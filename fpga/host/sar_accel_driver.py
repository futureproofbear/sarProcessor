"""Host driver for the SAR FFT accelerator (fpga/rtl) on the PolarFire SoC.

Drives the fabric over its AXI4-Lite register map (fpga/regmap.md): mmaps the
control registers via UIO, shares contiguous DDR buffers with the fabric AXI
master (udmabuf), and runs the pad -> 2-D BFP FFT -> detect datapath.

The fabric implements exactly the offloaded half of src/form_image_pfa_fixed.py,
so the public entry point is a DROP-IN for fixedpoint.focus_fixed:

    from sar_accel_driver import SarFftAccel
    accel = SarFftAccel()                       # opens /dev/uio0 + /dev/udmabuf0
    fixed_mag, (er, ea) = accel.focus_fixed(g2, NBITS)   # g2 = resampled k-space

i.e. in form_image_pfa_fixed.main() replace
    fixed_mag, (er, ea) = fp.focus_fixed(g2, NBITS, NBITS_TW)
with
    fixed_mag, (er, ea) = accel.focus_fixed(g2, NBITS)
and the rest (geo scaling, geocode, GeoTIFF) is unchanged. The CPU still does the
resample/window (ref.resample_kspace) and geocode; the fabric does pad+FFT+detect.

The resample/window/quantize stay in float/CPU on purpose -- the fabric input is
the already-resampled k-space `g2`, quantized here to int16 with a known input
exponent that is folded back into the output scale.

Run a laptop self-test (no hardware -- uses an in-process emulation of the
fabric) to exercise the quant/pack/scale plumbing against the NumPy reference:

    python fpga/host/sar_accel_driver.py --selftest
"""
import os
import sys
import time
import math
import mmap
import struct
import argparse
from pathlib import Path

import numpy as np

# input quantization shares fit_scale with the fixed-point reference
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
import fixedpoint as fp                                   # noqa: E402

# ----------------------------- register map -------------------------------- #
R_CTRL     = 0x00   # bit0 START (self-clearing), bit1 SOFT_RESET
R_STATUS   = 0x04   # bit0 DONE, bit1 BUSY, bit2 ERR, bit3 IRQ
R_IRQ_EN   = 0x08
R_M        = 0x0C
R_N        = 0x10
R_M2       = 0x14
R_N2       = 0x18
R_SIG_LO   = 0x1C
R_SIG_HI   = 0x20
R_BUF_LO   = 0x24
R_BUF_HI   = 0x28
R_OUT_LO   = 0x2C
R_OUT_HI   = 0x30
R_EXP_R    = 0x34
R_EXP_A    = 0x38
R_ID       = 0x3C

DESIGN_ID  = 0x5341_5246    # "SARF"
ST_DONE, ST_BUSY, ST_ERR, ST_IRQ = 1 << 0, 1 << 1, 1 << 2, 1 << 3


def _to_pow2(n):
    return 1 << int(math.ceil(math.log2(n)))


# ----------------------- input quant / output scale ------------------------ #
def quantize_input(g2, nbits):
    """Quantize the resampled k-space `g2` (complex) to packed int16 words and
    return (sig_u32, input_exp). Matches fixedpoint.fit_scale + truncating quant
    (floor) used by focus_fixed, but emits the integer grid the fabric reads:
    word = (uint16(im) << 16) | uint16(re)."""
    lsb, input_exp = fp.fit_scale(g2, nbits)
    full = 2 ** (nbits - 1) - 1
    re = np.clip(np.floor(g2.real / lsb), -full - 1, full).astype(np.int32)
    im = np.clip(np.floor(g2.imag / lsb), -full - 1, full).astype(np.int32)
    sig_u32 = ((im & 0xFFFF) << 16) | (re & 0xFFFF)
    return sig_u32.astype(np.uint32), input_exp


def descale_output(out_u32, input_exp, exp_r, exp_a):
    """Fabric magnitude (uint32, integer units) -> real-unit float32 magnitude.
    The block-floating-point right-shifts (exp_r over the range FFTs, exp_a over
    the azimuth FFTs) and the input LSB exponent are undone by one global scale:
        mag_float = OUT * 2^(input_exp + exp_r + exp_a)
    A single global scale only sets absolute brightness; the GeoTIFF path
    normalizes, so relative pixel values are what matter."""
    scale = 2.0 ** (int(input_exp) + int(exp_r) + int(exp_a))
    return (out_u32.astype(np.float64) * scale).astype(np.float32)


# --------------------------- accelerator base ------------------------------ #
class SarFftAccelBase:
    """Shared orchestration; subclasses provide register + DMA transport."""

    # -- transport hooks (subclass implements) --
    def _wr(self, off, val):       raise NotImplementedError
    def _rd(self, off):            raise NotImplementedError
    def _sig_view(self, nwords):   raise NotImplementedError   # writable uint32 view
    def _out_view(self, nwords):   raise NotImplementedError   # readable uint32 view
    def _phys(self):               raise NotImplementedError   # (sig, buf, out) phys
    def _wait_done(self, timeout): raise NotImplementedError

    def run(self, M, N, M2, N2, sig_u32, timeout=10.0):
        """Low-level: load sig, program registers, launch, return
        (out_u32 [M2*N2], exp_r, exp_a). Pure register-map sequence."""
        idv = self._rd(R_ID)
        if idv != DESIGN_ID:
            raise RuntimeError(f"accelerator ID 0x{idv:08x} != 0x{DESIGN_ID:08x}")

        self._sig_view(M * N)[:] = sig_u32                  # host -> DDR (SIG)
        sig_p, buf_p, out_p = self._phys()

        self._wr(R_M, M);   self._wr(R_N, N)
        self._wr(R_M2, M2); self._wr(R_N2, N2)
        self._wr(R_SIG_LO, sig_p & 0xFFFFFFFF); self._wr(R_SIG_HI, sig_p >> 32)
        self._wr(R_BUF_LO, buf_p & 0xFFFFFFFF); self._wr(R_BUF_HI, buf_p >> 32)
        self._wr(R_OUT_LO, out_p & 0xFFFFFFFF); self._wr(R_OUT_HI, out_p >> 32)

        self._wr(R_CTRL, 0x1)                               # START
        self._wait_done(timeout)

        st = self._rd(R_STATUS)
        if st & ST_ERR:
            raise RuntimeError("STATUS.ERR -- N2/M2 must equal the built FFT "
                               "lengths (FFT_LEN_R/FFT_LEN_A); see fpga/README.md")
        exp_r = self._rd(R_EXP_R) & 0x1F
        exp_a = self._rd(R_EXP_A) & 0x1F
        out_u32 = np.array(self._out_view(M2 * N2), dtype=np.uint32)  # DDR -> host
        return out_u32, exp_r, exp_a

    def focus_fixed(self, g2, nbits=16, nbits_tw=18, timeout=10.0):
        """Drop-in for fixedpoint.focus_fixed: fixed-point focus of an already
        resampled+windowed k-space `g2`. Returns (detected_magnitude float32
        [M2,N2], (exps_range, exps_azimuth)).

        `nbits_tw` is accepted for signature parity; the twiddle width is fixed
        in the CoreFFT build. The returned exponent lists carry the fabric's two
        max block exponents (not per-stage), so er[-1]-er[0] = total range guard
        bits and ea[-1]-ea[0] = total azimuth guard bits, matching how
        form_image_pfa_fixed.py consumes them."""
        M, N = g2.shape
        M2, N2 = _to_pow2(M), _to_pow2(N)
        sig_u32, input_exp = quantize_input(g2, nbits)
        out_u32, exp_r, exp_a = self.run(M, N, M2, N2, sig_u32.ravel(), timeout)
        mag = descale_output(out_u32, input_exp, exp_r, exp_a).reshape(M2, N2)
        return mag, ([0, int(exp_r)], [0, int(exp_a)])


# --------------------------- hardware backend ------------------------------ #
class _Udmabuf:
    """Contiguous DDR pool shared with the fabric AXI master (u-dma-buf).
    Carves page-aligned SIG / BUF / OUT sub-regions and hands out NumPy views.

    Assumes the accelerator is on the PolarFire SoC *cache-coherent* FIC (the
    recommended path), so no manual cache flush/invalidate is needed. For a
    non-coherent FIC, allocate udmabuf with sync ops and call them around run()."""
    def __init__(self, name="udmabuf0"):
        base = f"/sys/class/u-dma-buf/{name}"
        self.size = int(open(f"{base}/size").read())
        self.phys = int(open(f"{base}/phys_addr").read().strip(), 16)
        self._fd = os.open(f"/dev/{name}", os.O_RDWR | os.O_SYNC)
        self._mm = mmap.mmap(self._fd, self.size)
        self._off = 0

    def _carve(self, nbytes):
        nbytes = (nbytes + 0xFFF) & ~0xFFF                  # page align
        if self._off + nbytes > self.size:
            raise MemoryError(f"udmabuf {self.size} B too small for SIG+BUF+OUT")
        start = self._off
        self._off += nbytes
        return start

    def reserve(self, sig_bytes, buf_bytes, out_bytes):
        self._off = 0
        self._sig = self._carve(sig_bytes)
        self._buf = self._carve(buf_bytes)
        self._out = self._carve(out_bytes)
        return (self.phys + self._sig, self.phys + self._buf, self.phys + self._out)

    def view(self, which, nwords):
        off = {"sig": self._sig, "out": self._out}[which]
        return np.frombuffer(self._mm, dtype=np.uint32, count=nwords, offset=off)


class SarFftAccel(SarFftAccelBase):
    """Real hardware: AXI4-Lite via UIO, DDR buffers via u-dma-buf."""
    def __init__(self, uio="/dev/uio0", udmabuf="udmabuf0",
                 reg_size=0x1000, use_irq=False):
        self._fd = os.open(uio, os.O_RDWR | os.O_SYNC)
        self._regs = mmap.mmap(self._fd, reg_size)
        self._pool = _Udmabuf(udmabuf)
        self._use_irq = use_irq
        self._sig_p = self._buf_p = self._out_p = 0

    # registers
    def _wr(self, off, val):
        self._regs[off:off + 4] = struct.pack("<I", int(val) & 0xFFFFFFFF)

    def _rd(self, off):
        return struct.unpack("<I", self._regs[off:off + 4])[0]

    # buffers (reserved lazily on first run from the requested sizes)
    def _ensure(self, M, N, M2, N2):
        self._sig_p, self._buf_p, self._out_p = self._pool.reserve(
            M * N * 4, M2 * N2 * 4, M2 * N2 * 4)
        self._M, self._N, self._M2, self._N2 = M, N, M2, N2

    def _phys(self):
        return self._sig_p, self._buf_p, self._out_p

    def _sig_view(self, nwords):
        return self._pool.view("sig", nwords)

    def _out_view(self, nwords):
        return self._pool.view("out", nwords)

    def _wait_done(self, timeout):
        if self._use_irq:
            self._wr(R_IRQ_EN, 1)
            os.read(self._fd, 4)                            # blocks on UIO irq
            self._wr(R_IRQ_EN, 1)                           # re-arm
            return
        t0 = time.perf_counter()
        while not (self._rd(R_STATUS) & (ST_DONE | ST_ERR)):
            if time.perf_counter() - t0 > timeout:
                raise TimeoutError("accelerator DONE not seen")
            time.sleep(0.0005)

    def run(self, M, N, M2, N2, sig_u32, timeout=10.0):
        self._ensure(M, N, M2, N2)
        return super().run(M, N, M2, N2, sig_u32, timeout)


# --------------------------- mock backend ---------------------------------- #
class MockSarFftAccel(SarFftAccelBase):
    """In-process emulation for laptop development / CI: implements the same
    register + buffer protocol but computes the result with the bit-faithful
    NumPy model the RTL is built from (fpga/scripts/gen_vectors.hw_model). No
    /dev/uio, no hardware. Validates the driver plumbing end to end."""
    def __init__(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
        import gen_vectors                                 # noqa: E402
        self._model = gen_vectors.hw_model
        self._regs = {R_ID: DESIGN_ID, R_STATUS: 0}
        self._sig = self._out = None
        self._exp_r = self._exp_a = 0

    def _wr(self, off, val):
        self._regs[off] = int(val) & 0xFFFFFFFF
        if off == R_CTRL and (val & 0x1):
            self._compute()

    def _rd(self, off):
        return self._regs.get(off, 0)

    def _sig_view(self, nwords):
        self._sig = np.zeros(nwords, np.uint32)
        return self._sig

    def _out_view(self, nwords):
        return self._out

    def _phys(self):
        return 0x1000, 0x100000, 0x200000                  # arbitrary, unused

    def _wait_done(self, timeout):
        return                                             # _compute already ran

    def _compute(self):
        M, N = self._regs[R_M], self._regs[R_N]
        M2, N2 = self._regs[R_M2], self._regs[R_N2]
        if N2 != _to_pow2(N) or M2 != _to_pow2(M):
            self._regs[R_STATUS] = ST_DONE | ST_ERR
            return
        w = self._sig.reshape(M, N)
        re = (w & 0xFFFF).astype(np.int16)                 # int16 wrap
        im = (w >> 16).astype(np.int16)
        sig = re.astype(np.int64) + 1j * im.astype(np.int64)
        g = self._model(sig, M2, N2)
        self._out = g["out"].ravel().astype(np.uint32)
        self._exp_r, self._exp_a = g["exp_r"], g["exp_a"]
        self._regs[R_EXP_R] = self._exp_r
        self._regs[R_EXP_A] = self._exp_a
        self._regs[R_STATUS] = ST_DONE


# --------------------------------- self-test ------------------------------- #
def _selftest():
    rng = np.random.default_rng(0)
    M, N = 12, 9
    # a couple of tones + noise -> realistic resampled k-space
    y, x = np.mgrid[0:M, 0:N]
    g2 = (7000 * np.exp(2j * np.pi * (3 * y / M + 2 * x / N))
          + 3000 * np.exp(2j * np.pi * (1 * y / M + 4 * x / N))
          + (rng.standard_normal((M, N)) + 1j * rng.standard_normal((M, N))) * 40
          ).astype(np.complex64)

    accel = MockSarFftAccel()
    mag, (er, ea) = accel.focus_fixed(g2, nbits=16)
    ref_mag, _ = fp.focus_fixed(g2, 16, 18)                # NumPy fixed-point ref

    # normalize (only relative pixel structure is meaningful)
    a = mag / (mag.max() + 1e-30)
    b = ref_mag / (ref_mag.max() + 1e-30)
    corr = float(np.corrcoef(a.ravel(), b.ravel())[0, 1])
    print(f"selftest: focus_fixed via driver+mock  shape={mag.shape}  "
          f"EXP_R={er[-1]} EXP_A={ea[-1]}")
    print(f"  correlation vs fixedpoint.focus_fixed = {corr:.4f}  (want ~1.0)")
    assert mag.shape == (_to_pow2(M), _to_pow2(N))
    assert corr > 0.99, f"driver output decorrelated from reference ({corr:.3f})"
    print("  PASS")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="SAR FFT accelerator host driver")
    ap.add_argument("--selftest", action="store_true",
                    help="run laptop emulation self-test (no hardware)")
    ap.add_argument("--uio", default="/dev/uio0")
    ap.add_argument("--udmabuf", default="udmabuf0")
    a = ap.parse_args()
    if a.selftest:
        _selftest()
    else:
        accel = SarFftAccel(uio=a.uio, udmabuf=a.udmabuf)
        print(f"opened {a.uio} + {a.udmabuf}; ID=0x{accel._rd(R_ID):08x}")
        print("use accel.focus_fixed(g2, nbits) -- see module docstring.")
