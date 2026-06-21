"""Generate input + golden simulation vectors for the SAR FFT accelerator.

This is the *executable specification* of what the RTL computes. It models the
fabric pipeline exactly as `rtl/sar_ctrl.sv` + the behavioral `corefft_model.sv`
implement it:

    pad to pow2 -> ifftshift (top-bit toggle) -> per-frame-BFP range FFT ->
    normalize rows to common exponent -> per-frame-BFP azimuth FFT ->
    normalize cols to common exponent -> fftshift -> detect |re,im| (floor isqrt)

It is deliberately INDEPENDENT of src/fixedpoint.py: that reference uses a
global-per-stage BFP, whereas a CoreFFT-based datapath does per-frame BFP. This
model matches the hardware; the testbench checks RTL == this model (tolerance +
correlation), and we also print correlation vs the ideal float FFT as a sanity
check that the architecture is algorithmically right.

Usage:
    python fpga/scripts/gen_vectors.py --m 10 --n 6      # M x N input, padded to pow2
Outputs (fpga/sim/vectors/):
    params.vh        Verilog `define header (dims, addresses, exponents, tol)
    sig.hex          input k-space, M*N words, {im[15:0],re[15:0]} hex
    golden.hex       golden magnitude image, M2*N2 words, uint32 hex
    meta.json        human-readable summary
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

# ----- fixed-point datapath parameters (keep in sync with rtl params) -------- #
DIN_W = 16          # CoreFFT input width (bits, signed)
DOUT_W = 16         # CoreFFT output width (bits, signed) -> BUF word is int16 cplx
MAG_W = 32          # output magnitude width (uint32)

VEC_DIR = Path(__file__).resolve().parent.parent / "sim" / "vectors"


def to_pow2(n):
    return 1 << int(math.ceil(math.log2(n)))


def s16(x):
    """wrap a python int array to signed 16-bit (two's complement), as int."""
    return ((np.asarray(x, dtype=np.int64) + 0x8000) & 0xFFFF) - 0x8000


def corefft_bfp(x, dout_w):
    """Behavioral CoreFFT: forward FFT of complex-int line `x`, block-floating-
    point scaled to signed `dout_w`. Returns (y_int_complex, blk_exp).

    Mirrors sim/corefft_model.sv: full forward DFT (no 1/N), then right-shift by
    the smallest blk so the result fits signed dout_w, truncating (floor) the
    fractional bits -- the PolarFire datapath truncates rather than rounds."""
    X = np.fft.fft(x.astype(np.complex128))                 # forward, unscaled
    maxabs = max(float(np.abs(X.real).max()), float(np.abs(X.imag).max()))
    full = 2 ** (dout_w - 1) - 1
    if maxabs == 0.0:
        return np.zeros(len(x), dtype=np.complex128), 0
    blk = max(0, int(math.ceil(math.log2(maxabs / full))))  # right shifts (>=0)
    yr = np.clip(np.floor(X.real / (2 ** blk)), -full - 1, full)
    yi = np.clip(np.floor(X.imag / (2 ** blk)), -full - 1, full)
    return yr + 1j * yi, blk


def arsh(v, s):
    """arithmetic right shift of a (possibly complex-int) value by s (floor)."""
    return np.floor(v.real / (2 ** s)) + 1j * np.floor(v.imag / (2 ** s))


def hw_model(sig, m2, n2):
    """Full fabric pipeline on int-complex `sig` (M x N). Returns dict with the
    golden magnitude image (M2 x N2, uint32) plus exp_r/exp_a, matching the RTL."""
    M, N = sig.shape
    hm, hn = m2 // 2, n2 // 2

    # ---- PASS 1: range FFT over each padded row (col ifftshift = toggle bit) --
    buf = np.zeros((m2, n2), dtype=np.complex128)
    exp_r = np.zeros(m2, dtype=np.int64)
    for r in range(m2):
        line = np.zeros(n2, dtype=np.complex128)
        if r < M:
            for q in range(N):                       # pad col q valid if q<N
                line[q ^ hn] = sig[r, q]             # ifftshift cols
        y, e = corefft_bfp(line, DOUT_W)
        buf[r] = y
        exp_r[r] = e
    max_r = int(exp_r.max())

    # ---- PASS 2: azimuth FFT over each col (row ifftshift + renorm to max_r) --
    img = np.zeros((m2, n2), dtype=np.complex128)
    exp_a = np.zeros(n2, dtype=np.int64)
    for c in range(n2):
        line = np.zeros(m2, dtype=np.complex128)
        for p in range(m2):
            row = p ^ hm                              # ifftshift rows
            line[p] = arsh(buf[row, c], max_r - exp_r[row])
        y, e = corefft_bfp(line, DOUT_W)
        img[:, c] = y
        exp_a[c] = e
    max_a = int(exp_a.max())

    # ---- DETECT: fftshift (toggle both) + renorm to max_a + |re,im| (floor) ---
    out = np.zeros((m2, n2), dtype=np.uint32)
    for i in range(m2):
        for j in range(n2):
            v = arsh(img[i ^ hm, j ^ hn], max_a - exp_a[j ^ hn])
            mag = int(math.isqrt(int(v.real) ** 2 + int(v.imag) ** 2))   # floor sqrt
            out[i, j] = min(mag, (1 << MAG_W) - 1)
    return {"out": out, "exp_r": max_r, "exp_a": max_a, "img": img}


def ideal_mag(sig, m2, n2):
    """Ideal float 2-D FFT magnitude (no quantization) for an architecture sanity
    check: pad, ifftshift, fft2, fftshift, |.|."""
    pad = np.zeros((m2, n2), dtype=np.complex128)
    pad[: sig.shape[0], : sig.shape[1]] = sig
    return np.abs(np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(pad))))


def make_scene(M, N, seed=1):
    """Synthetic complex k-space: a couple of point targets (=> sinc image) plus
    low-level noise, so the image has real structure and dynamic range to focus.
    Quantized to int16, the format the host would pre-load into DDR."""
    rng = np.random.default_rng(seed)
    y, x = np.mgrid[0:M, 0:N]
    sig = np.zeros((M, N), dtype=np.complex128)
    for (fy, fx, a) in [(2.0, 1.0, 9000.0), (5.0, 3.0, 4000.0)]:   # tones -> points
        sig += a * np.exp(2j * np.pi * (fy * y / M + fx * x / N))
    sig += (rng.standard_normal((M, N)) + 1j * rng.standard_normal((M, N))) * 50.0
    return s16(np.round(sig.real)) + 1j * s16(np.round(sig.imag))


def pack(c):
    re = int(c.real) & 0xFFFF
    im = int(c.imag) & 0xFFFF
    return (im << 16) | re


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--m", type=int, default=10, help="input rows (pulses)")
    ap.add_argument("--n", type=int, default=6, help="input cols (samples)")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    M, N = args.m, args.n
    m2, n2 = to_pow2(M), to_pow2(N)
    sig = make_scene(M, N, args.seed)
    g = hw_model(sig, m2, n2)
    out = g["out"]

    # architecture sanity: hw magnitude vs ideal float fft magnitude
    ref = ideal_mag(sig, m2, n2)
    corr = float(np.corrcoef(out.ravel().astype(float), ref.ravel())[0, 1])

    VEC_DIR.mkdir(parents=True, exist_ok=True)
    # input k-space, row-major
    with open(VEC_DIR / "sig.hex", "w") as f:
        for r in range(M):
            for c in range(N):
                f.write(f"{pack(sig[r, c]):08x}\n")
    # golden magnitude, row-major
    with open(VEC_DIR / "golden.hex", "w") as f:
        for i in range(m2):
            for j in range(n2):
                f.write(f"{int(out[i, j]):08x}\n")

    # DDR byte addresses for the three buffers (word = 4 bytes); keep them apart.
    sig_words, buf_words, out_words = M * N, m2 * n2, m2 * n2
    sig_addr = 0x0000_1000
    buf_addr = sig_addr + ((sig_words * 4 + 0xFFF) & ~0xFFF) + 0x1000
    out_addr = buf_addr + ((buf_words * 4 + 0xFFF) & ~0xFFF) + 0x1000

    # tolerance: per-frame double-precision DFT in the SV model vs numpy fft here
    # differ by a few ULP -> at most a few LSB after floor; addressing bugs are
    # orders of magnitude larger, so a small absolute tol still catches them.
    tol = 3

    params = VEC_DIR / "params.vh"
    with open(params, "w") as f:
        f.write("// auto-generated by gen_vectors.py -- do not edit\n")
        f.write(f"`define VEC_M   {M}\n")
        f.write(f"`define VEC_N   {N}\n")
        f.write(f"`define VEC_M2  {m2}\n")
        f.write(f"`define VEC_N2  {n2}\n")
        f.write(f"`define VEC_SIG_ADDR 64'h{sig_addr:016x}\n")
        f.write(f"`define VEC_BUF_ADDR 64'h{buf_addr:016x}\n")
        f.write(f"`define VEC_OUT_ADDR 64'h{out_addr:016x}\n")
        f.write(f"`define VEC_EXP_R {g['exp_r']}\n")
        f.write(f"`define VEC_EXP_A {g['exp_a']}\n")
        f.write(f"`define VEC_MAG_TOL {tol}\n")
        f.write(f'`define VEC_SIG_HEX "sig.hex"\n')
        f.write(f'`define VEC_GOLDEN_HEX "golden.hex"\n')

    meta = {
        "M": M, "N": N, "M2": m2, "N2": n2,
        "sig_addr": hex(sig_addr), "buf_addr": hex(buf_addr), "out_addr": hex(out_addr),
        "exp_r": g["exp_r"], "exp_a": g["exp_a"], "mag_tol": tol,
        "out_max": int(out.max()), "arch_corr_vs_float_fft": round(corr, 4),
    }
    (VEC_DIR / "meta.json").write_text(json.dumps(meta, indent=2))

    print(f"wrote vectors to {VEC_DIR}")
    print(f"  scene {M}x{N} -> pad {m2}x{n2}   EXP_R={g['exp_r']} EXP_A={g['exp_a']}")
    print(f"  out max {int(out.max())}   arch corr vs float FFT = {corr:.4f}  (want ~1.0)")
    print(f"  mag tolerance for TB = +/-{tol} LSB")


if __name__ == "__main__":
    main()
