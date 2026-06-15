"""Fixed-point / block-floating-point emulation for the SAR focuser.

Emulates, in NumPy, what the PolarFire fabric would do: quantize the signal and
twiddles to N-bit fixed point and run a block-floating-point (BFP) FFT that
rescales per stage to hold dynamic range in a fixed word width. Used to (a) size
the datapath bit widths, (b) measure the per-stage scale shifting BFP needs, and
(c) quantify the information loss vs the float reference.

Run as a script for the quantization study:
    python src/fixedpoint.py            # uses the scene in config.yaml
"""
import sys
import math
from pathlib import Path

import numpy as np

# ----------------------------- fixed-point core ---------------------------- #
def fit_scale(arr, nbits):
    """Smallest power-of-2 LSB so |arr| fits in signed nbits. Returns (lsb, exp)."""
    M = max(float(np.abs(arr.real).max()), float(np.abs(arr.imag).max()))
    if M == 0.0:
        return 1.0, 0
    full = 2 ** (nbits - 1) - 1
    exp = math.ceil(math.log2(M / full))
    return 2.0 ** exp, exp


def quant(arr, lsb, nbits):
    """Round complex arr to a signed nbits grid with step `lsb` (saturating)."""
    full = 2 ** (nbits - 1) - 1
    re = np.clip(np.round(arr.real / lsb), -full - 1, full)
    im = np.clip(np.round(arr.imag / lsb), -full - 1, full)
    return (re + 1j * im) * lsb


def _bitrev_perm(n):
    bits = int(round(math.log2(n)))
    return np.array([int(format(i, f"0{bits}b")[::-1], 2) for i in range(n)])


def fft1d_bfp(x, nbits, nbits_tw, perm):
    """Radix-2 DIT FFT along the last axis with block-floating-point requantize
    after every stage. Vectorized across all other axes (one 'line' per row).
    Returns (transformed, per_stage_exponents)."""
    n = x.shape[-1]
    stages = int(round(math.log2(n)))
    x = x[..., perm]
    lsb, e = fit_scale(x, nbits)
    x = quant(x, lsb, nbits)
    exps = [e]
    lsb_tw = 2.0 ** -(nbits_tw - 1)                 # twiddles in ~[-1, 1)
    for s in range(1, stages + 1):
        m = 1 << s; mh = m >> 1
        w = quant(np.exp(-2j * np.pi * np.arange(mh) / m), lsb_tw, nbits_tw).astype(x.dtype, copy=False)
        xr = x.reshape(*x.shape[:-1], n // m, m)
        a = xr[..., :mh]; b = xr[..., mh:]
        t = w * b                                    # complex multiply (grows)
        x = np.concatenate([a + t, a - t], axis=-1).reshape(*x.shape[:-1], n)
        lsb, e = fit_scale(x, nbits)                 # BFP: rescale to word width
        x = quant(x, lsb, nbits)
        exps.append(e)
    return x, exps


def fft2_bfp(x, nbits, nbits_tw=None):
    """2-D BFP FFT (range axis then azimuth axis). Returns (img, (exps_r, exps_a))."""
    nbits_tw = nbits_tw or nbits
    pr = _bitrev_perm(x.shape[-1])
    y, er = fft1d_bfp(x, nbits, nbits_tw, pr)
    pa = _bitrev_perm(x.shape[-2])
    y, ea = fft1d_bfp(np.swapaxes(y, -1, -2), nbits, nbits_tw, pa)
    return np.swapaxes(y, -1, -2), (er, ea)


# ------------------------------- assessment -------------------------------- #
def dynamic_range_db(mag):
    """Usable dynamic range: brightest target over the speckle/background floor."""
    hi = np.percentile(mag, 99.99)
    lo = np.percentile(mag[mag > 0], 5) if np.any(mag > 0) else 1e-12
    return 20 * np.log10(hi / (lo + 1e-12))


def compare(ref_mag, test_mag):
    """Information-loss metrics between float ref and fixed-point test images."""
    r = ref_mag / (ref_mag.max() + 1e-30)
    t = test_mag / (test_mag.max() + 1e-30)
    err = t - r
    rms_ref = np.sqrt(np.mean(r ** 2))
    rms_err = np.sqrt(np.mean(err ** 2)) + 1e-30
    snr = 20 * np.log10(rms_ref / rms_err)
    return {
        "nrmse": rms_err / rms_ref,
        "snr_db": snr,
        "enob": (snr - 1.76) / 6.02,            # effective number of bits
        "corr": float(np.corrcoef(r.ravel(), t.ravel())[0, 1]),
        "dr_ref_db": dynamic_range_db(ref_mag),
        "dr_test_db": dynamic_range_db(test_mag),
    }


# ----------------------- fixed-point PFA focuser --------------------------- #
def _to_pow2(n):
    return 1 << int(math.ceil(math.log2(n)))


def focus_fixed(sig_win, nbits, nbits_tw=None):
    """Fixed-point focus of an already-resampled+windowed signal: quantize input,
    zero-pad to power-of-2 (as the fabric does), BFP 2-D FFT, detect. Returns
    (detected_magnitude, bfp_exponents)."""
    m2, n2 = _to_pow2(sig_win.shape[0]), _to_pow2(sig_win.shape[1])
    pad = np.zeros((m2, n2), dtype=np.complex64)   # complex64 to halve RAM at full res
    pad[:sig_win.shape[0], :sig_win.shape[1]] = sig_win
    lsb, _ = fit_scale(pad, nbits)
    pad = quant(pad, lsb, nbits)                    # input quantization
    img, exps = fft2_bfp(np.fft.ifftshift(pad), nbits, nbits_tw)
    img = np.fft.fftshift(img)
    return np.abs(img).astype(np.float32), exps


# --------------------------------- study ----------------------------------- #
def run_study(crop=2048, bits=(8, 10, 12, 14, 16, 18), out_dir=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import form_image_pfa as ref

    out_dir = Path(out_dir or ref.OUTPUT_DIR); out_dir.mkdir(parents=True, exist_ok=True)
    rdr = ref.open_phase_history(str(ref.LOCAL_CPHD))
    nv, ns = rdr.get_data_size_as_tuple()[0]
    P = min(crop, _to_pow2(min(nv, ns)) // 2)        # power-of-2 center crop
    r0, c0 = (nv - P) // 2, (ns - P) // 2
    sig = np.asarray(rdr.read_chip((r0, r0 + P, 1), (c0, c0 + P, 1), index=0),
                     dtype=np.complex128)
    rdr.close()
    win = sig * np.outer(np.hamming(P), np.hamming(P))
    print(f"study crop {P}x{P} from {ref.KEY.split('/')[2]}\n")

    # float reference (double precision, same transform)
    ref_mag = np.abs(np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(win))))

    print(f"{'bits':>4} {'SNR dB':>7} {'ENOB':>5} {'corr':>6} {'NRMSE':>7} "
          f"{'DR ref':>7} {'DR fix':>7} {'BFP guard(r/a)':>15}")
    rows = []
    for nb in bits:
        m2, n2 = _to_pow2(P), _to_pow2(P)
        pad = np.zeros((m2, n2), np.complex128); pad[:P, :P] = win
        lsb, _ = fit_scale(pad, nb); padq = quant(pad, lsb, nb)
        fimg, (er, ea) = fft2_bfp(np.fft.ifftshift(padq), nb)
        test_mag = np.abs(np.fft.fftshift(fimg))
        # crop ref to padded grid for fair compare
        rmag = np.abs(np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(pad))))
        c = compare(rmag, test_mag); c["bits"] = nb
        # BFP guard bits = cumulative block-exponent growth across the FFT stages
        c["shift_r"], c["shift_a"] = er[-1] - er[0], ea[-1] - ea[0]
        rows.append(c)
        print(f"{nb:>4} {c['snr_db']:>7.1f} {c['enob']:>5.1f} {c['corr']:>6.3f} "
              f"{c['nrmse']:>7.4f} {c['dr_ref_db']:>6.1f}d {c['dr_test_db']:>6.1f}d "
              f"{c['shift_r']:>7}/{c['shift_a']:<6}")

    # plots
    b = [r["bits"] for r in rows]
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    ax[0].plot(b, [r["snr_db"] for r in rows], "o-", label="SNR (dB)")
    ax[0].plot(b, [r["dr_test_db"] for r in rows], "s-", label="usable DR (dB)")
    ax[0].axhline(rows[-1]["dr_ref_db"], ls="--", c="k", label="float DR")
    ax[0].set_xlabel("datapath bits"); ax[0].set_ylabel("dB")
    ax[0].set_title("Information loss vs bit width"); ax[0].legend(); ax[0].grid(True)
    ax[1].plot(b, [r["enob"] for r in rows], "o-")
    ax[1].set_xlabel("datapath bits"); ax[1].set_ylabel("ENOB (bits)")
    ax[1].set_title("Effective number of bits"); ax[1].grid(True)
    fig.tight_layout(); fig.savefig(out_dir / "fixedpoint_study.png", dpi=120)
    print(f"\nwrote {out_dir/'fixedpoint_study.png'}")
    return rows


if __name__ == "__main__":
    run_study()
