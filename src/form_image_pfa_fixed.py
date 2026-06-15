"""Fixed-point version of form_image_pfa.py (FPGA-datapath emulation).

Same PFA focuser, but the offloaded datapath (resample -> window -> 2-D FFT ->
detect) runs in fixed point with a block-floating-point FFT (see fixedpoint.py).
This is what the PolarFire fabric would compute. Geocoding/GeoTIFF stays the
float CPU step (unchanged from the reference), so it is not repeated here.

It loads the scene from config.yaml (decimated by default so the Python BFP FFT
is fast), forms BOTH the float and the fixed-point image of the SAME pipeline,
and reports the information loss + the BFP scale-shift schedule.

    python src/form_image_pfa_fixed.py            # 16-bit by default
"""
import sys
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
import form_image_pfa as ref
import fixedpoint as fp

C = ref.C
NBITS = 16             # datapath bit width to emulate (matches CoreFFT config)
NBITS_TW = 18          # twiddle bit width (18x18 DSP)
FULL = True            # True -> full resolution; False -> decimate to TARGET_MAX
TARGET_MAX = 1536      # used only when FULL is False (pad -> 2048)


def resample_kspace(sig, freq, ax, ay):
    """Polar->Cartesian keystone resample + window (mirrors form_image_pfa.pfa,
    returns the windowed k-space that feeds the 2-D FFT)."""
    m, n = sig.shape
    kmag = 2.0 * freq / C
    dx, dy = ax.mean(), ay.mean()
    dn = math.hypot(dx, dy); dx, dy = dx / dn, dy / dn
    cx, cy = -dy, dx
    pr = ax * dx + ay * dy
    pc = ax * cx + ay * cy
    kr = kmag * pr[:, None]
    tan_phi = pc / pr
    kr_lo, kr_hi = kr.max(axis=1).min(), kr.min(axis=1).max()
    if kr_lo > kr_hi:
        kr_lo, kr_hi = kr.min(), kr.max()
    KR = np.linspace(kr_lo, kr_hi, n)
    g1 = np.empty((m, n), np.complex128)
    for i in range(m):
        xp = kr[i]; row = sig[i]
        if xp[0] > xp[-1]:
            xp = xp[::-1]; row = sig[i, ::-1]
        g1[i] = (np.interp(KR, xp, row.real, left=0, right=0)
                 + 1j * np.interp(KR, xp, row.imag, left=0, right=0))
    kc_max = np.abs(KR[:, None] * tan_phi[None, :]).max()
    KC = np.linspace(-kc_max, kc_max, m)
    order = np.argsort(tan_phi); tan_s = tan_phi[order]; g1s = g1[order]
    g2 = np.empty((m, n), np.complex128)
    for j in range(n):
        src = KR[j] * tan_s
        g2[:, j] = (np.interp(KC, src, g1s[:, j].real, left=0, right=0)
                    + 1j * np.interp(KC, src, g1s[:, j].imag, left=0, right=0))
    return g2 * np.outer(np.hamming(m), np.hamming(n))


def main():
    rdr = ref.open_phase_history(str(ref.LOCAL_CPHD))
    meta = rdr.cphd_meta
    nv, ns = rdr.get_data_size_as_tuple()[0]
    mu = 1 if FULL else max(1, math.ceil(nv / TARGET_MAX))
    nu = 1 if FULL else max(1, math.ceil(ns / TARGET_MAX))
    print(f"[fixed] scene {ref.KEY.split('/')[2]}  {nv}x{ns}  decimate {mu}x{nu}  "
          f"NBITS={NBITS} (tw {NBITS_TW})")

    sc0 = rdr.read_pvp_variable("SC0", 0)[::mu]
    scss = rdr.read_pvp_variable("SCSS", 0)[::mu]
    tx = rdr.read_pvp_variable("TxPos", 0)[::mu]
    rcvp = rdr.read_pvp_variable("RcvPos", 0)[::mu]
    srp = rdr.read_pvp_variable("SRPPos", 0)[::mu]
    plan = meta.SceneCoordinates.ReferenceSurface.Planar
    uiax = np.array([plan.uIAX.X, plan.uIAX.Y, plan.uIAX.Z])
    uiay = np.array([plan.uIAY.X, plan.uIAY.Y, plan.uIAY.Z])
    u = (srp - 0.5 * (tx + rcvp))
    u = u / np.linalg.norm(u, axis=1, keepdims=True)
    ax, ay = u @ uiax, u @ uiay

    sig = np.asarray(rdr.read_chip((0, nv, mu), (0, ns, nu), index=0), np.complex128)
    rdr.close()
    k = np.arange(sig.shape[1])
    freq = sc0[:, None] + k[None, :] * (scss[:, None] * nu)

    g2 = resample_kspace(sig, freq, ax, ay).astype(np.complex64)
    m2, n2 = fp._to_pow2(g2.shape[0]), fp._to_pow2(g2.shape[1])
    print(f"[fixed] resampled k-space {g2.shape} -> FFT {m2}x{n2}; focusing (BFP)...")

    fixed_mag, (er, ea) = fp.focus_fixed(g2, NBITS, NBITS_TW)
    print(f"  fixed dynamic range : {fp.dynamic_range_db(fixed_mag):.1f} dB")
    print(f"  BFP guard bits (range/azimuth FFT): {er[-1]-er[0]} / {ea[-1]-ea[0]}")

    # compare to the same-pipeline float reference only when small enough to fit
    if m2 * n2 <= 4096 * 4096:
        pad = np.zeros((m2, n2), np.complex64); pad[:g2.shape[0], :g2.shape[1]] = g2
        float_mag = np.abs(np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(pad))))
        c = fp.compare(float_mag, fixed_mag)
        print(f"  vs float: SNR {c['snr_db']:.1f} dB, ENOB {c['enob']:.1f}, "
              f"corr {c['corr']:.4f}, DR loss {c['dr_ref_db']-c['dr_test_db']:.1f} dB")

    # save the full fixed-point SAR image (dB)
    ref_p = np.percentile(fixed_mag, 99.7)
    dbimg = 20 * np.log10(fixed_mag / (ref_p + 1e-12) + 1e-6)
    out = ref.OUTPUT_DIR / f"{ref._stem}_fixed{NBITS}bit_full.png"
    ref.OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(9, 9))
    plt.imshow(dbimg, cmap="gray", vmin=-30, vmax=5, origin="lower")
    plt.title(f"{ref.KEY.split('/')[2]} - fixed {NBITS}-bit ({m2}x{n2})")
    plt.tight_layout(); plt.savefig(out, dpi=140)
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
