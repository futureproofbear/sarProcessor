"""Storage-to-storage board runner: focus a CPHD on the PolarFire SoC fabric.

The on-board counterpart of src/form_image_pfa_fixed.py. Same pipeline, same
GeoTIFF, but the pad -> 2-D BFP FFT -> detect runs on the fabric accelerator
(fpga/rtl) via sar_accel_driver instead of the NumPy emulation:

    [CPU]  open CPHD + PVP  ->  resample_kspace (polar->Cartesian) + window
    [FPGA] accel.focus_fixed(g2)  =  pad -> ifftshift -> 2-D BFP FFT -> detect
    [CPU]  scale geo to padded grid  ->  geocode (ECEF->UTM)  ->  GeoTIFF

The resample/window/geocode are imported from the verified reference
(src/form_image_pfa.py) so the float, NumPy-fixed and FPGA pipelines cannot
drift: they differ ONLY in where the FFT/detect arithmetic runs.

    # on the board (needs bitstream + /dev/uio0 + /dev/udmabuf0):
    python fpga/host/run_on_board.py --in <scene>_CPHD.cphd --out /data/out

    # anywhere (laptop CI): same pipeline, fabric emulated in NumPy:
    python fpga/host/run_on_board.py --in <scene>_CPHD.cphd --out out --backend mock
"""
import sys
import argparse
from pathlib import Path

import numpy as np

REF_DIR = Path(__file__).resolve().parents[2] / "src"
sys.path.insert(0, str(REF_DIR))
import form_image_pfa as ref                              # noqa: E402

from sar_accel_driver import SarFftAccel, MockSarFftAccel  # noqa: E402

NBITS = 16


def make_accel(name, uio, udmabuf):
    if name == "fpga":
        return SarFftAccel(uio=uio, udmabuf=udmabuf)
    if name == "mock":
        return MockSarFftAccel()
    raise ValueError(f"unknown backend {name}")


def process(cphd_path, out_dir, accel, deci_pulse=1, deci_sample=1):
    cphd_path, out_dir = Path(cphd_path), Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    t = ref.StepTimer()

    rdr = ref.open_phase_history(str(cphd_path))
    meta = rdr.cphd_meta
    sgn = int(meta.Global.SGN)
    assert sgn < 0, "fabric BFP FFT is forward-only; matches SGN=-1"
    nv, ns = rdr.get_data_size_as_tuple()[0]
    mu, nu = max(1, deci_pulse), max(1, deci_sample)

    sc0 = rdr.read_pvp_variable("SC0", 0)[::mu]
    scss = rdr.read_pvp_variable("SCSS", 0)[::mu]
    tx = rdr.read_pvp_variable("TxPos", 0)[::mu]
    rcvp = rdr.read_pvp_variable("RcvPos", 0)[::mu]
    srp = rdr.read_pvp_variable("SRPPos", 0)[::mu]
    plan = meta.SceneCoordinates.ReferenceSurface.Planar
    uiax = np.array([plan.uIAX.X, plan.uIAX.Y, plan.uIAX.Z])
    uiay = np.array([plan.uIAY.X, plan.uIAY.Y, plan.uIAY.Z])
    u = srp - 0.5 * (tx + rcvp)
    u = u / np.linalg.norm(u, axis=1, keepdims=True)
    ax, ay = u @ uiax, u @ uiay
    t.lap("open + PVP")

    sig = np.asarray(rdr.read_chip((0, nv, mu), (0, ns, nu), index=0), np.complex64)
    rdr.close()
    k = np.arange(sig.shape[1])
    freq = sc0[:, None] + k[None, :] * (scss[:, None] * nu)
    t.lap("read signal")

    # CPU: identical resample + window as the reference (imported, not copied)
    g2, geo = ref.resample_kspace(sig, freq, ax, ay)
    t.lap("resample + window")

    # FPGA: pad -> 2-D BFP FFT -> detect
    fixed_mag, (er, ea) = accel.focus_fixed(g2, NBITS)
    m2, n2 = fixed_mag.shape
    print(f"  [{getattr(accel,'name','fpga')}] focused {fixed_mag.shape}  "
          f"EXP_R={er[-1]} EXP_A={ea[-1]}  "
          f"dyn.range {ref_dr(fixed_mag):.1f} dB")
    t.lap("fabric pad+FFT+detect")

    # scale pixel spacing to the padded pow2 grid, exactly as focus_fixed does
    geo = dict(geo)
    geo["dr"] *= g2.shape[1] / n2
    geo["dc"] *= g2.shape[0] / m2

    stem = cphd_path.stem.replace("_CPHD", "")
    out_tif = out_dir / f"{stem}_fpga{NBITS}bit_detected.tif"
    ref.save_detected_geotiff(fixed_mag, geo, srp[0], uiax, uiay, str(out_tif),
                              ref.GEO_EPSG, flip_col=ref.FLIP_COL,
                              flip_row=ref.FLIP_ROW, dtype="float32")
    t.lap("geocode + GeoTIFF")
    return out_tif


def ref_dr(mag):
    hi = np.percentile(mag, 99.99)
    lo = np.percentile(mag[mag > 0], 5) if np.any(mag > 0) else 1e-12
    return 20 * np.log10(hi / (lo + 1e-12))


def main():
    ap = argparse.ArgumentParser(description="Focus a CPHD on the PolarFire SoC fabric")
    ap.add_argument("--in", dest="inp", required=True, help="<scene>_CPHD.cphd")
    ap.add_argument("--out", default="/data/out")
    ap.add_argument("--backend", default="fpga", choices=["fpga", "mock"])
    ap.add_argument("--uio", default="/dev/uio0")
    ap.add_argument("--udmabuf", default="udmabuf0")
    ap.add_argument("--deci-pulse", type=int, default=1)
    ap.add_argument("--deci-sample", type=int, default=1)
    a = ap.parse_args()
    accel = make_accel(a.backend, a.uio, a.udmabuf)
    out = process(a.inp, a.out, accel, a.deci_pulse, a.deci_sample)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
