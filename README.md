# sarProcessor

Lightweight SAR image formation from **Umbra CPHD** (compensated phase history
data) on a laptop. Downloads a public Umbra open-data CPHD, focuses it with the
Polar Format Algorithm, and writes a detected, geocoded GeoTIFF.

## Layout

```
sarProcessor/
├── src/
│   └── form_image_pfa.py     # the pipeline (download → focus → detect → geocode)
├── data/                     # local mirror of the Umbra S3 bucket layout
│   └── sar-data/tasks/<AOI>/<task-uuid>/<capture>/…_CPHD.cphd, …_GEC.tif, …_METADATA.json
├── output/                   # all generated products
│   ├── centerfield_pfa.png           # focused image (dB quick-view)
│   ├── centerfield_detected.tif      # detected, geocoded GeoTIFF (EPSG:32612)
│   ├── centerfield_GEC_quicklook.png # Umbra reference product, for comparison
│   └── geocode_final.png             # my output vs GEC overlay (validation)
└── README.md
```

Paths in the script are anchored to the project root, so it runs the same from
any working directory.

## Run

```bash
python src/form_image_pfa.py
```

First run downloads the ~196 MB CPHD into `data/` (anonymous HTTPS, no AWS
credentials); later runs reuse the cache. The script prints a measured
resource/time estimate before any heavy compute.

Key knobs at the top of `src/form_image_pfa.py`:
- `MODE` — `"pfa"` (geometrically correct, ~12 s) or `"quicklook"` (single 2-D FFT, ~7 s)
- `DECIMATE_PULSE` / `DECIMATE_SAMPLE` — trade resolution for speed
- `ESTIMATE_ONLY` — print the estimate and stop
- `SAVE_GEOTIFF`, `OUT_TIF`, `GEO_EPSG`, `FLIP_COL/ROW` — detected GeoTIFF output

## Requires

`numpy`, `scipy`, `matplotlib`, `sarpy`, `rasterio`, `pyproj` (all already installed).

## Ports to the PolarFire SoC Icicle Kit

- [`mpfs/`](mpfs/) — CPU + FPGA co-design (HLS templates, host-driven).
- [`fpga/`](fpga/) — **synthesizable RTL accelerator**: a fabric-mastered 2-D
  block-floating-point FFT + detect that reads the resampled k-space from LPDDR4,
  forms the image one line at a time, and writes the detected magnitude back to
  LPDDR4. Libero project (`MPFS250T-FCVG484EES`) + SystemVerilog + self-checking
  testbench + golden vectors. Implements the fabric half of
  [`src/form_image_pfa_fixed.py`](src/form_image_pfa_fixed.py). See
  [`fpga/README.md`](fpga/README.md).
