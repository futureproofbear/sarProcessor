# MPFS port — embedded SAR focuser for the PolarFire SoC Icicle Kit

CPU + FPGA co-design that runs the `form_image_pfa.py` workload on the
**MPFS-Icicle-Kit-ES** as a **storage-to-storage batch processor**: CPHD +
metadata are pre-loaded onto the board's storage, focused on-board, and the
geocoded GeoTIFF is written back to storage. No network link.

```
/data/in/*_CPHD.cphd (+ _METADATA.json)        /data/out/*_detected.tif
        │                                                ▲
   [CPU] parse + build tables                            │
        │              ┌──────── shared LPDDR4 ────────┐ │
        └──► signal ──►│  [FPGA] resample→window→FFT   │ │
                       │         →corner-turn→FFT→detect│ │
                       └────────────► detected ─────────┘ │
                                  [CPU] geocode + GeoTIFF ─┘
```

The laptop reference [`src/form_image_pfa.py`](../src/form_image_pfa.py) stays
the **algorithm source of truth**; this port reuses it (it is *not* duplicated)
and adds the storage queue, the CPU/FPGA partition, and the FPGA kernels.

## Layout
```
mpfs/
├── host/                 # CPU side (Python, runs on the U54 Linux)
│   ├── sar_pipeline.py   #   storage-in → tables → focus → geocode → storage-out + --watch daemon
│   └── accel.py          #   accelerator seam: NumpyBackend (CPU) | FpgaBackend (fabric, stub)
├── fpga/                 # FPGA side (HLS templates + build/integration docs)
│   ├── sar_accel_top.cpp #   on-fabric datapath skeleton  (UNVERIFIED template)
│   ├── fft1d.cpp         #   streaming BFP FFT kernel      (UNVERIFIED template)
│   └── README.md         #   Libero/SmartHLS flow, DMA/coherent port, verification
└── regmap.md             # AXI4-Lite control register map (host <-> fabric contract)
```

## What works today vs. what's a template
- **Works & verified:** the CPU host path (`--backend numpy`) runs end-to-end
  and is **pixel-identical** to the laptop reference's GeoTIFF. This is also the
  board's CPU-only fallback mode (slow, but correct).
- **Template / not yet built:** the FPGA kernels (`fpga/*.cpp`) and the
  `FpgaBackend` body. They define the design and the host↔fabric contract but
  require synthesis, simulation, place/route, and a bitstream + UIO/CMA driver.

## Run (CPU reference — works on laptop or board)
```bash
# single scene
python mpfs/host/sar_pipeline.py --in <path>/..._CPHD.cphd --out mpfs/output --backend numpy

# batch daemon over pre-loaded storage (the board deployment model)
python mpfs/host/sar_pipeline.py --watch /data/in --out /data/out --backend numpy
```
On the board, switch to `--backend fpga` once the bitstream + driver are in
place; the rest of the command is identical.

## Why this partition
The 2-D FFT dominates compute and is what the U54 cores are worst at (600 MHz,
no SIMD); it maps cleanly onto the fabric's 784 DSP blocks. The CPU keeps the
I/O, parsing, geometry/table prep, map projection, and GeoTIFF encoding it is
already fine at. Across a pre-loaded queue, the read / focus / encode stages
pipeline so both engines stay busy. See [fpga/README.md](fpga/README.md) and
[regmap.md](regmap.md) for the hardware details.
