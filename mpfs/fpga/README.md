# FPGA accelerator (PolarFire SoC fabric)

Offloads the compute-dominant SAR focusing datapath from the U54 CPUs:
**range resample → window → 2-D FFT (range FFT → corner-turn → azimuth FFT) →
detect**. The CPU host drives it over AXI4-Lite (see [../regmap.md](../regmap.md))
and shares DDR buffers with it.

## Status — read this first
These files are **unsynthesized starting templates**. They have **not** been
compiled in SmartHLS, simulated, placed/routed, or run on hardware:
- `fft1d.cpp` — streaming radix-2 block-floating-point FFT (template)
- `sar_accel_top.cpp` — top-level dataflow stitching the kernels (skeleton; the
  corner-turn body is intentionally left as a stub)

For production, prefer the **Microchip CoreFFT IP** (verified, characterized)
over the hand-written `fft1d.cpp`, and treat `sar_accel_top.cpp` as the dataflow
spec to implement/verify, not as drop-in RTL.

## Datapath
```
signal[M,N] ─DMA─► resample(KR) ─► window× ─► 1-D FFT ─┐
                                                       ▼
                                              corner-turn (tiled BRAM)
                                                       │
detected[M,N] ◄─DMA─ detect|·| ◄─ 1-D FFT ◄─ resample(KC) ◄┘
```
- **Power-of-2 FFT lengths** (`FFT_LEN_R`, `FFT_LEN_A`, e.g. 4096 × 8192): the
  raw 5634 × 4319 sizes are slow/odd, so the host zero-pads. Padding also
  improves the focus (free image-domain interpolation).
- **Block floating point, 18-bit** fixed point → matches the 784 18×18 DSP
  blocks and holds the ~50 dB scene dynamic range. `BFP_SHIFT` is reported back
  so the host can scale consistently with the laptop reference.
- **Parallelism**: one FFT core ≈ 24–40 DSPs of 784 → instantiate several and
  process multiple lines concurrently.

## Build flow (Libero SoC + SmartHLS)
1. **Generate twiddles**: produce `TW_RE[]`/`TW_IN[]` ROMs for the chosen NFFT.
2. **HLS**: `shls` build of the kernels (or instantiate CoreFFT instead of
   `fft1d`). Verify with C/RTL co-simulation against the CPU golden output
   (`host` numpy backend) using the speckle-smoothed correlation check.
3. **Libero**: build the fabric design — accelerator + `CoreAXI4DMAController`
   (or a SmartHLS datamover) + AXI4-Lite control, connected to the SoC's
   **cache-coherent** AXI port (FIC). Add the FFT cores' DSP/RAM.
4. **Place/route/timing** at the chosen fabric clock (~150 MHz target).
5. Export the bitstream + a device-tree overlay exposing the accelerator as a
   **UIO** device and a **CMA** pool for the shared buffers.

## Host integration
`host/accel.py::FpgaBackend.focus()` mmaps the AXI4-Lite registers via UIO,
fills the CMA buffers (signal/KR/KC/tanphi/window), starts the core, waits on
the done IRQ, and reads the detected image back. The rest of the pipeline
(parse, table prep, geocode, GeoTIFF) is unchanged CPU code in
`host/sar_pipeline.py`.

## Verification oracle
The CPU reference (`--backend numpy`) is **pixel-identical to the laptop
reference** `src/form_image_pfa.py`. Use its detected GeoTIFF as golden and
compare the FPGA output with the same metric used during bring-up (smoothed
correlation r → 1.0 modulo fixed-point, plus a point-target PSLR check).
