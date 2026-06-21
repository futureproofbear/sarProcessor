# Libero build — SAR FFT accelerator on PolarFire SoC

Target board: **MPFS-Icicle-Kit-ES** — `MPFS250T-FCVG484EES` + 2 GB LPDDR4.

Two stages:
1. **Accelerator block** (`build_sar_fft.tcl`): project, RTL, CoreFFT IP,
   constraints, synth/place-route/timing of `sar_fft_top`. Closes timing on the
   fabric datapath in isolation.
2. **SoC SmartDesign** (manual / `soc_integration.tcl` skeleton): drop the
   accelerator into a top design with the MSS, LPDDR4, FIC ports and clocks, then
   generate the device bitstream.

## 1. Build the accelerator block
```
cd fpga/libero
libero SCRIPT:build_sar_fft.tcl
```
Set `FFT_LEN_R` / `FFT_LEN_A` at the top of the Tcl to your scene's padded
power-of-2 dimensions (default 8192 × 8192). The two CoreFFT cores are sized from
these, and the host **must** program `N2 = FFT_LEN_R`, `M2 = FFT_LEN_A` or the
core raises `STATUS.ERR`.

`SAR_USE_COREFFT` is defined for synthesis so `corefft_wrap` binds the real
CoreFFT IP. (Simulation leaves it undefined and uses `sim/corefft_model.sv`.)
Confirm the CoreFFT parameter keys in `corefft_config.tcl` against your installed
CoreFFT version, and finish wiring the core's load/unload handshake to the four
stream signals inside `corefft_wrap.sv` (the `ifdef SAR_USE_COREFFT` branch).

## 2. SoC SmartDesign integration
Assemble a top SmartDesign (`PF_SOC_TOP`) containing:

| Block | Role |
|-------|------|
| **PolarFire SoC MSS** | U54 cores + **LPDDR4 controller** (2 GB). Enable one **FIC** as a *master into the fabric* (drives the accelerator's AXI4-Lite control) and one **FIC** as a *slave from the fabric* (the accelerator's AXI4 master → DDR). Use the **cache-coherent** path for zero-copy with the U54s, or a non-coherent FIC for peak bandwidth with explicit cache flush/invalidate around START/DONE. |
| **`sar_fft_top`** | the accelerator. `clk`/`rstn` from the MSS fabric clock + reset. `s_axil_*` ← MSS-master FIC. `m_axi_*` → MSS-slave FIC (to LPDDR4). `irq` → an MSS fabric interrupt (`F2H` GPIO/PLIC) for the done IRQ. |
| **CCC / PLL** | fabric clock (start at 100 MHz, push to ~150 MHz). |
| **CoreReset / INIT_MONITOR** | reset sequencing, DDR/PLL lock gating. |

Most LPDDR4 and MSS pin/IO constraints come from the **PolarFire SoC MSS
configurator** and the **Icicle Kit board component**; import those rather than
hand-writing them.

`soc_integration.tcl` is a commented skeleton of these `sd_*` SmartDesign calls
— complete the MSS `.cfg`/component reference for your Libero version (the MSS
configuration is best done once in the GUI and exported).

## 3. Bitstream + Linux bring-up
- Generate the bitstream from `PF_SOC_TOP`, export the FlashPro/`.job` file, and
  program the Icicle Kit.
- Device tree: expose the accelerator's AXI4-Lite window as a **UIO** device and
  reserve a **CMA** pool for the `SIG` / `BUF` / `OUT` DDR buffers (contiguous, so
  the fabric AXI master sees one physical extent).
- Host driver: mmap the UIO registers, fill `SIG` with the resampled/windowed/
  quantized k-space (int16 complex), set dims + addresses (`fpga/regmap.md`),
  write `CTRL.START`, wait on the done IRQ, read `EXP_R`/`EXP_A`, then read the
  magnitude image from `OUT` and finish the GeoTIFF exactly as
  `src/form_image_pfa_fixed.py` does. The CPU half of
  [`mpfs/host/sar_pipeline.py`](../../mpfs/host/sar_pipeline.py) already does the
  parse / resample / geocode; point its accelerator seam at this register map.

## Resource sketch (MPFS250T has 784 math blocks, ~17 Mb LSRAM)
- 2 × CoreFFT (8192-pt, 16-bit, BFP): the dominant DSP/LSRAM consumer; well
  within budget on the 250T. Shrink with decimation if you target a smaller die.
- `sar_ctrl` line/stage buffers: 2 × 8192 × 32-bit BRAM + two small exponent RAMs.
- The single-beat DDR master is small; throughput (not area) is its cost — see
  the burst-grouping note in `fpga/README.md` for the optimization path.
