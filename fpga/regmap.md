# SAR FFT accelerator — AXI4-Lite control register map

Contract between the host (U54 Linux, or any AXI master) and the fabric
accelerator `sar_fft_top`. Offsets are byte offsets from the AXI4-Lite base
address. All registers are 32-bit. 64-bit DDR addresses occupy a lo/hi pair.

| Offset | Name           | R/W | Description |
|--------|----------------|-----|-------------|
| 0x00   | `CTRL`         | RW  | bit0 `START` (write 1 to launch; self-clears), bit1 `SOFT_RESET` |
| 0x04   | `STATUS`       | R   | bit0 `DONE`, bit1 `BUSY`, bit2 `ERR`, bit3 `IRQ` |
| 0x08   | `IRQ_EN`       | RW  | bit0 enable done-interrupt; write `STATUS.IRQ`=1 path |
| 0x0C   | `M`            | RW  | input rows (pulses), un-padded (`1…M2`) |
| 0x10   | `N`            | RW  | input cols (samples), un-padded (`1…N2`) |
| 0x14   | `M2`           | RW  | azimuth FFT length, power of 2 (rows after pad) |
| 0x18   | `N2`           | RW  | range FFT length, power of 2 (cols after pad) |
| 0x1C   | `SIG_ADDR_LO`  | RW  | DDR addr[31:0] of input k-space (int16 cplx, M×N, row-major) |
| 0x20   | `SIG_ADDR_HI`  | RW  | DDR addr[63:32] of input k-space |
| 0x24   | `BUF_ADDR_LO`  | RW  | DDR addr[31:0] of intermediate buffer (int16 cplx, M2×N2) |
| 0x28   | `BUF_ADDR_HI`  | RW  | DDR addr[63:32] of intermediate buffer |
| 0x2C   | `OUT_ADDR_LO`  | RW  | DDR addr[31:0] of output magnitude (uint32, M2×N2, row-major) |
| 0x30   | `OUT_ADDR_HI`  | RW  | DDR addr[63:32] of output magnitude |
| 0x34   | `EXP_R`        | R   | max block exponent of the range (pass-1) FFTs |
| 0x38   | `EXP_A`        | R   | max block exponent of the azimuth (pass-2) FFTs |
| 0x3C   | `ID`           | R   | constant `0x53415246` ("SARF") — design id / liveness |

## Data formats

- **Input k-space (`SIG`)**: one complex sample per 32-bit word, packed
  `{ im[15:0], re[15:0] }`, each a signed two's-complement 16-bit integer. Layout
  is row-major `M×N` (row = pulse, col = sample). The host produces this by
  resampling + windowing + quantizing the CPHD exactly as
  `form_image_pfa_fixed.py` does, then truncating to int16 with a known input
  exponent it keeps for the final scale.
- **Intermediate (`BUF`)**: same packing, `M2×N2` row-major. Owned by the fabric;
  the host only supplies a scratch DDR region of `M2*N2*4` bytes.
- **Output (`OUT`)**: one `uint32` magnitude per pixel, `M2×N2` row-major,
  already `fftshift`-ed (DC at centre). The host reconstructs float magnitude as
  `mag_float = OUT · 2^(input_exp + EXP_R + EXP_A)` (a single global scale; the
  GeoTIFF path normalizes anyway, so only relative pixel values matter).

## Handshake
1. Host writes the resampled/windowed/quantized k-space to `SIG`, allocates `BUF`
   and `OUT` scratch in DDR (CMA-contiguous), and sets `M, N, M2, N2` and the
   three address pairs.
2. Host writes `CTRL.START = 1`.
3. Fabric runs PASS 1 (range FFTs) → PASS 2 (azimuth FFTs, corner-turn in DDR) →
   DETECT, streaming magnitudes to `OUT`. `STATUS.BUSY` is high throughout.
4. Fabric sets `STATUS.DONE` (and `STATUS.IRQ` + the UIO interrupt if `IRQ_EN`).
5. Host reads `EXP_R`, `EXP_A`, then the image from `OUT`.

## Memory port (MSS)
The AXI4 master connects to a PolarFire SoC **Fabric Interface Controller (FIC)**
slave that routes to the LPDDR4 controller. Use the **cache-coherent** path
(through the MSS coherent switch) for zero-copy with the U54 cores, or the
non-coherent FIC for peak bandwidth with explicit cache flush/invalidate around
START/DONE. See [libero/build_sar_fft.tcl](libero/build_sar_fft.tcl).
