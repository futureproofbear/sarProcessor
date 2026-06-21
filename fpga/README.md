# SAR focuser — synthesizable RTL accelerator (PolarFire SoC Icicle Kit)

A self-contained **fabric-mastered** SAR image-formation accelerator for the
**MPFS-Icicle-Kit-ES** (`MPFS250T-FCVG484EES`, 2 GB LPDDR4). The fabric reads the
pre-loaded k-space from DDR4, forms the image, and writes the detected magnitude
image back to DDR4 — processing **one line (pulse / range bin) at a time** so it
needs only a couple of line buffers of on-chip RAM, not the whole image.

This is the **RTL** sibling of the HLS templates in [`../mpfs/`](../mpfs/). The
HLS version is host-driven (the CPU fills DDR buffers and the fabric is a kernel);
**this** version makes the fabric an AXI master that owns the DDR traffic itself
and runs the standard 2-D FFT corner-turn out of DDR.

## Scope (what the fabric does vs. what the CPU does)

The Python reference [`src/form_image_pfa_fixed.py`](../src/form_image_pfa_fixed.py)
deliberately splits the workload. The **CPU half** (polar→Cartesian *resample*,
Hamming window, input quantization, geocode, GeoTIFF) stays in float on the U54
cores — fixed-point resample would lose accuracy, which is why the reference
keeps it in float. The **fabric half** is exactly
[`fixedpoint.py::focus_fixed`](../src/fixedpoint.py#L116):

```
zero-pad to pow2 ─► ifftshift ─► 2-D FFT (range FFT │ corner-turn │ azimuth FFT) ─► fftshift ─► detect |·|
```

So the **"raw data" the user pre-loads into DDR4 is the already
resampled + windowed + quantized complex k-space** (signed 16-bit re / 16-bit im).
The fabric does the pad → FFT → detect and writes back a magnitude image; the CPU
multiplies by the reported block exponent and writes the GeoTIFF, exactly as
`form_image_pfa_fixed.py` does today.

## Datapath

```
                ┌──────────────────── LPDDR4 (set by host) ────────────────────┐
   SIG (int16 cplx, M×N) ──┐                          ┌── BUF (int16 cplx, M2×N2) ──┐   OUT (uint32 mag, M2×N2)
                           ▼                          ▼                             ▼            ▲
   PASS 1 (range)   load row r (zero-pad, col ifftshift)  ─► CoreFFT(N2) ─► store BUF row r ; exp_r[r]
   PASS 2 (azimuth) load BUF col c (row ifftshift, ≫ to common exp_r) ─► CoreFFT(M2) ─► store BUF col c ; exp_a[c]
   DETECT           load BUF row (fftshift, ≫ to common exp_a) ─► |re,im| (isqrt) ─► store OUT row
```

The 2-D FFT is the classic row-transform / **corner-turn** / column-transform.
The corner-turn is done purely in DDR addressing (pass 2 reads BUF column-strided),
so no transpose buffer is needed on-chip — only one line.

### Key simplifications (why the addressing is tiny)
- **`ifftshift` / `fftshift` on a power-of-2 length is a roll-by-half, which is
  just *toggling the top index bit*.** So all three FFT-shift steps reduce to an
  XOR on a row/column address (`^ (N2/2)`, `^ (M2/2)`). See `sar_ctrl.sv`.
- **Zero-pad is implicit**: rows `r ≥ M` and columns `q ≥ N` are simply never
  read from DDR; the line buffer is pre-cleared, so they contribute zeros.
- **Block-floating-point bookkeeping**: CoreFFT does per-frame BFP and returns a
  block exponent per line. To keep pixels mutually consistent across lines, each
  pass records every line's exponent, tracks the max, and the *next* stage right-
  shifts each line to the common (max) exponent as it loads it. The two max
  exponents (`EXP_R`, `EXP_A`) are reported to the host so it can reconstruct the
  true float magnitude (`mag_float = OUT · 2^(input_exp + EXP_R + EXP_A)`).

## Block diagram (fabric)

```
            AXI4-Lite (control)              AXI4 master (DDR, via MSS FIC)
   host ───────────────► ┌───────────────┐ ──────────────────► LPDDR4
                         │  axil_regs    │
                         └──────┬────────┘
                                │ regs (addrs, M,N,M2,N2, start)
                         ┌──────▼────────┐     ┌──────────────┐
                         │   sar_ctrl    │────►│ axi_master_rw│◄─► DDR
                         │  (pass FSM,   │     └──────────────┘
                         │   addressing, │     ┌──────────────┐
                         │   BFP shift,  │────►│ corefft_wrap │ (CoreFFT IP)
                         │   exp RAMs)   │◄────│  + blk_exp   │
                         └──────┬────────┘     └──────────────┘
                                │ detect
                         ┌──────▼────────┐
                         │   isqrt mag   │
                         └───────────────┘
```

## Files
```
fpga/
├── rtl/
│   ├── sar_fft_top.sv     # top: AXI4-Lite slave + AXI4 master + ctrl + corefft + detect
│   ├── sar_ctrl.sv        # pass FSM, line addressing, BFP normalize, exp RAMs, detect
│   ├── axil_regs.sv       # AXI4-Lite control/status register file
│   ├── axi_master_rw.sv   # AXI4 read/write master (single-beat, strided-capable)
│   ├── corefft_wrap.sv    # uniform handshake around Microchip CoreFFT
│   └── isqrt.sv           # pipelined integer sqrt for |re,im|
├── sim/
│   ├── corefft_model.sv   # behavioral CoreFFT (real-arithmetic DFT + BFP) for sim
│   ├── axi_ddr_model.sv   # behavioral AXI4 slave / DDR memory for sim
│   ├── tb_sar_fft_top.sv  # self-checking top testbench (loads vectors, checks golden)
│   └── vectors/           # generated by scripts/gen_vectors.py
├── host/
│   ├── sar_accel_driver.py # U54 driver: UIO regs + udmabuf DDR; focus_fixed() drop-in
│   └── run_on_board.py     # storage-to-storage runner (CPHD → fabric focus → GeoTIFF)
├── scripts/
│   └── gen_vectors.py     # bit-faithful Python model → input + golden hex vectors
├── libero/
│   ├── build_sar_fft.tcl  # Libero project: target, sources, CoreFFT, MSS/FIC, flow
│   ├── corefft_config.tcl  # CoreFFT IP configuration
│   └── constraints/
│       ├── sar_fft.sdc    # clocks / timing
│       └── sar_fft.pdc    # fabric I/O + FIC placement notes
├── regmap.md              # AXI4-Lite register map (host ↔ fabric contract)
└── README.md              # this file
```

## Verification status — read this first
What has and hasn't been run, honestly:
- **Done here**: `gen_vectors.py` runs (numpy) and its hardware model correlates
  **1.0000** with an ideal float 2-D FFT — i.e. the addressing, corner-turn, BFP
  bookkeeping and detect are algorithmically correct. The golden vectors are
  generated. The RTL was written to match that model exactly and was self-reviewed
  against it line by line (PASS-1/2 addressing, ifftshift-as-bit-toggle, BFP
  renormalization, detect), and for BRAM read-latency / AXI-handshake correctness.
- **Set up but NOT executed here** (no HDL simulator in this environment):
  `tb_sar_fft_top.sv` drives the whole design against a behavioral DDR + behavioral
  CoreFFT and checks the output magnitude vs the golden image (tolerance-based).
  **Run it before trusting the RTL** — `sim/run_modelsim.do` in the ModelSim/
  QuestaSim that ships with Libero (or Verilator). It prints `RESULT: PASS/FAIL`.
- **Requires Libero + licensed CoreFFT**: synthesis, place/route, timing closure,
  bitstream, board bring-up. `corefft_model.sv` is a *behavioral stand-in* with the
  ports `corefft_wrap` expects; synthesis defines `SAR_USE_COREFFT` to bind the real
  IP. The real CoreFFT's internal rounding differs from the model, so the on-board
  result is **not bit-identical** — which is why the testbench oracle is tolerance +
  correlation, the right check for a fixed-point core.

## Build / verify
```bash
# 1. generate vectors (needs numpy only — no CPHD/sarpy)
python fpga/scripts/gen_vectors.py --m 10 --n 6        # → fpga/sim/vectors/

# 2. simulate (example with Verilator; ModelSim flow in libero/README notes)
#    compile rtl/*.sv + sim/*.sv, top = tb_sar_fft_top, run; it prints PASS/FAIL.

# 3. build the bitstream
#    Libero SoC ≥ 2023.1, then:  libero SCRIPT:fpga/libero/build_sar_fft.tcl
```

## Host driver (U54 Linux)
[host/sar_accel_driver.py](host/sar_accel_driver.py) drives the fabric over the
register map: mmaps the AXI4-Lite registers via **UIO**, shares contiguous DDR
buffers via **u-dma-buf**, loads `SIG`, programs dims+addresses, starts, waits on
DONE (poll or IRQ), and reconstructs float magnitude from `OUT` and the reported
exponents. Its `focus_fixed(g2, nbits)` is a **drop-in for
`fixedpoint.focus_fixed`**, so the only change to
[../src/form_image_pfa_fixed.py](../src/form_image_pfa_fixed.py) is:
```python
accel = SarFftAccel()                                   # /dev/uio0 + /dev/udmabuf0
fixed_mag, (er, ea) = accel.focus_fixed(g2, NBITS)      # was fp.focus_fixed(...)
```
[host/run_on_board.py](host/run_on_board.py) is the full storage-to-storage
runner (CPHD → CPU resample → fabric focus → geocode → GeoTIFF).

Develop/CI without hardware: every host path also runs against an in-process
emulation of the fabric (the bit-faithful model), so the plumbing is testable on
a laptop:
```bash
python fpga/host/sar_accel_driver.py --selftest   # PASS, corr 1.0000 vs NumPy ref
python fpga/host/run_on_board.py --in <scene>_CPHD.cphd --out out --backend mock
```
The host must program `N2 = FFT_LEN_R`, `M2 = FFT_LEN_A` (the built CoreFFT
lengths) or the driver raises on `STATUS.ERR`.

See [regmap.md](regmap.md) for the control interface and
[libero/build_sar_fft.tcl](libero/build_sar_fft.tcl) for the hardware build.
