# Silicon Iso-Test + HLS-FFT Build Runbook

Reliable, repeatable procedures for isolating SAR kernels on silicon, coherent DDR reads,
SmartHLS validation, and the HLS-FFT fabric rebuild. Written after a long 2026-07-04 session
that re-derived these too many times. **Follow this before improvising.**

## 1. JTAG hygiene (DO NOT skip)
- **NEVER `taskkill /F` openocd/gdb** — wedges the FlashPro6 DM, needs board power-cycle.
  Clean shutdown: `python -c "import socket,time; s=socket.create_connection(('localhost',4444),5); time.sleep(.5); s.sendall(b'shutdown\n'); time.sleep(1.5); s.close()"` (openocd telnet port 4444), THEN kill the orphaned gdb (safe once openocd exited).
- gdb scripts must end with `monitor resume` + `monitor shutdown`. The trailing
  "Remote communication error. Target disconnected." AFTER "shutdown command invoked" is **benign**.
- **Capture gdb output** — either `set logging file <path>` + `set logging on` in the .gdb, OR redirect
  the runner stdout to a file. NEVER `>/dev/null` (you lose the read values → wasted board run).
- **openocd startup**: `sleep 14` after launch before gdb connects (hart-examine race). Runner template:
  `run_status_probe.sh` (openocd + gdb -x <script.gdb>).
- Scene load = 1.5 MB over JTAG ≈ **2.4 min** (gdb prints nothing during `restore`). Full 128 MB OUT dump is impractical (hours); dump 256-row bands (4 MB) only.
- `libero.exe` lingers after PROGRAMDEVICE but **does NOT hold the FlashPro6** (released at "PROGRAM PASSED"). Don't over-wait — just launch openocd; it'll grab the FP6.

## 2. DDR + kernel-control map
| Buffer | Addr | Notes |
|---|---|---|
| SIG | `0x88000000` | scene / ping-pong. row R = `0x88000000 + R*0x8000` (8192 cplx u32) |
| SCRATCH | `0x98000000` | intermediate. row R = `0x98000000 + R*0x8000` |
| OUT | `0xA8000000` | uint16 magnitude image. row R = `0xA8000000 + R*0x4000` |
| TABLES | `0xB0000000` | geometry/coeffs/mailbox (CPU-read, cacheability per MPU) |
| COEF_IDX(0)/WQ(0) | `0xB0148000` / `0xB0158000` | int32[Np] / int16[Np] |
| mailbox | `0xB0058000` | +0 cmd, +4 base, +8 len, +C result, +10 status, +14 seq |
| SAR_PROG | `0xB0059100` | +0 pass, +4 idx, +8 total, +C heartbeat |

- **DDR is `0x80000000`–`0xBFFFFFFF` only. `≥0xC0000000` = ABOVE-DDR decode error** (NOT a cached/
  non-cached alias — cacheability is MPU-config, not address-aliased). Don't read `0xC8…`/`0xE8…`.
- Kernel control: `K_CORNER_TURN 0x60000000`, `K_WINDOW 0x60001000`, `K_DETECT 0x60002000`,
  `K_RESAMPLE 0x60003000`, `K_FFT 0x60004000`. Regs: `START +0x08` (write 1=go, read 0=done),
  `ARG0 +0xc, ARG1 +0x10, ARG2 +0x14, ARG3 +0x18`. **Never read an unused slave (e.g. 0x60005000) — hangs AXI un-haltably.**
- Kernel arg contracts: `detect(in,out)` no count (DN=8192²); `resample(in,idx,wq,out)`;
  `window(in,hamr,hamc,out)`; `corner_turn(src,dst)`; `fft_kernel(src,dst,nrows)`.

## 3. Coherent DDR read (FIC0 is non-coherent)
The fabric kernels read/write DDR via FIC0; the hart/gdb see L2. To read what a kernel actually wrote:
- **`call (void) flush_l2_cache(1)` from gdb** — evicts L2, so a subsequent *cached* read fetches
  physical DDR. (Also the way to push a gdb-loaded input to DDR before arming a kernel.) VERIFIED:
  load pattern → CRC(L2)=pattern → call flush → CRC(post-evict)=pattern ⇒ flush delivers to DDR.
- Mid-pipeline, SIG/SCRATCH data rows are **uncached** (kernels write via FIC0, hart never caches them;
  per-line resample flushes keep L2 cold) → a direct read already hits DDR. But `call flush` mid-run
  can perturb the sequencer (observed a restart) — read directly when possible.
- **CRC localization**: mailbox CRC32 (cmd `0x43524333`, zlib-compatible) over 16 MB of SIG/SCRATCH/OUT
  after a PIPE run (post-flush → DDR) pinpoints where data survives vs zeros. zero-CRC(16 MB)=`0xa47ca14a`.
- **GOTCHA**: resampled k-space cols 0–4 are legit **edge zero-fill** (first ~12 KC-grid pts out of
  range → idx=−1). Read **col 5+** to see real data. A truncated `x/8xw` (first line only) nearly
  mis-blamed pass-2/window when the FFT was the culprit.

## 4. Single-kernel isolation pattern (the workhorse)
`jtag_full/{detect,fft,resample}_iso*.gdb` + `run_*.sh`. Structure:
1. `monitor reset halt` + boot (`resume`, sleep 28–30 s, `arp_halt`, `thread 2`)
2. `restore <pattern>.bin binary <SIG>` (e.g. `fft_test_row.bin` = const re=1000)
3. pre-clear the dst first words (so a stale value can't fool you)
4. `call (void) flush_l2_cache(1)` (input → DDR)
5. arm the kernel (set ARG regs + START=1), `resume`, sleep, `arp_halt`, read START (0=done)
6. `call (void) flush_l2_cache(1)` (evict dst), read dst
- **Known-good expectations** (const re=1000 = `0x03e80000`): detect→`0x03e803e8` (mag 1000);
  resample identity coeffs→`0x03e80000` (passthrough); **FFT→DC delta `~0x7D000000` (32000 = 8192·1000>>8), NOT flat `0x00030000`** (flat = broken passthrough).

## 5. SmartHLS validation
- **vsim** is at `C:/Microchip/Libero_SoC_2025.2/Libero_SoC/QuestaSim_Pro/win64/` — **add to PATH**
  (the shls setup script wrongly points to `ModelSim_Pro`). `command -v vsim` must resolve first.
- `shls cosim` (RTL vs C) currently **segfaults in its C-testbench wrapper** (0xC0000005) — a tooling
  bug, not the design (`shls sw` runs clean). `shls sim` needs a custom Verilog TB.
- **C-logic validation** (does the fix compute right): `shls sw` + `python tb/gen_and_check.py gen <case>`
  / `check <case>` (cases: tone/twotone/pointtarget/random). tone → corr≈1.0, peak bin 137. **sw-sim
  passing does NOT prove the RTL** (the broken FFT also passed sw-sim). RTL truth = silicon fft_iso2.
- Regen RTL from fixed HLS: `shls hw` (produces `hls_output/rtl/*.v` + `scripts/libero/create_hdl_plus.tcl`).

## 6. HLS-FFT fabric rebuild + program
- `bash mpfs/fpga/run_hlsfft_build.sh` (3 stages, board-free, ~12 min): create project+cores+MSS+HDL+
  +assembly → stage constraints → synth/PnR/**VERIFYTIMING**/bitstream. Gates on DONE markers.
- **Check timing** before trusting: `SETUP nviol=0`, `HOLD nviol=0`, `TIMING_MET`, then `BITSTREAM_READY`
  + `BUILD_HLSFFT_DONE`. (Libero silently programs timing-failing bitstreams — see [[always-check-timing-closure]].)
- Program (board on): `libero.exe SCRIPT:program_hlsfft.tcl LOGFILE:...` → expect `PROGRAMDEVICE OK` +
  "Chain programming PASSED". Fabric-only change → firmware (eNVM) untouched, no reflash needed.
- Prereqs: `LM_LICENSE_FILE=C:\Users\lkwangsi\Documents\github\polarfire-soc\License.dat`;
  `libero.exe` at `C:/Microchip/Libero_SoC_2025.2/Libero_SoC/Designer/bin/`. No stale synth (synbatch
  zombies corrupt synth → host reboot clears them).
- **GOTCHA: a leftover `libero.exe` (from the previous program/build) holds a LOCK on `libero_hlsfft/`
  → `create_fresh_project_hlsfft.tcl`'s `file delete -force` fails "permission denied ... SAR_TOP.smat.seg"
  → STAGE 1 FAILED in ~18 s.** Fix: `taskkill //F //PID <libero>` FIRST (libero.exe is safe to /F-kill —
  the openocd/gdb no-force-kill rule is ONLY about the FlashPro6 DM, not libero), then re-run the build.
  Check `tasklist | grep libero` is empty before launching a rebuild.
- **When iterating the HLS kernel** (edit .hpp): `shls hw` regenerates RTL from the header (dependency
  tracking DOES pick up header edits — verified). Then `run_hlsfft_build.sh` picks up the new RTL via
  `create_hdl_plus.tcl`. Full cycle edit→hw→build→program→fft_iso2 ≈ 15 min.
- **Background chains**: launch long chains with the Bash tool's `run_in_background`, NOT a trailing `&`
  in a normal call (the tool waits and times out at 2 min even though `&` detaches — confusing). Append
  a sentinel (`CHAIN_DONE $(date)`) to the log so a Monitor can detect end.
- **`set_root` fails on RE-OPEN of a post-bitstream project** ("Please select a root ... set_root failed").
  The build session's `set_root -module {SAR_TOP::work}` works (fresh hierarchy) but `program_hlsfft.tcl`
  re-opening the finished project cannot re-select the root. **FIX: program INSIDE the build session** —
  build_full_prog_hlsfft.tcl now runs `run_tool PROGRAMDEVICE` right after export (root still set). Don't
  rely on a separate program_hlsfft.tcl for a fresh rebuild.
- **FALSE `TIMING_MET` gate**: the gate reads `designer/SAR_TOP/pinslacks.txt`; if the impl got named
  `impl2` (dirty project residue) that file is MISSING → the reader left `sv=0` → false TIMING_MET →
  bitstream silently not generated. FIXED: missing report now forces `sv=999` (fail). If you see
  `designer/impl2` instead of `designer/SAR_TOP`, the project is dirty → `rm -rf libero_hlsfft` (it's a
  regeneratable build artifact) and rebuild clean.

## 7. Debugging-methodology learnings (meta — apply these first)
Hard-won from the 2026-07-04 all-zero-image debug (see SAR_PIPELINE_STATUS.md):
- **"RETURN=0 / stage completes" ≠ "data is correct".** The pipeline reported RETURN=0 for a whole
  session while emitting an all-zero image. Always verify DATA (CRC / read-back / correlate), not just
  completion. Same trap as [[always-check-timing-closure]] ("stage completes"≠"data correct"≠"timing met").
- **A data-independent stage running fast proves nothing about the data.** The resample is a
  gather+lerp — it runs identically on zeros. "Resample sped up" did NOT mean data flowed.
- **Isolate every stage on silicon before blaming one.** The workhorse was the single-kernel iso test
  (§4): flush a known input to DDR, arm ONE kernel, read its output. That localized the failure to the
  FFT while proving resample/detect/corner-turn/coherency all work. Don't debug the whole pipeline.
- **Measure the input/output boundary of the suspect stage, not just the final output.** The FFT was
  confirmed as the zero-source only by reading its INPUT (rich) and OUTPUT (zero) directly — inference
  from "everything else works" was nearly wrong (a truncated `x/8xw` read of edge zero-fill columns
  briefly mis-blamed the wrong stage; read col 5+ past the edge zero-fill).
- **C-simulation passing does NOT prove the RTL.** SmartHLS `shls sw` + a numpy check passed at corr
  0.9999 for an FFT whose synthesized RTL was a passthrough. Only silicon (or RTL cosim, if it worked)
  is ground truth for HLS. Budget for the possibility that HLS output ≠ HLS source semantics.
- **When an HLS kernel is intractable, move the stage to the CPU.** A plain-C version on the U54 is
  provably correct, firmware-only (fast iteration), and fully controllable — a valid escape hatch when
  a synthesis bug resists multiple structural fixes and cosim is blocked. Trade throughput for correctness
  + iteration speed during bring-up; optimize later.
- **Image-correctness gotchas:** (a) a fixed-point FFT needs a block-exponent (BFP), not per-stage
  truncation, or the AC content rounds to zero (DC-only image). (b) SAR/FFT output matches the golden
  only "up to orientation" — always run an 8-dihedral + transpose search (correlate_cpufft.py), and mask
  saturated pixels before correlating (speckle is unforgiving; a few % saturation tanks the raw number).
- **Iteration-cost awareness:** a fabric rebuild is ~40 min and the Libero flow is fragile; a firmware
  rebuild is ~1.5 min. Push logic to firmware during bring-up whenever correctness allows — it turned an
  intractable multi-rebuild loop into minutes-per-iteration.

## 8. CoreFFT in-place range-FFT (fabric FFT, 2026-07-08) — build + iso-test
Sub-project to run the range-FFT on the **in-place CoreFFT** (8192-pt, 16-bit, conditional BFP)
instead of the CPU FFT. Wrapper `mpfs/fpga/corefft_inplace_wrap.v` (elastic LSRAM FIFO + SCALE_EXP)
is sim-validated vs the real core (see memory `corefft-streaming-vs-inplace`). CoreFFT STREAMING
maxes at 4096-pt + no BFP → 8192 REQUIRES in-place.

**Rebuild the CoreFFT bitstream (headless, ~1 hr, timing-gated):** the `libero_sar` SmartDesign is
the deleted-`.cxf` state, so use the VM-netlist flow — `libero.exe SCRIPT:mpfs/fpga/build_corefft_vm.tcl`
(fresh project `libero_corefft_vm`, `-vm_netlist_flow TRUE`, imports the surviving 62.5-MHz
`SAR_TOP_NL.vm`, associates `SAR_TOP_derived_constraints.sdc` (has the 62.5/7.8125 `create_generated_clock`)
+ `sar_fft_cdc.sdc` + `io/sar_io.pdc`, P&R, gate on `pinslacks.txt`, export). Result: **TIMING MET 0/0
(315,348 pins) → `SAR_TOP_corefft.job` (12.12 MB, FABRIC+SNVM)**. Preserved at
`mpfs/fpga/bitstreams/SAR_TOP_corefft.job`. GOTCHAS: `new_project` rejects `-instantiate_mss_component`
(use the minimal signature); `export_prog_job` needs `file mkdir $exportdir` first; the `.tcl` runs
setup-only first (`STOP_AFTER_SETUP 1`) to fail-fast on API errors before the ~1 hr P&R.

**⚠️ PROGRAM IT RIGHT (the mistake to never repeat):** program the fabric **FABRIC-ONLY**
(`SAR_TOP_corefft.job`, no eNVM), then **re-flash the APP** to eNVM with `bash mpfs/host/run_program.sh`
(`mpfsBootmodeProgrammer` --bootmode 1, via `fpgenprog` — reliable, NOT OpenOCD). **Boot mode 1 + the
APP is the debug state — the app cooperates with JTAG halt.** Do NOT build/flash an **HSS** eNVM
(`build_corefft_bootable.tcl` / boot-mode-1 HSS client): HSS does NOT cooperate with JTAG halt →
`openocd: "Target not halted" / gdb connection rejected`, and you must power-cycle. Re-flashing the
app is REQUIRED after any fabric program that touches eNVM (§6). `mpfs/fpga/bm1/` is run_program.sh's
working dir — `mkdir` it if a cleanup removed it.

**Run the CoreFFT iso-test:** `bash mpfs/host/run_corefft_iso.sh` — generates 8 known 8192-pt rows
(`fft_golden.py`), loads to `SIG`, drives `fft_feeder(0x60004000)→CoreFFT→fft_unloader(0x60005000)`
directly over JTAG (`jtag_full/corefft_iso.gdb.tmpl`), reads back `SCRATCH`, correlates each row vs
the **scale-invariant** BFP golden (CoreFFT's block exponent differs by a power of 2 — corr/nrmse
absorb it, proven in QuestaSim). Uses the §4 pattern: boot (resume, sleep 30, arp_halt), restore
input, `flush_l2_cache(1)` (input→DDR), arm feeder/unloader, `flush_l2_cache(1)` (evict dst), dump.
Offline plumbing self-checks corr=1.0. NOTE: in the CoreFFT build `0x60005000` is the unloader (a
REAL slave) — in the HLS build it's an unused slave (§2: reading it hangs AXI). Host-path GOTCHA:
run_corefft_iso.sh must pass **Windows (`C:/`) paths** to the Windows-native gdb (`restore`/`dump`/ELF)
— MSYS `/c/` paths silently fail "No such file". gdb runs with `-batch </dev/null` so a script error
can't park it at the prompt for 16 min. `CASES=impulse bash run_corefft_iso.sh` runs one row.

**⚠️ SILICON RESULT 2026-07-08 — the fabric's OLD `corefft_stream64_adapter` WEDGES.** First on-silicon
CoreFFT run: input loads fine (`SIG[0]=0x7d000000`), but after arming, **`feeder busy=1` never clears /
`unloader busy=0`** and SCRATCH is unwritten (pre-cleared `SCRATCH[0]` stays 0; rest = stale DDR) →
corr≈0. Reproduces at **1 row (4096 beats) too** → a fundamental FIRST-FRAME stall, not a between-frame
re-arm. Confirms `TIMEOUT_FFT1` is a real feeder/CoreFFT-handshake wedge, NOT a timing artifact (timing
MET 0/0). BUT this fabric (`SAR_TOP_NL.vm`) has the OLD adapter — the NEW sim-validated
`corefft_inplace_wrap` (better elastic FIFO + LSRAM show-ahead) is NOT in it. SmartDebug ROOT-CAUSED it (2026-07-08): the CoreFFT is FINE — `FFT:buf_ready_r=1` (ready, twiddle-init
done), `sync_ngrst delayLine=1` (out of reset). The wedge is the **`fft_feeder` SmartHLS read master**:
`FEED/axi4slv_inst/rd_controller` shows `arvalid=0`, `rd_cnt=0`, `rd_data_valid=0`, and the HLS loop
counter `FEED/…/fft_feeder_BB_4_phi_reg=0` — i.e. the read master **issues ZERO AXI reads** despite
correct config (readback confirmed `src=0x88000000`, `nbeats=4096` via feeder_diag.gdb) and `busy=1`.
Config landed, kernel started, but the read engine never fires -> read FIFO empty -> loop stuck at i=0
-> nothing reaches CoreFFT. This is the SAME class of SmartHLS-2025.2 synth bug as the K_FFT butterfly
([[m3-pipeline-silicon-status]]): cosim-PASS, silicon-DEAD RTL. The `fft_feeder` was never
silicon-validated. FIX: replace `fft_feeder` with a hand-written Verilog AXI read-burst master
(mem->stream, feeding the gearbox) — bypass SmartHLS (unreliable here). NOT the CoreFFT, NOT clocks, NOT
timing, NOT corefft_inplace_wrap. (SmartDebug on libero_corefft_vm/corefft_vm.prjx; diag:
mpfs/host/jtag_full/feeder_diag.gdb reads the ARG regs back.)
**PHASE A DONE (2026-07-08): `mpfs/fpga/fft_feeder_v.v` written + sim-validated** (sim/fft_feeder_v_tb.v,
AXI read-slave BFM + backpressuring stream sink): reads 600 beats via 4KB-aware multi-burst INCR, streams
IN ORDER under backpressure -> 600/600 errors=0 PASS. Single-outstanding AXI4 read master -> elastic LSRAM
FIFO -> AXI4-Stream; AXI4-Lite ctrl matches the HLS reg map (+0x08 START, +0x0c src, +0x10 nbeats). Watch:
`fifo_room` MUST be declared wide (a 1-bit decl truncated 512->0 and the AR branch never fired). PHASE B
(pending) = swap fft_feeder->fft_feeder_v in SAR_TOP + rebuild; blocked by deleted-.cxf (netlist-splice
the module in SAR_TOP_NL.vm, OR reconstruct the SmartDesign). For integration the read master's AXI ID
width must match the DIC initiator port (DIC=8-bit ID -> ID_FIX/sar_axi_idconv -> 4-bit FIC_0; 32-bit addr,
zero-extended 32->38 by ID_FIX — see AMBA_ARCHITECTURE.md §4).

See also `SAR_PIPELINE_STATUS.md` (status + latency roadmap), `SMARTDEBUG_RUNBOOK.md`,
`LIBERO_HEADLESS_PLAYBOOK.md`, `SAR_PIPELINE_PROCESS.md`.
