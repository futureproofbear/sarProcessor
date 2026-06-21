# LPDDR4 sustained write-throughput test — PolarFire SoC Icicle Kit

A bare-metal benchmark that **sustains writes to the board's DDR for 10 s**,
measures the achieved write throughput, then **reads every word back and
verifies it**. Runs on one U54 application core through the Microchip MPFS HAL.

> **Memory part.** The Icicle Kit (`MPFS250T-FCVG484EES`) ships with **2 GB
> LPDDR4**, driven by the MSS DDR controller. There is no discrete DDR4 on the
> standard kit — the method here is identical regardless, the number is just
> labelled LPDDR4 so it isn't mis-reported.

## What it measures

A single U54 core (600 MHz, in-order, no SIMD) streaming 64-bit stores into a
**non-cached write-combine** DDR window. That deliberately measures the
**CPU→DRAM write path**, not L2 cache bandwidth:

- **Non-cached** → every store leaves the core for the DDR controller.
- **Write-combine** → stores are coalesced into burst beats, the way a real
  streaming writer should hit LPDDR4.

This is the honest "how fast can a core push bytes into DDR" figure. It is
**not** the DRAM's theoretical peak — that needs all four U54s plus fabric
masters hammering the controller in parallel (see *Extending*).

## How correctness is guaranteed

Word `i` written during pass `p` holds `(p << 40) | (i & ((1<<40)-1))`:

- low 40 bits = the address → catches stuck/mis-addressed writes,
- high bits = the pass number → catches a word a later pass failed to update.

The write loop only checks the clock **at pass boundaries**, so when it stops
the whole buffer sits at one known pass and readback recomputes every expected
word with no per-word bookkeeping. A `fence` after each pass drains the
write-combine buffer to DRAM before the next timing sample, and again before
readback so we verify what is actually in DRAM, not what is still in flight.

## Files

```
ddr_throughput/
├── src/
│   ├── ddr_bench.c / .h        # portable measure core (no HAL dependency)
│   ├── ddr_bench_config.h      # region address, size, duration, RTC rate
│   └── application/
│       ├── hart0/e51.c         # monitor core: wake the app core, idle
│       └── hart1/u54_1.c       # run the test, report over MMUART0
└── test/
    ├── host_test.c             # runs the SAME core on a PC
    └── Makefile                # `make test`
```

`ddr_bench.c` has no HAL dependency, so the integrity logic is testable off the
board (`test/`) and the HAL glue stays thin.

## Verify the logic on your PC first (no board)

```bash
cd ddr_throughput/test
make test      # uses any host gcc/clang (or SoftConsole's riscv gcc as CC)
```

It builds the core with `-DDDR_BENCH_HOST` (rdtime/fence emulated via
`clock_gettime`), runs a short pass over an 8 MiB buffer, and asserts: a clean
run reports **zero** errors, the buffer matches the documented pattern, and an
injected bit-flip **is** caught. This validates the write/verify contract before
you ever flash hardware.

## Build for the board

These files drop into the Microchip **`mpfs-bare-metal-c-template`** (or the
`icicle-kit-reference-design` bare-metal project), which already contains the
`mpfs_hal` and `mss_mmuart` driver that `e51.c`/`u54_1.c` include:

1. Copy `src/application/hart0/e51.c` and `src/application/hart1/u54_1.c` over
   the template's versions (or merge the benchmark call into yours).
2. Add `src/ddr_bench.c`, `src/ddr_bench.h`, `src/ddr_bench_config.h` to the
   project and the `src` include path.
3. Build the **DDR-Release** configuration (program runs from cached DDR; the
   test region is far above it — see below). Flash/run over the USB-UART.
4. Open the Icicle USB-UART at **115200 8N1** to read the report.

If you boot through the **HSS** instead of bare-metal-from-eNVM, `e51.c` is
unused — call `ddr_bench_run()` from your U54 payload's entry exactly as
`u54_1.c` does.

## Configuration — `ddr_bench_config.h`

| macro | default | meaning |
|-------|---------|---------|
| `DDR_BENCH_BASE_ADDR` | `0x1840000000` | 64-bit non-cached **write-combine** DDR window + 1 GiB |
| `DDR_BENCH_SIZE_BYTES` | 256 MiB | working set swept each pass |
| `DDR_BENCH_DURATION_SEC` | 10 | sustained-write window |
| `DDR_BENCH_RTC_FREQ_HZ` | 1 000 000 | `rdtime` tick rate (Icicle MSS RTC = 1 MHz) |

**The two things to get right:**

- **`BASE_ADDR` must not overlap the program.** The app links into low cached
  DDR (`0x10_0000_0000…`); the default base sits ~1 GiB into the
  write-combine alias, well clear of code/stack/heap. Move it only if your MSS
  configurator places the program above 1 GiB. To benchmark a *different* alias,
  point it at `0x14_00000000` (plain non-cached, ordered, lower number) or
  `0x10_00000000` (cached — but then you are measuring **cache**, not DRAM).
- **`RTC_FREQ_HZ` must match your design.** If the reference design changes the
  RTC clock from 1 MHz, every time/throughput figure scales wrong.

## Reading the report

```
=== PolarFire SoC Icicle Kit - LPDDR4 sustained write test ===
region base : 0x0000001840000000 (non-cached write-combine)
region size : 256 MiB
duration    : 10 s
running, please wait...

--- results ---
passes        : <N>
bytes written : <N * 256 MiB>
write time    : ~10000000 us
WRITE THRPUT  : <X.Y> MB/s          <- the headline number (MB = 1e6)
readback time : <us>
read thrput   : <X.Y> MB/s
--- verify ---
PASS: all 33554432 words read back correctly.
done.
```

`PASS` means every one of the 256 MiB / 8 words survived the 10 s of sustained
writes intact. A `FAIL` prints the count and the first mismatching
index/expected/actual so a fault can be localized.

## Extending to true DRAM saturation

This v1 is intentionally one core (simplicity, and an honest per-core figure).
To push toward the LPDDR4 controller's limit, fan the same `write_pass` out
across `u54_2…u54_4` on disjoint sub-ranges of the buffer, start them together,
and sum the per-core byte counts over a shared wall-clock window. The verify
phase is unchanged — each core owns its slice. The repo's fabric AXI master
(`fpga/rtl/axi_master_rw.sv`) is the other route to higher sustained bandwidth.
