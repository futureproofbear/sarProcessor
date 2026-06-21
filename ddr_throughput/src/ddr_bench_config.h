/*
 * ddr_bench_config.h - tunables for the LPDDR4 sustained-write benchmark.
 *
 * Target: PolarFire SoC Icicle Kit (MPFS250T-FCVG484EES), on-board 2 GB LPDDR4
 * reached through the MSS DDR controller. The kit has no discrete DDR4; the
 * methodology is identical, only the part name differs.
 */
#ifndef DDR_BENCH_CONFIG_H
#define DDR_BENCH_CONFIG_H

/*
 * Where to write.
 *
 * We use the 64-bit NON-CACHED WRITE-COMBINE DDR window (0x18_0000_0000).
 * Rationale:
 *   - Non-cached  -> every store actually leaves the core for the DDR
 *     controller, so we measure DRAM write bandwidth, not L2 cache bandwidth.
 *   - Write-combine -> the MSS coalesces stores into burst-sized beats, which
 *     is how a real streaming writer should hit LPDDR4. (Plain non-cached at
 *     0x14_0000_0000 also works and is fully ordered, but reports a lower,
 *     un-combined number.)
 *   - 64-bit window -> 2 GB is fully addressable and the test region sits far
 *     from the program image (which links into low cached DDR), so the
 *     benchmark cannot scribble over its own code/stack/heap.
 *
 * BASE puts the region 1 GiB into the window (DDR physical ~1 GiB with the
 * stock Icicle SEG mapping). Move it only if your MSS configurator places the
 * program above 1 GiB.
 */
#define DDR_BENCH_BASE_ADDR   0x1840000000ULL   /* WCB window + 1 GiB */
#define DDR_BENCH_SIZE_BYTES  (256ULL * 1024 * 1024)   /* 256 MiB working set */

/* How long to sustain the write phase. */
#define DDR_BENCH_DURATION_SEC   10U

/*
 * RISC-V `time` CSR (rdtime) tick rate. On the Icicle Kit the MSS RTC / CLINT
 * mtime is clocked at 1 MHz, so one tick == 1 us. If your reference design
 * changes the RTC clock, fix this or the throughput number will be wrong.
 */
#define DDR_BENCH_RTC_FREQ_HZ    1000000ULL

#endif /* DDR_BENCH_CONFIG_H */
