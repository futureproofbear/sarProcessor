/*
 * ddr_bench.c - sustained LPDDR4 write-throughput benchmark core.
 *
 * The verifiable pattern: word i during pass p holds (p << 40) | (i & MASK).
 * The low 40 bits encode the address (catches mis-addressed / stuck-address
 * writes; a 256 MiB buffer is only 2^25 words, so 40 bits is ample headroom),
 * the high bits encode the pass number (catches a word that a later pass
 * failed to overwrite). Because every store is just `tag | i`, the inner loop
 * costs one OR and one store - we measure memory, not arithmetic.
 *
 * The write loop only checks the clock at pass boundaries, so when it stops the
 * entire buffer is uniformly at the last completed pass and readback can
 * recompute every expected word with no per-word bookkeeping.
 */
#include "ddr_bench.h"

#define DDR_BENCH_TAG_SHIFT 40
#define DDR_BENCH_ADDR_MASK ((1ULL << DDR_BENCH_TAG_SHIFT) - 1ULL)

uint64_t ddr_bench_word(uint64_t pass, uint64_t i)
{
    return (pass << DDR_BENCH_TAG_SHIFT) | (i & DDR_BENCH_ADDR_MASK);
}

#if defined(DDR_BENCH_HOST)
/* Host build: emulate rdtime/fence so the core logic can be unit tested. */
#include <time.h>
static inline uint64_t bench_now_ticks(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    /* 1 MHz ticks to match DDR_BENCH_RTC_FREQ_HZ on the board. */
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}
static inline void bench_fence(void) { __asm__ volatile("" ::: "memory"); }
#else
/* Target build: RV64 time CSR and a full memory fence. */
static inline uint64_t bench_now_ticks(void)
{
    uint64_t t;
    __asm__ volatile("rdtime %0" : "=r"(t));
    return t;
}
static inline void bench_fence(void)
{
    /* Drain the write-combine buffer / order stores ahead of the next read. */
    __asm__ volatile("fence" ::: "memory");
}
#endif

/* One full sweep, 8x unrolled. `buf` is volatile so every store is emitted. */
static void write_pass(volatile uint64_t *buf, uint64_t nwords, uint64_t pass)
{
    uint64_t tag = pass << DDR_BENCH_TAG_SHIFT;
    uint64_t i = 0;
    for (; i + 8 <= nwords; i += 8) {
        buf[i + 0] = tag | (i + 0);
        buf[i + 1] = tag | (i + 1);
        buf[i + 2] = tag | (i + 2);
        buf[i + 3] = tag | (i + 3);
        buf[i + 4] = tag | (i + 4);
        buf[i + 5] = tag | (i + 5);
        buf[i + 6] = tag | (i + 6);
        buf[i + 7] = tag | (i + 7);
    }
    for (; i < nwords; ++i) {
        buf[i] = tag | i;
    }
}

void ddr_bench_run(volatile uint64_t *buf, uint64_t nwords,
                   uint64_t duration_ticks, ddr_bench_result_t *res)
{
    res->passes = 0;
    res->bytes_written = 0;
    res->write_ticks = 0;
    res->readback_ticks = 0;
    res->errors = 0;
    res->first_bad_index = 0;
    res->first_bad_expected = 0;
    res->first_bad_actual = 0;

    if (nwords == 0) {
        return;
    }

    /* ---- write phase: sustain until the deadline, on pass boundaries ---- */
    uint64_t t0 = bench_now_ticks();
    uint64_t deadline = t0 + duration_ticks;
    uint64_t pass = 0;
    do {
        write_pass(buf, nwords, pass);
        bench_fence();          /* push this sweep to DRAM before we time it */
        pass++;
    } while (bench_now_ticks() < deadline);
    uint64_t t1 = bench_now_ticks();

    res->passes = pass;
    res->bytes_written = pass * nwords * sizeof(uint64_t);
    res->write_ticks = t1 - t0;

    /* ---- verify phase: read every word back from DRAM and compare ---- */
    uint64_t last_pass = pass - 1;
    uint64_t tag = last_pass << DDR_BENCH_TAG_SHIFT;
    uint64_t r0 = bench_now_ticks();
    for (uint64_t i = 0; i < nwords; ++i) {
        uint64_t got = buf[i];
        uint64_t want = tag | (i & DDR_BENCH_ADDR_MASK);
        if (got != want) {
            if (res->errors == 0) {
                res->first_bad_index = i;
                res->first_bad_expected = want;
                res->first_bad_actual = got;
            }
            res->errors++;
        }
    }
    res->readback_ticks = bench_now_ticks() - r0;
}
