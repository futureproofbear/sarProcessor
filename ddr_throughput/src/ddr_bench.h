/*
 * ddr_bench.h - sustained LPDDR4 write-throughput benchmark core.
 *
 * Pure C, no HAL dependency: it takes a DDR buffer and a duration and does the
 * write loop, the drain, and the readback verification. The HAL glue
 * (e51.c / u54_1.c) supplies the buffer and prints the result; a host test
 * exercises the very same code on a PC.
 */
#ifndef DDR_BENCH_H
#define DDR_BENCH_H

#include <stdint.h>

typedef struct {
    uint64_t passes;          /* full sweeps of the buffer completed         */
    uint64_t bytes_written;   /* passes * buffer_bytes                        */
    uint64_t write_ticks;     /* rdtime ticks spent in the write phase        */
    uint64_t readback_ticks;  /* rdtime ticks spent verifying                 */
    uint64_t errors;          /* mismatching words found on readback          */
    uint64_t first_bad_index; /* word index of the first mismatch (if any)    */
    uint64_t first_bad_expected;
    uint64_t first_bad_actual;
} ddr_bench_result_t;

/*
 * Write a verifiable pattern over [buf, buf+nwords) repeatedly until
 * duration_ticks of rdtime have elapsed (the in-progress sweep is always
 * finished, so the whole buffer ends at a single known pass), drain to DRAM,
 * then read every word back and check it. Results land in *res.
 *
 * buf must point into a non-cached DDR window (see ddr_bench_config.h).
 */
void ddr_bench_run(volatile uint64_t *buf, uint64_t nwords,
                   uint64_t duration_ticks, ddr_bench_result_t *res);

/* Expected word value for index i after a given pass. Exposed for testing. */
uint64_t ddr_bench_word(uint64_t pass, uint64_t i);

#endif /* DDR_BENCH_H */
