/*
 * u54_1.c - PolarFire SoC application core (hart 1): the benchmark front end.
 *
 * Runs the sustained LPDDR4 write-throughput test and reports over MMUART0
 * (115200 8N1, the Icicle Kit's USB-UART). Formatting is done with tiny
 * integer helpers so the report does not depend on newlib float/long-long
 * printf support.
 */
#include "mpfs_hal/mss_hal.h"
#include "drivers/mss/mss_mmuart/mss_uart.h"

#include "ddr_bench.h"
#include "ddr_bench_config.h"

static void put_str(const char *s)
{
    MSS_UART_polled_tx_string(&g_mss_uart0_lo, (const uint8_t *)s);
}

/* Append decimal text for v into buf (caller-sized), return new length. */
static void put_u64(uint64_t v)
{
    char tmp[21];          /* 2^64 is 20 digits + NUL */
    int n = 0;
    if (v == 0) {
        put_str("0");
        return;
    }
    while (v > 0) {
        tmp[n++] = (char)('0' + (v % 10));
        v /= 10;
    }
    char out[21];
    int k = 0;
    while (n > 0) {
        out[k++] = tmp[--n];
    }
    out[k] = '\0';
    put_str(out);
}

static void put_hex64(uint64_t v)
{
    static const char digits[] = "0123456789abcdef";
    char out[19];
    out[0] = '0';
    out[1] = 'x';
    for (int i = 0; i < 16; ++i) {
        out[2 + i] = digits[(v >> ((15 - i) * 4)) & 0xF];
    }
    out[18] = '\0';
    put_str(out);
}

/* Print a MB/s figure (MB = 1e6) with one decimal place, from bytes and us. */
static void put_mbps(uint64_t bytes, uint64_t us)
{
    if (us == 0) {
        put_str("n/a");
        return;
    }
    uint64_t whole = bytes / us;              /* bytes/us == MB/s */
    uint64_t tenths = (bytes * 10ULL / us) % 10ULL;
    put_u64(whole);
    put_str(".");
    put_u64(tenths);
    put_str(" MB/s");
}

void u54_1(void)
{
    clear_soft_interrupt();
    set_csr(mie, MIP_MSIP);
    __enable_irq();

    MSS_UART_init(&g_mss_uart0_lo, MSS_UART_115200_BAUD,
                  MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY |
                  MSS_UART_ONE_STOP_BIT);

    put_str("\r\n=== PolarFire SoC Icicle Kit - LPDDR4 sustained write test ===\r\n");
    put_str("region base : ");
    put_hex64(DDR_BENCH_BASE_ADDR);
    put_str(" (non-cached write-combine)\r\n");
    put_str("region size : ");
    put_u64(DDR_BENCH_SIZE_BYTES / (1024 * 1024));
    put_str(" MiB\r\nduration    : ");
    put_u64(DDR_BENCH_DURATION_SEC);
    put_str(" s\r\nrunning, please wait...\r\n");

    volatile uint64_t *buf = (volatile uint64_t *)DDR_BENCH_BASE_ADDR;
    uint64_t nwords = DDR_BENCH_SIZE_BYTES / sizeof(uint64_t);
    uint64_t duration_ticks =
        (uint64_t)DDR_BENCH_DURATION_SEC * DDR_BENCH_RTC_FREQ_HZ;

    ddr_bench_result_t r;
    ddr_bench_run(buf, nwords, duration_ticks, &r);

    uint64_t write_us = r.write_ticks * 1000000ULL / DDR_BENCH_RTC_FREQ_HZ;
    uint64_t read_us = r.readback_ticks * 1000000ULL / DDR_BENCH_RTC_FREQ_HZ;

    put_str("\r\n--- results ---\r\n");
    put_str("passes        : ");
    put_u64(r.passes);
    put_str("\r\nbytes written : ");
    put_u64(r.bytes_written);
    put_str("\r\nwrite time    : ");
    put_u64(write_us);
    put_str(" us\r\nWRITE THRPUT  : ");
    put_mbps(r.bytes_written, write_us);
    put_str("\r\nreadback time : ");
    put_u64(read_us);
    put_str(" us\r\nread thrput   : ");
    put_mbps(DDR_BENCH_SIZE_BYTES, read_us);
    put_str("\r\n\r\n--- verify ---\r\n");

    if (r.errors == 0) {
        put_str("PASS: all ");
        put_u64(nwords);
        put_str(" words read back correctly.\r\n");
    } else {
        put_str("FAIL: ");
        put_u64(r.errors);
        put_str(" mismatched words. first at index ");
        put_u64(r.first_bad_index);
        put_str("\r\n  expected ");
        put_hex64(r.first_bad_expected);
        put_str("\r\n  got      ");
        put_hex64(r.first_bad_actual);
        put_str("\r\n");
    }

    put_str("\r\ndone.\r\n");
    for (;;) {
        __asm volatile("wfi");
    }
}
