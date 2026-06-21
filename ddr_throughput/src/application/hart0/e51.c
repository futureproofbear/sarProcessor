/*
 * e51.c - PolarFire SoC monitor core (hart 0).
 *
 * Minimal role for this benchmark: enable the MMUART0 clock, wake the first
 * application core (u54_1), then idle. All measurement and reporting happens on
 * u54_1 so a single hart owns the UART and the DDR test region.
 *
 * Matches the Microchip "mpfs-bare-metal-c-template" startup model. If your
 * project boots through the HSS instead, the HSS launches the u54 payload and
 * this file is unused - put the benchmark call from u54_1.c into your payload's
 * application entry instead.
 */
#include "mpfs_hal/mss_hal.h"

void e51(void)
{
    /* Route MMUART0 to the U54 application core and turn its clock on. */
    (void)mss_config_clk_rst(MSS_PERIPH_MMUART0, (uint8_t)MPFS_HAL_FIRST_HART,
                             PERIPHERAL_ON);

    /* Enable software interrupts, then release hart 1 from its boot WFI. */
    clear_soft_interrupt();
    set_csr(mie, MIP_MSIP);
    __enable_irq();
    raise_soft_interrupt(1U);

    for (;;) {
        __asm volatile("wfi");
    }
}
