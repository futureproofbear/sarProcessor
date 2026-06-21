# sar_fft.sdc -- timing constraints for the SAR FFT accelerator block.
#
# Fabric clock. Target ~150 MHz; relax to 100 MHz first if CoreFFT + the DDR
# master do not close timing, then optimize. In the SoC the clock comes from a
# PolarFire SoC CCC/PLL (e.g. the MSS fabric clock); rename to match.

create_clock -name {clk} -period 6.667 [ get_ports {clk} ]   ;# 150 MHz

# Asynchronous, externally-synchronized reset.
set_false_path -from [ get_ports {rstn} ]

# AXI4-Lite (control) and AXI4 (DDR master) are all synchronous to clk in this
# block, so no cross-clock constraints here. When the AXI master crosses into a
# different MSS FIC clock domain in the SoC SmartDesign, add the appropriate
# set_clock_groups / max_delay on that boundary there.

# I/O budgets are not meaningful for this block on its own (its ports connect to
# the MSS, not device pins). Constrain them at the SoC top level instead.
