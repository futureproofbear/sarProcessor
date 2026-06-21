# soc_integration.tcl -- SmartDesign skeleton: drop sar_fft_top into a PolarFire
# SoC top with the MSS (LPDDR4 + FICs), a CCC clock, and reset.
#
# SKELETON / [VERSION]: the MSS is configured once in the MSS configurator GUI
# and exported as a component; reference that component name below. Connection
# (`sd_connect_pins`) names follow the MSS/FIC bus interfaces of your config.
# Run after build_sar_fft.tcl (same project), then set PF_SOC_TOP as root and
# generate the bitstream.

set TOP   "PF_SOC_TOP"
set ACCEL "sar_fft_top"
set MSS   "PFSOC_MSS_C0"     ;# <- your exported MSS component name

create_smartdesign -sd_name $TOP

# --- instantiate the blocks ---
sd_instantiate_component -sd_name $TOP -component_name $MSS   -instance_name "u_mss"
sd_instantiate_hdl_module -sd_name $TOP -hdl_module_name $ACCEL -instance_name "u_accel"
# CCC (fabric clock) + CoreReset -- add from the IP catalog:
# sd_instantiate_component -sd_name $TOP -component_name "FCCC_C0"      -instance_name "u_ccc"
# sd_instantiate_component -sd_name $TOP -component_name "CORERESET_PF_C0" -instance_name "u_rst"

# --- clock + reset to the accelerator ---
# sd_connect_pins -sd_name $TOP -pin_names {u_ccc:OUT0_FABCLK_0 u_accel:clk}
# sd_connect_pins -sd_name $TOP -pin_names {u_rst:FABRIC_RESET_N u_accel:rstn}

# --- control plane: MSS-master FIC  ->  accelerator AXI4-Lite slave ---
# Use a FIC configured as an AXI4 *initiator* into the fabric. Connect its AXI4
# (or AXI4-Lite) bus interface to u_accel's s_axil_* (group the s_axil_* pins as
# an AXI4-Lite mirrored interface, or connect pin-by-pin).
# sd_connect_pins -sd_name $TOP -pin_names {u_mss:FIC_0_AXI4_INITIATOR u_accel:S_AXIL}

# --- data plane: accelerator AXI4 master -> MSS-slave FIC -> LPDDR4 ---
# Use a FIC configured as an AXI4 *target* (to the coherent or non-coherent DDR
# path). Connect u_accel's m_axi_* to it.
# sd_connect_pins -sd_name $TOP -pin_names {u_accel:M_AXI u_mss:FIC_1_AXI4_TARGET}

# --- done interrupt ---
# sd_connect_pins -sd_name $TOP -pin_names {u_accel:irq u_mss:FABRIC_IRQ_0}

# --- LPDDR4 / SGMII / MMUART etc. are inside the MSS component + board PDC ---

sd_save -sd_name $TOP
generate_component -component $TOP
set_root -module "$TOP::work"
# run_tool -name {SYNTHESIZE}
# run_tool -name {PLACEROUTE}
# run_tool -name {GENERATEPROGRAMMINGFILE}
puts "PF_SOC_TOP skeleton created -- finish the MSS component + FIC connections."
