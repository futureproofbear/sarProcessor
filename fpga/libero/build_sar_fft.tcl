# build_sar_fft.tcl -- Libero SoC project builder for the SAR FFT accelerator.
#
# Target: MPFS250T-FCVG484EES (PolarFire SoC, Icicle Kit ES) + 2 GB LPDDR4.
#
# This builds the FABRIC ACCELERATOR block (sar_fft_top) as the synthesis root:
# project + RTL + CoreFFT IP + constraints + synth/place-route/bitstream. The
# full-SoC wiring (MSS + LPDDR4 + FIC + clocks) is a SmartDesign assembled on top
# of this block -- see libero/README.md and soc_integration.tcl for that step.
#
# Run:   libero SCRIPT:build_sar_fft.tcl        (or `source` it in the Tcl console)
# Libero SoC 2023.1+ assumed; commands marked [VERSION] may need minor tweaks.

set PRJ      "sar_fft_prj"
set RTL      "../rtl"
set CON      "constraints"

# ---- power-of-2 FFT lengths = the scene's padded dimensions ----
# Centerfield scene is 5634 x 4319 -> pad to 8192 x 8192. Override here (and in
# the host) to match your decimation; both CoreFFT cores are sized from these.
set FFT_LEN_R 8192   ;# range  FFT length (= N2, cols)
set FFT_LEN_A 8192   ;# azimuth FFT length (= M2, rows)

# -------------------------------------------------------------------- project --
new_project \
    -location $PRJ \
    -name {sar_fft} \
    -project_description {SAR PFA 2-D BFP FFT focuser (fabric-mastered DDR)} \
    -hdl {VERILOG} \
    -family {PolarFireSoC} \
    -die {MPFS250T_ES} \
    -package {FCVG484} \
    -speed {STD} \
    -die_voltage {1.0} \
    -part_range {EXT} \
    -instantiate_in_smartdesign 1 \
    -ondemand_build_dh 1

# ------------------------------------------------------------------- sources --
foreach f { isqrt.sv axi_master_rw.sv axil_regs.sv corefft_wrap.sv \
            sar_ctrl.sv sar_fft_top.sv } {
    import_files -convert_EDN_to_HDL 0 -hdl_source "$RTL/$f"
}

# Bind the real CoreFFT IP (not the behavioral sim model): define SAR_USE_COREFFT
# for synthesis. [VERSION] Synplify-Pro define hook; adjust if your flow differs.
configure_tool -name {SYNTHESIZE} \
    -params {SYNPLIFY_OPTIONS:set_option -hdl_define -set "SAR_USE_COREFFT"}

# ----------------------------------------------------------------- CoreFFT IP --
# Generates two CoreFFT cores (range length FFT_LEN_R, azimuth length FFT_LEN_A),
# both 16-bit, forward, block-floating-point. corefft_wrap instantiates them.
source corefft_config.tcl
sar_configure_corefft "CoreFFT_R" $FFT_LEN_R
sar_configure_corefft "CoreFFT_A" $FFT_LEN_A

# --------------------------------------------------------------------- root ----
set_root -module {sar_fft_top::work}

# -------------------------------------------------------------- constraints ----
import_files -io_pdc "$CON/sar_fft.pdc"
import_files -sdc    "$CON/sar_fft.sdc"
organize_tool_files -tool {SYNTHESIZE} -file "$CON/sar_fft.sdc" \
    -module {sar_fft_top::work} -input_type {constraint}
organize_tool_files -tool {PLACEROUTE} \
    -file "$CON/sar_fft.sdc" -file "$CON/sar_fft.pdc" \
    -module {sar_fft_top::work} -input_type {constraint}

# ----------------------------------------------------------------------- flow --
run_tool -name {SYNTHESIZE}
run_tool -name {PLACEROUTE}
run_tool -name {VERIFYTIMING}
run_tool -name {GENERATEPROGRAMMINGDATA}
# For the full SoC, generate the bitstream from the top SmartDesign instead:
# run_tool -name {GENERATEPROGRAMMINGFILE}

puts "============================================================="
puts " sar_fft_top built. Review the timing report (VERIFYTIMING)."
puts " Next: assemble the SoC SmartDesign (MSS + LPDDR4 + FIC) per"
puts " libero/README.md, then generate the device bitstream."
puts "============================================================="
