# run_modelsim.do -- compile + run the SAR FFT testbench in ModelSim/QuestaSim
# (the simulator that ships with Libero SoC).
#
# Usage (from fpga/sim/vectors/, so the .hex + params.vh resolve in the CWD):
#     cd fpga/sim/vectors
#     vsim -c -do ../run_modelsim.do
#
# Generate the vectors first:  python ../../scripts/gen_vectors.py --m 10 --n 6
# The testbench binds the behavioral corefft_model (SAR_USE_COREFFT is NOT
# defined here); for a CoreFFT-IP build that swap happens in Libero instead.

if {[file exists work]} { vdel -all }
vlib work

set RTL ../../rtl
set SIM ..

vlog -sv +incdir+. \
    $RTL/isqrt.sv \
    $RTL/axi_master_rw.sv \
    $RTL/axil_regs.sv \
    $RTL/corefft_wrap.sv \
    $RTL/sar_ctrl.sv \
    $RTL/sar_fft_top.sv \
    $SIM/corefft_model.sv \
    $SIM/axi_ddr_model.sv \
    $SIM/tb_sar_fft_top.sv

vsim -voptargs=+acc work.tb_sar_fft_top
run -all
quit -f
