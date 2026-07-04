# run_m1.do -- M1 CoreFFT co-simulation in QuestaSim/ModelSim.
#
# Usage (from this sim/ dir):
#   python ../../host/fft_golden.py gen --n 64 --bits 16 --tw-bits 16 --twiddle --out fft_vectors
#   vsim -c -do run_m1.do
#   python ../../host/fft_golden.py check --expected fft_vectors/random_out.hex \
#          --actual rtl_out.hex --corr-min 0.9999 --nrmse-max 0.05
#
# Swap to the REAL CoreFFT: replace corefft_behav.v in the vlog line with the
# Libero-generated core sources, and (for full size) regen vectors with --n 8192
# and set -gPOINTS=8192 below. Everything else is unchanged.
#
# CASE / N are overridable:  vsim -c -do "set CASE tone; set N 64; do run_m1.do"

if {![info exists CASE]} { set CASE random }
if {![info exists N]}    { set N 64 }

vlib work
vlog -sv corefft_behav.v ../corefft_fft_tb.v
vsim -c -gPOINTS=$N -gWIDTH=16 -gEXPW=5 \
     -gIN_HEX=fft_vectors/${CASE}_in.hex -gOUT_HEX=rtl_out.hex \
     corefft_fft_tb
run -all
quit -f
