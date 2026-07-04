# run_m1_corefft.do -- M1 co-sim with the REAL Libero-generated CoreFFT IP.
# Run from sim/:  vsim -c -do run_m1_corefft.do
# (generate vectors first: python ../../host/fft_golden.py gen --n 8192 --bits 16 --tw-bits 16 --twiddle --out fft_vectors)

set GEN ../libero_corefft/component/work/COREFFT_C0/COREFFT_C0_0
set CFT ../libero_corefft/component/work/COREFFT_C0
set DC  ../libero_corefft/component/Actel/DirectCore/COREFFT/8.1.100/rtl/in_place/vlog/core

set PF C:/Microchip/Libero_SoC_2025.2/Libero_SoC/Designer/lib/vlog/polarfire.v

vlib work
# PolarFire device primitive sim models (RAM1K20, MACC_PA, CFG1, GND, VCC).
# -sv: this library uses SystemVerilog constructs.
vlog -sv $PF
# Generated CoreFFT RTL (per COREFFT_C0_manifest.txt)
vlog $GEN/rtl/in_place/vlog/core/COREFFT_C0_COREFFT_C0_0_lsram_g5.v \
     $GEN/twiddle32.v \
     $GEN/rtl/in_place/vlog/core/COREFFT.v \
     $GEN/rtl/in_place/vlog/core/fftDp.v \
     $GEN/rtl/in_place/vlog/core/COREFFT_TOP.v \
     $DC/mac_lib.v $DC/cmplx.v $DC/kit.v $DC/fftSm.v \
     $CFT/COREFFT_C0.v
# Shim (COREFFT -> COREFFT_C0) + the shared testbench
vlog -sv corefft_shim.v ../corefft_fft_tb.v

vsim -c -gPOINTS=8192 -gWIDTH=16 -gEXPW=4 \
     -gIN_HEX=fft_vectors/random_in.hex -gOUT_HEX=rtl_out_corefft.hex \
     corefft_fft_tb
run -all
quit -f
