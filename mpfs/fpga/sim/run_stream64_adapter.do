set GEN ../libero_corefft/component/work/COREFFT_C0/COREFFT_C0_0
set CFT ../libero_corefft/component/work/COREFFT_C0
set DC  ../libero_corefft/component/Actel/DirectCore/COREFFT/8.1.100/rtl/in_place/vlog/core
set PF C:/Microchip/Libero_SoC_2025.2/Libero_SoC/Designer/lib/vlog/polarfire.v
vlib work
vlog -sv $PF
vlog $GEN/rtl/in_place/vlog/core/COREFFT_C0_COREFFT_C0_0_lsram_g5.v $GEN/twiddle32.v $GEN/rtl/in_place/vlog/core/COREFFT.v $GEN/rtl/in_place/vlog/core/fftDp.v $GEN/rtl/in_place/vlog/core/COREFFT_TOP.v $DC/mac_lib.v $DC/cmplx.v $DC/kit.v $DC/fftSm.v $CFT/COREFFT_C0.v
vlog -sv corefft_shim.v ../corefft_stream64_adapter.v corefft_stream64_tb.v
vsim -c -gPOINTS=8192 -gIN_HEX=fft_vectors/random_in.hex -gOUT_HEX=rtl_out_stream64.hex corefft_stream64_tb
run -all
quit -f
