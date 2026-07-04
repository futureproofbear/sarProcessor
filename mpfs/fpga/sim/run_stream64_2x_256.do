set GEN ../libero_corefft256/component/work/COREFFT_C0/COREFFT_C0_0
set CFT ../libero_corefft256/component/work/COREFFT_C0
set DC  ../libero_corefft256/component/Actel/DirectCore/COREFFT/8.1.100/rtl/in_place/vlog/core
set PF C:/Microchip/Libero_SoC_2025.2/Libero_SoC/Designer/lib/vlog/polarfire.v
vlib work256
vmap work work256
vlog -sv -work work256 $PF
vlog -work work256 $GEN/rtl/in_place/vlog/core/COREFFT_C0_COREFFT_C0_0_lsram_g5.v $GEN/twiddle32.v $GEN/rtl/in_place/vlog/core/COREFFT.v $GEN/rtl/in_place/vlog/core/fftDp.v $GEN/rtl/in_place/vlog/core/COREFFT_TOP.v $DC/mac_lib.v $DC/cmplx.v $DC/kit.v $DC/fftSm.v $CFT/COREFFT_C0.v
vlog -sv -work work256 corefft_shim.v ../corefft_stream64_adapter.v corefft_stream64_2x_tb.v
vsim -c -gPOINTS=256 -gIN_HEX=fft_vectors/random_in.hex corefft_stream64_2x_tb
run -all
quit -f
