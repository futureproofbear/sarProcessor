# corefft_config.tcl -- configure + generate Microchip CoreFFT instances.
#
# Defines a helper that creates one CoreFFT core of a given length, matching the
# datapath that corefft_wrap / sar_ctrl expect: 16-bit complex, forward FFT,
# block-floating-point (the core returns a block exponent we report as EXP_R/A).
#
# [VERSION] CoreFFT parameter names vary across IP releases. Open the CoreFFT
# config GUI once to confirm the exact -params keys for your installed version,
# then mirror them here. Key settings, however named:
#   N (transform length)            = $len   (power of 2)
#   Forward/Inverse                 = Forward
#   Input/output data width         = 16
#   Twiddle width                   = 18  (matches the 18x18 math blocks)
#   Block floating point            = ENABLED (exposes the block-exponent output)
#   Rounding                        = TRUNCATE (matches the fixed-point study)
#   Memory                          = LSRAM/uSRAM as fitted

proc sar_configure_corefft {inst len} {
    # download the core from the vault if needed (no-op if already present)
    catch { download_core -vlnv {Actel:DirectCore:CoreFFT:7.0.103} -location {./vault} }

    create_and_configure_core \
        -core_vlnv {Actel:DirectCore:CoreFFT:7.0.103} \
        -component_name $inst \
        -params [list \
            "N:$len" \
            "DIRECTION:0" \
            "DWIDTH:16" \
            "TWIDTH:18" \
            "BFP:1" \
            "ROUND:0" ]
    generate_component -component $inst
    puts "configured CoreFFT '$inst' : N=$len, 16-bit, forward, BFP, truncating"
}
