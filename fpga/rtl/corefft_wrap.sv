// corefft_wrap.sv -- uniform handshake around the FFT engine.
//
// Contract (one frame):
//   1. consumer pulses `start`.
//   2. wrap holds `in_ready` high; consumer streams FFT_LEN complex samples in
//      natural order 0..FFT_LEN-1 (transfer on in_valid & in_ready).
//   3. wrap computes a forward, block-floating-point FFT.
//   4. wrap streams FFT_LEN results in natural order on out_valid, with `out_last`
//      on the final beat, `blk_exp` stable across the output, and `done` pulsing
//      at the end.  blk_exp = number of right-shifts the BFP applied.
//
// Synthesis: define SAR_USE_COREFFT to bind the Microchip CoreFFT IP (configured
// by libero/corefft_config.tcl). The CoreFFT port names vary by IP version --
// the instantiation below is a TEMPLATE: match it to your generated core, and
// implement the small load/unload adapter the core needs. By default (no define)
// the behavioral `corefft_model` is bound, which is what the testbench uses.
`default_nettype none
module corefft_wrap #(
    parameter int FFT_LEN  = 8192,
    parameter int LOG2_LEN = 13,
    parameter int DIN_W    = 16,
    parameter int DOUT_W   = 16,
    parameter int EXPW     = 5
) (
    input  wire                 clk,
    input  wire                 rstn,
    input  wire                 start,
    input  wire                 in_valid,
    input  wire [DIN_W-1:0]     in_re,
    input  wire [DIN_W-1:0]     in_im,
    output wire                 in_ready,
    output wire                 out_valid,
    output wire [DOUT_W-1:0]    out_re,
    output wire [DOUT_W-1:0]    out_im,
    output wire                 out_last,
    output wire [EXPW-1:0]      blk_exp,
    output wire                 busy,
    output wire                 done
);
`ifdef SAR_USE_COREFFT
    // ---------------------------------------------------------------------- //
    //  REAL CoreFFT INTEGRATION (template -- complete for your IP version)    //
    // ---------------------------------------------------------------------- //
    //  CoreFFT is a load -> run -> unload core with an internal frame RAM and
    //  a BLK_EXP (block exponent) output. Typical flow: assert the load enable,
    //  stream FFT_LEN samples, pulse START/GO, wait DONE, then read the result
    //  RAM while sampling BLK_EXP. Wire that sequencing here so the four
    //  handshake signals above behave per the contract.  Example skeleton:
    //
    //  CoreFFT #(.N(FFT_LEN), .NB(DIN_W), .NTW(18)) u_core (
    //      .CLK(clk), .ARST_N(rstn),
    //      .START(start), .DI_RE(in_re), .DI_IM(in_im), .DI_VALID(in_valid),
    //      .DI_READY(in_ready), .DO_RE(out_re), .DO_IM(out_im),
    //      .DO_VALID(out_valid), .DO_LAST(out_last), .BLK_EXP(blk_exp),
    //      .BUSY(busy), .DONE(done));
    //
    //  Until wired, error out so an unconfigured synthesis build fails loudly:
    initial $error("corefft_wrap: SAR_USE_COREFFT set but CoreFFT instance not wired in");
    assign in_ready = 1'b0; assign out_valid = 1'b0; assign out_re = '0;
    assign out_im = '0; assign out_last = 1'b0; assign blk_exp = '0;
    assign busy = 1'b0; assign done = 1'b0;
`else
    // ---- behavioral model (simulation / default) ----
    corefft_model #(
        .FFT_LEN (FFT_LEN), .LOG2_LEN(LOG2_LEN),
        .DIN_W   (DIN_W),   .DOUT_W  (DOUT_W), .EXPW(EXPW)
    ) u_model (
        .clk(clk), .rstn(rstn), .start(start),
        .in_valid(in_valid), .in_re(in_re), .in_im(in_im), .in_ready(in_ready),
        .out_valid(out_valid), .out_re(out_re), .out_im(out_im),
        .out_last(out_last), .blk_exp(blk_exp), .busy(busy), .done(done)
    );
`endif
endmodule
`default_nettype wire
