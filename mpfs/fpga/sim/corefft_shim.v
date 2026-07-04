// corefft_shim.v -- adapts the Libero-generated COREFFT_C0 wrapper to the
// `COREFFT` module name/params that corefft_fft_tb.v instantiates, so the same
// testbench drives the REAL CoreFFT IP (in-place, POINTS=8192, WIDTH=16,
// conditional BFP, SCALE_EXP enabled). Config is baked into COREFFT_C0; the
// shim params are accepted for TB compatibility and ignored.
`timescale 1ns/1ps
module COREFFT #(parameter integer WIDTH = 16, parameter integer POINTS = 8192,
                 parameter integer SCALE = 0, parameter integer SCALE_EXP_ON = 1) (
  input  wire CLK, SLOWCLK, NGRST,
  input  wire signed [WIDTH-1:0] DATAI_RE, DATAI_IM,
  input  wire DATAI_VALID, READ_OUTP,
  output wire signed [WIDTH-1:0] DATAO_RE, DATAO_IM,
  output wire DATAO_VALID, BUF_READY, OUTP_READY,
  output wire [3:0] SCALE_EXP            // real core: ceil(log2(POINTS))+1 = 4 bits @ 8192
);
  COREFFT_C0 u (
    .CLK(CLK), .SLOWCLK(SLOWCLK), .NGRST(NGRST),
    .DATAI_RE(DATAI_RE), .DATAI_IM(DATAI_IM), .DATAI_VALID(DATAI_VALID),
    .READ_OUTP(READ_OUTP),
    .DATAO_RE(DATAO_RE), .DATAO_IM(DATAO_IM), .DATAO_VALID(DATAO_VALID),
    .BUF_READY(BUF_READY), .OUTP_READY(OUTP_READY), .SCALE_EXP(SCALE_EXP)
  );
endmodule
