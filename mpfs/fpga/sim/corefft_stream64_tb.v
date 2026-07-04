// corefft_stream64_tb.v -- verify corefft_stream64_adapter + real CoreFFT: stream
// the golden input in as 64-bit beats (2 samples/beat), capture 64-bit output beats,
// unpack to per-sample hex for fft_golden.py. Proves the folded 64<->32 gearbox.
`timescale 1ns/1ps
module corefft_stream64_tb;
  parameter integer W = 16;
  parameter integer POINTS = 8192;            // must be even
  parameter         IN_HEX  = "fft_vectors/random_in.hex";
  parameter         OUT_HEX = "rtl_out_stream64.hex";

  reg clk=0, slowclk=0, ngrst=0;
  always #5  clk = ~clk;
  always #40 slowclk = ~slowclk;

  wire [W-1:0] datai_re, datai_im, datao_re, datao_im;
  wire datai_valid, buf_ready, datao_valid, outp_ready, read_outp;
  wire [3:0] scale_exp;
  reg  [4*W-1:0] s_tdata=0; reg s_tvalid=0; wire s_tready;
  wire [4*W-1:0] m_tdata;   wire m_tvalid; reg m_tready=0;

  COREFFT #(.WIDTH(W), .POINTS(POINTS)) fft (
    .CLK(clk), .SLOWCLK(slowclk), .NGRST(ngrst),
    .DATAI_RE(datai_re), .DATAI_IM(datai_im), .DATAI_VALID(datai_valid), .READ_OUTP(read_outp),
    .DATAO_RE(datao_re), .DATAO_IM(datao_im), .DATAO_VALID(datao_valid),
    .BUF_READY(buf_ready), .OUTP_READY(outp_ready), .SCALE_EXP(scale_exp));

  corefft_stream64_adapter #(.W(W)) ad (
    .clk(clk), .resetn(ngrst),
    .s_axis_tdata(s_tdata), .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
    .datai_re(datai_re), .datai_im(datai_im), .datai_valid(datai_valid), .buf_ready(buf_ready),
    .datao_re(datao_re), .datao_im(datao_im), .datao_valid(datao_valid),
    .outp_ready(outp_ready), .read_outp(read_outp),
    .m_axis_tdata(m_tdata), .m_axis_tvalid(m_tvalid), .m_axis_tready(m_tready));

  reg [31:0] inmem [0:POINTS-1];
  integer i, o, fo;
  initial begin
    $readmemh(IN_HEX, inmem);
    ngrst = 0; repeat (4) @(posedge slowclk); ngrst = 1; @(posedge clk);
    // feed POINTS/2 beats of 2 samples each (lower=sample0, upper=sample1)
    i = 0;
    while (i < POINTS/2) begin
      s_tvalid <= 1'b1;
      s_tdata  <= {inmem[2*i+1], inmem[2*i]};
      @(posedge clk);
      if (s_tvalid && s_tready) i = i + 1;
    end
    s_tvalid <= 1'b0;
    // capture POINTS/2 beats -> POINTS samples
    fo = $fopen(OUT_HEX, "w");
    m_tready <= 1'b1; o = 0;
    while (o < POINTS/2) begin
      @(posedge clk);
      if (m_tvalid && m_tready) begin
        $fwrite(fo, "%08x\n", m_tdata[31:0]);   // sample0
        $fwrite(fo, "%08x\n", m_tdata[63:32]);  // sample1
        o = o + 1;
      end
    end
    m_tready <= 1'b0; $fclose(fo);
    $display("stream64 adapter done: wrote %s  SCALE_EXP=%0d", OUT_HEX, scale_exp);
    $finish;
  end
  initial begin #8_000_000; $display("TIMEOUT"); $finish; end
endmodule
