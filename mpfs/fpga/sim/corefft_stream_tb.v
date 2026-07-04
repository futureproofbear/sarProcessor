// corefft_stream_tb.v -- verify corefft_stream_adapter + the REAL CoreFFT together:
// push the golden input vector in through the AXI4-Stream interface, capture the
// result out through the stream, and write rtl_out_stream.hex for fft_golden.py.
// Same data path the DMA will drive; proves the adapter bridges stream<->CoreFFT.
`timescale 1ns/1ps
module corefft_stream_tb;
  parameter integer W = 16;
  parameter integer POINTS = 64;
  parameter         IN_HEX  = "fft_vectors/random_in.hex";
  parameter         OUT_HEX = "rtl_out_stream.hex";

  reg clk=0, slowclk=0, ngrst=0;
  always #5  clk = ~clk;
  always #40 slowclk = ~slowclk;

  // adapter <-> CoreFFT
  wire [W-1:0] datai_re, datai_im, datao_re, datao_im;
  wire datai_valid, buf_ready, datao_valid, outp_ready, read_outp;
  wire [3:0] scale_exp;
  // stream sides
  reg  [2*W-1:0] s_tdata=0; reg s_tvalid=0; wire s_tready;
  wire [2*W-1:0] m_tdata;   wire m_tvalid; reg m_tready=0;

  COREFFT #(.WIDTH(W), .POINTS(POINTS)) fft (
    .CLK(clk), .SLOWCLK(slowclk), .NGRST(ngrst),
    .DATAI_RE(datai_re), .DATAI_IM(datai_im), .DATAI_VALID(datai_valid),
    .READ_OUTP(read_outp),
    .DATAO_RE(datao_re), .DATAO_IM(datao_im), .DATAO_VALID(datao_valid),
    .BUF_READY(buf_ready), .OUTP_READY(outp_ready), .SCALE_EXP(scale_exp));

  corefft_stream_adapter #(.W(W)) ad (
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

    // ---- stream the input in (beat transfers when tvalid & tready) ----
    i = 0;
    while (i < POINTS) begin
      s_tvalid <= 1'b1;
      s_tdata  <= inmem[i];
      @(posedge clk);
      if (s_tvalid && s_tready) i = i + 1;
    end
    s_tvalid <= 1'b0;

    // ---- capture the output stream ----
    fo = $fopen(OUT_HEX, "w");
    m_tready <= 1'b1;
    o = 0;
    while (o < POINTS) begin
      @(posedge clk);
      if (m_tvalid && m_tready) begin
        $fwrite(fo, "%08x\n", m_tdata);
        o = o + 1;
      end
    end
    m_tready <= 1'b0;
    $fclose(fo);
    $display("stream adapter done: wrote %s  SCALE_EXP=%0d", OUT_HEX, scale_exp);
    $finish;
  end

  initial begin #5_000_000; $display("TIMEOUT"); $finish; end
endmodule
