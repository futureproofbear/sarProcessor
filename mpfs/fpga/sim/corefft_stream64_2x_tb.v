// corefft_stream64_2x_tb.v -- reproduce the silicon FFT-compute hang.
// Unlike corefft_stream64_tb.v (single transform, then s_tvalid drops), this TB
// mimics the HARDWARE feeder: s_tvalid is held HIGH continuously (the feeder always
// has the next transform's beat ready), so datai_valid stays asserted THROUGH the
// FFT's compute window. With the buggy gearbox (datai_valid = s_axis_tvalid) that
// keeps ticking CoreFFT's load counter during compute -> spurious load_done ->
// smStartFFT re-fires -> compute never finishes -> no output (HANG).
// With the fix (datai_valid = s_axis_tvalid & buf_ready) datai_valid drops during
// compute and the transform completes.
//
// PASS = one full transform of output beats emerges under continuous tvalid.
// FAIL/HANG = watchdog fires with no output.
`timescale 1ns/1ps
module corefft_stream64_2x_tb;
  parameter integer W = 16;
  parameter integer POINTS = 8192;                 // must be even
  parameter         IN_HEX  = "fft_vectors/random_in.hex";

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
  integer bi;                                       // beat index (feeder)
  integer o;                                        // captured output beats

  // present beat bi, cyclic over the golden transform (samples wrap every POINTS)
  wire [31:0] beat_lo = inmem[(2*bi)   % POINTS];
  wire [31:0] beat_hi = inmem[(2*bi+1) % POINTS];

  // ---- continuous feeder: s_tvalid NEVER drops (this is the key vs the old TB) ----
  always @(posedge clk or negedge ngrst) begin
    if (!ngrst) begin s_tvalid <= 1'b0; bi <= 0; end
    else begin
      s_tvalid <= 1'b1;
      s_tdata  <= {beat_hi, beat_lo};
      if (s_tvalid && s_tready) bi <= bi + 1;
    end
  end

  // ---- capture: PASS as soon as one full transform of output emerges ----
  initial begin
    $readmemh(IN_HEX, inmem);
    ngrst = 0; repeat (4) @(posedge slowclk); ngrst = 1; @(posedge clk);
    m_tready <= 1'b1; o = 0;
    // One transform = POINTS/2 output beats. Capture PAST that into transform 2 to prove
    // the transform->transform RE-ARM works (silicon hangs somewhere; sim only ever tested
    // ONE transform before). If output stalls at POINTS/2, re-arm is the silicon bug.
    while (o < (POINTS/2) + 16) begin
      @(posedge clk);
      if (m_tvalid && m_tready) begin
        o = o + 1;
        if (o == 1)              $display("  [xfrm1] FIRST output beat at t=%0t", $time);
        if (o == POINTS/2)       $display("  [xfrm1] transform 1 FULLY out (%0d beats) at t=%0t", o, $time);
        if (o == (POINTS/2)+1)   $display("  [xfrm2] transform 2 output STARTED -> RE-ARM OK at t=%0t", $time);
      end
    end
    $display("RESULT: PASS -- TWO transforms streamed back-to-back (re-arm works), %0d beats", o);
    $finish;
  end

  // ---- heartbeat: prove sim-time is actually advancing (vs a delta-loop freeze) ----
  initial forever begin
    #50_000;   // every 50 us of sim-time
    $display("  [hb] t=%0t  buf_ready=%b outp_ready=%b datai_valid=%b datao_valid=%b out_beats=%0d",
             $time, buf_ready, outp_ready, datai_valid, datao_valid, o);
  end

  // ---- hang watchdog ----
  initial begin
    #4_000_000;
    $display("RESULT: FAIL/HANG -- out_beats stalled at %0d (POINTS/2=%0d = end of transform 1 => RE-ARM HANG); outp_ready=%b buf_ready=%b datao_valid=%b datai_valid=%b feeder_bi=%0d",
             o, POINTS/2, outp_ready, buf_ready, datao_valid, datai_valid, bi);
    $finish;
  end
endmodule
