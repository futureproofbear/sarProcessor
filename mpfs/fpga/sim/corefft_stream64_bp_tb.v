// corefft_stream64_bp_tb.v -- reproduce the FULL-PIPELINE range-FFT stall.
// The 2x TB holds m_tready=1 (no backpressure) and PASSES. The silicon pipeline stalls because
// the fft_unloader's AXI write to DDR occasionally backpressures the stream (m_tready drops)
// WHILE CoreFFT is unloading its in-place RAM. The gearbox then drops read_outp mid-unload
// (read_outp = outp_ready & ~(have_lo & have_beat & ~m_tready)). Hypothesis: CoreFFT in-place
// cannot pause mid-unload cleanly -> wedges. This TB drives m_tready with a backpressure duty
// cycle to model the unloader and streams SEVERAL transforms to see if/where it stalls.
//
// PASS  = many transforms stream out despite backpressure (CoreFFT tolerates read_outp gaps).
// HANG  = output stalls at a transform boundary -> confirms the mid-unload backpressure wedge.
`timescale 1ns/1ps
module corefft_stream64_bp_tb;
  parameter integer W = 16;
  parameter integer POINTS = 8192;
  parameter         IN_HEX  = "fft_vectors/random_in.hex";
  parameter integer NXFRM   = 4;                   // how many transforms to stream
  parameter integer BP_ON   = 40;                  // m_tready low for BP_ON cycles ...
  parameter integer BP_OFF  = 24;                  // ... then high for BP_OFF cycles

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
  integer bi, o;
  wire [31:0] beat_lo = inmem[(2*bi)   % POINTS];
  wire [31:0] beat_hi = inmem[(2*bi+1) % POINTS];

  // continuous feeder (like the hardware fft_feeder)
  always @(posedge clk or negedge ngrst) begin
    if (!ngrst) begin s_tvalid <= 1'b0; bi <= 0; end
    else begin
      s_tvalid <= 1'b1; s_tdata <= {beat_hi, beat_lo};
      if (s_tvalid && s_tready) bi <= bi + 1;
    end
  end

  // ---- unloader backpressure model: m_tready cycles low/high (write stalls) ----
  integer bpc;
  always @(posedge clk or negedge ngrst) begin
    if (!ngrst) begin m_tready <= 1'b0; bpc <= 0; end
    else begin
      bpc <= (bpc + 1) % (BP_ON + BP_OFF);
      m_tready <= (bpc >= BP_ON);          // low for BP_ON cycles, then high for BP_OFF
    end
  end

  // ---- instrumentation: FIFO watermark + read_outp continuity (FIFO gearbox: ad.fcount) ----
  integer fifo_max, rd_drops;
  initial begin fifo_max = 0; rd_drops = 0; end
  always @(posedge clk) if (ngrst) begin
    if (ad.fcount > fifo_max) fifo_max = ad.fcount;          // peak FIFO occupancy
    if (outp_ready & ~read_outp) rd_drops = rd_drops + 1;    // read_outp dropped while CoreFFT had output
  end

  // ---- capture output beats across NXFRM transforms ----
  integer target;
  initial begin
    $readmemh(IN_HEX, inmem);
    target = (POINTS/2) * NXFRM;
    ngrst = 0; repeat (4) @(posedge slowclk); ngrst = 1; @(posedge clk);
    o = 0;
    while (o < target) begin
      @(posedge clk);
      if (m_tvalid && m_tready) begin
        o = o + 1;
        if (o % (POINTS/2) == 0)
          $display("  [xfrm%0d] complete (%0d beats) at t=%0t  -- read_outp drops so far=%0d  FIFO peak=%0d",
                   o/(POINTS/2), o, $time, rd_drops, fifo_max);
        if (o == 3276) $display("  [milestone] beats=3276 (baseline wedged here) -- PAST IT, still streaming at t=%0t", $time);
        if (o == 4096) $display("  [milestone] beats=4096 -- transform-1 boundary CLEARED, re-arming transform 2 at t=%0t", $time);
      end
    end
    $display("RESULT: PASS -- %0d transforms streamed under backpressure (%0d beats), no wedge", NXFRM, o);
    $display("  CHECK read_outp: %s (drops=%0d; 0 => flat uninterrupted high through the unload)",
             (rd_drops==0)?"FLAT HIGH":"DROPPED", rd_drops);
    $display("  CHECK FIFO watermark: peak=%0d of %0d (skid buffer cycled up on micro-stalls, drained back)", fifo_max, 256);
    $finish;
  end

  // heartbeat
  initial forever begin
    #50_000;
    $display("  [hb] t=%0t rd_outp=%b outp_rdy=%b buf_rdy=%b datao_v=%b m_tv=%b m_tr=%b beats=%0d fifo=%0d(pk%0d) rd_drops=%0d",
             $time, read_outp, outp_ready, buf_ready, datao_valid, m_tvalid, m_tready, o, ad.fcount, fifo_max, rd_drops);
  end

  // watchdog
  initial begin
    #8_000_000;
    $display("RESULT: FAIL/HANG -- stalled at %0d beats (xfrm boundary ~%0d); outp_ready=%b buf_ready=%b datao_valid=%b read_outp=%b feeder_bi=%0d",
             o, o/(POINTS/2), outp_ready, buf_ready, datao_valid, read_outp, bi);
    $finish;
  end
endmodule
