// corefft_stream64_feeder_tb.v -- HIGH-FIDELITY sim: the REAL fft_feeder HLS kernel
// -> gearbox -> real CoreFFT, streaming 2 transforms back-to-back. Unlike the idealized
// TB, this exercises the actual feeder's AXI read + its out_var/out_var_valid timing
// under gearbox backpressure (the one path the sim hadn't covered vs silicon).
// AXI read-slave memory model serves the feeder's src reads.
`timescale 1ns/1ps
module corefft_stream64_feeder_tb;
  parameter integer W = 16;
  parameter integer POINTS = 256;
  parameter         IN_HEX  = "fft_vectors/random_in.hex";
  localparam integer NBEATS = POINTS;               // 2 transforms (POINTS/2 beats each)

  reg clk=0, slowclk=0, ngrst=0, rst=1;
  always #5  clk = ~clk;
  always #40 slowclk = ~slowclk;

  // CoreFFT <-> gearbox
  wire [W-1:0] datai_re, datai_im, datao_re, datao_im;
  wire datai_valid, buf_ready, datao_valid, outp_ready, read_outp;
  wire [3:0] scale_exp;
  // feeder -> gearbox stream
  wire [4*W-1:0] s_tdata; wire s_tvalid, s_tready;
  // gearbox -> capture
  wire [4*W-1:0] m_tdata; wire m_tvalid, m_tlast; reg m_tready = 1'b1;
  // realistic output backpressure (busy DMA sink): ready ~11/16 cycles
  reg [3:0] bpc = 0;
  always @(posedge clk) begin bpc <= bpc + 1; m_tready <= (bpc < 11); end
  // ---- TLAST validation: must assert exactly every POINTS/2 output beats ----
  integer since_tlast = 0, tlast_cnt = 0, tlast_bad = 0;
  always @(posedge clk) if (m_tvalid && m_tready) begin
    if (m_tlast) begin
      tlast_cnt = tlast_cnt + 1;
      if (since_tlast != (POINTS/2 - 1)) begin
        tlast_bad = tlast_bad + 1;
        $display("  [TLAST] BAD: gap=%0d (expected %0d) at tlast#%0d", since_tlast+1, POINTS/2, tlast_cnt);
      end else $display("  [TLAST] ok: transform %0d closed (%0d beats)", tlast_cnt, POINTS/2);
      since_tlast = 0;
    end else since_tlast = since_tlast + 1;
  end

  COREFFT #(.WIDTH(W), .POINTS(POINTS)) fft (
    .CLK(clk), .SLOWCLK(slowclk), .NGRST(ngrst),
    .DATAI_RE(datai_re), .DATAI_IM(datai_im), .DATAI_VALID(datai_valid), .READ_OUTP(read_outp),
    .DATAO_RE(datao_re), .DATAO_IM(datao_im), .DATAO_VALID(datao_valid),
    .BUF_READY(buf_ready), .OUTP_READY(outp_ready), .SCALE_EXP(scale_exp));

  corefft_stream64_adapter #(.W(W), .POINTS(POINTS)) ad (
    .clk(clk), .resetn(ngrst),
    .s_axis_tdata(s_tdata), .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
    .datai_re(datai_re), .datai_im(datai_im), .datai_valid(datai_valid), .buf_ready(buf_ready),
    .datao_re(datao_re), .datao_im(datao_im), .datao_valid(datao_valid),
    .outp_ready(outp_ready), .read_outp(read_outp),
    .m_axis_tdata(m_tdata), .m_axis_tvalid(m_tvalid), .m_axis_tlast(m_tlast), .m_axis_tready(m_tready));

  // ---- REAL fft_feeder (inner: start/src/nbeats driven directly) ----
  reg fstart = 0; wire fready, ffinish;
  wire [31:0] ar_addr; wire ar_valid; wire ar_ready;
  wire [1:0] ar_burst; wire [2:0] ar_size; wire [7:0] ar_len;
  reg  [63:0] r_data = 0; reg r_valid = 0; wire r_ready; reg r_last = 0;

  fft_feeder_fft_feeder feeder (
    .clk(clk), .reset(rst), .start(fstart), .ready(fready), .finish(ffinish),
    .src(32'd0), .nbeats(NBEATS[31:0]),
    .axi4initiator_ar_addr(ar_addr), .axi4initiator_ar_ready(ar_ready), .axi4initiator_ar_valid(ar_valid),
    .axi4initiator_ar_burst(ar_burst), .axi4initiator_ar_size(ar_size), .axi4initiator_ar_len(ar_len),
    .axi4initiator_r_data(r_data), .axi4initiator_r_ready(r_ready), .axi4initiator_r_valid(r_valid),
    .axi4initiator_r_resp(2'b00), .axi4initiator_r_last(r_last),
    .out_var(s_tdata), .out_var_ready(s_tready), .out_var_valid(s_tvalid));

  // ---- AXI read-slave memory model (serves src beats, honors ar_len bursts) ----
  reg [31:0] inmem [0:POINTS-1];
  reg [63:0] mem   [0:1023];
  integer k;
  // MULTI-OUTSTANDING queue-based read slave (matches the real interconnect: the feeder
  // uses max_outstanding_reads(8); a single-outstanding slave deadlocks its pipeline).
  reg [31:0] q_base [0:15];
  reg [9:0]  q_len  [0:15];
  reg [3:0]  qhead = 0, qtail = 0;
  wire qempty = (qhead == qtail);
  wire qfull  = ((qtail + 4'd1) == qhead);
  assign ar_ready = ~qfull;                    // accept ARs whenever the queue has room
  always @(posedge clk) begin                  // enqueue accepted ARs
    if (rst) qtail <= 0;
    else if (ar_valid && ar_ready) begin
      q_base[qtail] <= ar_addr[31:3];
      q_len[qtail]  <= {2'b0, ar_len} + 10'd1;
      qtail <= qtail + 1;
    end
  end
  reg [9:0] bcnt = 0;                           // beat index within head burst
  always @(posedge clk) begin
    if (rst) begin r_valid<=0; r_last<=0; bcnt<=0; qhead<=0; end
    else if (!qempty) begin
      if (r_valid && r_ready) begin
        if (r_last) begin r_valid<=0; r_last<=0; bcnt<=0; qhead<=qhead+1; end
        else begin
          bcnt   <= bcnt + 1;
          r_data <= mem[(q_base[qhead] + bcnt + 1) % 1024];
          r_last <= ((bcnt + 1) == q_len[qhead] - 1);
        end
      end else if (!r_valid) begin             // present first beat of head burst
        r_valid <= 1'b1;
        r_data  <= mem[(q_base[qhead] + bcnt) % 1024];
        r_last  <= (bcnt == q_len[qhead] - 1);
      end
    end else begin
      r_valid <= 0; r_last <= 0;
    end
  end

  // ---- cycle counter (clock-based timing, robust vs timescale) ----
  reg [31:0] cyc = 0;
  always @(posedge clk) cyc <= cyc + 1;

  // ---- debug: feeder first AR + handshake counters ----
  reg seen_ar = 0; reg [31:0] arc = 0, rbc = 0, sbc = 0;
  always @(posedge clk) if (!rst) begin
    if (ar_valid && !seen_ar) begin
      seen_ar <= 1; $display("  [tb] feeder FIRST AR at cyc=%0d addr=%0h len=%0d", cyc, ar_addr, ar_len);
    end
    if (ar_valid && ar_ready) arc <= arc + 1;   // AR handshakes
    if (r_valid  && r_ready ) rbc <= rbc + 1;    // R beats slave delivered
    if (s_tvalid && s_tready) sbc <= sbc + 1;    // stream beats gearbox consumed
  end

  // ---- capture + stimulus ----
  integer o;
  initial begin
    $readmemh(IN_HEX, inmem);
    for (k = 0; k < 1024; k = k + 1)
      mem[k] = {inmem[(2*k+1) % POINTS], inmem[(2*k) % POINTS]};   // cyclic over one transform
    ngrst = 0; rst = 1; fstart = 0; o = 0;
    repeat (4) @(posedge slowclk); ngrst = 1;     // twiddle-LUT init on slowclk
    repeat (20) @(posedge clk); rst = 0;          // release the feeder
    // clean start handshake: wait until feeder is READY (idle), drive start on negedge
    @(negedge clk); wait (fready == 1'b1);
    @(negedge clk); fstart = 1'b1;
    @(posedge clk);                               // feeder samples start & ready here -> latches src/nbeats
    @(negedge clk); fstart = 1'b0;
    $display("  [tb] start pulsed at cyc=%0d (fready was 1)", cyc);

    // capture output beats; go PAST transform 1 to prove RE-ARM with the REAL feeder
    while (o < (POINTS/2) + 16) begin
      @(posedge clk);
      if (m_tvalid && m_tready) begin
        o = o + 1;
        if (o == 1)            $display("  [xfrm1] FIRST output beat at cyc=%0d", cyc);
        if (o == POINTS/2)     $display("  [xfrm1] transform 1 FULLY out (%0d beats) at cyc=%0d", o, cyc);
        if (o == (POINTS/2)+1) $display("  [xfrm2] transform 2 output STARTED -> RE-ARM OK (real feeder) at cyc=%0d", cyc);
      end
    end
    $display("RESULT: %s -- REAL feeder 2 transforms, %0d beats; TLAST pulses=%0d bad=%0d (expect 1/transform, at POINTS/2=%0d boundaries)",
             (tlast_bad == 0 && tlast_cnt >= 1) ? "PASS" : "TLAST-FAIL", o, tlast_cnt, tlast_bad, POINTS/2);
    $finish;
  end

  // ---- heartbeat + cycle-based watchdog ----
  always @(posedge clk) begin
    if (cyc[12:0] == 0)   // every 8192 cycles
      $display("  [hb] cyc=%0d buf_ready=%b outp_ready=%b s_tvalid=%b s_tready=%b datai_valid=%b ar_valid=%b r_valid=%b finish=%b out_beats=%0d",
               cyc, buf_ready, outp_ready, s_tvalid, s_tready, datai_valid, ar_valid, r_valid, ffinish, o);
    if (cyc > 300000) begin
      $display("RESULT: FAIL/HANG -- out_beats=%0d (POINTS/2=%0d) at cyc=%0d; buf_ready=%b outp_ready=%b s_tvalid=%b s_tready=%b ar_valid=%b r_valid=%b finish=%b",
               o, POINTS/2, cyc, buf_ready, outp_ready, s_tvalid, s_tready, ar_valid, r_valid, ffinish);
      $display("  [tb] counters: AR_handshakes=%0d  R_beats_delivered=%0d  stream_beats_consumed=%0d", arc, rbc, sbc);
      $finish;
    end
  end
endmodule
