// sar_fft_top.sv -- top level of the SAR 2-D FFT focuser accelerator.
//
// AXI4-Lite slave (control, see fpga/regmap.md) + AXI4 master (DDR via an MSS
// FIC). Instantiates the control FSM, the DDR read/write master, and two CoreFFT
// wrappers (range length FFT_LEN_R = N2, azimuth length FFT_LEN_A = M2). The FSM
// drives one generic FFT port; this level muxes it onto the selected core.
//
// CoreFFT lengths are fixed at synthesis: set FFT_LEN_R / FFT_LEN_A to the
// scene's padded power-of-2 dimensions and configure CoreFFT to match
// (libero/corefft_config.tcl). The host must program N2 = FFT_LEN_R and
// M2 = FFT_LEN_A or the core reports STATUS.ERR.
`default_nettype none
module sar_fft_top #(
    parameter int FFT_LEN_R = 8192,
    parameter int LOG2_R    = 13,
    parameter int FFT_LEN_A = 8192,
    parameter int LOG2_A    = 13,
    parameter int ADDR_W    = 64,
    parameter int AXI_ID_W  = 4,
    parameter int DIN_W     = 16,
    parameter int DOUT_W    = 16,
    parameter int EXPW      = 5,
    // derived
    parameter int NMAX      = (FFT_LEN_R > FFT_LEN_A) ? FFT_LEN_R : FFT_LEN_A,
    parameter int LOG2_NMAX = (LOG2_R > LOG2_A) ? LOG2_R : LOG2_A
) (
    input  wire                  clk,
    input  wire                  rstn,
    output wire                  irq,

    // ---- AXI4-Lite slave (control) ----
    input  wire [7:0]            s_axil_awaddr,
    input  wire                  s_axil_awvalid,
    output wire                  s_axil_awready,
    input  wire [31:0]           s_axil_wdata,
    input  wire [3:0]            s_axil_wstrb,
    input  wire                  s_axil_wvalid,
    output wire                  s_axil_wready,
    output wire [1:0]            s_axil_bresp,
    output wire                  s_axil_bvalid,
    input  wire                  s_axil_bready,
    input  wire [7:0]            s_axil_araddr,
    input  wire                  s_axil_arvalid,
    output wire                  s_axil_arready,
    output wire [31:0]           s_axil_rdata,
    output wire [1:0]            s_axil_rresp,
    output wire                  s_axil_rvalid,
    input  wire                  s_axil_rready,

    // ---- AXI4 master (DDR) ----
    output wire [AXI_ID_W-1:0]   m_axi_awid,
    output wire [ADDR_W-1:0]     m_axi_awaddr,
    output wire [7:0]            m_axi_awlen,
    output wire [2:0]            m_axi_awsize,
    output wire [1:0]            m_axi_awburst,
    output wire                  m_axi_awvalid,
    input  wire                  m_axi_awready,
    output wire [31:0]           m_axi_wdata,
    output wire [3:0]            m_axi_wstrb,
    output wire                  m_axi_wlast,
    output wire                  m_axi_wvalid,
    input  wire                  m_axi_wready,
    input  wire [AXI_ID_W-1:0]   m_axi_bid,
    input  wire [1:0]            m_axi_bresp,
    input  wire                  m_axi_bvalid,
    output wire                  m_axi_bready,
    output wire [AXI_ID_W-1:0]   m_axi_arid,
    output wire [ADDR_W-1:0]     m_axi_araddr,
    output wire [7:0]            m_axi_arlen,
    output wire [2:0]            m_axi_arsize,
    output wire [1:0]            m_axi_arburst,
    output wire                  m_axi_arvalid,
    input  wire                  m_axi_arready,
    input  wire [AXI_ID_W-1:0]   m_axi_rid,
    input  wire [31:0]           m_axi_rdata,
    input  wire [1:0]            m_axi_rresp,
    input  wire                  m_axi_rlast,
    input  wire                  m_axi_rvalid,
    output wire                  m_axi_rready
);
    // ---- regs <-> ctrl ----
    wire        start_pulse, soft_reset, irq_en;
    wire [15:0] r_M, r_N, r_M2, r_N2;
    wire [63:0] r_sig, r_buf, r_out;
    wire        core_busy, core_done, core_err;
    wire [EXPW-1:0] exp_r, exp_a;
    wire        rstn_i = rstn & ~soft_reset;

    axil_regs u_regs (
        .clk(clk), .rstn(rstn),
        .awaddr(s_axil_awaddr), .awvalid(s_axil_awvalid), .awready(s_axil_awready),
        .wdata(s_axil_wdata), .wstrb(s_axil_wstrb), .wvalid(s_axil_wvalid), .wready(s_axil_wready),
        .bresp(s_axil_bresp), .bvalid(s_axil_bvalid), .bready(s_axil_bready),
        .araddr(s_axil_araddr), .arvalid(s_axil_arvalid), .arready(s_axil_arready),
        .rdata(s_axil_rdata), .rresp(s_axil_rresp), .rvalid(s_axil_rvalid), .rready(s_axil_rready),
        .start_pulse(start_pulse), .soft_reset(soft_reset), .irq_en(irq_en),
        .r_M(r_M), .r_N(r_N), .r_M2(r_M2), .r_N2(r_N2),
        .r_sig_addr(r_sig), .r_buf_addr(r_buf), .r_out_addr(r_out),
        .core_busy(core_busy), .core_done(core_done), .core_err(core_err),
        .exp_r(exp_r), .exp_a(exp_a), .irq(irq)
    );

    // ---- ctrl <-> axi master ----
    wire                 ax_start, ax_we;
    wire [ADDR_W-1:0]    ax_addr;
    wire [23:0]          ax_stride;
    wire [15:0]          ax_len;
    wire                 ax_busy, ax_done;
    wire                 ax_r_valid;
    wire [31:0]          ax_r_data;
    wire                 ax_wd_ready, ax_wd_valid;
    wire [31:0]          ax_wd_data;

    axi_master_rw #(
        .ADDR_W(ADDR_W), .DATA_W(32), .ID_W(AXI_ID_W), .LEN_W(16), .STRIDE_W(24)
    ) u_axi (
        .clk(clk), .rstn(rstn_i),
        .cmd_start(ax_start), .cmd_we(ax_we), .cmd_addr(ax_addr),
        .cmd_stride(ax_stride), .cmd_len(ax_len), .cmd_busy(ax_busy), .cmd_done(ax_done),
        .m_r_valid(ax_r_valid), .m_r_data(ax_r_data), .m_r_ready(1'b1),
        .m_wd_ready(ax_wd_ready), .m_wd_valid(ax_wd_valid), .m_wd_data(ax_wd_data),
        .awid(m_axi_awid), .awaddr(m_axi_awaddr), .awlen(m_axi_awlen), .awsize(m_axi_awsize),
        .awburst(m_axi_awburst), .awvalid(m_axi_awvalid), .awready(m_axi_awready),
        .wdata(m_axi_wdata), .wstrb(m_axi_wstrb), .wlast(m_axi_wlast),
        .wvalid(m_axi_wvalid), .wready(m_axi_wready),
        .bid(m_axi_bid), .bresp(m_axi_bresp), .bvalid(m_axi_bvalid), .bready(m_axi_bready),
        .arid(m_axi_arid), .araddr(m_axi_araddr), .arlen(m_axi_arlen), .arsize(m_axi_arsize),
        .arburst(m_axi_arburst), .arvalid(m_axi_arvalid), .arready(m_axi_arready),
        .rid(m_axi_rid), .rdata(m_axi_rdata), .rresp(m_axi_rresp), .rlast(m_axi_rlast),
        .rvalid(m_axi_rvalid), .rready(m_axi_rready)
    );

    // ---- ctrl <-> FFT (generic, muxed below) ----
    wire             fft_sel, fft_start;
    wire             fft_in_valid;
    wire [DIN_W-1:0] fft_in_re, fft_in_im;
    wire             fft_in_ready;
    wire             fft_out_valid;
    wire [DOUT_W-1:0] fft_out_re, fft_out_im;
    wire             fft_out_last;
    wire [EXPW-1:0]  fft_blk_exp;
    wire             fft_busy, fft_done;

    sar_ctrl #(
        .FFT_LEN_R(FFT_LEN_R), .FFT_LEN_A(FFT_LEN_A),
        .NMAX(NMAX), .LOG2_NMAX(LOG2_NMAX),
        .DIN_W(DIN_W), .DOUT_W(DOUT_W), .EXPW(EXPW), .ADDR_W(ADDR_W)
    ) u_ctrl (
        .clk(clk), .rstn(rstn_i),
        .start(start_pulse),
        .M(r_M), .N(r_N), .M2(r_M2), .N2(r_N2),
        .sig_addr(r_sig), .buf_addr(r_buf), .out_addr(r_out),
        .busy(core_busy), .done(core_done), .err(core_err),
        .exp_r_out(exp_r), .exp_a_out(exp_a),
        .ax_start(ax_start), .ax_we(ax_we), .ax_addr(ax_addr),
        .ax_stride(ax_stride), .ax_len(ax_len), .ax_busy(ax_busy), .ax_done(ax_done),
        .ax_r_valid(ax_r_valid), .ax_r_data(ax_r_data),
        .ax_wd_ready(ax_wd_ready), .ax_wd_valid(ax_wd_valid), .ax_wd_data(ax_wd_data),
        .fft_sel(fft_sel), .fft_start(fft_start),
        .fft_in_valid(fft_in_valid), .fft_in_re(fft_in_re), .fft_in_im(fft_in_im),
        .fft_in_ready(fft_in_ready),
        .fft_out_valid(fft_out_valid), .fft_out_re(fft_out_re), .fft_out_im(fft_out_im),
        .fft_out_last(fft_out_last), .fft_blk_exp(fft_blk_exp),
        .fft_busy(fft_busy), .fft_done(fft_done)
    );

    // ---- two CoreFFT cores (range / azimuth), muxed by fft_sel ----
    wire rdy_r, ov_r, ol_r, bz_r, dn_r;  wire [DOUT_W-1:0] or_r, oi_r;  wire [EXPW-1:0] be_r;
    wire rdy_a, ov_a, ol_a, bz_a, dn_a;  wire [DOUT_W-1:0] or_a, oi_a;  wire [EXPW-1:0] be_a;

    corefft_wrap #(.FFT_LEN(FFT_LEN_R), .LOG2_LEN(LOG2_R),
                   .DIN_W(DIN_W), .DOUT_W(DOUT_W), .EXPW(EXPW)) u_fft_r (
        .clk(clk), .rstn(rstn_i),
        .start(fft_start & ~fft_sel),
        .in_valid(fft_in_valid & ~fft_sel), .in_re(fft_in_re), .in_im(fft_in_im),
        .in_ready(rdy_r),
        .out_valid(ov_r), .out_re(or_r), .out_im(oi_r), .out_last(ol_r),
        .blk_exp(be_r), .busy(bz_r), .done(dn_r)
    );
    corefft_wrap #(.FFT_LEN(FFT_LEN_A), .LOG2_LEN(LOG2_A),
                   .DIN_W(DIN_W), .DOUT_W(DOUT_W), .EXPW(EXPW)) u_fft_a (
        .clk(clk), .rstn(rstn_i),
        .start(fft_start & fft_sel),
        .in_valid(fft_in_valid & fft_sel), .in_re(fft_in_re), .in_im(fft_in_im),
        .in_ready(rdy_a),
        .out_valid(ov_a), .out_re(or_a), .out_im(oi_a), .out_last(ol_a),
        .blk_exp(be_a), .busy(bz_a), .done(dn_a)
    );

    assign fft_in_ready  = fft_sel ? rdy_a : rdy_r;
    assign fft_out_valid = fft_sel ? ov_a  : ov_r;
    assign fft_out_re    = fft_sel ? or_a  : or_r;
    assign fft_out_im    = fft_sel ? oi_a  : oi_r;
    assign fft_out_last  = fft_sel ? ol_a  : ol_r;
    assign fft_blk_exp   = fft_sel ? be_a  : be_r;
    assign fft_busy      = fft_sel ? bz_a  : bz_r;
    assign fft_done      = fft_sel ? dn_a  : dn_r;
endmodule
`default_nettype wire
