// axi_master_rw.sv -- minimal AXI4 read/write master, single outstanding,
// single-beat transactions on a strided address pattern.
//
// One command moves `len` words to/from DDR at addresses
//   cmd_addr, cmd_addr+stride, cmd_addr+2*stride, ...  (stride in BYTES).
// Single-beat (AxLEN=0) keeps the master simple and correct for the arbitrary
// strides the corner-turn needs; the design trades throughput for low on-chip
// memory (one line at a time), as specified. Contiguous lines use stride = the
// word size. Burst grouping is a later optimization (see fpga/README.md).
//
// Read : words come out on the m_r_* stream in issue order.
// Write: words are pulled from the m_wd_* stream in issue order.
`default_nettype none
module axi_master_rw #(
    parameter int ADDR_W   = 64,
    parameter int DATA_W   = 32,
    parameter int ID_W     = 4,
    parameter int LEN_W    = 16,            // max words per command
    parameter int STRIDE_W = 24             // max stride in bytes
) (
    input  wire                 clk,
    input  wire                 rstn,

    // command interface (from sar_ctrl)
    input  wire                 cmd_start,  // 1-cycle pulse
    input  wire                 cmd_we,     // 1 = write, 0 = read
    input  wire [ADDR_W-1:0]    cmd_addr,
    input  wire [STRIDE_W-1:0]  cmd_stride, // bytes between consecutive words
    input  wire [LEN_W-1:0]     cmd_len,    // number of words (>= 1)
    output reg                  cmd_busy,
    output reg                  cmd_done,   // 1-cycle pulse at completion

    // read data out (valid/ready)
    output reg                  m_r_valid,
    output reg  [DATA_W-1:0]    m_r_data,
    input  wire                 m_r_ready,

    // write data in (valid/ready)
    output reg                  m_wd_ready,
    input  wire                 m_wd_valid,
    input  wire [DATA_W-1:0]    m_wd_data,

    // ---- AXI4 master ----
    output reg  [ID_W-1:0]      awid,
    output reg  [ADDR_W-1:0]    awaddr,
    output wire [7:0]           awlen,
    output wire [2:0]           awsize,
    output wire [1:0]           awburst,
    output reg                  awvalid,
    input  wire                 awready,

    output reg  [DATA_W-1:0]    wdata,
    output wire [DATA_W/8-1:0]  wstrb,
    output wire                 wlast,
    output reg                  wvalid,
    input  wire                 wready,

    input  wire [ID_W-1:0]      bid,
    input  wire [1:0]           bresp,
    input  wire                 bvalid,
    output reg                  bready,

    output reg  [ID_W-1:0]      arid,
    output reg  [ADDR_W-1:0]    araddr,
    output wire [7:0]           arlen,
    output wire [2:0]           arsize,
    output wire [1:0]           arburst,
    output reg                  arvalid,
    input  wire                 arready,

    input  wire [ID_W-1:0]      rid,
    input  wire [DATA_W-1:0]    rdata,
    input  wire [1:0]           rresp,
    input  wire                 rlast,
    input  wire                 rvalid,
    output reg                  rready
);
    localparam [2:0] AXSIZE = $clog2(DATA_W/8);   // bytes-per-beat encoding

    assign awlen   = 8'd0;          // single beat
    assign arlen   = 8'd0;
    assign awsize  = AXSIZE;
    assign arsize  = AXSIZE;
    assign awburst = 2'b01;         // INCR
    assign arburst = 2'b01;
    assign wstrb   = {(DATA_W/8){1'b1}};
    assign wlast   = 1'b1;

    typedef enum logic [2:0] {
        S_IDLE, S_RBEAT, S_WGET, S_WADDR, S_WRESP
    } state_t;
    state_t state;

    reg [ADDR_W-1:0]    cur;
    reg [STRIDE_W-1:0]  stride;
    reg [LEN_W-1:0]     remaining;
    reg                 aw_sent, w_sent;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            state <= S_IDLE; cmd_busy <= 1'b0; cmd_done <= 1'b0;
            awvalid <= 1'b0; wvalid <= 1'b0; bready <= 1'b0;
            arvalid <= 1'b0; rready <= 1'b0;
            m_r_valid <= 1'b0; m_wd_ready <= 1'b0;
            awid <= '0; arid <= '0; cur <= '0; stride <= '0; remaining <= '0;
            aw_sent <= 1'b0; w_sent <= 1'b0; awaddr <= '0; araddr <= '0;
            wdata <= '0; m_r_data <= '0;
        end else begin
            cmd_done <= 1'b0;
            case (state)
                S_IDLE: begin
                    m_r_valid <= 1'b0; m_wd_ready <= 1'b0; rready <= 1'b0;
                    if (cmd_start) begin
                        cur       <= cmd_addr;
                        stride    <= cmd_stride;
                        remaining <= cmd_len;
                        cmd_busy  <= 1'b1;
                        if (cmd_we) state <= S_WGET;
                        else begin araddr <= cmd_addr; arvalid <= 1'b1; state <= S_RBEAT; end
                    end else begin
                        cmd_busy <= 1'b0;
                    end
                end

                // ---------------- READ ----------------
                // Single outstanding read; m_r_valid is a 1-cycle pulse per word
                // (sar_ctrl captures it straight into a line-buffer BRAM, so it
                // is always ready -- rready is held high internally).
                S_RBEAT: begin
                    m_r_valid <= 1'b0;
                    rready    <= 1'b1;
                    if (arvalid && arready) arvalid <= 1'b0;
                    if (rvalid && rready) begin
                        m_r_data  <= rdata;
                        m_r_valid <= 1'b1;
                        remaining <= remaining - 1'b1;
                        cur       <= cur + stride;
                        if (remaining == 1) begin
                            state    <= S_IDLE;
                            cmd_busy <= 1'b0;
                            cmd_done <= 1'b1;
                            rready   <= 1'b0;
                        end else begin
                            araddr  <= cur + stride;
                            arvalid <= 1'b1;
                        end
                    end
                end

                // ---------------- WRITE ---------------
                S_WGET: begin
                    m_wd_ready <= 1'b1;
                    if (m_wd_valid && m_wd_ready) begin
                        wdata      <= m_wd_data;
                        m_wd_ready <= 1'b0;
                        awaddr     <= cur;
                        awvalid    <= 1'b1;
                        wvalid     <= 1'b1;
                        aw_sent    <= 1'b0;
                        w_sent     <= 1'b0;
                        state      <= S_WADDR;
                    end
                end
                S_WADDR: begin
                    if (awvalid && awready) begin awvalid <= 1'b0; aw_sent <= 1'b1; end
                    if (wvalid  && wready ) begin wvalid  <= 1'b0; w_sent  <= 1'b1; end
                    if ((aw_sent || (awvalid && awready)) &&
                        (w_sent  || (wvalid  && wready ))) begin
                        bready <= 1'b1;
                        state  <= S_WRESP;
                    end
                end
                S_WRESP: begin
                    if (bvalid && bready) begin
                        bready    <= 1'b0;
                        remaining <= remaining - 1'b1;
                        cur       <= cur + stride;
                        if (remaining == 1) begin
                            state    <= S_IDLE;
                            cmd_busy <= 1'b0;
                            cmd_done <= 1'b1;
                        end else begin
                            state <= S_WGET;
                        end
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
