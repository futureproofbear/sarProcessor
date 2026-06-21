// axil_regs.sv -- AXI4-Lite control/status register file for sar_fft_top.
// Register map: see fpga/regmap.md. 32-bit registers, byte addresses 0x00..0x3C.
`default_nettype none
module axil_regs #(
    parameter int ADDR_W = 8                 // enough for 0x00..0x3C
) (
    input  wire                 clk,
    input  wire                 rstn,

    // ---- AXI4-Lite slave ----
    input  wire [ADDR_W-1:0]    awaddr,
    input  wire                 awvalid,
    output reg                  awready,
    input  wire [31:0]          wdata,
    input  wire [3:0]           wstrb,
    input  wire                 wvalid,
    output reg                  wready,
    output reg  [1:0]           bresp,
    output reg                  bvalid,
    input  wire                 bready,
    input  wire [ADDR_W-1:0]    araddr,
    input  wire                 arvalid,
    output reg                  arready,
    output reg  [31:0]          rdata,
    output reg  [1:0]           rresp,
    output reg                  rvalid,
    input  wire                 rready,

    // ---- to/from core ----
    output reg                  start_pulse,  // 1-cycle when CTRL.START written 1
    output reg                  soft_reset,
    output reg                  irq_en,
    output reg  [15:0]          r_M,
    output reg  [15:0]          r_N,
    output reg  [15:0]          r_M2,
    output reg  [15:0]          r_N2,
    output reg  [63:0]          r_sig_addr,
    output reg  [63:0]          r_buf_addr,
    output reg  [63:0]          r_out_addr,
    input  wire                 core_busy,
    input  wire                 core_done,    // 1-cycle pulse at completion
    input  wire                 core_err,
    input  wire [4:0]           exp_r,
    input  wire [4:0]           exp_a,
    output wire                 irq           // level, = done_latch & irq_en
);
    localparam [31:0] DESIGN_ID = 32'h5341_5246;   // "SARF"

    reg done_latch;
    assign irq = done_latch & irq_en;

    // ---------------- write channel ----------------
    reg        aw_hs, w_hs;
    reg [ADDR_W-1:0] waddr_q;
    wire [5:0] widx = waddr_q[ADDR_W-1:2];

    always_ff @(posedge clk) begin
        if (!rstn) begin
            awready <= 1'b0; wready <= 1'b0; bvalid <= 1'b0; bresp <= 2'b00;
            aw_hs <= 1'b0; w_hs <= 1'b0; waddr_q <= '0;
            start_pulse <= 1'b0; soft_reset <= 1'b0; irq_en <= 1'b0;
            r_M <= '0; r_N <= '0; r_M2 <= '0; r_N2 <= '0;
            r_sig_addr <= '0; r_buf_addr <= '0; r_out_addr <= '0;
            done_latch <= 1'b0;
        end else begin
            start_pulse <= 1'b0;
            soft_reset  <= 1'b0;
            if (core_done) done_latch <= 1'b1;

            // latch AW
            if (awvalid && !aw_hs) begin awready <= 1'b1; waddr_q <= awaddr; aw_hs <= 1'b1; end
            else awready <= 1'b0;
            // latch W
            if (wvalid && !w_hs) begin wready <= 1'b1; w_hs <= 1'b1; end
            else wready <= 1'b0;

            // commit write when both seen
            if (aw_hs && w_hs && !bvalid) begin
                case (widx)
                    6'd0: begin                          // CTRL
                        if (wdata[0]) begin start_pulse <= 1'b1; done_latch <= 1'b0; end
                        if (wdata[1]) soft_reset <= 1'b1;
                    end
                    6'd2: irq_en     <= wdata[0];
                    6'd3: r_M        <= wdata[15:0];
                    6'd4: r_N        <= wdata[15:0];
                    6'd5: r_M2       <= wdata[15:0];
                    6'd6: r_N2       <= wdata[15:0];
                    6'd7: r_sig_addr[31:0]  <= wdata;
                    6'd8: r_sig_addr[63:32] <= wdata;
                    6'd9: r_buf_addr[31:0]  <= wdata;
                    6'd10: r_buf_addr[63:32] <= wdata;
                    6'd11: r_out_addr[31:0]  <= wdata;
                    6'd12: r_out_addr[63:32] <= wdata;
                    default: ;
                endcase
                bvalid <= 1'b1; bresp <= 2'b00;
                aw_hs <= 1'b0; w_hs <= 1'b0;
            end
            if (bvalid && bready) bvalid <= 1'b0;
        end
    end

    // ---------------- read channel ----------------
    wire [5:0] ridx = araddr[ADDR_W-1:2];
    always_ff @(posedge clk) begin
        if (!rstn) begin
            arready <= 1'b0; rvalid <= 1'b0; rresp <= 2'b00; rdata <= '0;
        end else begin
            if (arvalid && !rvalid) begin
                arready <= 1'b1;
                rvalid  <= 1'b1;
                rresp   <= 2'b00;
                case (ridx)
                    6'd0: rdata <= {31'b0, core_busy};
                    6'd1: rdata <= {28'b0, irq, core_err, core_busy, done_latch};
                    6'd2: rdata <= {31'b0, irq_en};
                    6'd3: rdata <= {16'b0, r_M};
                    6'd4: rdata <= {16'b0, r_N};
                    6'd5: rdata <= {16'b0, r_M2};
                    6'd6: rdata <= {16'b0, r_N2};
                    6'd7: rdata <= r_sig_addr[31:0];
                    6'd8: rdata <= r_sig_addr[63:32];
                    6'd9: rdata <= r_buf_addr[31:0];
                    6'd10: rdata <= r_buf_addr[63:32];
                    6'd11: rdata <= r_out_addr[31:0];
                    6'd12: rdata <= r_out_addr[63:32];
                    6'd13: rdata <= {27'b0, exp_r};
                    6'd14: rdata <= {27'b0, exp_a};
                    6'd15: rdata <= DESIGN_ID;
                    default: rdata <= 32'b0;
                endcase
            end else begin
                arready <= 1'b0;
                if (rvalid && rready) rvalid <= 1'b0;
            end
        end
    end
endmodule
`default_nettype wire
