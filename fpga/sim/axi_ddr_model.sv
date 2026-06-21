// axi_ddr_model.sv -- behavioral AXI4 slave / DDR memory for simulation.
// Single outstanding read and single outstanding write (matches axi_master_rw),
// single-beat INCR, with a few cycles of latency to exercise the handshakes.
// 32-bit data, byte addresses; word = addr>>2.
`default_nettype none
module axi_ddr_model #(
    parameter int ADDR_W    = 64,
    parameter int ID_W      = 4,
    parameter int MEM_WORDS = 1 << 16,
    parameter int LAT       = 3
) (
    input  wire                 clk,
    input  wire                 rstn,

    input  wire [ID_W-1:0]      awid,
    input  wire [ADDR_W-1:0]    awaddr,
    input  wire [7:0]           awlen,
    input  wire [2:0]           awsize,
    input  wire [1:0]           awburst,
    input  wire                 awvalid,
    output reg                  awready,
    input  wire [31:0]          wdata,
    input  wire [3:0]           wstrb,
    input  wire                 wlast,
    input  wire                 wvalid,
    output reg                  wready,
    output reg  [ID_W-1:0]      bid,
    output reg  [1:0]           bresp,
    output reg                  bvalid,
    input  wire                 bready,

    input  wire [ID_W-1:0]      arid,
    input  wire [ADDR_W-1:0]    araddr,
    input  wire [7:0]           arlen,
    input  wire [2:0]           arsize,
    input  wire [1:0]           arburst,
    input  wire                 arvalid,
    output reg                  arready,
    output reg  [ID_W-1:0]      rid,
    output reg  [31:0]          rdata,
    output reg  [1:0]           rresp,
    output reg                  rlast,
    output reg                  rvalid,
    input  wire                 rready
);
    reg [31:0] mem [0:MEM_WORDS-1];

    // ---------------- read ----------------
    typedef enum logic [1:0] {R_IDLE, R_WAIT, R_RESP} rstate_t;
    rstate_t rs;
    reg [ADDR_W-1:0] raddr;
    reg [ID_W-1:0]   rid_q;
    integer rcnt;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            rs <= R_IDLE; arready <= 1'b1; rvalid <= 1'b0; rlast <= 1'b0;
            rdata <= '0; rresp <= 2'b00; rid <= '0; rcnt <= 0;
        end else begin
            case (rs)
                R_IDLE: begin
                    arready <= 1'b1;
                    if (arvalid && arready) begin
                        raddr   <= araddr; rid_q <= arid;
                        arready <= 1'b0; rcnt <= LAT; rs <= R_WAIT;
                    end
                end
                R_WAIT: begin
                    if (rcnt == 0) begin
                        rdata <= mem[raddr[ADDR_W-1:2]];
                        rid <= rid_q; rresp <= 2'b00; rlast <= 1'b1; rvalid <= 1'b1;
                        rs <= R_RESP;
                    end else rcnt <= rcnt - 1;
                end
                R_RESP: begin
                    if (rvalid && rready) begin
                        rvalid <= 1'b0; rlast <= 1'b0; arready <= 1'b1; rs <= R_IDLE;
                    end
                end
                default: rs <= R_IDLE;
            endcase
        end
    end

    // ---------------- write ----------------
    typedef enum logic [1:0] {W_IDLE, W_DATA, W_RESP} wstate_t;
    wstate_t ws;
    reg [ADDR_W-1:0] waddr;
    reg [ID_W-1:0]   wid_q;
    integer bcnt;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            ws <= W_IDLE; awready <= 1'b1; wready <= 1'b0; bvalid <= 1'b0;
            bresp <= 2'b00; bid <= '0; bcnt <= 0;
        end else begin
            case (ws)
                W_IDLE: begin
                    awready <= 1'b1; wready <= 1'b0; bvalid <= 1'b0;
                    if (awvalid && awready) begin
                        waddr <= awaddr; wid_q <= awid;
                        awready <= 1'b0; wready <= 1'b1; ws <= W_DATA;
                    end
                end
                W_DATA: begin
                    wready <= 1'b1;
                    if (wvalid && wready) begin
                        mem[waddr[ADDR_W-1:2]] <= wdata;
                        wready <= 1'b0; bcnt <= LAT; ws <= W_RESP;
                    end
                end
                W_RESP: begin
                    if (!bvalid) begin
                        if (bcnt == 0) begin bvalid <= 1'b1; bresp <= 2'b00; bid <= wid_q; end
                        else bcnt <= bcnt - 1;
                    end else if (bvalid && bready) begin
                        bvalid <= 1'b0; awready <= 1'b1; ws <= W_IDLE;
                    end
                end
                default: ws <= W_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
