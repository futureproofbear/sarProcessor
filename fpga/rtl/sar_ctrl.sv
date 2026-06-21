// sar_ctrl.sv -- SAR 2-D FFT focuser control + datapath FSM.
//
// Drives the whole storage-to-storage flow, one line at a time, mastering DDR
// through an axi_master_rw and a (muxed) corefft_wrap, with the corner-turn done
// purely in DDR addressing. See fpga/README.md for the architecture.
//
//   PASS 1 (range)   : per row r in 0..M2-1
//       clear line buffer; read SIG row r contiguous, scatter into linebuf with
//       the column ifftshift (index ^ hn) and implicit zero-pad (q>=N stays 0);
//       N2-pt FFT; store BUF row r; record exp_r[r], track max_r.
//   PASS 2 (azimuth) : per col c in 0..N2-1   (corner-turn = strided BUF read)
//       read BUF column c (one word per row, stride N2), apply the row ifftshift
//       (write index ^ hm) and renormalize each row to the common max_r exponent;
//       M2-pt FFT; store BUF column c in place; record exp_a[c], track max_a.
//   DETECT           : per output row i in 0..M2-1
//       read BUF row (i ^ hm); for each col j take linebuf[j ^ hn] (fftshift),
//       renormalize to max_a, |re,im| via isqrt; store OUT row i (uint32).
//
// ifftshift/fftshift on a power-of-2 length == roll by half == toggle the top
// index bit, so all shifts are a single XOR with hm (=M2/2) or hn (=N2/2).
//
// Line buffers are inferred block RAM (registered read), so each streaming loop
// uses ADDR -> RD(bubble) -> DATA states to absorb the 1-cycle read latency.
`default_nettype none
module sar_ctrl #(
    parameter int FFT_LEN_R = 8192,         // built range  FFT length (= N2)
    parameter int FFT_LEN_A = 8192,         // built azimuth FFT length (= M2)
    parameter int NMAX      = 8192,
    parameter int LOG2_NMAX = 13,
    parameter int DIN_W     = 16,
    parameter int DOUT_W    = 16,
    parameter int MAG_W     = 32,
    parameter int ADDR_W    = 64,
    parameter int STRIDE_W  = 24,
    parameter int LEN_W     = 16,
    parameter int EXPW      = 5
) (
    input  wire                 clk,
    input  wire                 rstn,

    // control / status
    input  wire                 start,
    input  wire [15:0]          M,
    input  wire [15:0]          N,
    input  wire [15:0]          M2,
    input  wire [15:0]          N2,
    input  wire [ADDR_W-1:0]    sig_addr,
    input  wire [ADDR_W-1:0]    buf_addr,
    input  wire [ADDR_W-1:0]    out_addr,
    output reg                  busy,
    output reg                  done,
    output reg                  err,
    output reg  [EXPW-1:0]      exp_r_out,
    output reg  [EXPW-1:0]      exp_a_out,

    // AXI master command + data streams
    output reg                  ax_start,
    output reg                  ax_we,
    output reg  [ADDR_W-1:0]    ax_addr,
    output reg  [STRIDE_W-1:0]  ax_stride,
    output reg  [LEN_W-1:0]     ax_len,
    input  wire                 ax_busy,
    input  wire                 ax_done,
    input  wire                 ax_r_valid,
    input  wire [31:0]          ax_r_data,
    input  wire                 ax_wd_ready,
    output wire                 ax_wd_valid,
    output wire [31:0]          ax_wd_data,

    // FFT (muxed in top; ctrl selects via fft_sel)
    output reg                  fft_sel,    // 0 = range (N2), 1 = azimuth (M2)
    output reg                  fft_start,
    output wire                 fft_in_valid,
    output wire [DIN_W-1:0]     fft_in_re,
    output wire [DIN_W-1:0]     fft_in_im,
    input  wire                 fft_in_ready,
    input  wire                 fft_out_valid,
    input  wire [DOUT_W-1:0]    fft_out_re,
    input  wire [DOUT_W-1:0]    fft_out_im,
    input  wire                 fft_out_last,
    input  wire [EXPW-1:0]      fft_blk_exp,
    input  wire                 fft_busy,
    input  wire                 fft_done
);
    // ---- line / scratch memories (inferred simple-dual-port BRAM) ----
    (* ram_style = "block" *) reg [31:0]      linebuf  [0:NMAX-1]; // {im,re} int16
    (* ram_style = "block" *) reg [31:0]      stagebuf [0:NMAX-1]; // FFT out / mag
    reg [EXPW-1:0]  exp_r_mem [0:NMAX-1];
    reg [EXPW-1:0]  exp_a_mem [0:NMAX-1];
    reg [LOG2_NMAX-1:0] lb_raddr, sb_raddr, er_raddr, ea_raddr;
    reg [31:0]      lb_rdata, sb_rdata;
    reg [EXPW-1:0]  er_rdata, ea_rdata;

    // ---- latched parameters ----
    reg [15:0] q_M, q_N, q_M2, q_N2, hm, hn, flen, wlen;
    reg [63:0] q_sig, q_buf, q_out;

    // ---- bookkeeping ----
    reg [EXPW-1:0] max_r, max_a;
    reg [15:0] r_idx, c_idx, i_idx;
    reg [15:0] beat, clr_idx, feed_idx, cap_idx, wr_idx, jdx;

    localparam [1:0] PASS_RANGE = 2'd0, PASS_AZ = 2'd1, PASS_DET = 2'd2;
    reg [1:0] pass;

    // detect arithmetic (registered isqrt)
    reg          sq_start;
    reg  [31:0]  sq_rad;
    wire [15:0]  sq_root;
    wire         sq_done;

    isqrt #(.RADW(32)) u_sqrt (
        .clk(clk), .rstn(rstn), .start(sq_start), .rad(sq_rad),
        .root(sq_root), .done(sq_done)
    );

    typedef enum logic [4:0] {
        S_IDLE, S_P1_CLR, S_P1_RD,
        S_FEED_ADDR, S_FEED_RD, S_FEED_DATA, S_CAP,
        S_WR_ADDR, S_WR_RD, S_WR_DATA, S_WR_WAIT,
        S_P2_RD,
        S_DET_RD, S_DET_MAG_ADDR, S_DET_MAG_RD, S_DET_MAG_CALC, S_DET_MAG_SQ,
        S_DONE
    } state_t;
    state_t state;

    assign fft_in_valid = (state == S_FEED_DATA);
    assign fft_in_re    = lb_rdata[15:0];
    assign fft_in_im    = lb_rdata[31:16];
    assign ax_wd_valid  = (state == S_WR_DATA);
    assign ax_wd_data   = sb_rdata;

    // detect: renormalize column to max_a, then |re,im|
    wire [4:0]         det_sh  = max_a - ea_rdata;
    wire signed [15:0] det_re  = $signed(lb_rdata[15:0])  >>> det_sh;
    wire signed [15:0] det_im  = $signed(lb_rdata[31:16]) >>> det_sh;
    wire [31:0]        det_re2 = det_re * det_re;
    wire [31:0]        det_im2 = det_im * det_im;

    // pass-2 capture: renormalize physical row to max_r (exp pre-read via er_rdata)
    wire [4:0]         p2_sh   = max_r - er_rdata;
    wire signed [15:0] p2_re   = $signed(ax_r_data[15:0])  >>> p2_sh;
    wire signed [15:0] p2_im   = $signed(ax_r_data[31:16]) >>> p2_sh;

    // toggled-index helper (detect source column = fftshift of output column)
    wire [15:0] scol = jdx ^ hn;

    always_ff @(posedge clk) begin
        // registered memory reads (every cycle)
        lb_rdata <= linebuf[lb_raddr];
        sb_rdata <= stagebuf[sb_raddr];
        er_rdata <= exp_r_mem[er_raddr];
        ea_rdata <= exp_a_mem[ea_raddr];

        if (!rstn) begin
            state <= S_IDLE; busy <= 1'b0; done <= 1'b0; err <= 1'b0;
            ax_start <= 1'b0; ax_we <= 1'b0; fft_start <= 1'b0; fft_sel <= 1'b0;
            sq_start <= 1'b0;
            max_r <= '0; max_a <= '0; exp_r_out <= '0; exp_a_out <= '0;
            r_idx <= '0; c_idx <= '0; i_idx <= '0;
            beat <= '0; clr_idx <= '0; feed_idx <= '0; cap_idx <= '0;
            wr_idx <= '0; jdx <= '0;
            lb_raddr <= '0; sb_raddr <= '0; er_raddr <= '0; ea_raddr <= '0;
        end else begin
            done <= 1'b0; ax_start <= 1'b0; fft_start <= 1'b0; sq_start <= 1'b0;

            case (state)
            // ---------------------------------------------------------------
            S_IDLE: begin
                busy <= 1'b0;
                if (start) begin
                    // CoreFFT lengths are fixed at build time: the host must set
                    // N2/M2 to the built FFT lengths (see fpga/README.md).
                    if (N2 != FFT_LEN_R[15:0] || M2 != FFT_LEN_A[15:0]) begin
                        err <= 1'b1; done <= 1'b1;       // reject, signal host
                    end else begin
                        q_M <= M; q_N <= N; q_M2 <= M2; q_N2 <= N2;
                        q_sig <= sig_addr; q_buf <= buf_addr; q_out <= out_addr;
                        hm <= M2 >> 1; hn <= N2 >> 1;
                        flen <= N2; fft_sel <= 1'b0; pass <= PASS_RANGE;
                        max_r <= '0; max_a <= '0;
                        r_idx <= '0; clr_idx <= '0;
                        busy <= 1'b1; err <= 1'b0;
                        state <= S_P1_CLR;
                    end
                end
            end

            // ---------------- PASS 1: range FFT over rows ------------------
            S_P1_CLR: begin
                linebuf[clr_idx] <= 32'b0;
                if (clr_idx == q_N2 - 1) begin
                    clr_idx <= '0; beat <= '0;
                    if (r_idx < q_M) begin
                        ax_addr   <= q_sig + ({{(ADDR_W-32){1'b0}}, (r_idx * q_N)} << 2);
                        ax_stride <= 24'd4; ax_len <= q_N;
                        ax_we     <= 1'b0; ax_start <= 1'b1;
                        state     <= S_P1_RD;
                    end else begin
                        feed_idx <= '0; fft_start <= 1'b1; state <= S_FEED_ADDR;
                    end
                end else clr_idx <= clr_idx + 1'b1;
            end
            S_P1_RD: begin
                if (ax_r_valid) begin
                    linebuf[beat ^ hn] <= ax_r_data;     // column ifftshift
                    beat <= beat + 1'b1;
                end
                if (ax_done) begin
                    feed_idx <= '0; fft_start <= 1'b1; state <= S_FEED_ADDR;
                end
            end

            // ---------------- shared FEED / CAP / WRITE --------------------
            S_FEED_ADDR: begin
                lb_raddr <= feed_idx[LOG2_NMAX-1:0];
                state    <= S_FEED_RD;
            end
            S_FEED_RD: state <= S_FEED_DATA;            // BRAM read latency bubble
            S_FEED_DATA: begin
                if (fft_in_ready) begin                 // fft_in_valid is state-decoded
                    if (feed_idx == flen - 1) begin
                        cap_idx <= '0; state <= S_CAP;
                    end else begin
                        feed_idx <= feed_idx + 1'b1;
                        state    <= S_FEED_ADDR;
                    end
                end
            end
            S_CAP: begin
                if (fft_out_valid) begin
                    stagebuf[cap_idx] <= {fft_out_im, fft_out_re};
                    cap_idx           <= cap_idx + 1'b1;
                    if (fft_out_last) begin
                        if (pass == PASS_RANGE) begin
                            exp_r_mem[r_idx] <= fft_blk_exp;
                            if (fft_blk_exp > max_r) max_r <= fft_blk_exp;
                            ax_addr   <= q_buf + ({{(ADDR_W-32){1'b0}}, (r_idx * q_N2)} << 2);
                            ax_stride <= 24'd4; ax_len <= q_N2;
                        end else begin // PASS_AZ : store column c in place
                            exp_a_mem[c_idx] <= fft_blk_exp;
                            if (fft_blk_exp > max_a) max_a <= fft_blk_exp;
                            ax_addr   <= q_buf + ({{(ADDR_W-16){1'b0}}, c_idx} << 2);
                            ax_stride <= (q_N2 << 2); ax_len <= q_M2;
                        end
                        wlen <= flen; wr_idx <= '0;
                        ax_we <= 1'b1; ax_start <= 1'b1;
                        state <= S_WR_ADDR;
                    end
                end
            end
            S_WR_ADDR: begin
                sb_raddr <= wr_idx[LOG2_NMAX-1:0];
                state    <= S_WR_RD;
            end
            S_WR_RD: state <= S_WR_DATA;                // BRAM read latency bubble
            S_WR_DATA: begin
                if (ax_wd_ready) begin                  // ax_wd_valid is state-decoded
                    if (wr_idx == wlen - 1) state <= S_WR_WAIT;
                    else begin
                        wr_idx <= wr_idx + 1'b1;
                        state  <= S_WR_ADDR;
                    end
                end
            end
            S_WR_WAIT: begin
                if (ax_done) begin
                    case (pass)
                    PASS_RANGE: begin
                        if (r_idx == q_M2 - 1) begin
                            pass <= PASS_AZ; fft_sel <= 1'b1; flen <= q_M2;
                            c_idx <= '0; beat <= '0;
                            ax_addr   <= q_buf;                  // column 0 base
                            ax_stride <= (q_N2 << 2); ax_len <= q_M2;
                            ax_we <= 1'b0; ax_start <= 1'b1;
                            state <= S_P2_RD;
                        end else begin
                            r_idx <= r_idx + 1'b1; clr_idx <= '0; state <= S_P1_CLR;
                        end
                    end
                    PASS_AZ: begin
                        if (c_idx == q_N2 - 1) begin
                            pass <= PASS_DET; i_idx <= '0; beat <= '0;
                            exp_a_out <= max_a; exp_r_out <= max_r;
                            ax_addr   <= q_buf + ({{(ADDR_W-32){1'b0}}, ((16'd0 ^ hm) * q_N2)} << 2);
                            ax_stride <= 24'd4; ax_len <= q_N2;
                            ax_we <= 1'b0; ax_start <= 1'b1;
                            state <= S_DET_RD;
                        end else begin
                            c_idx <= c_idx + 1'b1; beat <= '0;
                            ax_addr   <= q_buf + ({{(ADDR_W-16){1'b0}}, (c_idx + 1'b1)} << 2);
                            ax_stride <= (q_N2 << 2); ax_len <= q_M2;
                            ax_we <= 1'b0; ax_start <= 1'b1;
                            state <= S_P2_RD;
                        end
                    end
                    default: begin // PASS_DET
                        if (i_idx == q_M2 - 1) state <= S_DONE;
                        else begin
                            i_idx <= i_idx + 1'b1; beat <= '0;
                            ax_addr   <= q_buf + ({{(ADDR_W-32){1'b0}}, (((i_idx + 1'b1) ^ hm) * q_N2)} << 2);
                            ax_stride <= 24'd4; ax_len <= q_N2;
                            ax_we <= 1'b0; ax_start <= 1'b1;
                            state <= S_DET_RD;
                        end
                    end
                    endcase
                end
            end

            // ---------------- PASS 2: azimuth FFT over columns ------------
            S_P2_RD: begin
                er_raddr <= beat[LOG2_NMAX-1:0];        // pre-read exp_r[row=beat]
                if (ax_r_valid) begin
                    // er_rdata == exp_r_mem[beat] (beat stable between slow beats)
                    linebuf[beat ^ hm] <= {p2_im[15:0], p2_re[15:0]};  // row ifftshift
                    beat <= beat + 1'b1;
                end
                if (ax_done) begin
                    feed_idx <= '0; fft_start <= 1'b1; state <= S_FEED_ADDR;
                end
            end

            // ---------------- DETECT --------------------------------------
            S_DET_RD: begin
                if (ax_r_valid) begin
                    linebuf[beat] <= ax_r_data;         // BUF row (i^hm), natural order
                    beat <= beat + 1'b1;
                end
                if (ax_done) begin
                    jdx <= '0; state <= S_DET_MAG_ADDR;
                end
            end
            S_DET_MAG_ADDR: begin
                lb_raddr <= scol[LOG2_NMAX-1:0];        // fftshift column
                ea_raddr <= scol[LOG2_NMAX-1:0];
                state    <= S_DET_MAG_RD;
            end
            S_DET_MAG_RD:   state <= S_DET_MAG_CALC;     // BRAM read latency bubble
            S_DET_MAG_CALC: begin
                sq_rad   <= det_re2 + det_im2;
                sq_start <= 1'b1;
                state    <= S_DET_MAG_SQ;
            end
            S_DET_MAG_SQ: begin
                if (sq_done) begin
                    stagebuf[jdx] <= {16'b0, sq_root};   // uint32 magnitude
                    if (jdx == q_N2 - 1) begin
                        wlen <= q_N2; wr_idx <= '0;
                        ax_addr   <= q_out + ({{(ADDR_W-32){1'b0}}, (i_idx * q_N2)} << 2);
                        ax_stride <= 24'd4; ax_len <= q_N2;
                        ax_we <= 1'b1; ax_start <= 1'b1;
                        state <= S_WR_ADDR;
                    end else begin
                        jdx <= jdx + 1'b1; state <= S_DET_MAG_ADDR;
                    end
                end
            end

            S_DONE: begin
                done <= 1'b1; busy <= 1'b0;
                exp_r_out <= max_r; exp_a_out <= max_a;
                state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
