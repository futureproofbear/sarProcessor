// corefft_model.sv -- BEHAVIORAL CoreFFT stand-in (simulation only).
//
// Implements the corefft_wrap handshake contract with a real-arithmetic forward
// DFT plus per-frame block-floating-point scaling, matching the Python reference
// fpga/scripts/gen_vectors.py::corefft_bfp exactly in intent (floor truncation).
// NOT synthesizable -- it uses `real` and a DFT loop. The real Microchip CoreFFT
// replaces it for synthesis; on hardware the result is not bit-identical (the
// core's internal rounding differs), so the testbench checks with a tolerance.
`default_nettype none
module corefft_model #(
    parameter int FFT_LEN  = 8192,
    parameter int LOG2_LEN = 13,
    parameter int DIN_W    = 16,
    parameter int DOUT_W   = 16,
    parameter int EXPW     = 5
) (
    input  wire                 clk,
    input  wire                 rstn,
    input  wire                 start,
    input  wire                 in_valid,
    input  wire [DIN_W-1:0]     in_re,
    input  wire [DIN_W-1:0]     in_im,
    output reg                  in_ready,
    output reg                  out_valid,
    output reg  [DOUT_W-1:0]    out_re,
    output reg  [DOUT_W-1:0]    out_im,
    output reg                  out_last,
    output reg  [EXPW-1:0]      blk_exp,
    output reg                  busy,
    output reg                  done
);
    localparam real PI   = 3.14159265358979323846;
    localparam real FULL = (2.0 ** (DOUT_W-1)) - 1.0;

    real xr [0:FFT_LEN-1];
    real xi [0:FFT_LEN-1];
    integer yr [0:FFT_LEN-1];
    integer yi [0:FFT_LEN-1];

    typedef enum logic [1:0] {S_IDLE, S_LOAD, S_OUT} state_t;
    state_t state;
    integer ld, od;

    // saturating cast of an integer to signed DOUT_W
    function automatic [DOUT_W-1:0] sat(input integer v);
        integer hi, lo;
        begin
            hi = (1 <<< (DOUT_W-1)) - 1;
            lo = -(1 <<< (DOUT_W-1));
            if (v > hi) v = hi;
            if (v < lo) v = lo;
            sat = v[DOUT_W-1:0];
        end
    endfunction

    task automatic compute_dft;
        integer k, n, blk;
        real ang, sr, si, ar, ai, maxabs, scale;
        begin
            maxabs = 0.0;
            // forward DFT, unscaled: X[k] = sum_n x[n] exp(-j2pi kn/N)
            for (k = 0; k < FFT_LEN; k = k + 1) begin
                sr = 0.0; si = 0.0;
                for (n = 0; n < FFT_LEN; n = n + 1) begin
                    ang = -2.0 * PI * k * n / FFT_LEN;
                    ar  = $cos(ang); ai = $sin(ang);
                    sr  = sr + xr[n]*ar - xi[n]*ai;
                    si  = si + xr[n]*ai + xi[n]*ar;
                end
                // stash in y as scaled-by-1 reals via integer holders later;
                // first pass just track max magnitude component
                if (sr < 0.0) begin if (-sr > maxabs) maxabs = -sr; end
                else          begin if ( sr > maxabs) maxabs =  sr; end
                if (si < 0.0) begin if (-si > maxabs) maxabs = -si; end
                else          begin if ( si > maxabs) maxabs =  si; end
            end
            // choose block exponent: smallest blk>=0 with maxabs/2^blk <= FULL
            blk = 0;
            if (maxabs > 0.0)
                while (maxabs > FULL * (2.0 ** blk)) blk = blk + 1;
            blk_exp = blk[EXPW-1:0];
            scale   = 2.0 ** blk;
            // second DFT pass with scaling + floor truncation
            for (k = 0; k < FFT_LEN; k = k + 1) begin
                sr = 0.0; si = 0.0;
                for (n = 0; n < FFT_LEN; n = n + 1) begin
                    ang = -2.0 * PI * k * n / FFT_LEN;
                    ar  = $cos(ang); ai = $sin(ang);
                    sr  = sr + xr[n]*ar - xi[n]*ai;
                    si  = si + xr[n]*ai + xi[n]*ar;
                end
                yr[k] = $rtoi($floor(sr/scale));
                yi[k] = $rtoi($floor(si/scale));
            end
        end
    endtask

    always_ff @(posedge clk) begin
        if (!rstn) begin
            state <= S_IDLE; in_ready <= 1'b0; out_valid <= 1'b0; out_last <= 1'b0;
            busy <= 1'b0; done <= 1'b0; ld <= 0; od <= 0; blk_exp <= '0;
            out_re <= '0; out_im <= '0;
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    out_valid <= 1'b0; out_last <= 1'b0;
                    if (start) begin
                        ld <= 0; in_ready <= 1'b1; busy <= 1'b1; state <= S_LOAD;
                    end else busy <= 1'b0;
                end
                S_LOAD: begin
                    if (in_valid && in_ready) begin
                        // blocking so the last sample is in xr/xi before compute_dft
                        xr[ld] = $itor($signed(in_re));
                        xi[ld] = $itor($signed(in_im));
                        if (ld == FFT_LEN-1) begin
                            in_ready <= 1'b0;
                            compute_dft();          // fills yr/yi/blk_exp
                            od <= 0;
                            out_valid <= 1'b1;
                            out_re <= sat(yr[0]);
                            out_im <= sat(yi[0]);
                            out_last <= (FFT_LEN == 1);
                            state <= S_OUT;
                        end else ld <= ld + 1;
                    end
                end
                S_OUT: begin
                    if (out_valid) begin
                        if (od == FFT_LEN-1) begin
                            out_valid <= 1'b0; out_last <= 1'b0; busy <= 1'b0;
                            done <= 1'b1; state <= S_IDLE;
                        end else begin
                            out_re   <= sat(yr[od+1]);
                            out_im   <= sat(yi[od+1]);
                            out_last <= (od+1 == FFT_LEN-1);
                            od <= od + 1;
                        end
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
