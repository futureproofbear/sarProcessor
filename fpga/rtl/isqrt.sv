// isqrt.sv -- multicycle integer square root, root = floor(sqrt(rad)).
//
// Restoring digit-by-digit algorithm, 2 radicand bits per iteration
// (RADW/2 iterations). Used by the detect stage for |re,im| = sqrt(re^2+im^2).
// Latency = RADW/2 + 1 cycles after `start`; result held until next `start`.
`default_nettype none
module isqrt #(
    parameter int RADW = 32                 // radicand width (even)
) (
    input  wire                 clk,
    input  wire                 rstn,
    input  wire                 start,      // 1-cycle pulse with rad valid
    input  wire [RADW-1:0]      rad,
    output reg  [RADW/2-1:0]    root,       // floor(sqrt(rad))
    output reg                  done        // 1-cycle pulse when root valid
);
    localparam int ITER = RADW/2;

    reg [RADW-1:0]        radbuf;            // shifts left 2 bits/iter, MSB-first
    reg [RADW+1:0]        rem;               // running remainder
    reg [RADW/2-1:0]      q;                 // partial root
    reg [$clog2(ITER+1)-1:0] cnt;
    reg                   busy;

    // rem' = (rem<<2) | next-2-bits ; test = (q<<2)|1
    wire [RADW+1:0] rem_sh   = (rem << 2) | radbuf[RADW-1 -: 2];
    wire [RADW/2+1:0] test   = {q, 2'b01};
    wire            ge       = (rem_sh >= {{(RADW+2-(RADW/2+2)){1'b0}}, test});

    always_ff @(posedge clk) begin
        if (!rstn) begin
            busy <= 1'b0; done <= 1'b0; root <= '0;
            rem <= '0; q <= '0; cnt <= '0; radbuf <= '0;
        end else begin
            done <= 1'b0;
            if (start) begin
                radbuf <= rad;
                rem    <= '0;
                q      <= '0;
                cnt    <= ITER[$clog2(ITER+1)-1:0] - 1'b1;
                busy   <= 1'b1;
            end else if (busy) begin
                if (ge) begin
                    rem <= rem_sh - {{(RADW+2-(RADW/2+2)){1'b0}}, test};
                    q   <= {q[RADW/2-2:0], 1'b1};
                end else begin
                    rem <= rem_sh;
                    q   <= {q[RADW/2-2:0], 1'b0};
                end
                radbuf <= radbuf << 2;
                if (cnt == '0) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    root <= ge ? {q[RADW/2-2:0], 1'b1} : {q[RADW/2-2:0], 1'b0};
                end else begin
                    cnt <= cnt - 1'b1;
                end
            end
        end
    end
endmodule
`default_nettype wire
