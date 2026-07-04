// corefft_behav.v -- BEHAVIORAL stand-in for the Microchip CoreFFT (in-place),
// for validating the M1 co-sim flow (corefft_fft_tb.v + fft_golden vectors +
// fft_golden.py check) in QuestaSim BEFORE the real CoreFFT IP is generated.
//
// It exposes the exact in-place CoreFFT port list and computes the DFT with
// double-precision behavioral arithmetic, then emits a conditional-block-floating-
// point output: it picks the smallest right-shift SCALE_EXP that fits the result
// in WIDTH bits, like the real core (FFT result = DATAO * 2^SCALE_EXP). It is NOT
// synthesizable and NOT bit-exact to the BFP emulator -- it is the true transform,
// so it passes the checker's TOLERANCE mode (corr/nrmse), proving the harness.
//
// When the real CoreFFT is generated in Libero, delete this file and compile the
// generated core instead -- corefft_fft_tb.v instantiates `COREFFT` unchanged.

`timescale 1ns/1ps
module COREFFT #(parameter integer WIDTH = 16, parameter integer POINTS = 64,
                 parameter integer SCALE = 0, parameter integer SCALE_EXP_ON = 1) (
  input  wire CLK, SLOWCLK, NGRST,
  input  wire signed [WIDTH-1:0] DATAI_RE, DATAI_IM,
  input  wire DATAI_VALID, READ_OUTP,
  output reg  signed [WIDTH-1:0] DATAO_RE, DATAO_IM,
  output reg  DATAO_VALID,
  output reg  BUF_READY,
  output reg  OUTP_READY,
  output reg  [4:0] SCALE_EXP
);
  real xr [0:POINTS-1], xi [0:POINTS-1];
  real yr [0:POINTS-1], yi [0:POINTS-1];
  integer nin, nout, k, n, shift;
  real ang, cc, ss, acc_r, acc_i, maxm, m, full, sc;
  localparam real PI = 3.14159265358979323846;
  reg [1:0] st;
  localparam LOAD=0, COMP=1, OUT=2;

  always @(posedge CLK or negedge NGRST) begin
    if (!NGRST) begin
      nin <= 0; nout <= 0; st <= LOAD;
      BUF_READY <= 1'b1; OUTP_READY <= 1'b0; DATAO_VALID <= 1'b0; SCALE_EXP <= 0;
    end else begin
      case (st)
        LOAD: begin
          DATAO_VALID <= 1'b0;
          if (DATAI_VALID && BUF_READY) begin
            xr[nin] = $itor(DATAI_RE); xi[nin] = $itor(DATAI_IM);
            nin = nin + 1;
            if (nin == POINTS) begin
              BUF_READY <= 1'b0;
              // ---- behavioral DFT: X[k] = sum_n x[n] exp(-j2pi kn/N) ----
              maxm = 0.0;
              for (k = 0; k < POINTS; k = k + 1) begin
                acc_r = 0.0; acc_i = 0.0;
                for (n = 0; n < POINTS; n = n + 1) begin
                  ang = -2.0*PI*((k*n) % POINTS)/POINTS;
                  cc = $cos(ang); ss = $sin(ang);
                  acc_r = acc_r + xr[n]*cc - xi[n]*ss;
                  acc_i = acc_i + xr[n]*ss + xi[n]*cc;
                end
                yr[k] = acc_r; yi[k] = acc_i;
                if (acc_r >  maxm) maxm = acc_r; if (-acc_r > maxm) maxm = -acc_r;
                if (acc_i >  maxm) maxm = acc_i; if (-acc_i > maxm) maxm = -acc_i;
              end
              // ---- conditional BFP: smallest shift so |y| fits signed WIDTH ----
              full = (1 << (WIDTH-1)) - 1; shift = 0; sc = 1.0;
              while (maxm/sc > full) begin shift = shift + 1; sc = sc*2.0; end
              for (k = 0; k < POINTS; k = k + 1) begin
                yr[k] = yr[k]/sc; yi[k] = yi[k]/sc;          // downscale by 2^shift
              end
              SCALE_EXP <= shift[4:0];
              nout <= 0; OUTP_READY <= 1'b1; st <= OUT;
            end
          end
        end
        OUT: begin
          if (READ_OUTP && nout < POINTS) begin
            DATAO_RE <= $rtoi(yr[nout] >= 0.0 ? yr[nout]+0.5 : yr[nout]-0.5);
            DATAO_IM <= $rtoi(yi[nout] >= 0.0 ? yi[nout]+0.5 : yi[nout]-0.5);
            DATAO_VALID <= 1'b1;
            nout = nout + 1;
          end else begin
            DATAO_VALID <= 1'b0;
            if (nout == POINTS) OUTP_READY <= 1'b0;
          end
        end
      endcase
    end
  end
endmodule
