// sar_accel_top.cpp -- SAR focuser top-level dataflow (HLS TEMPLATE, UNVERIFIED).
//
// STATUS: architectural skeleton for Microchip SmartHLS. NOT synthesized or
// simulated. It shows the on-fabric datapath and the DDR/DMA boundary that the
// host (host/sar_pipeline.py + accel.py FpgaBackend) drives via the registers
// in ../regmap.md. The control args below mirror that register map.
//
// Datapath (the work offloaded from the U54 CPUs):
//
//   signal[M,N] --DMA--> [range resample (KR)] -> [window x] -> [1-D FFT] ----+
//                                                                             |
//                                                            corner-turn (BRAM tiles)
//                                                                             |
//   detected[M,N] <--DMA-- [detect |.|] <- [1-D FFT] <- [azimuth resample (KC)]+
//
// What stays on the CPU (NOT here): CPHD/metadata parse, KR/KC/window/geometry
// table prep, ECEF->UTM geocode, GeoTIFF encode, storage I/O.

#include <hls_stream.h>
#include "fft1d.cpp"

// AXI master to coherent DDR for each buffer; AXI4-Lite for control regs.
#pragma SDS data zero_copy(signal, kr, kc, tanphi, win, out)

extern void fft1d(hls::stream<cplx>&, hls::stream<cplx>&, int&);

// Linear-interpolation resampler: emit OUT samples on a uniform grid `grid`
// from input samples whose source coordinates are `src` (monotonic). One line.
static void resample_line(const cplx *inln, const float *src, const float *grid,
                          int n_in, int n_out, hls::stream<cplx> &out) {
    int j = 0;
RESAMP:
    for (int i = 0; i < n_out; i++) {
#pragma HLS pipeline II=1
        float g = grid[i];
        while (j < n_in - 2 && src[j + 1] < g) j++;
        float t = (g - src[j]) / (src[j + 1] - src[j] + 1e-12f);
        cplx a = inln[j], b = inln[j + 1], o;
        o.re = a.re + (data_t)t * (b.re - a.re);
        o.im = a.im + (data_t)t * (b.im - a.im);
        out.write((g < src[0] || g > src[n_in - 1]) ? cplx{0, 0} : o);   // zero-fill edges
    }
}

// Top level: the host sets dims + buffer addresses, then CTRL.START.
void sar_accel_top(cplx *signal, const float *kr, const float *kc,
                   const float *tanphi, const float *win, unsigned char *out,
                   int M, int N, int FFT_LEN_R, int FFT_LEN_A, int &bfp_shift) {
#pragma HLS dataflow

    static cplx after_range[/*M*FFT_LEN_R*/ NFFT * 32];   // sized at build
#pragma HLS bind_storage variable=after_range type=ram_t2p impl=uram

    // --- PASS 1: per pulse -> range resample -> window -> range FFT ---------
PASS1:
    for (int p = 0; p < M; p++) {
        hls::stream<cplx> s_rs, s_fft;
#pragma HLS stream variable=s_rs depth=64
#pragma HLS stream variable=s_fft depth=64
        // kr holds this pulse's source coords; uniform target grid is implicit.
        resample_line(&signal[p * N], &kr[p * N], /*grid*/ kr, N, FFT_LEN_R, s_rs);
        // window multiply folded into the FFT input stream
        hls::stream<cplx> s_win;
#pragma HLS stream variable=s_win depth=64
        for (int i = 0; i < FFT_LEN_R; i++) {
#pragma HLS pipeline II=1
            cplx v = s_rs.read();
            v.re *= (data_t)win[i]; v.im *= (data_t)win[i];
            s_win.write(v);
        }
        int sh;
        fft1d(s_win, s_fft, sh);
        for (int i = 0; i < FFT_LEN_R; i++) after_range[p * FFT_LEN_R + i] = s_fft.read();
    }

    // --- CORNER-TURN: transpose via tiled BRAM (range-major -> azimuth-major)
    // (omitted body: tiled read/write so DDR access stays burst-friendly)

    // --- PASS 2: per range bin -> azimuth resample -> azimuth FFT -> detect --
PASS2:
    for (int b = 0; b < FFT_LEN_R; b++) {
        hls::stream<cplx> s_rs, s_fft;
#pragma HLS stream variable=s_rs depth=64
#pragma HLS stream variable=s_fft depth=64
        // azimuth source coords for this bin: KC scaled by tan(phi) per pulse.
        resample_line(/*column b*/ &after_range[b], /*src*/ tanphi, kc, M, FFT_LEN_A, s_rs);
        int sh;
        fft1d(s_rs, s_fft, sh);
        bfp_shift = sh;
        for (int i = 0; i < FFT_LEN_A; i++) {
#pragma HLS pipeline II=1
            cplx v = s_fft.read();
            // detect: magnitude -> uint8 (host applies the dB/percentile view)
            float mag = hls::sqrtf((float)(v.re * v.re + v.im * v.im));
            int q = (int)(mag);            // scaling/AGC configured by BFP_SHIFT
            out[i * FFT_LEN_R + b] = (q > 255) ? 255 : (unsigned char)q;
        }
    }
}
