/*
 * sar_sequencer.c -- PFA pipeline sequencer (see sar_sequencer.h).
 *
 * Buffer plan (1 GiB LPDDR4, 256 MiB per complex frame so buffers are reused):
 *   SIG     0x88000000  input signal; reused as scratch-2 once resample consumes it
 *   SCRATCH 0x98000000  primary intermediate
 *   OUT     0xA8000000  final detected magnitude (uint16)
 *   tables  0xB0000000  resample idx/wq + window taper (host-loaded)
 *
 *   resample : SIG     -> SCRATCH
 *   window   : SCRATCH -> SCRATCH  (element-wise, in place)
 *   FFT range: SCRATCH -> SCRATCH  (per-row, in place: a row is read before its
 *                                   transform is written back)
 *   corner   : SCRATCH -> SIG      (transpose needs a distinct buffer; SIG is free)
 *   FFT azim : SIG     -> SIG
 *   detect   : SIG     -> OUT
 */
#include "mpfs_hal/mss_hal.h"     /* flush_l2_cache -- FIC0 is non-coherent (see below) */
#include "sar_sequencer.h"
#include "sar_kernels.h"
#include "ddr_sar_layout.h"
#include "sar_resample_coeffs.h"
#include "sar_accel_driver.h"     /* sar_job_t, sar_job_load (M, N from the host job) */
#include "sar_fft.h"              /* sar_cpu_fft -- CPU FFT (HLS K_FFT butterfly broken on silicon) */

/* Fixed geometry baked into the kernels + CoreFFT (POINTS = 8192). */
#define SAR_GRID          8192u
#define SAR_FRAME_SAMPLES ((uint64_t)SAR_GRID * SAR_GRID)        /* complex samples */
#define SAR_FRAME_BEATS   ((uint32_t)(SAR_FRAME_SAMPLES / 2u))   /* 2 samples / 64-bit beat */
#define SAR_DEFAULT_SPINS 0x40000000u

/* 32-bit address views the fabric masters drive onto FIC0 -> DDR. */
#define BUF_SIG      ((uint32_t)SAR_SIG_ADDR)
#define BUF_SCRATCH  ((uint32_t)SAR_SCRATCH_ADDR)
#define BUF_OUT      ((uint32_t)SAR_OUT_ADDR)

/* ---- FFT pass (HLS fft_kernel) --------------------------------------------
 * The CoreFFT streaming chain (fft_feeder -> gearbox -> CoreFFT -> fft_unloader) is
 * REPLACED by a single plain-AXI HLS kernel (K_FFT, control SLAVE4). fft_kernel reads
 * `src`, does a forward 8192-pt FFT per row (unconditional 1/8192 scaling, numerically
 * validated to >0.9997 correlation vs the float golden), and writes `dst` -- all via
 * one self-contained AXI4 read+write master. No native-handshake IP, no dual-master
 * streaming, no per-transform re-arm: it joins the well-behaved plain-kernel datapath,
 * sidestepping the pipeline-context stall that wedged the CoreFFT streaming path.
 * HLS_ARG0 = src, HLS_ARG1 = dst, HLS_ARG2 = nrows, HLS_START = go/done. */
#define SAR_PROG_ADDR     0xB0059100u   /* progress: [0]=pass(1/2) [1]=cur idx [2]=total [3]=heartbeat (JTAG-pollable) */
#define SAR_PROG(pass,idx,tot) do { volatile uint32_t *pg=(volatile uint32_t*)(uintptr_t)SAR_PROG_ADDR; \
    pg[0]=(uint32_t)(pass); pg[1]=(uint32_t)(idx); pg[2]=(uint32_t)(tot); pg[3]++; } while (0)

/* One FFT pass over the whole frame: transform all SAR_GRID rows of `src` (each an
 * 8192-pt row FFT) into `dst`. Runs on the MSS U54 CPU (sar_cpu_fft) -- the HLS K_FFT
 * kernel's butterfly network drops the twiddle term on silicon (identity/passthrough,
 * a SmartHLS synthesis bug; see m3 memory 2026-07-04). Everything else in the pipeline
 * is fabric; only the FFT moved to the CPU. Returns 0 = OK. */
static int fft_pass(uint32_t src, uint32_t dst, uint32_t spins)
{
    (void)spins;
    /* FIC0 non-coherent: flush so the CPU reads the kernel-written `src` from DDR (not
     * stale L2), run the FFT, then flush so the CPU-written `dst` reaches DDR for the
     * next fabric kernel's FIC0 read. */
    flush_l2_cache(1u);
    __asm volatile ("fence rw, rw");
    sar_cpu_fft((const uint32_t *)(uintptr_t)src, (uint32_t *)(uintptr_t)dst, SAR_GRID);
    __asm volatile ("fence rw, rw");
    flush_l2_cache(1u);
    return 0;
}

/* ---- on-MSS keystone resample: 2 passes, coefficients computed per line -----
 * pass 1 (range): each real pulse row of SIG (N samples) is resampled to the
 *   padded width Np and written to SCRATCH at its tan_phi-sorted row (invord[i]),
 *   so SCRATCH ends up pulse-sorted; padded rows are then zeroed.
 * transpose SCRATCH -> SIG so range bins (columns) become rows.
 * pass 2 (azimuth): each range-bin row (M sorted pulses) is resampled to Mp,
 *   leaving the resampled k-space in SCRATCH (range x cross).
 * The resample kernel runs one line per call; the MSS double-buffers the next
 * line's coefficients (bank b^1) while the current line (bank b) streams. */
static int resample_2pass(const sar_geom_t *g, uint32_t spins)
{
    float *f32 = (float *)(uintptr_t)SAR_COEF_LINE_F32;
    const int32_t *invord = (const int32_t *)(uintptr_t)SAR_INVORDER_ADDR;
    const uint32_t Np = g->Np, Mp = g->Mp;
    int b = 0;

    /* PASS 1 (range) */
    sar_coeffs_pass1(g, 0, f32, (int32_t *)(uintptr_t)SAR_COEF_IDX(0),
                                (int16_t *)(uintptr_t)SAR_COEF_WQ(0));
    for (uint32_t i = 0; i < g->M; i++) {
        SAR_PROG(1u, i, g->M);
        sar_reg_w(K_RESAMPLE, HLS_ARG0, BUF_SIG + i * g->N * 4u);            /* in  (N-wide) */
        sar_reg_w(K_RESAMPLE, HLS_ARG1, (uint32_t)SAR_COEF_IDX(b));
        sar_reg_w(K_RESAMPLE, HLS_ARG2, (uint32_t)SAR_COEF_WQ(b));
        sar_reg_w(K_RESAMPLE, HLS_ARG3, BUF_SCRATCH + (uint32_t)invord[i] * Np * 4u);
        /* FIC0 non-coherent: the idx/wq coeffs just computed by the MSS (bank b) live in
         * L2, not DDR. Flush before the kernel reads them via FIC0, else it gathers with
         * stale coeffs. (Per-line; whole-L2 flush is the only granularity this HAL has.) */
        flush_l2_cache(1u);
        sar_k_start(K_RESAMPLE);
        if (i + 1u < g->M)
            sar_coeffs_pass1(g, i + 1u, f32, (int32_t *)(uintptr_t)SAR_COEF_IDX(b ^ 1),
                                             (int16_t *)(uintptr_t)SAR_COEF_WQ(b ^ 1));
        if (!sar_k_wait(K_RESAMPLE, spins)) return 0;
        b ^= 1;
    }
    /* zero padded pulse rows (M..Mp-1) for clean FFT zero-padding (CPU clear; a
     * candidate for a fabric memset if this dominates runtime) */
    {
        volatile uint64_t *z = (volatile uint64_t *)(uintptr_t)(BUF_SCRATCH + g->M * Np * 4u);
        uint64_t words = ((uint64_t)(Mp - g->M) * Np) / 2u;   /* 2 complex int16 / 64-bit */
        for (uint64_t w = 0; w < words; w++) z[w] = 0u;
    }

    /* transpose SCRATCH(Mp x Np) -> SIG(Np x Mp) */
    sar_reg_w(K_CORNER_TURN, HLS_ARG0, BUF_SCRATCH);
    sar_reg_w(K_CORNER_TURN, HLS_ARG1, BUF_SIG);
    sar_k_start(K_CORNER_TURN);
    if (!sar_k_wait(K_CORNER_TURN, spins)) return 0;

    /* PASS 2 (azimuth) */
    sar_coeffs_pass2(g, 0, f32, (int32_t *)(uintptr_t)SAR_COEF_IDX(0),
                                (int16_t *)(uintptr_t)SAR_COEF_WQ(0));
    b = 0;
    for (uint32_t j = 0; j < Np; j++) {
        SAR_PROG(2u, j, Np);
        sar_reg_w(K_RESAMPLE, HLS_ARG0, BUF_SIG     + j * Mp * 4u);   /* in  (Mp-wide, M valid) */
        sar_reg_w(K_RESAMPLE, HLS_ARG1, (uint32_t)SAR_COEF_IDX(b));
        sar_reg_w(K_RESAMPLE, HLS_ARG2, (uint32_t)SAR_COEF_WQ(b));
        sar_reg_w(K_RESAMPLE, HLS_ARG3, BUF_SCRATCH + j * Mp * 4u);   /* out (Mp-wide) */
        flush_l2_cache(1u);   /* flush just-computed coeffs L2 -> DDR (non-coherent FIC0) */
        sar_k_start(K_RESAMPLE);
        if (j + 1u < Np)
            sar_coeffs_pass2(g, j + 1u, f32, (int32_t *)(uintptr_t)SAR_COEF_IDX(b ^ 1),
                                             (int16_t *)(uintptr_t)SAR_COEF_WQ(b ^ 1));
        if (!sar_k_wait(K_RESAMPLE, spins)) return 0;
        b ^= 1;
    }
    return 1;
}

/* Debug: arm the unloader + start the feeder, do NOT wait -> hold the streaming path live for
 * SmartDebug (see sar_sequencer.h). Range-FFT config: SCRATCH -> (stream) -> SCRATCH. */
void sar_fft_hold(void)
{
    __asm volatile ("fence rw, rw");
    sar_reg_w(K_FFT_UNLOADER, HLS_ARG1, SAR_FRAME_BEATS);
    sar_reg_w(K_FFT_UNLOADER, HLS_ARG0, BUF_SCRATCH);
    sar_k_start(K_FFT_UNLOADER);
    sar_reg_w(K_FFT_FEEDER, HLS_ARG1, SAR_FRAME_BEATS);
    sar_reg_w(K_FFT_FEEDER, HLS_ARG0, BUF_SCRATCH);
    sar_k_start(K_FFT_FEEDER);
    /* return immediately; feeder + unloader run/stall in fabric, holding the handshake */
}

/* Debug: run ONLY the range-FFT pass (SIG -> SCRATCH), skipping the ~10 min resample. Fast
 * iteration on the feeder/CoreFFT/unloader streaming path.
 * Returns fft_pass status (0 OK, 1 feeder stall, 2 unloader stall); DMADBG @0xB0059200 on a stall. */
__attribute__((used)) int sar_fft_pass_test(void)
{
    __asm volatile ("fence rw, rw");
    /* DECOUPLED src/dst (SIG -> SCRATCH) so range-FFT input and output never alias. */
    return fft_pass(BUF_SIG, BUF_SCRATCH, 0x00200000u);
}

sar_seq_status_t sar_form_image(uint32_t spin_limit)
{
    uint32_t spins = spin_limit ? spin_limit : SAR_DEFAULT_SPINS;

    /* scene dims come from the host job descriptor; padded grid is the fixed
     * size baked into the kernels + CoreFFT (square, SAR_GRID). */
    sar_job_t job;
    if (sar_job_load(&job) != SAR_OK) return SAR_SEQ_BAD_JOB;
    sar_geom_t g = {
        .M = job.M, .N = job.N, .Mp = SAR_GRID, .Np = SAR_GRID,
        .f0    = (const float *)(uintptr_t)SAR_F0_ADDR,
        .df    = (const float *)(uintptr_t)SAR_DF_ADDR,
        .pr    = (const float *)(uintptr_t)SAR_PR_ADDR,
        .tan_s = (const float *)(uintptr_t)SAR_TANS_ADDR,
        .KR    = (const float *)(uintptr_t)SAR_KRGRID_ADDR,
        .KC    = (const float *)(uintptr_t)SAR_KCGRID_ADDR,
    };

    /* Make CPU-prepared DDR (signal + geometry) visible to the fabric masters.
     * If FIC0 is used non-coherently, replace these fences with explicit
     * cache flush(before)/invalidate(after) of the touched DDR regions. */
    __asm volatile ("fence rw, rw");

    /* 1) keystone resample (2-pass, MSS-computed coeffs): -> SCRATCH */
    if (!resample_2pass(&g, spins)) return SAR_SEQ_TIMEOUT_RESAMPLE;

    /* 2) window (2-D Hamming: fabric forms hamr[j]*hamc[k]): SCRATCH -> SCRATCH */
    sar_reg_w(K_WINDOW, HLS_ARG0, BUF_SCRATCH);                  /* in  */
    sar_reg_w(K_WINDOW, HLS_ARG1, (uint32_t)SAR_HAMR_ADDR);      /* range taper [Np] */
    sar_reg_w(K_WINDOW, HLS_ARG2, (uint32_t)SAR_HAMC_ADDR);      /* cross taper [Mp] */
    sar_reg_w(K_WINDOW, HLS_ARG3, BUF_SCRATCH);                  /* out */
    sar_k_start(K_WINDOW);
    if (!sar_k_wait(K_WINDOW, spins)) return SAR_SEQ_TIMEOUT_WINDOW;

    /* 3) range FFT: SCRATCH -> SIG (DECOUPLED src/dst -- an in-place FFT feeding-and-
     *    draining the SAME DDR page stalls at transform 1 on silicon: the DMA is still
     *    flushing transform t's output while the feeder pulls transform t+1's input over
     *    the shared interconnect, so CoreFFT drops BUF_READY and the pipeline locks up.
     *    SIG is free after resample, so ping-pong SCRATCH<->SIG keeps read/write on
     *    separate 256 MB pages. VALIDATED on silicon: decoupled fft_pass streams past
     *    transform 1 (in-place stalled at idx=1). */
    { int r = fft_pass(BUF_SCRATCH, BUF_SIG, spins);
      if (r == 1) return SAR_SEQ_TIMEOUT_FFT1;          /* feeder stalled */
      if (r == 2) return SAR_SEQ_TIMEOUT_DMA; }          /* DMA S2MM stalled (range) */

    /* 4) corner-turn (transpose): SIG -> SCRATCH (range-FFT output is now in SIG) */
    sar_reg_w(K_CORNER_TURN, HLS_ARG0, BUF_SIG);
    sar_reg_w(K_CORNER_TURN, HLS_ARG1, BUF_SCRATCH);
    sar_k_start(K_CORNER_TURN);
    if (!sar_k_wait(K_CORNER_TURN, spins)) return SAR_SEQ_TIMEOUT_CORNER;

    /* 5) azimuth FFT: SCRATCH -> SIG (DECOUPLED, same ping-pong as range FFT) */
    { int r = fft_pass(BUF_SCRATCH, BUF_SIG, spins);
      if (r == 1) return SAR_SEQ_TIMEOUT_FFT2;          /* feeder stalled */
      if (r == 2) return SAR_SEQ_TIMEOUT_DMA; }          /* DMA S2MM stalled (azimuth) */

    /* 6) detect (sqrt(I^2+Q^2)): SIG -> OUT (azimuth-FFT output is in SIG) */
    sar_reg_w(K_DETECT, HLS_ARG0, BUF_SIG);
    sar_reg_w(K_DETECT, HLS_ARG1, BUF_OUT);
    sar_k_start(K_DETECT);
    if (!sar_k_wait(K_DETECT, spins)) return SAR_SEQ_TIMEOUT_DETECT;

    /* Ensure fabric writes to OUT land in DDR before the host JTAG-dumps it.
     * (Invalidate the OUT region if it was cached non-coherently.) */
    __asm volatile ("fence rw, rw");
    return SAR_SEQ_OK;
}
