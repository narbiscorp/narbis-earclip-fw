/*
 * nc_ibi.h — fixed-point inter-beat-interval (beat) detector.
 *
 * Consumes one band-passed PPG sample (nc_dsp output, selected channel)
 * per call and emits nc_ibi_rec_t beat records (proto.h wire struct).
 * Pure C11; the per-sample path is integer-only (Q8/Q15/Q31 + int64).
 * double is used exactly once, in nc_ibi_init(), to derive the threshold
 * decay coefficient — init/reconfig is slow-path per the module spec.
 *
 * Algorithm (handoff §5.8): rectified derivative -> slope-sum (SSF) over
 * ssf_win_ms -> adaptive threshold = thr_frac x decaying peak tracker
 * (floor ibi_thr_min) -> peak capture with parabolic sub-sample timing
 * (interp_en) -> refractory max(refract_ms, refract_frac_pct x the median
 * clean interval) -> IBI range check [ibi_min_ms, ibi_max_ms] ->
 * confidence 0..100.
 *
 * Knobs are read from the registry in nc_ibi_init() only (LIVE knob edits
 * take effect at the next init/reconfig). The MEASURED sample period is
 * passed in and must never be assumed nominal; nc_ibi_set_ts() tracks the
 * timing EMA cheaply (integer-only) between reconfigs.
 *
 * Emission contract (exact, relied on by t_ibi.c and the BLE layer):
 *  - A committed peak always advances the internal beat clock (t_last).
 *  - First beat ever, or a beat whose distance to the previous beat
 *    exceeds ibi_max_ms (dropout/off-ear span): the record is emitted
 *    with ibi_ms = 0 and NC_IBIF_FIRST_AFTER_GAP — the timestamp is
 *    valid, the interval is not (there is none). Plausibility context
 *    (previous IBI) resets.
 *  - Distance below ibi_min_ms (double-fire class): nothing is emitted
 *    for that beat, but t_last still advances (spec) so the next beat
 *    measures from it.
 *  - gated=true samples: detection is suppressed entirely (returns
 *    false, any in-progress capture is aborted) and the confidence
 *    context (amplitude EMA + previous IBI) resets per handoff §5.9.
 *    SSF/threshold state keeps running (gating is annotation, samples
 *    are never dropped); on gate exit the SSF ring restarts so stale
 *    artifact-era energy cannot fire the detector (<= one-window
 *    warm-up). The first record emitted after a gated span carries
 *    NC_IBIF_GATED_CTX.
 *  - Plausibility (ibi_art_en, proto 1.3): an interval more than
 *    ibi_art_tol_pct off BOTH the median of the last 5 accepted intervals
 *    AND the last accepted one is flagged NC_IBIF_ARTIFACT, confidence
 *    capped at NC_IBI_ART_CONF_MAX. It is still emitted — flagged, never
 *    dropped. Requiring both references is what keeps paced breathing
 *    clean: respiratory swings of +-25 % move beat to beat in small steps,
 *    which a lagging median alone would flag at every peak (synthetic
 *    6 /min RSA: median-only 60-78 % false flags, both-references 0 %).
 *    On the two first guided H10 recordings (2026-10-09) at 15 %: 77 % of
 *    the junk intervals flagged, 1 of 324 good rest / foot-tap beats.
 *    Four consecutive flagged intervals that agree with each other (within
 *    tol) are a genuine rate change: they become the new reference. The
 *    reference survives gating (heart rate does not jump during a few
 *    seconds of motion) and resets with FIRST_AFTER_GAP.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "narbis/proto.h"

/* SSF ring capacity: ssf_win_ms max (200 ms) at 500 sps = 100 entries,
 * headroom for measured-period undershoot. */
#define NC_IBI_SSF_MAX 128

#define NC_IBI_ART_REF      5    /* accepted intervals in the median       */
#define NC_IBI_ART_ANCHOR   4    /* agreeing flagged intervals = new rate  */
#define NC_IBI_ART_CONF_MAX 20   /* confidence cap on an ARTIFACT record   */

/* Plausibility reference (see the contract above). Exposed for tests. */
typedef struct {
    uint16_t ref[NC_IBI_ART_REF];  /* last accepted intervals, ring */
    uint8_t  n;                    /* valid entries                 */
    uint8_t  idx;                  /* next write                    */
    uint16_t last;                 /* last accepted interval        */
    uint16_t run[NC_IBI_ART_ANCHOR];  /* consecutive flagged intervals */
    uint8_t  run_n;
} nc_ibi_plaus_t;

void nc_ibi_plaus_reset(nc_ibi_plaus_t *p);

/* true = ARTIFACT. tol_pct 5..50. The first 3 intervals after a reset are
 * accepted unchecked (warm-up). */
bool nc_ibi_plaus_check(nc_ibi_plaus_t *p, uint16_t ibi_ms, uint8_t tol_pct);

typedef struct {
    /* measured timing + derived config (nc_ibi_init / nc_ibi_set_ts) */
    uint32_t ts_us;            /* measured sample period               */
    int32_t  decay_q31;        /* per-sample peak_track decay          */
    uint16_t ssf_win;          /* SSF window, samples, 1..NC_IBI_SSF_MAX */
    uint32_t refract_smp;      /* refractory, samples                  */

    /* knob snapshot taken at init (set_ts re-derives from these) */
    uint16_t k_ssf_win_ms, k_thr_tau_ms, k_refract_ms;
    uint16_t k_ibi_min_ms, k_ibi_max_ms;
    int32_t  k_thr_frac_x100, k_thr_min;
    bool     k_interp_en;
    bool     k_art_en;
    uint8_t  k_art_tol_pct;
    uint8_t  k_refract_frac_pct; /* adaptive refractory, % of the median clean IBI */
    nc_ibi_plaus_t plaus;

    /* derivative + slope-sum */
    int32_t  bp_prev;
    bool     have_bp;
    bool     was_gated;
    int32_t  ring[NC_IBI_SSF_MAX];  /* rectified derivative history    */
    uint16_t ring_idx;
    int64_t  ssf_sum;
    int64_t  ssf_prev;         /* SSF one sample ago (peak neighbor)   */

    /* adaptive threshold: Q8 running peak, decays every sample        */
    int64_t  peak_track;

    /* peak capture in progress */
    bool     in_peak;
    bool     want_next;        /* next SSF sample is the +1 neighbor   */
    int64_t  y0, ym1, yp1;     /* running max and its neighbors        */
    uint64_t t_peak_us;
    uint32_t refract_cnt;

    /* light motion since the last committed beat (nc_ibi_note_motion) */
    bool     motion_since_beat;

    /* beat/interval context */
    bool     t_last_valid;
    uint64_t t_last_us;
    uint16_t prev_ibi_ms;      /* 0 = no plausibility context          */
    int64_t  amp_ema;          /* EMA of accepted SSF peaks; 0 = reset */
    bool     gated_since_beat;
} nc_ibi_t;

/* Reads the IBI knob block and derives all rate-dependent state from the
 * measured sample period. Call again to apply knob changes (reconfig). */
void nc_ibi_init(nc_ibi_t *s, uint32_t ts_us_measured);

/* Track the timing EMA between reconfigs. Integer-only (cheap): safe to
 * call per batch. If the SSF window length crosses an integer sample
 * boundary the ring restarts (<= one window of warm-up). */
void nc_ibi_set_ts(nc_ibi_t *s, uint32_t ts_us_measured);

/* Feed one band-passed sample. Returns true when *out was filled with an
 * emitted beat record (out is untouched otherwise). t_us is the sample's
 * capture timestamp; gated is the artifact-gate flag for this sample. */
bool nc_ibi_update(nc_ibi_t *s, int32_t bp, uint64_t t_us, bool gated,
                   nc_ibi_rec_t *out);

/* Light motion this sample (nc_gate_motion): the next emitted beat carries
 * NC_IBIF_MOTION — found through motion that did not gate (proto 1.4). */
static inline void nc_ibi_note_motion(nc_ibi_t *s) { s->motion_since_beat = true; }

/* AGC step compensation (nc_agcstep.h). nc_ibi_scale: the band-passed
 * input just changed scale by r (Q16, 1/8..8) — the derivative ring, SSF,
 * threshold tracker, peak capture and amplitude EMA follow, so the
 * threshold stays where the pulse is and no beat is lost or invented.
 * nc_ibi_restart_amp: an unscalable step — amplitude state cleared (the
 * threshold re-learns from the floor within a beat or two); the beat
 * clock and the rate references are kept, so the next interval is
 * measured from the last real beat. */
void nc_ibi_scale(nc_ibi_t *s, int32_t r_q16);
void nc_ibi_restart_amp(nc_ibi_t *s);
