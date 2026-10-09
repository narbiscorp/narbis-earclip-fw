/*
 * nc_agc.h — AGC policy engine (handoff §5.6). Pure decision logic:
 * the caller (main/agc_task) samples DC levels, calls nc_agc_evaluate,
 * applies the returned actions to the AFE4404, does the settle
 * blanking (agc_settle_ms) and emits the NC_EV_AGC_* events. Nothing
 * here touches hardware, allocates, or uses float.
 *
 * Locked policy ladder, per evaluation (gated by agc_en, !frozen and
 * agc_hold_ms since the last emitted action):
 *   (1) per channel: LED current ±agc_step_ma toward the target band,
 *       clamped to the per-channel [min,max] knobs and the hard LED
 *       abs-max clamps below;
 *   (2) per channel, if the LED is railed and agc_offdac_en: one
 *       offset-DAC step toward center (positive DC too high -> more
 *       negative code), range -15..15;
 *   (3) shared TIA gain, one notch in GAIN-VALUE order:
 *       DOWN  iff EITHER channel saturates with its LED at min and its
 *             offdac exhausted (or offdac disabled);
 *       UP    iff BOTH channels sit below the band bottom with LEDs at
 *             max and offdac exhausted (or disabled).
 *       At most one GAIN action per evaluate. A gain step invalidates
 *       decisions made under the old gain, so it is emitted alone plus
 *       offdac recenter (code 0) suggestions for any channel whose
 *       offdac is nonzero.
 * Consequence of the locked rule set: a channel that is above the band
 * top but NOT saturating, with LED at min and offdac at -15, has no
 * remaining move — the loop parks there until the plant changes.
 *
 * Any emitted action restarts the agc_hold_ms window (one hold window
 * covers the caller's settle blanking).
 *
 * Acquire -> lock (agc_mode = 1, the default). Every ladder action is
 * a step in the raw signal, and a step every couple of seconds is
 * poison for beat detection and for recording raw data to develop it.
 * So the ladder only runs while ACQUIRING — from stream start
 * (nc_agc_init), wear-on, unfreeze, or a re-acquire — and then LOCKS,
 * after which nothing is actuated at all:
 *   - BEATS:  agc_lock_beats consecutive clean beats (nc_agc_on_beat)
 *             with neither channel clipping and both inside the lock
 *             band (target ± agc_lock_band_pct), or
 *   - STABLE: both channels in the target deadband, nothing to do,
 *             for agc_lock_stable_ms (covers a weak pulse the detector
 *             cannot see yet — the level is right, stop touching it).
 * While LOCKED it re-acquires only when the signal is unusable for long
 * enough to be worth a step, and never sooner than agc_reacq_min_s
 * after it locked:
 *   - SAT:    either channel clipping on every evaluation for
 *             agc_reacq_sat_ms (clipped samples are lost data), or
 *   - DRIFT:  either channel outside the lock band for
 *             agc_reacq_drift_ms with NO clean beat in that time (if
 *             beats are still coming, the signal is doing its job).
 * Wear-on and unfreeze also re-acquire (the caller reports wear-on via
 * nc_agc_reacquire; freeze is an input). agc_mode = 0 keeps the legacy
 * continuous ladder, which never locks.
 *
 * Off-ear hold and put-back restore (first on-ear runs: the clip went
 * on the table, the SAT re-acquire walked the LEDs down chasing an
 * empty clip, and back on the ear it took longer than 30 s to climb
 * back and lock):
 *   - HOLD:    once this stream has locked, while the caller says
 *              off_ear nothing is evaluated — no ladder, no SAT/DRIFT
 *              runs — for at most agc_offear_hold_s (0 = never hold).
 *              The bound keeps a wrong "off" verdict (a worn clip driven
 *              to the rail) from freezing the gain for good.
 *   - RESTORE: every lock while worn records LED / gain / offdac. A
 *              re-acquire with reason WEAR then first sets them back (gain
 *              + LEDs, then the offdacs one hold window later) and only
 *              then lets the ladder run. The record survives
 *              nc_agc_restart (a new stream): the same wearer's settings
 *              beat a climb from the minimum. nc_agc_init clears it.
 *   - PROBE:   before the first lock of a stream, while off_ear, the
 *              ladder may only move the LEDs, never above
 *              agc_probe_max_ma, and nothing locks. An empty clip
 *              otherwise walks the offdac and gain all the way down
 *              chasing its own saturation (or the LEDs up to max chasing
 *              nothing). The ear then went on to a crushed gain, and it
 *              locked there on a tiny pulse with junk beats (second on-ear
 *              run, v0.3.0-27). Probing keeps the clip at settings where an
 *              ear reads in range, so the wear detector can see it; worn
 *              lifts the cap.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "narbis/nc_types.h"   /* nc_agc_state_t / nc_agc_reason_t, NC_IBIF_* */

#define NC_AGC_CH_IR    0
#define NC_AGC_CH_RED   1

/* AFE4404 ADC positive full-scale in counts (21-bit positive range).
 * Target band = agc_target_pct% of this, ± agc_deadband_pct%. */
#define NC_AGC_FS_COUNTS    (1 << 21)

/* SFH 7016 DC abs-max clamps. Values mirror board.h LED_IR_MAX_MA /
 * LED_RED_MAX_MA (narbis_core cannot include main/ headers); enforced
 * here IN ADDITION to the [min,max] knobs, whose ranges already cap at
 * the same values. */
#define NC_AGC_HARD_MAX_IR_MA   50
#define NC_AGC_HARD_MAX_RED_MA  40

/* AFE4404 offset DAC: 4-bit magnitude + polarity -> usable -15..+15. */
#define NC_AGC_OFFDAC_MIN   (-15)
#define NC_AGC_OFFDAC_MAX   15

/* Snapshot of the plant as measured over the last AGC period.
 * Index 0 = IR (LED1), 1 = RED (LED2). dc_raw is the tracked DC in ADC
 * counts; sat means the channel clipped near rail during the window
 * (its dc_raw is then a lower bound, treated as "too high"). */
typedef struct {
    int32_t  dc_raw[2];
    bool     sat[2];
    uint8_t  led_ma[2];
    uint8_t  rf_code;
    int8_t   offdac[2];
    uint64_t now_ms;
    bool     frozen;     /* NC_OP_AGC_FREEZE */
    bool     off_ear;    /* wear detector: not worn (false = worn / unknown) */
} nc_agc_in_t;

typedef enum {
    NC_AGC_NONE = 0,
    NC_AGC_LED,
    NC_AGC_OFFDAC,
    NC_AGC_GAIN
} nc_agc_kind_t;

/* One requested actuation. Only the field matching `kind` (plus `chan`
 * for LED/OFFDAC) is meaningful; the others are zeroed. Values are
 * absolute targets, not deltas. */
typedef struct {
    nc_agc_kind_t kind;
    uint8_t  chan;
    uint8_t  led_ma;
    int8_t   offdac;
    uint8_t  rf_code;
} nc_agc_act_t;

typedef struct {
    uint64_t last_step_ms;
    bool     stepped_once;   /* false -> hold gate open on first call */
    /* acquire -> lock (see header). Run flags pair with their *_since
     * stamps because 0 ms is a valid time on the host-test clock. */
    uint8_t  state;          /* nc_agc_state_t                          */
    uint8_t  reason;         /* nc_agc_reason_t of the last transition   */
    uint8_t  reacq_count;    /* LOCKED -> ACQUIRE transitions, sat 255   */
    uint8_t  clean_beats;    /* consecutive clean beats since last action */
    bool     changed;        /* transition not yet taken by the caller   */
    bool     quiet;          /* ACQUIRE: in-band idle run in progress    */
    bool     sat_run;        /* LOCKED: clipping run in progress         */
    bool     drift_run;      /* LOCKED: out-of-lock-band run in progress */
    bool     beat_seen;      /* a clean beat has ever been reported      */
    uint64_t quiet_since_ms;
    uint64_t sat_since_ms;
    uint64_t drift_since_ms;
    uint64_t locked_ms;
    uint64_t last_beat_ms;   /* time of the last clean beat              */
    /* off-ear hold (cleared by nc_agc_init / nc_agc_restart) */
    bool     locked_this_stream; /* LOCKED reached since nc_agc_init     */
    bool     offear_run;     /* off_ear on every evaluation since ...    */
    uint64_t offear_since_ms;
    uint8_t  restore_phase;  /* 0 none, 1 gain + LEDs, 2 offdacs pending */
    /* put-back restore record (kept by nc_agc_restart, cleared by
     * nc_agc_init) */
    bool     lock_cfg_valid;
    uint8_t  lock_led_ma[2];
    uint8_t  lock_rf_code;
    int8_t   lock_offdac[2];
} nc_agc_t;

/* -> ACQUIRE with reason START, and a pending change edge. Clears the
 * put-back restore record. */
void nc_agc_init(nc_agc_t *s);

/* nc_agc_init for a new stream on the same device: keeps the restore
 * record (zero-initialised storage = no record). */
void nc_agc_restart(nc_agc_t *s);

/* Returns the number of actions written to out[] (0..4: worst case is
 * one GAIN + two OFFDAC recenters, or one LED/OFFDAC per channel).
 * Always 0 while LOCKED unless this very call decided to re-acquire. */
int nc_agc_evaluate(nc_agc_t *s, const nc_agc_in_t *in, nc_agc_act_t out[4]);

/* Feed every IBI record (flags NC_IBIF_*). Clean = an interval with none
 * of GATED_CTX / AGC_SETTLING / FIRST_AFTER_GAP; anything else breaks the
 * consecutive run. INTERPOLATED (sub-sample peak refinement, set on nearly
 * every good beat) does not count against a beat. Same clock as
 * nc_agc_in_t.now_ms. */
void nc_agc_on_beat(nc_agc_t *s, uint16_t ibi_ms, uint8_t ibi_flags,
                    uint64_t now_ms);

/* Restart acquisition from outside (wear-on). LOCKED -> ACQUIRE with
 * `reason` and a change edge; already ACQUIRE -> counters restart, no
 * edge; OFF (frozen/disabled) -> ignored, the unfreeze re-acquires.
 * Reason WEAR with a lock record also schedules the RESTORE (header). */
void nc_agc_reacquire(nc_agc_t *s, uint8_t reason);

/* True once per state transition (then clears) — the caller emits
 * NC_EV_AGC_STATE {state, reason, reacq_count} on it. */
bool nc_agc_take_change(nc_agc_t *s);

/* TIA RF code <-> gain-value rank. The AFE4404 TIA_GAIN RF field is
 * NOT monotonic in code; ascending gain value order is
 *   10k 25k 50k 100k 250k 500k 1M 2M  ==  codes 5 4 3 2 1 0 6 7.
 * rank: 0 (10k) .. 7 (2M); nc_agc_rf_rank returns -1 for codes > 7. */
int nc_agc_rf_rank(uint8_t rf_code);
uint8_t nc_agc_rf_code_at(int rank);   /* rank clamped to 0..7 */
