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
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

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
} nc_agc_t;

void nc_agc_init(nc_agc_t *s);

/* Returns the number of actions written to out[] (0..4: worst case is
 * one GAIN + two OFFDAC recenters, or one LED/OFFDAC per channel). */
int nc_agc_evaluate(nc_agc_t *s, const nc_agc_in_t *in, nc_agc_act_t out[4]);

/* TIA RF code <-> gain-value rank. The AFE4404 TIA_GAIN RF field is
 * NOT monotonic in code; ascending gain value order is
 *   10k 25k 50k 100k 250k 500k 1M 2M  ==  codes 5 4 3 2 1 0 6 7.
 * rank: 0 (10k) .. 7 (2M); nc_agc_rf_rank returns -1 for codes > 7. */
int nc_agc_rf_rank(uint8_t rf_code);
uint8_t nc_agc_rf_code_at(int rank);   /* rank clamped to 0..7 */
