/*
 * nc_wear.h — on-ear / wear detector (handoff §5.12 item 2). Pure C11.
 *
 * Runs at 1 Hz on the IR channel: DC transmission level + band-pass
 * power (the pulsatile signature — an open clip or a table top has DC
 * but no plausible pulse).
 *
 * Worn DC band (transmissive earlobe): below the band = dark (clip in
 * a case / LED blocked), above it = open clip / direct LED-PD coupling
 * near rail. Hysteresis is on the UPPER edge via the two knobs:
 *
 *   entry band (not worn -> worn): wear_dark_thr < dc < wear_off_thr
 *   stay  band (worn -> stays)   : wear_dark_thr < dc < wear_on_thr
 *
 * dc is the ambient-subtracted IR DC (amb_subtract, default on). The
 * defaults (dark 100000, off 1900000, on 2050000) come from the first
 * V2.1 earlobe runs: worn with the AGC locked ~565k, empty clip in air
 * ~35k, empty clip on a table drives the RAW IR to the rail (caught by
 * the rail rule below, since amb-subtracted it can still read in band).
 * The original 2000/10000/20000 sat ~40x below any locked earlobe, so
 * nothing was ever "worn". The stay band is a superset of the entry
 * band, so a DC hovering between the two upper thresholds cannot
 * chatter. NOTE the task text ("worn when
 * wear_dark_thr < dc < wear_on_thr ... off path uses wear_off_thr")
 * assigns the knobs the other way around, but that reading makes the
 * stay band NARROWER than the entry band and oscillates with period
 * wear_off_s for any DC between the two thresholds — see the final
 * review note. Both thresholds are bench-tuned knobs either way.
 *
 * BP-power plausibility floor (knob-free heuristic, documented per
 * spec): 64 * ibi_thr_min^2. Rationale: ibi_thr_min is the smallest
 * band-pass amplitude the IBI detector treats as a beat; a 1 s power
 * sum of a sine at that amplitude is ~fs/2 * a^2 (>= 25 * a^2 at the
 * slowest 50 sps rate), so 64 * a^2 sits above noise yet is reached by
 * any signal the beat detector could ever trigger on.
 *
 * Transitions: worn after 2 consecutive in-band ticks; off after
 * wear_off_s consecutive out-of-band ticks (any good tick resets the
 * countdown). Attack asymmetry is deliberate: turning on is cheap,
 * turning off pauses streaming and arms offear_sleep_s.
 *
 * Two faster ways off, both from the first on-ear runs (an off-ear
 * clip kept producing junk "beats" for the whole 30 s countdown):
 *  - rail: wear_rail_s whole seconds with the RAW IR at the ADC rail.
 *    A locked earlobe sits mid-scale; an empty clip facing a surface
 *    is pure LED->PD coupling. A rail second is also never good.
 *  - jolt: the accelerometer saw the clip being handled (gate accel
 *    energy >= wear_jolt_thr) within the last wear_jolt_win_s. Then
 *    the countdown is wear_jolt_off_s. A jolt alone never drops a
 *    wearer: it only shortens a countdown the optical checks started,
 *    so head movement with a good pulse changes nothing.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define NC_WEAR_ON_TICKS 2   /* consecutive good ticks to declare worn */

typedef struct {
    bool     worn;
    uint16_t on_streak;
    uint32_t off_streak;
    uint8_t  rail_streak;    /* consecutive rail seconds while worn      */
    uint16_t jolt_age_s;     /* seconds since the last jolt (saturating) */
} nc_wear_t;

/* One second of evidence. */
typedef struct {
    int32_t dc_ir;        /* tracked IR DC in ADC counts                 */
    int64_t bp_power_1s;  /* sum of bp^2 over the last second (int64 —
                           * 24-bit samples squared at 500 sps can pass
                           * 2^53)                                       */
    bool    rail;         /* >= 90 % of the second's RAW IR samples at
                           * the ADC rail                                */
    bool    jolt;         /* accel energy reached wear_jolt_thr          */
} nc_wear_in_t;

void nc_wear_init(nc_wear_t *w);

/* Call exactly once per second while acquiring. */
void nc_wear_tick(nc_wear_t *w, const nc_wear_in_t *in);

/* Optical evidence only (no rail / jolt): nc_wear_tick with both false. */
void nc_wear_tick_1hz(nc_wear_t *w, int32_t dc_ir, int64_t bp_power_1s);

bool nc_wear_is_worn(const nc_wear_t *w);
