/*
 * nc_agcstep.h — AGC step compensation for the DSP + beat chain
 * (v0.3.0-35). Pure C11, integer per-sample path.
 *
 * Why: an AGC actuation (LED current, TIA gain, offset DAC) is a step in
 * the raw signal. Fed straight into the chain it rings the band-pass,
 * pumps the beat threshold up to the transient's size, and the 3 s
 * threshold decay then blinds the detector for up to ~10 s per step —
 * the user's "you lose IBIs for a really long time after auto gain
 * recalibration". The firmware knows each step exactly: LED new/old per
 * channel, TIA Rf_new/Rf_old for both.
 *
 * What, per sample, around one AGC message's AFE writes [t_lo, t_hi]:
 *  - samples up to t_lo: untouched (PASS);
 *  - samples that may straddle the switch (after t_lo, and the first one
 *    after t_hi): the chain is fed a straight-line continuation of the
 *    last two inputs instead (HOLD). Only the DSP input — the raw stream
 *    and the recordings are untouched;
 *  - the first certainly-new sample (APPLY): each channel's chain and the
 *    beat detector are rescaled by r, then the DC tracker is rebased so
 *    hp continues smoothly. Whatever the r model got wrong, and any
 *    offset-DAC step, lands in the DC instead of the pulse.
 *  A channel whose ratio is unknown (LEDs from 0, a failed write) or
 *  outside 1/8..8 is re-primed instead: no ringing, the threshold
 *  re-learns from the floor.
 *
 * Handshake (sys_task applies, dsp_task consumes; the caller transports):
 *  guard(t_lo)            right before the writes: hold anything later
 *                         until the step is described (the sample path
 *                         can run between the writes and arm);
 *  arm(t_lo, t_hi, r[2])  right after them. Arms merge: ratios multiply,
 *                         the window widens.
 * HOLD is capped at NC_AGCSTEP_HOLD_MAX samples so a lost arm can never
 * freeze the chain.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "narbis/nc_dsp.h"
#include "narbis/nc_ibi.h"
#include "narbis/nc_gate.h"

#define NC_AGCSTEP_ONE        65536
#define NC_AGCSTEP_R_MIN      (NC_AGCSTEP_ONE / 8)
#define NC_AGCSTEP_R_MAX      (NC_AGCSTEP_ONE * 8)
#define NC_AGCSTEP_HOLD_MAX   8

typedef enum {
    NC_AGCSTEP_PASS = 0,
    NC_AGCSTEP_HOLD,
    NC_AGCSTEP_APPLY,
} nc_agcstep_act_t;

typedef struct {
    bool     guard;              /* writes under way, step not described */
    uint64_t guard_t_lo_us;
    bool     armed;              /* described, waiting for its switch    */
    uint64_t t_lo_us, t_hi_us;
    int32_t  r_q16[2];           /* 0 = unscalable: re-prime             */
    bool     seen_after;         /* the first sample after t_hi was held */
    uint8_t  held;               /* consecutive HOLD samples             */
    uint8_t  hist;               /* valid history samples, 0..2          */
    int32_t  x1[2], x2[2];       /* last two chain inputs (x1 newest)    */
    int32_t  hp1[2], hp2[2];     /* last two hp outputs                  */
} nc_agcstep_t;

void nc_agcstep_init(nc_agcstep_t *st);

/* num/den as Q16, 0 when den == 0, num == 0 or outside 1/8..8. */
int32_t nc_agcstep_ratio_q16(uint32_t num, uint32_t den);
/* a * b (Q16), 0 when either is 0 or the product leaves 1/8..8. */
int32_t nc_agcstep_chain_q16(int32_t a_q16, int32_t b_q16);

void nc_agcstep_guard(nc_agcstep_t *st, uint64_t t_lo_us);
void nc_agcstep_arm(nc_agcstep_t *st, uint64_t t_lo_us, uint64_t t_hi_us,
                    const int32_t r_q16[2]);

/* One sample through the compensated front end. x[c] = this sample's chain
 * input for channel c (ambient subtracted by the caller if enabled). Runs
 * nc_dsp_run on both channels into o[]; at APPLY it also rescales `ibi`
 * (the beat detector on channel `sel`, may be NULL) and rebases `ds` (the
 * DC-step detector fed with o[sel].dc, may be NULL). */
nc_agcstep_act_t nc_agcstep_run(nc_agcstep_t *st, nc_chan_dsp_t ch[2],
                                nc_ibi_t *ibi, int sel, nc_gate_dcstep_t *ds,
                                uint64_t t_us, const int32_t x[2],
                                nc_dsp_out_t o[2]);
