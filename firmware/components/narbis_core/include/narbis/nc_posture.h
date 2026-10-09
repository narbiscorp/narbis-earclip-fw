/*
 * nc_posture.h — "hanging from an earlobe" from the accelerometer alone.
 * Pure C11; knobs via nc_knob_get; no float.
 *
 * Used while nobody is connected (standby): the earclip cannot run the
 * optical wear detector for free, but the accelerometer can tell a clip
 * hanging from a lobe (first V2.1 on-ear runs: gravity (-0.99, -0.09,
 * -0.07) g, i.e. along -x) from one lying on a table ((0.13, -0.10,
 * -0.99) g). Ear posture starts local sensing, and the optical detector
 * then confirms or rejects it.
 *
 * Per sample: a 1-pole low-pass per axis (alpha 2^-5, ~0.64 s at 50 Hz)
 * estimates gravity; the posture test is scale-free (direction only, so
 * accel full-scale and resolution do not matter):
 *     along = sign * g[axis] > 0  and  along^2 * 10000 >= cos_pct^2 * |g|^2
 * The verdict flips only after ear_hold_ms of consecutive samples on the
 * other side, both ways, so a head turn does not drop it and a clip
 * picked up and waved about does not raise it.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int32_t  lp_q5[3];   /* gravity estimate, 5 fractional bits */
    bool     primed;
    bool     ear;        /* debounced verdict                  */
    uint32_t run;        /* consecutive samples disagreeing with `ear` */
} nc_posture_t;

void nc_posture_init(nc_posture_t *p);

/* One accelerometer sample (raw counts, any full scale) at odr_hz.
 * Returns the debounced ear-posture verdict. */
bool nc_posture_feed(nc_posture_t *p, int16_t x, int16_t y, int16_t z,
                     uint16_t odr_hz);

bool nc_posture_is_ear(const nc_posture_t *p);
