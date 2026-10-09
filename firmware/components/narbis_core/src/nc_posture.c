/*
 * nc_posture.c — ear posture from gravity direction. See nc_posture.h.
 */
#include <string.h>
#include "narbis/nc_posture.h"
#include "narbis/nc_knobs.h"

void nc_posture_init(nc_posture_t *p)
{
    memset(p, 0, sizeof(*p));
}

bool nc_posture_is_ear(const nc_posture_t *p)
{
    return p->ear;
}

bool nc_posture_feed(nc_posture_t *p, int16_t x, int16_t y, int16_t z,
                     uint16_t odr_hz)
{
    const int32_t s[3] = { x, y, z };
    if (!p->primed) {
        for (int i = 0; i < 3; i++) p->lp_q5[i] = s[i] * 32;
        p->primed = true;
    }
    /* >> on negatives is arithmetic (floor), as everywhere in the core. */
    for (int i = 0; i < 3; i++) {
        p->lp_q5[i] += (s[i] * 32 - p->lp_q5[i]) >> 5;
    }

    const int code = nc_knob_get(KNOB_EAR_AXIS);
    const int axis = code / 2;
    const int32_t g_axis = p->lp_q5[axis] >> 5;
    const int64_t along = (code & 1) ? -(int64_t)g_axis : (int64_t)g_axis;
    int64_t mag2 = 0;
    for (int i = 0; i < 3; i++) {
        const int64_t g = p->lp_q5[i] >> 5;
        mag2 += g * g;
    }
    const int64_t cos_pct = nc_knob_get(KNOB_EAR_COS_PCT);
    /* |g| <= ~2^15 * sqrt(3): along^2 * 10^4 < 2^45, no overflow. */
    const bool ok = along > 0 && mag2 > 0 &&
                    along * along * 10000 >= cos_pct * cos_pct * mag2;

    if (ok == p->ear) {
        p->run = 0;
        return p->ear;
    }
    const uint32_t need = ((uint32_t)nc_knob_get(KNOB_EAR_HOLD_MS) * odr_hz) / 1000u;
    if (++p->run >= (need > 0 ? need : 1)) {
        p->ear = ok;
        p->run = 0;
    }
    return p->ear;
}
