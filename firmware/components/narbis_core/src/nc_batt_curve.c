/*
 * nc_batt_curve.c — 1S LiPo mV -> % via piecewise-linear table.
 *
 * Knots are a generic 1S Li-po rest-voltage curve; the flat 3.68-3.92 V
 * plateau gets the fine steps, the steep tails are single segments.
 * Interpolation is integer round-to-nearest; monotonicity follows from
 * each segment being monotone with exact agreement at the knots.
 */
#include "narbis/nc_batt_curve.h"
#include <stddef.h>

static const struct { uint16_t mv; uint8_t pct; } curve[] = {
    { 3300,   0 },
    { 3500,   5 },
    { 3680,  10 },
    { 3730,  20 },
    { 3760,  30 },
    { 3790,  40 },
    { 3820,  50 },
    { 3870,  60 },
    { 3920,  70 },
    { 3980,  80 },
    { 4060,  90 },
    { 4200, 100 },
};
#define CURVE_N (sizeof curve / sizeof curve[0])

uint8_t nc_batt_pct(uint16_t mv)
{
    if (mv <= curve[0].mv) return 0;
    if (mv >= curve[CURVE_N - 1].mv) return 100;

    size_t i = 1;
    while (curve[i].mv < mv) i++;   /* mv in (curve[i-1].mv, curve[i].mv] */

    uint32_t span_mv  = (uint32_t)(curve[i].mv - curve[i - 1].mv);
    uint32_t span_pct = (uint32_t)(curve[i].pct - curve[i - 1].pct);
    /* worst case (mv-lo)*span_pct ~= 200*10 — no overflow anywhere */
    uint32_t num = (uint32_t)(mv - curve[i - 1].mv) * span_pct + span_mv / 2;
    return (uint8_t)(curve[i - 1].pct + num / span_mv);
}
