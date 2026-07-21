/*
 * nc_batt_curve.h — LiPo 1S voltage -> state-of-charge percent.
 *
 * Piecewise-linear over a generic 1S Li-po discharge table (rest
 * voltage; the LP401515 at C/20-ish loads sits close enough — refine
 * from bench data if needed). Integer-only, round-to-nearest, clamped
 * to [0, 100] outside 3300..4200 mV. Monotonic non-decreasing by
 * construction (proved over the full sweep in t_batt_curve.c).
 */
#pragma once
#include <stdint.h>

uint8_t nc_batt_pct(uint16_t mv);
