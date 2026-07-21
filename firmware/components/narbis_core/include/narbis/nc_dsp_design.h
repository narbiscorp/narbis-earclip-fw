/*
 * nc_dsp_design.h — runtime filter design for non-default DSP corner
 * knobs (hp_fc_x100 / bp_lo_x100 / bp_hi_x100).
 *
 * SLOW PATH ONLY: double-precision math is explicitly allowed here
 * (init/reconfig, never per-sample). Mirrors tools/goldens/
 * gen_dsp_coeffs.py operation-for-operation so that the default
 * corners reproduce the committed golden tables (host-tested to
 * <= 1 LSB per Q30 coefficient).
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "narbis/nc_dsp.h"

/* 4th-order Butterworth band-pass -> 2 Q30 SOS sections, gain split
 * sqrt(k) per section, section order matching zpk2sos 'nearest'
 * (sec0 = far-from-unit pole pair + zeros{-1,-1}, sec1 = near-unit
 * pair + zeros{+1,+1}). Returns false (out untouched) if the request
 * is degenerate after clamping or a quantized pole radius >= 0.9995 —
 * caller falls back to the golden defaults. Corners in centi-Hz. */
bool nc_dsp_design_bp(uint16_t fs_sps, uint16_t lo_x100, uint16_t hi_x100,
                      nc_bq_coeff_t out[2]);

/* DC-tracker alpha for an arbitrary hp corner:
 * round(2^31 * (1 - exp(-2*pi*fc/fs))). Clamped to >= 1. */
int32_t nc_dsp_design_alpha_q31(uint16_t fs_sps, uint16_t fc_x100);
