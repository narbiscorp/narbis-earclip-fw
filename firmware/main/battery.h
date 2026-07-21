/*
 * battery.h — cell voltage measurement.
 *
 * Board facts (board.h): PIN_BATT_ADC = GPIO1 = ADC1_CH1, fed from
 * VBAT_CELL through a 1 M : 1 M divider (BATT_DIVIDER_NUM = 2) with a
 * 100 nF filter cap. The divider taps the CELL side of the LM66100 load
 * switch, so readings stay honest while charging (expect the CV plateau
 * near 4.2 V). Source impedance ≈ 500 k — every reading averages
 * BATT_OVERSAMPLE raw conversions.
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"

/* ADC oneshot unit + curve-fitting calibration. If eFuse calibration
 * data is unavailable the driver logs a warning and falls back to the
 * nominal transfer function (readings then miss the ±50 mV DoD). */
esp_err_t battery_init(void);

/*
 * One averaged measurement. mv = calibrated pin mV × BATT_DIVIDER_NUM
 * (i.e. cell millivolts); raw_avg = mean raw ADC code over
 * BATT_OVERSAMPLE reads (exposed for NC_OP_TEST_BATT_RAW). Either out
 * pointer may be NULL.
 */
esp_err_t battery_read_mv(uint16_t *mv, uint16_t *raw_avg);

/* Convenience: cell mV + percent via nc_batt_pct(). NULLs allowed. */
esp_err_t battery_status(uint16_t *mv, uint8_t *pct);
