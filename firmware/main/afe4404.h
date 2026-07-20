/*
 * afe4404.h — AFE4404 PPG analog front end driver (I2C 0x58, RESETZ GPIO21).
 *
 * Threading: every entry point takes a driver-internal mutex, so calls may
 * come from any task. Nothing here is ISR-safe — the ADC_RDY ISR must only
 * timestamp + notify a task, which then calls afe4404_read_frame().
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "narbis/nc_types.h"

/* Full bring-up: rev-proof RESETZ sequence (drive high, >=10 ms, 25-50 us
 * low pulse, high, >=2 ms), shared-I2C attach, static config (ENSEPGAIN=0,
 * TIA RF=100k CF=5pF, LED currents 0, offset DACs 0), then apply_rate().
 * Works on V2.1 (RESETZ pull-up) and V2.2 (pull-down: cold boot is hardware
 * PWDN; the initial high + 10 ms wait covers the tACTIVE recovery).
 * Also the recovery path after afe4404_powerdown_hw() — hardware power-down
 * loses all registers. */
esp_err_t afe4404_init(nc_rate_t rate);

/* Rate switch: CONTROL1 TIMEREN=0, counter held in reset while the whole
 * per-rate table is written, then CONTROL1 = TIMEREN | NUMAV written last.
 * Sampling restarts from count 0; caller handles stream-side bookkeeping. */
esp_err_t afe4404_apply_rate(nc_rate_t rate);

/* Reads LED1VAL (IR, 0x2C), LED2VAL (red, 0x2A) and, if want_amb, ALED1VAL
 * (0x2D); sign-extends 24-bit two's complement; out->t_us = t_us_isr,
 * out->amb = 0 when !want_amb. Call after each ADC_RDY, from a task. */
esp_err_t afe4404_read_frame(nc_ppg_sample_t *out, uint64_t t_us_isr,
                             bool want_amb);

/* Clamps to LED_IR_MAX_MA / LED_RED_MAX_MA (board.h) BEFORE the DAC-code
 * conversion round(mA*63/50), then writes LEDCNTRL (0x22). */
esp_err_t afe4404_set_led_ma(uint8_t ir_ma, uint8_t red_ma);

/* rf_code/cf_code 0..7 per SBAS689D tables; writes TIA_GAIN (0x21).
 * ENSEPGAIN stays 0 (single gain shared by both phases). */
esp_err_t afe4404_set_tia(uint8_t rf_code, uint8_t cf_code);

/* phase: 0 LED1(IR), 1 LED2(red), 2 AMB1, 3 AMB2. code -15..+15 maps to
 * OFFDAC magnitude 0-15 (~0-7 uA), sign -> polarity bit. Other phases keep
 * their cached codes (0x3A is written as a whole). */
esp_err_t afe4404_set_offdac(uint8_t phase, int8_t code);

/* Drives RESETZ low and leaves it low: >200 us low = hardware power-down,
 * ~8 uA, ALL REGISTERS LOST. Caller owns gpio_hold_en/deep-sleep holds and
 * must go through afe4404_init() to run again. */
esp_err_t afe4404_powerdown_hw(void);

/* Readback of any CONFIG register via the CONTROL0 REG_READ wrap.
 * ONLY call while streaming is stopped (TIMEREN=0): the wrap rewrites
 * CONTROL0, which is unsafe while the timing engine runs. Data registers
 * 0x2A-0x2F do not need this — afe4404_read_frame reads them directly. */
esp_err_t afe4404_reg_read(uint8_t reg, uint32_t *val);

/* Cached actuals (post-clamp) for STATUS/AGC. */
uint8_t afe4404_cur_ir_ma(void);
uint8_t afe4404_cur_red_ma(void);
uint8_t afe4404_cur_rf(void);
uint8_t afe4404_cur_cf(void);
