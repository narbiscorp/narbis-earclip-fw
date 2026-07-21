/*
 * acq.h — acquisition control surface (implemented by app_tasks.c /
 * acq_afe.c / acq_imu.c / dsp_task.c). Called ONLY from sys_task.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "narbis/nc_types.h"

esp_err_t acq_init(void);                 /* queues + tasks, once at boot */

esp_err_t acq_ppg_start(nc_rate_t rate);  /* AFE up + afe/dsp running     */
esp_err_t acq_ppg_stop(void);             /* AFE powered down             */
esp_err_t acq_ppg_set_rate(nc_rate_t rate); /* live rate switch           */
esp_err_t acq_accel_start(void);
esp_err_t acq_accel_stop(void);
bool acq_ppg_running(void);
bool acq_accel_running(void);

/* Measured mean sample period (EMA over inter-frame deltas) in us —
 * the truth the IBI math uses; internal osc is only +-1%. */
uint32_t acq_measured_ts_us(void);

/* AGC settle blanking: dsp_task flags samples until this device time.
 * Written by sys_task after each actuation; single u64 word, read on
 * the sample path (use a 32-bit split or portMUX on RV32 — impl detail,
 * document the choice). */
void acq_set_blank_until(uint64_t t_us);

/* Batch stream flag bits owned by sys_task (USB present, wear-off):
 * OR-ed into every outgoing PPG batch's flags field. */
void acq_set_extra_ppg_flags(uint8_t set_mask, uint8_t clear_mask);

/* Selftest support: re-register the streaming ADC_RDY ISR after selftest
 * released the pin (its detach must NOT leave the pin handler-less, or
 * PPG never streams again until reboot). Sys-task context only. */
esp_err_t acq_afe_rdy_isr_restore(void);

/* Console raw-sample tap: arm captures the next n frames (dsp_task fills
 * a small ring); read drains them. Single consumer (console). */
void acq_tap_arm(uint16_t n);
int  acq_tap_read(nc_ppg_sample_t *out, int max);

/* AGC freeze latch (NC_OP_AGC_FREEZE / manual override): sys_task owns
 * the freeze POLICY and calls this; dsp_task samples the latch on each
 * slow AGC tick. [Appended by the acquisition agent per task
 * authorization — single-bool atomic, no other coupling.] */
void acq_set_agc_frozen(bool frozen);

/* Latest per-second aggregates for STATUS/wear (dsp_task publishes). */
typedef struct {
    int32_t dc_ir, dc_red;
    int64_t bp_power_1s;
    uint16_t gate_duty_x100;   /* over last 60 s */
    bool gated;
    bool worn;
    uint16_t ibi_last_ms;
    uint8_t hr_bpm;
} acq_aggregates_t;
void acq_get_aggregates(acq_aggregates_t *out);
