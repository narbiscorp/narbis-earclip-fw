/*
 * acq_internal.h — private glue between app_tasks.c / acq_afe.c /
 * acq_imu.c / dsp_task.c. NOT a public contract; nothing outside the
 * acquisition layer may include this.
 *
 * Cross-task data discipline:
 *  - u32/u8/bool cross-task scalars are C11 _Atomic (lock-free on RV32);
 *  - the only cross-task u64 (AGC blank_until) lives behind a portMUX in
 *    dsp_task.c because RV32 has no 64-bit atomic load;
 *  - everything else is single-owner file-static state.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "narbis/nc_types.h"
#include "narbis/nc_knobs.h"
#include "narbis/nc_agc.h"

/* Task handles (created once by acq_init; ISRs and WDT add/delete use
 * them, so acq_init must finish before any acq_*_start runs). */
extern TaskHandle_t g_acq_afe_task;
extern TaskHandle_t g_acq_imu_task;
extern TaskHandle_t g_acq_dsp_task;

/* Task entry points (app_tasks.c creates; each module implements own). */
void acq_afe_task_run(void *arg);
void acq_imu_task_run(void *arg);
void acq_dsp_task_run(void *arg);

/* Per-module one-time init (mutexes + GPIO/ISR wiring, interrupt left
 * DISABLED). Called from acq_init after gpio_install_isr_service. */
esp_err_t acq_afe_module_init(void);
esp_err_t acq_imu_module_init(void);

/* Non-blocking event_q push helpers (app_tasks.c). Timestamp is taken
 * inside; a full queue silently drops (events are advisory). */
void acq_event_post(uint8_t type, const uint8_t *data, uint8_t len);
void acq_error_event(uint16_t code, uint32_t arg); /* NC_EV_ERROR wrap */

/* imu_task -> dsp_task: latest accel window energy (nc_gate_accel_feed
 * output). Zeroed on accel stop so a halted accelerometer can never
 * hold the gate closed with stale motion energy. */
extern _Atomic uint32_t g_acq_acc_energy;

/* dsp_task -> afe_task: per-sample gate/settle annotation for the PPG
 * batcher (NC_PPGF_GATE | NC_PPGF_AGC_SETTLING only; USB/WEAR_OFF bits
 * are sys_task's, via acq_set_extra_ppg_flags). */
extern _Atomic uint8_t g_acq_batch_bits;

/* sys_task -> dsp_task: AGC freeze latch (acq_set_agc_frozen). */
extern _Atomic bool g_acq_agc_frozen;

/* acq_afe.c -> dsp_task.c: request a DSP-side reset before the next
 * sample is processed. full=true (acquisition start) also resets gate,
 * AGC context (incl. offset-DAC shadow — afe4404_init zeroed the DACs),
 * wear, HR/IBI seq and the gate-duty ring; full=false (live rate
 * switch) re-inits only the rate-derived state (filters, IBI, measured
 * timestamp, tick counters). */
void acq_dsp_request_reset(nc_rate_t rate, bool full);

/* Saturation threshold in counts: sat_pct% of the AFE positive full
 * scale (2^21). Single definition shared by the batch CLIPPED flag
 * (afe_task) and the gate/AGC sat inputs (dsp_task). */
static inline int32_t acq_sat_thr_counts(void)
{
    return (int32_t)(((int64_t)nc_knob_get(KNOB_SAT_PCT) * NC_AGC_FS_COUNTS) / 100);
}
