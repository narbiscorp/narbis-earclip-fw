/*
 * diag.h — diagnostics: reset reason, coredump presence, runtime counters.
 * Counters are plain u32 with single-writer discipline (each counter has
 * exactly one incrementing task/ISR); 32-bit aligned loads are atomic on
 * RV32 so readers (STATUS/console) take no lock.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    uint32_t i2c_err;          /* afe_task / imu_task I2C failures      */
    uint32_t notify_drop;      /* ble_tx staging drops (all streams)    */
    uint32_t ppg_overrun;      /* afe_rdy_q or ppg_queue full           */
    uint32_t fifo_overrun;     /* LIS2DH12 FIFO OVRN                    */
    uint32_t agc_steps;        /* actuations                            */
    uint32_t gate_transitions;
    uint32_t queue_hw_ppg;     /* high-water marks                      */
    uint32_t queue_hw_ble;
} diag_counters_t;

extern diag_counters_t g_diag;

esp_err_t diag_init(void);           /* log reset reason + coredump state */
const char *diag_reset_reason_str(void);
bool diag_coredump_present(void);
void diag_console_dump(void);        /* 'stats' command body */
