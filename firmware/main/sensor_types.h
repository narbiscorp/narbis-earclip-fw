/*
 * sensor_types.h — FreeRTOS-side queue contract (old spec §5, kept stable).
 *
 * Boundary rule: afe_task is the ONLY producer of ppg_queue and pushes
 * every frame raw and untouched; dsp_task is its consumer. imu_queue is
 * a tee for BLE/debug — the gate consumes accel through the atomic
 * energy publish in dsp_task, never through this queue.
 */
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "narbis/nc_types.h"

#define PPG_QUEUE_DEPTH   128   /* 256 ms of headroom at 500 sps */
#define IMU_QUEUE_DEPTH   64
#define AFE_RDY_Q_DEPTH   8     /* ISR timestamps awaiting frame read */
#define EVENT_Q_DEPTH     32
#define SYS_Q_DEPTH       16
#define BLE_TX_Q_DEPTH    12

extern QueueHandle_t ppg_queue;    /* items: nc_ppg_sample_t   */
extern QueueHandle_t imu_queue;    /* items: nc_accel_sample_t */
extern QueueHandle_t afe_rdy_q;    /* items: uint64_t t_us (from ISR) */
extern QueueHandle_t event_q;      /* items: nc_event_t        */
