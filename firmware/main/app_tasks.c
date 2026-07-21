/*
 * app_tasks.c — queue + task bring-up and the sys_task mailbox posts
 * (handoff §5.5/§5.11 glue; acq.h acq_init, app_msgs.h externs).
 *
 * Ownership: this file creates every queue and every task exactly once.
 * sys_task's BODY lives in sys_task.c (other agent); its entry prototype
 * is the declared contract `void sys_task_run(void *)`.
 *
 * Priority ladder (rationale):
 *   afe 23  — must drain afe_rdy_q within one sample period (2 ms @500sps)
 *   imu 20  — FIFO watermark gives ~500 ms of slack, but the drain burst
 *             (32 samples over I2C) must not be starved by dsp
 *   dsp 19  — per-sample math; ppg_queue gives 256 ms of buffer
 *   sys 12  — control plane; everything above preempts it freely
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "acq.h"
#include "app_msgs.h"
#include "sensor_types.h"
#include "acq_internal.h"

static const char *TAG = "acq_tasks";

/* ------------------------------------------------------------------ */
/* Queue + task handles (sensor_types.h / app_msgs.h externs)          */
/* ------------------------------------------------------------------ */
QueueHandle_t sys_q;
QueueHandle_t ppg_queue;
QueueHandle_t imu_queue;
QueueHandle_t afe_rdy_q;
QueueHandle_t event_q;

TaskHandle_t g_acq_afe_task;
TaskHandle_t g_acq_imu_task;
TaskHandle_t g_acq_dsp_task;

/* sys_task.c (other agent) provides the body; we own the creation. */
extern void sys_task_run(void *arg);

static bool s_inited;

/* ------------------------------------------------------------------ */
/* sys_q posts — 0-timeout everywhere: hot paths must never block on   */
/* the control mailbox; a full sys_q means sys_task is wedged and the  */
/* caller's diag counter is the record of it.                          */
/* ------------------------------------------------------------------ */
bool sys_post(const sys_msg_t *msg)
{
    return sys_q != NULL && xQueueSend(sys_q, msg, 0) == pdTRUE;
}

bool sys_post_from_isr(const sys_msg_t *msg, BaseType_t *hpw)
{
    return sys_q != NULL && xQueueSendFromISR(sys_q, msg, hpw) == pdTRUE;
}

/* ------------------------------------------------------------------ */
/* event_q helpers (shared by the acquisition files)                   */
/* ------------------------------------------------------------------ */
void acq_event_post(uint8_t type, const uint8_t *data, uint8_t len)
{
    nc_event_t ev;
    ev.t_us = (uint64_t)esp_timer_get_time();
    ev.type = type;
    if (len > sizeof(ev.data)) {
        len = sizeof(ev.data);
    }
    ev.len = len;
    if (len > 0) {
        memcpy(ev.data, data, len);
    }
    /* 0-timeout: events are advisory; the drainer's seq numbers do not
     * cover event_q losses and that is accepted (EVENT is best-effort). */
    (void)xQueueSend(event_q, &ev, 0);
}

void acq_error_event(uint16_t code, uint32_t arg)
{
    /* NC_EVLEN_ERROR wire payload after t_us: u16 code, u32 arg (LE). */
    uint8_t d[6];
    d[0] = (uint8_t)(code & 0xFF);
    d[1] = (uint8_t)(code >> 8);
    d[2] = (uint8_t)(arg & 0xFF);
    d[3] = (uint8_t)((arg >> 8) & 0xFF);
    d[4] = (uint8_t)((arg >> 16) & 0xFF);
    d[5] = (uint8_t)((arg >> 24) & 0xFF);
    acq_event_post(NC_EV_ERROR, d, sizeof(d));
}

/* ------------------------------------------------------------------ */
/* acq_init — queues first (tasks block on them at entry), then GPIO   */
/* ISR service + module wiring, then tasks. Runs once from app_main    */
/* before anything can call the acq_* control surface.                 */
/* ------------------------------------------------------------------ */
esp_err_t acq_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    sys_q     = xQueueCreate(SYS_Q_DEPTH,     sizeof(sys_msg_t));
    ppg_queue = xQueueCreate(PPG_QUEUE_DEPTH, sizeof(nc_ppg_sample_t));
    imu_queue = xQueueCreate(IMU_QUEUE_DEPTH, sizeof(nc_accel_sample_t));
    afe_rdy_q = xQueueCreate(AFE_RDY_Q_DEPTH, sizeof(uint64_t));
    event_q   = xQueueCreate(EVENT_Q_DEPTH,   sizeof(nc_event_t));
    if (sys_q == NULL || ppg_queue == NULL || imu_queue == NULL ||
        afe_rdy_q == NULL || event_q == NULL) {
        ESP_LOGE(TAG, "queue alloc failed");
        return ESP_ERR_NO_MEM;
    }

    /* IRAM ISR service: the ADC_RDY/INT1 handlers stay serviceable
     * during flash-cache-off windows (NVS commits while streaming). */
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio isr service: %s", esp_err_to_name(err));
        return err;
    }

    err = acq_afe_module_init();
    if (err != ESP_OK) {
        return err;
    }
    err = acq_imu_module_init();
    if (err != ESP_OK) {
        return err;
    }

    /* Stack sizes are BYTES (ESP-IDF xTaskCreate). Create the data
     * pipeline before sys_task so the control plane can never observe
     * half-built acquisition tasks. */
    if (xTaskCreate(acq_dsp_task_run, "dsp", 4096, NULL, 19, &g_acq_dsp_task) != pdPASS ||
        xTaskCreate(acq_imu_task_run, "imu", 3072, NULL, 20, &g_acq_imu_task) != pdPASS ||
        xTaskCreate(acq_afe_task_run, "afe", 3072, NULL, 23, &g_acq_afe_task) != pdPASS ||
        xTaskCreate(sys_task_run,     "sys", 4096, NULL, 12, NULL) != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return ESP_ERR_NO_MEM;
    }

    s_inited = true;
    ESP_LOGI(TAG, "queues + tasks up (afe 23/3072, imu 20/3072, dsp 19/4096, sys 12/4096)");
    return ESP_OK;
}
