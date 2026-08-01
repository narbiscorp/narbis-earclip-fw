/*
 * acq_imu.c — LIS2DH12 acquisition path (handoff §5.11).
 *
 *   INT1 (GPIO17, active-high, FIFO watermark) ISR -> task notify ->
 *   imu_task: drain FIFO -> per sample: gate window energy (published
 *   to dsp_task via _Atomic u32) + imu_queue tee + accel batcher ->
 *   ble_tx_submit(BLE_CH_ACCEL) when full / 50 ms elapsed.
 *
 * The LIS2DH12 WTM interrupt is LEVEL-held while fill >= watermark but
 * the GPIO trigger is edge (POSEDGE): a drain always pulls fill below
 * the watermark so the line re-arms, and two belts cover the corners —
 * a kick-notify at start (line may already be high => no edge) and the
 * 1 s notify timeout, which turns into a poll drain while running.
 *
 * Internal use vs BLE emission are decoupled (spec): the gate energy
 * feed and imu_queue tee always run; the batcher only runs while the
 * acc_stream_en knob is set.
 *
 * s_imu_mtx guards the batcher + seq + overrun latch: imu_task holds it
 * per drain, sys_task inside accel_stop's flush. The gate energy ring
 * (s_gate_acc) is imu_task-only once running; acq_accel_start re-inits
 * it strictly before setting s_acc_running.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "board.h"
#include "acq.h"
#include "lis2dh12.h"
#include "ble_iface.h"
#include "diag.h"
#include "sensor_types.h"
#include "acq_internal.h"

#include "narbis/nc_gate.h"
#include "narbis/nc_proto_encode.h"

static const char *TAG = "acq_imu";

/* Fixed accel batch flush age (spec: "full / 50 ms"). Not a knob. */
#define ACCEL_BATCH_FLUSH_US 50000ULL

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static _Atomic bool s_acc_running;

/* imu -> dsp: latest window energy (defined here, extern'd in
 * acq_internal.h). Zeroed on stop so the gate never rides stale motion. */
_Atomic uint32_t g_acq_acc_energy;

/* imu_task-only once running (see header comment). */
static nc_gate_acc_t s_gate_acc;

/* s_imu_mtx-guarded block. */
static SemaphoreHandle_t s_imu_mtx;
static nc_accel_batch_t s_abatch;
static uint32_t s_acc_seq;
static bool s_ovr_latch;       /* batch-scoped: >=1 OVRN in this batch  */

static nc_acc_odr_t s_odr = NC_ODR_50;
static uint8_t s_fs = 1;       /* NC_ACCF_FS_MASK code, default ±4 g    */

/* imu_task-only: rate-limits the NC_ERR_FIFO_OVERRUN event to the
 * false->true transition of the per-drain OVRN report. */
static bool s_ovr_active;

/* ------------------------------------------------------------------ */
/* ISR                                                                 */
/* ------------------------------------------------------------------ */
static void IRAM_ATTR imu_int1_isr(void *arg)
{
    (void)arg;
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(g_acq_imu_task, &hpw);
    if (hpw == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/* ------------------------------------------------------------------ */
/* Module init (from acq_init). The lis2dh12 driver owns the pad
 * config (plain input, external 10 k pull-down defines idle level);
 * only the trigger type + handler registration belong to us.          */
/* ------------------------------------------------------------------ */
esp_err_t acq_imu_module_init(void)
{
    s_imu_mtx = xSemaphoreCreateMutex();
    if (s_imu_mtx == NULL) {
        return ESP_ERR_NO_MEM;
    }
    /* Bring the device up FIRST: lis2dh12_init owns the INT1 pad's
     * gpio_config and creates the I2C device handle that every later
     * lis2dh12_config/start depends on (nothing else on the normal
     * boot path calls it). A failure here means the accel is absent —
     * surface it like any other init failure. */
    esp_err_t err = lis2dh12_init();
    if (err != ESP_OK) {
#if NARBIS_BENCH_BUILD
        /* Bare-XIAO bench build: no mainboard, no accel. Boot proceeds
         * (BLE/console/OTA must work for pre-soldering prep); selftest
         * T01/T02 report the absence honestly. */
        ESP_LOGW(TAG, "BENCH: accel absent (%s) — continuing without it",
                 esp_err_to_name(err));
        return ESP_OK;
#else
        ESP_LOGE(TAG, "lis2dh12_init failed (%s)", esp_err_to_name(err));
        return err;
#endif
    }
    err = gpio_set_intr_type(PIN_ACC_INT1, GPIO_INTR_POSEDGE);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_isr_handler_add(PIN_ACC_INT1, imu_int1_isr, NULL);
    if (err != ESP_OK) {
        return err;
    }
    return gpio_intr_disable(PIN_ACC_INT1);   /* armed by accel_start */
}

/* ------------------------------------------------------------------ */
/* Batch helper — caller holds s_imu_mtx.                              */
/* ------------------------------------------------------------------ */
static void accel_flush_locked(void)
{
    if (s_abatch.n > 0) {
        uint8_t flags = (uint8_t)(s_fs & NC_ACCF_FS_MASK);
        if (s_ovr_latch) {
            flags |= NC_ACCF_FIFO_OVERRUN;
        }
        uint8_t *pkt = NULL;
        size_t len = nc_accel_batch_finish(&s_abatch, flags, &pkt);
        if (len > 0) {
            (void)ble_tx_submit(BLE_CH_ACCEL, pkt, (uint16_t)len);
        }
        s_acc_seq++;
    }
    s_ovr_latch = false;
    nc_accel_batch_reset(&s_abatch, s_acc_seq, (uint8_t)s_odr);
    nc_accel_batch_set_cap(&s_abatch, ble_att_payload_budget());
}

/* ------------------------------------------------------------------ */
/* imu_task                                                            */
/* ------------------------------------------------------------------ */
void acq_imu_task_run(void *arg)
{
    (void)arg;
    for (;;) {
        /* 1 s timeout doubles as the missed-edge poll (see file head).
         * imu_task is not WDT-subscribed — FIFO buffering makes it
         * latency-tolerant by design. */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

        if (!atomic_load(&s_acc_running)) {
            continue;
        }

        nc_accel_sample_t buf[LIS2DH12_FIFO_DEPTH];
        size_t n = 0;
        bool ovr = false;
        esp_err_t err = lis2dh12_read_fifo(buf, LIS2DH12_FIFO_DEPTH, &n,
                                           (uint64_t)esp_timer_get_time(),
                                           &ovr);
        if (err != ESP_OK) {
            g_diag.i2c_err++;
            continue;
        }

        /* Gate energy + tee run unconditionally (internal consumers are
         * not maskable by the BLE stream knob). */
        for (size_t i = 0; i < n; i++) {
            uint32_t e = nc_gate_accel_feed(&s_gate_acc, buf[i].x,
                                            buf[i].y, buf[i].z);
            atomic_store(&g_acq_acc_energy, e);
            (void)xQueueSend(imu_queue, &buf[i], 0);   /* tee, best-effort */
        }

        xSemaphoreTake(s_imu_mtx, portMAX_DELAY);

        if (ovr) {
            g_diag.fifo_overrun++;
            s_ovr_latch = true;
            if (!s_ovr_active) {
                acq_error_event(NC_ERR_FIFO_OVERRUN, g_diag.fifo_overrun);
            }
        }
        s_ovr_active = ovr;

        if (nc_knob_get(KNOB_ACC_STREAM_EN) != 0) {
            for (size_t i = 0; i < n; i++) {
                if (!nc_accel_batch_add(&s_abatch, buf[i].t_us, buf[i].x,
                                        buf[i].y, buf[i].z)) {
                    accel_flush_locked();
                    (void)nc_accel_batch_add(&s_abatch, buf[i].t_us,
                                             buf[i].x, buf[i].y, buf[i].z);
                }
            }
            if (s_abatch.n > 0 && n > 0 &&
                (buf[n - 1].t_us - s_abatch.t0_us) >= ACCEL_BATCH_FLUSH_US) {
                accel_flush_locked();
            }
        } else if (s_abatch.n > 0) {
            /* LIVE knob turned the BLE stream off mid-run: drop staged
             * samples rather than emit them arbitrarily later. */
            s_ovr_latch = false;
            nc_accel_batch_reset(&s_abatch, s_acc_seq, (uint8_t)s_odr);
            nc_accel_batch_set_cap(&s_abatch, ble_att_payload_budget());
        }

        xSemaphoreGive(s_imu_mtx);
    }
}

/* ------------------------------------------------------------------ */
/* Control surface (sys_task context only, per acq.h)                  */
/* ------------------------------------------------------------------ */
esp_err_t acq_accel_start(void)
{
    if (atomic_load(&s_acc_running)) {
        return ESP_OK;
    }

    nc_acc_odr_t odr = (nc_acc_odr_t)nc_knob_get(KNOB_ACC_ODR);
    uint8_t fs = (uint8_t)(nc_knob_get(KNOB_ACC_FS) & NC_ACCF_FS_MASK);
    if (odr >= NC_ODR_COUNT) {
        odr = NC_ODR_50;    /* knob range should prevent this; belt only */
    }

    esp_err_t err = lis2dh12_config(odr, fs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lis2dh12_config: %s", esp_err_to_name(err));
        acq_error_event(NC_ERR_ACCEL_INIT, (uint32_t)err);
        return err;
    }

    /* Window ring re-derived for the (possibly new) ODR/window knobs.
     * Safe: imu_task ignores everything while s_acc_running is false. */
    nc_gate_acc_init(&s_gate_acc, nc_acc_odr_hz(odr));
    atomic_store(&g_acq_acc_energy, 0);

    xSemaphoreTake(s_imu_mtx, portMAX_DELAY);
    s_odr = odr;
    s_fs = fs;
    s_acc_seq = 0;
    s_ovr_latch = false;
    nc_accel_batch_reset(&s_abatch, 0, (uint8_t)odr);
    nc_accel_batch_set_cap(&s_abatch, ble_att_payload_budget());
    xSemaphoreGive(s_imu_mtx);
    s_ovr_active = false;

    atomic_store(&s_acc_running, true);
    err = gpio_intr_enable(PIN_ACC_INT1);

    /* Kick one drain: if INT1 is already high (config raced the first
     * watermark) the edge was missed; the drain re-arms the line. */
    xTaskNotifyGive(g_acq_imu_task);

    ESP_LOGI(TAG, "accel start @%u Hz fs=%u", nc_acc_odr_hz(odr), fs);
    return err;
}

esp_err_t acq_accel_stop(void)
{
    if (!atomic_load(&s_acc_running)) {
        return ESP_OK;
    }
    atomic_store(&s_acc_running, false);
    (void)gpio_intr_disable(PIN_ACC_INT1);

    esp_err_t err = lis2dh12_powerdown();

    xSemaphoreTake(s_imu_mtx, portMAX_DELAY);
    accel_flush_locked();
    xSemaphoreGive(s_imu_mtx);

    /* A stopped accelerometer must read as "no motion", not "last
     * motion forever" — the gate compares this against gate_acc_thr. */
    atomic_store(&g_acq_acc_energy, 0);

    ESP_LOGI(TAG, "accel stop");
    return err;
}

bool acq_accel_running(void)
{
    return atomic_load(&s_acc_running);
}
