/*
 * acq_afe.c — AFE4404 acquisition path (handoff §5.5).
 *
 *   ADC_RDY ISR (GPIO16, ~250 ns rising pulse — NO glitch filter, it
 *   would eat the pulse) -> timestamp -> afe_rdy_q -> afe_task ->
 *   afe4404_read_frame -> ppg_queue (dsp tee, raw + untouched) +
 *   PPG batcher -> ble_tx_submit(BLE_CH_PPG).
 *
 * Concurrency model:
 *  - the ISR only timestamps and posts; a failed post sets a volatile
 *    flag the task folds into g_diag.ppg_overrun (no counters in ISRs);
 *  - the batcher (+ seq/flags/clip latches) is guarded by s_batch_mtx:
 *    afe_task holds it per frame, sys_task holds it inside
 *    start/stop/set_rate to flush/re-arm. Priority inheritance covers
 *    the afe(23) vs sys(12) inversion; the critical section is bounded
 *    (one ble_tx_submit copy, never blocking).
 *  - s_ppg_running is the produce/consume gate: sys clears it FIRST on
 *    stop/rate-switch, so in-flight rdy timestamps are dropped instead
 *    of being read from a powered-down / half-programmed AFE. A frame
 *    already inside afe4404_read_frame when stop lands is serialized by
 *    the driver's own mutex; worst case is one failed read counted in
 *    g_diag.i2c_err. Documented, accepted.
 *
 * All AFE config I2C (init/apply_rate/powerdown) happens here in the
 * callers' context — and acq.h restricts callers to sys_task.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "board.h"
#include "acq.h"
#include "afe4404.h"
#include "ble_iface.h"
#include "diag.h"
#include "sensor_types.h"
#include "acq_internal.h"

#include "narbis/nc_proto_encode.h"

static const char *TAG = "acq_afe";

/* Consecutive read failures before one NC_ERR_I2C event is emitted
 * (arg = count). One event per outage, not per failure. */
#define AFE_I2C_ERR_EVENT_AT 5

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static _Atomic bool s_ppg_running;

/* ISR -> task overflow flag: ISR stays minimal (no counter RMW there);
 * benign race — a lost re-set only delays one count. */
static volatile bool s_rdy_overflow;

/* Batcher block — every field below is s_batch_mtx-guarded. */
static SemaphoreHandle_t s_batch_mtx;
static nc_ppg_batch_t s_batch;
static uint32_t  s_ppg_seq;            /* seq of the batch being built  */
static nc_rate_t s_rate = NC_RATE_100;
static bool      s_amb_mode;           /* batch carries amb field       */
static bool      s_clipped;            /* >=1 near-rail sample in batch */
static bool      s_rate_flag_pending;  /* one-shot NC_PPGF_RATE_CHANGED */

/* sys_task-owned stream flag bits (USB present / wear-off), OR-ed into
 * every outgoing batch. Single writer (sys_task); CAS keeps the
 * set+clear update atomic against itself anyway. */
static _Atomic uint8_t s_extra_flags;

/* ------------------------------------------------------------------ */
/* ISR                                                                 */
/* ------------------------------------------------------------------ */
static void IRAM_ATTR afe_rdy_isr(void *arg)
{
    (void)arg;
    BaseType_t hpw = pdFALSE;
    uint64_t t = (uint64_t)esp_timer_get_time();
    if (xQueueSendFromISR(afe_rdy_q, &t, &hpw) != pdTRUE) {
        s_rdy_overflow = true;
    }
    if (hpw == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/* ------------------------------------------------------------------ */
/* Module init (from acq_init): mutex + GPIO16 wiring, ISR disabled.   */
/* ------------------------------------------------------------------ */
esp_err_t acq_afe_module_init(void)
{
    s_batch_mtx = xSemaphoreCreateMutex();
    if (s_batch_mtx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* ADC_RDY is a push-pull AFE output: input, no pulls, rising edge. */
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_ADC_RDY,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_isr_handler_add(PIN_ADC_RDY, afe_rdy_isr, NULL);
    if (err != ESP_OK) {
        return err;
    }
    /* handler_add enables the pin interrupt — disarm until ppg_start. */
    return gpio_intr_disable(PIN_ADC_RDY);
}

/* Re-register the streaming ADC_RDY ISR after a module (selftest) that
 * temporarily claimed the pin with its own handler releases it. Leaves
 * the interrupt disarmed, exactly like module init — ppg_start arms it. */
esp_err_t acq_afe_rdy_isr_restore(void)
{
    esp_err_t err = gpio_isr_handler_add(PIN_ADC_RDY, afe_rdy_isr, NULL);
    if (err == ESP_OK) {
        err = gpio_intr_disable(PIN_ADC_RDY);
    }
    return err;
}

/* ------------------------------------------------------------------ */
/* Batch helpers — caller holds s_batch_mtx.                           */
/* ------------------------------------------------------------------ */
static void ppg_flush_locked(void)
{
    if (s_batch.n > 0) {
        uint8_t flags = (uint8_t)(atomic_load(&g_acq_batch_bits) |
                                  atomic_load(&s_extra_flags));
        if (s_clipped) {
            flags |= NC_PPGF_CLIPPED;
        }
        if (s_rate_flag_pending) {
            flags |= NC_PPGF_RATE_CHANGED;   /* first batch at new rate */
            s_rate_flag_pending = false;
        }
        uint8_t *pkt = NULL;
        size_t len = nc_ppg_batch_finish(&s_batch, flags, &pkt);
        if (len > 0) {
            (void)ble_tx_submit(BLE_CH_PPG, pkt, (uint16_t)len);
        }
        s_ppg_seq++;
    }
    s_clipped = false;
    nc_ppg_batch_reset(&s_batch, s_ppg_seq, (uint8_t)s_rate, s_amb_mode);
    nc_ppg_batch_set_cap(&s_batch, ble_att_payload_budget());
}

/* ------------------------------------------------------------------ */
/* afe_task                                                            */
/* ------------------------------------------------------------------ */
void acq_afe_task_run(void *arg)
{
    (void)arg;
    uint32_t i2c_fail_streak = 0;

    for (;;) {
        /* WDT pattern while streaming: the queue wait is bounded at 1 s
         * so the loop always reaches esp_task_wdt_reset() well inside
         * the TWDT window even if ADC_RDY stops pulsing; when this task
         * is not WDT-subscribed (idle) the reset is a harmless
         * ESP_ERR_NOT_FOUND. */
        (void)esp_task_wdt_reset();

        uint64_t t_us;
        if (xQueueReceive(afe_rdy_q, &t_us, pdMS_TO_TICKS(1000)) != pdTRUE) {
            continue;
        }

        if (s_rdy_overflow) {
            s_rdy_overflow = false;
            g_diag.ppg_overrun++;
        }
        if (!atomic_load(&s_ppg_running)) {
            continue;   /* stale timestamp from before a stop/rate hold */
        }

        /* Ambient is read whenever it is streamed OR consumed by the
         * DSP subtractor (both LIVE knobs). */
        bool amb_stream = nc_knob_get(KNOB_AMB_STREAM) != 0;
        bool want_amb = amb_stream || nc_knob_get(KNOB_AMB_SUBTRACT) != 0;

        nc_ppg_sample_t s;
        esp_err_t err = afe4404_read_frame(&s, t_us, want_amb);
        if (err != ESP_OK) {
            g_diag.i2c_err++;
            if (++i2c_fail_streak == AFE_I2C_ERR_EVENT_AT) {
                acq_error_event(NC_ERR_I2C, i2c_fail_streak);
            }
            continue;
        }
        i2c_fail_streak = 0;

        /* Raw tee to dsp_task; drop-on-full, never block the AFE pace. */
        if (xQueueSend(ppg_queue, &s, 0) != pdTRUE) {
            g_diag.ppg_overrun++;
        } else {
            uint32_t depth = (uint32_t)uxQueueMessagesWaiting(ppg_queue);
            if (depth > g_diag.queue_hw_ppg) {
                g_diag.queue_hw_ppg = depth;
            }
        }

        /* --- batcher ------------------------------------------------ */
        xSemaphoreTake(s_batch_mtx, portMAX_DELAY);

        if (amb_stream != s_amb_mode) {
            /* LIVE amb_stream flip: stride changes, so the old-mode
             * partial goes out first and the batch re-arms. */
            ppg_flush_locked();          /* flushes under the OLD mode  */
            s_amb_mode = amb_stream;
            nc_ppg_batch_reset(&s_batch, s_ppg_seq, (uint8_t)s_rate, s_amb_mode);
            nc_ppg_batch_set_cap(&s_batch, ble_att_payload_budget());
        }

        int32_t thr = acq_sat_thr_counts();
        if (s.ir >= thr || s.ir <= -thr || s.red >= thr || s.red <= -thr ||
            (s_amb_mode && (s.amb >= thr || s.amb <= -thr))) {
            s_clipped = true;
        }

        if (!nc_ppg_batch_add(&s_batch, s.t_us, s.ir, s.red, s.amb)) {
            ppg_flush_locked();          /* full: flush, then re-add    */
            (void)nc_ppg_batch_add(&s_batch, s.t_us, s.ir, s.red, s.amb);
        }
        if (s_batch.n > 0 &&
            (s.t_us - s_batch.t0_us) >=
                (uint64_t)nc_knob_get(KNOB_PPG_BATCH_MS) * 1000ULL) {
            ppg_flush_locked();
        }

        xSemaphoreGive(s_batch_mtx);
    }
}

/* ------------------------------------------------------------------ */
/* Control surface (sys_task context only, per acq.h)                  */
/* ------------------------------------------------------------------ */
esp_err_t acq_ppg_start(nc_rate_t rate)
{
    if (rate >= NC_RATE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (atomic_load(&s_ppg_running)) {
        return (rate == s_rate) ? ESP_OK : acq_ppg_set_rate(rate);
    }

    /* Rev-proof bring-up (~15 ms: reset choreography + full config). */
    esp_err_t err = afe4404_init(rate);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "afe4404_init: %s", esp_err_to_name(err));
        acq_error_event(NC_ERR_AFE_INIT, (uint32_t)err);
        return err;
    }

    /* Discard anything queued from a previous run — the producer side
     * is quiet (ISR disabled, running=false), so resets cannot race a
     * concurrent send. */
    xQueueReset(afe_rdy_q);
    s_rdy_overflow = false;
    xQueueReset(ppg_queue);

    xSemaphoreTake(s_batch_mtx, portMAX_DELAY);
    s_rate = rate;
    s_ppg_seq = 0;                       /* seq contract: 0 at start    */
    s_clipped = false;
    s_rate_flag_pending = false;
    s_amb_mode = nc_knob_get(KNOB_AMB_STREAM) != 0;
    nc_ppg_batch_reset(&s_batch, 0, (uint8_t)rate, s_amb_mode);
    nc_ppg_batch_set_cap(&s_batch, ble_att_payload_budget());
    xSemaphoreGive(s_batch_mtx);

    /* Full DSP-side reset (filters, IBI, gate, AGC shadow, wear, duty). */
    acq_dsp_request_reset(rate, true);

    /* WDT covers the two hot tasks only while acquiring. Tolerate
     * "not inited" (TWDT config off) and "already added". */
    err = esp_task_wdt_add(g_acq_afe_task);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "wdt add afe: %s", esp_err_to_name(err));
    }
    err = esp_task_wdt_add(g_acq_dsp_task);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "wdt add dsp: %s", esp_err_to_name(err));
    }

    atomic_store(&s_ppg_running, true);
    err = gpio_intr_enable(PIN_ADC_RDY);
    ESP_LOGI(TAG, "ppg start @%u sps", nc_rate_sps(rate));
    return err;
}

esp_err_t acq_ppg_stop(void)
{
    if (!atomic_load(&s_ppg_running)) {
        return ESP_OK;
    }
    /* Order (spec): gate off producers, disarm ISR, drop WDT coverage,
     * kill the AFE, then flush the partial batch. */
    atomic_store(&s_ppg_running, false);
    (void)gpio_intr_disable(PIN_ADC_RDY);
    (void)esp_task_wdt_delete(g_acq_afe_task);
    (void)esp_task_wdt_delete(g_acq_dsp_task);

    esp_err_t err = afe4404_powerdown_hw();

    xSemaphoreTake(s_batch_mtx, portMAX_DELAY);
    ppg_flush_locked();
    xSemaphoreGive(s_batch_mtx);

    ESP_LOGI(TAG, "ppg stop");
    return err;
}

esp_err_t acq_ppg_set_rate(nc_rate_t rate)
{
    if (rate >= NC_RATE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!atomic_load(&s_ppg_running)) {
        return ESP_ERR_INVALID_STATE;    /* rate knob applies at start  */
    }
    if (rate == s_rate) {
        return ESP_OK;
    }

    uint8_t old_code = (uint8_t)s_rate;

    /* Hold the pipeline: frames read during TIMEREN=0 reprogramming
     * would be garbage, so in-flight timestamps are dropped instead. */
    atomic_store(&s_ppg_running, false);
    (void)gpio_intr_disable(PIN_ADC_RDY);
    xQueueReset(afe_rdy_q);
    s_rdy_overflow = false;

    /* Old-rate partial batch out before the header rate_code changes. */
    xSemaphoreTake(s_batch_mtx, portMAX_DELAY);
    ppg_flush_locked();
    xSemaphoreGive(s_batch_mtx);

    esp_err_t err = afe4404_apply_rate(rate);
    if (err != ESP_OK) {
        /* Half-programmed timing engine: leave the stream held (caller
         * decides stop/restart); do NOT re-enable the ISR. */
        ESP_LOGE(TAG, "apply_rate: %s", esp_err_to_name(err));
        acq_error_event(NC_ERR_AFE_INIT, (uint32_t)err);
        return err;
    }

    xQueueReset(ppg_queue);              /* no old-rate samples into new DSP */
    acq_dsp_request_reset(rate, false);  /* filters/IBI/ts re-init only */

    xSemaphoreTake(s_batch_mtx, portMAX_DELAY);
    s_rate = rate;                       /* seq continues (reset only at start) */
    s_rate_flag_pending = true;          /* arm NC_PPGF_RATE_CHANGED    */
    s_clipped = false;
    nc_ppg_batch_reset(&s_batch, s_ppg_seq, (uint8_t)rate, s_amb_mode);
    nc_ppg_batch_set_cap(&s_batch, ble_att_payload_budget());
    xSemaphoreGive(s_batch_mtx);

    uint8_t d[2] = { old_code, (uint8_t)rate };
    acq_event_post(NC_EV_RATE_CHANGE, d, sizeof(d));

    /* Timestamps stay monotonic by construction (esp_timer domain). */
    atomic_store(&s_ppg_running, true);
    err = gpio_intr_enable(PIN_ADC_RDY);
    ESP_LOGI(TAG, "rate %u -> %u sps", nc_rate_sps((nc_rate_t)old_code),
             nc_rate_sps(rate));
    return err;
}

bool acq_ppg_running(void)
{
    return atomic_load(&s_ppg_running);
}

void acq_set_extra_ppg_flags(uint8_t set_mask, uint8_t clear_mask)
{
    uint8_t v = atomic_load(&s_extra_flags);
    uint8_t nv;
    do {
        nv = (uint8_t)((v & (uint8_t)~clear_mask) | set_mask);
    } while (!atomic_compare_exchange_weak(&s_extra_flags, &v, nv));
}
