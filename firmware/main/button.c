/*
 * button.c — GPIO2 debounce front end.
 *
 * Pipeline: any-edge ISR -> (re)start a 30 ms one-shot esp_timer ->
 * timer callback samples the settled level -> if it differs from the
 * last CONFIRMED level, post SYS_BTN_EDGE to sys_q.
 *
 * Properties this buys:
 *  - every bounce restarts the window, so a confirmed edge means the
 *    line has been quiet for BUTTON_DEBOUNCE_MS — TVS/C22 ringing on
 *    the net (board.h) cannot produce edge pairs faster than 30 ms;
 *  - confirmed edges lag the physical edge by ~30 ms consistently, so
 *    durations measured between them (what nc_button cares about) are
 *    unbiased;
 *  - the ISR touches only esp_timer start/stop, which are IRAM-resident
 *    and use portENTER_CRITICAL_SAFE internally (esp_timer.c, verified
 *    v5.5.1) — ISR-legal, and no I2C / queue work in the ISR at all.
 *
 * Light-sleep wake: GPIO2 is an LP pin, so gpio_wakeup_enable() arms the
 * LP_IO level sampler (rtc_gpio_wakeup_enable path in gpio.c), which is
 * independent of the clock-gated digital interrupt matrix. That call
 * also forces the pad's digital interrupt type to LOW_LEVEL, so ANYEDGE
 * is restored immediately after — awake edge detection and asleep level
 * wake coexist. BENCH-VERIFY (bring-up checklist): a short press landing
 * entirely inside an idle light-sleep window must still register.
 */
#include "button.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "app_msgs.h"
#include "board.h"

static const char *TAG = "button";

static esp_timer_handle_t s_debounce_tmr;
static bool s_reported_pressed;   /* last level posted to sys_task */

static void IRAM_ATTR btn_isr(void *arg)
{
    (void)arg;
    /* Restart the quiet window on every edge. stop() on an idle timer
     * returns ESP_ERR_INVALID_STATE — expected, ignored. */
    esp_timer_stop(s_debounce_tmr);
    esp_timer_start_once(s_debounce_tmr, (uint64_t)BUTTON_DEBOUNCE_MS * 1000);
}

/* esp_timer task context. */
static void debounce_cb(void *arg)
{
    (void)arg;
    bool pressed = (gpio_get_level(PIN_BUTTON) == 0);   /* active low */
    if (pressed == s_reported_pressed) {
        return;   /* bounce settled back to the reported state */
    }
    s_reported_pressed = pressed;

    sys_msg_t m = {
        .type = SYS_BTN_EDGE,
        .u.btn = {
            .pressed = pressed,
            .t_ms = (uint32_t)(esp_timer_get_time() / 1000),
        },
    };
    if (!sys_post(&m)) {
        /* sys_q full: the edge is lost; the FSM self-heals on the next
         * confirmed edge (level-based sampling, not edge counting). */
        ESP_LOGW(TAG, "sys_q full, edge dropped");
    }
}

esp_err_t button_init(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,    /* no external pull-up on the net */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }

    const esp_timer_create_args_t targs = {
        .callback = debounce_cb,
        .name = "btn_deb",
    };
    err = esp_timer_create(&targs, &s_debounce_tmr);
    if (err != ESP_OK) {
        return err;
    }

    /* Acquisition tasks may have installed the shared GPIO ISR service
     * already (ADC_RDY / INT1) — INVALID_STATE means "already up". */
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = gpio_isr_handler_add(PIN_BUTTON, btn_isr, NULL);
    if (err != ESP_OK) {
        return err;
    }

    /* A press that is still in progress at init time belongs to the
     * wake gesture ("OFF + any press -> wake"), not to the FSM: start
     * from released so the eventual release edge is swallowed. */
    s_reported_pressed = false;

    /* Light-sleep wake (see file header). Order matters: wakeup_enable
     * rewrites the intr type, restore ANYEDGE afterwards. */
    err = gpio_wakeup_enable(PIN_BUTTON, GPIO_INTR_LOW_LEVEL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "gpio_wakeup_enable: %s", esp_err_to_name(err));
    }
    err = gpio_set_intr_type(PIN_BUTTON, GPIO_INTR_ANYEDGE);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_sleep_enable_gpio_wakeup();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_sleep_enable_gpio_wakeup: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "GPIO%d armed (debounce %d ms)", PIN_BUTTON, BUTTON_DEBOUNCE_MS);
    return ESP_OK;
}
