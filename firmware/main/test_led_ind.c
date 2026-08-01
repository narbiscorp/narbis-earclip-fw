/*
 * test_led_ind.c — TEST-build LED connect indicator. See test_led_ind.h
 * for the contract; this file is empty in production builds.
 *
 * Design notes:
 *  - The periodic timer runs for the life of the boot (2 posts/s to
 *    sys_q is negligible); ticks landing while RELEASED just re-check
 *    the re-arm condition. A full sys_q drops a tick — the blink skips
 *    one beat and self-heals.
 *  - Actuation goes through test_ops_ensure_bench_afe(), the same
 *    bench bring-up the TEST ops use, so if anything else powered the
 *    AFE down (selftest teardown, orderly stop) the next tick re-inits
 *    it transparently. The desired-vs-actual compare uses the driver's
 *    own post-clamp caches (afe4404_cur_*_ma), which an afe4404_init
 *    zeroes — a re-init therefore repaints automatically instead of
 *    trusting a stale local shadow.
 *  - Re-arm needs BOTH "acquisition stopped" and "disconnected": an
 *    operator's manual LED setup must survive until their session ends
 *    (spec: release until acq stops AND the device is disconnected).
 */
#include "test_led_ind.h"

#if NARBIS_TEST_MODE

#include <stdbool.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/gpio.h"

#include "acq.h"
#include "afe4404.h"
#include "app_msgs.h"
#include "ble_iface.h"
#include "test_ops.h"

static const char *TAG = "led_ind";

#define IND_IR_MA      25          /* 50 % of LED_IR_MAX_MA (50)  */
#define IND_RED_MA     20          /* 50 % of LED_RED_MAX_MA (40) */
#define IND_PERIOD_US  (500LL * 1000)

/* XIAO module onboard user LED (GPIO15, module-internal pin, active
 * LOW). Unlike the optical AFE indicator this one is never handed
 * over — it always shows power/link state on the bench: 1 Hz blink =
 * on + advertising, steady = connected. Invisible in the closed
 * enclosure, so there is no production cost to driving it. */
#define ONBOARD_LED_GPIO   15
#define ONBOARD_ON         0       /* active low */
#define ONBOARD_OFF        1

typedef enum { IND_OFF = 0, IND_OWNING, IND_RELEASED } ind_state_t;

static volatile ind_state_t s_ind = IND_OFF;
static bool s_phase_on;            /* pulse phase while disconnected */
static esp_timer_handle_t s_tmr;

/* esp_timer task: hop into sys context — every LED write must
 * serialize with test ops / AGC actuation there. */
static void ind_tick_cb(void *arg)
{
    (void)arg;
    const sys_msg_t m = { .type = SYS_TEST_IND };
    (void)sys_post(&m);            /* full queue: skip a beat */
}

void test_led_ind_start(void)
{
    if (s_ind != IND_OFF) {
        return;
    }
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << ONBOARD_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    if (gpio_config(&io) == ESP_OK) {
        gpio_set_level(ONBOARD_LED_GPIO, ONBOARD_OFF);
    }
    const esp_timer_create_args_t a = { .callback = ind_tick_cb,
                                        .name = "to_ind" };
    if (esp_timer_create(&a, &s_tmr) != ESP_OK) {
        ESP_LOGW(TAG, "timer create failed — indicator disabled");
        return;
    }
    s_ind = IND_OWNING;
    s_phase_on = false;
    esp_timer_start_periodic(s_tmr, IND_PERIOD_US);
    ind_tick_cb(NULL);             /* first paint ASAP via sys_task */
    ESP_LOGI(TAG, "connect indicator armed (1 Hz pulse @ 50%%)");
}

void test_led_ind_release(void)
{
    if (s_ind == IND_OWNING) {
        s_ind = IND_RELEASED;
        ESP_LOGI(TAG, "LEDs handed over (op)");
    }
}

void test_led_ind_poll(void)
{
    if (s_ind == IND_OFF) {
        return;
    }
    const bool conn = ble_is_connected();

    /* Onboard module LED first — never handed over, tracks link state
     * through every ownership transition below: steady = connected,
     * 1 Hz pulse = powered + advertising. */
    static bool s_ob_phase;
    s_ob_phase = !s_ob_phase;
    gpio_set_level(ONBOARD_LED_GPIO,
                   (conn || s_ob_phase) ? ONBOARD_ON : ONBOARD_OFF);

    /* (a) acquisition running: the AGC owns the optical LEDs. */
    if (acq_ppg_running()) {
        if (s_ind == IND_OWNING) {
            s_ind = IND_RELEASED;
            ESP_LOGI(TAG, "LEDs handed over (acquisition)");
        }
        return;
    }

    if (s_ind == IND_RELEASED) {
        if (conn) {
            return;                /* re-arm needs a disconnect too */
        }
        s_ind = IND_OWNING;
        s_phase_on = false;
        ESP_LOGI(TAG, "re-armed (idle + disconnected)");
    }

    /* OWNING: steady while connected, 1 Hz pulse while not. */
    bool on;
    if (conn) {
        on = true;
        s_phase_on = true;
    } else {
        s_phase_on = !s_phase_on;
        on = s_phase_on;
    }
    const uint8_t ir = on ? IND_IR_MA : 0;
    const uint8_t red = on ? IND_RED_MA : 0;

    if (test_ops_ensure_bench_afe() != ESP_OK) {
        ESP_LOGW(TAG, "bench AFE unavailable");
        return;
    }
    if (afe4404_cur_ir_ma() != ir || afe4404_cur_red_ma() != red) {
        (void)afe4404_set_led_ma(ir, red);
    }
}

#endif /* NARBIS_TEST_MODE */
