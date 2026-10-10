/*
 * test_led_ind.c — TEST-build LED connect indicator. See test_led_ind.h
 * for the contract; this file is empty in production builds.
 *
 * Design notes:
 *  - The periodic timer runs for the life of the boot (~33 ticks/s for
 *    the breathing repaint); ticks landing while RELEASED just re-check
 *    the re-arm condition. At most ONE tick is ever queued (pending
 *    flag), so the indicator can never crowd AGC / wear / CONTROL
 *    traffic out of sys_q; a busy sys_task just repaints less often.
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

#include <stdatomic.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/gpio.h"

#include "acq.h"
#include "afe4404.h"
#include "app_msgs.h"
#include "ble_iface.h"
#include "test_ops.h"
#include "narbis/nc_knobs.h"

static const char *TAG = "led_ind";

#define IND_IR_MA      25          /* 50 % of LED_IR_MAX_MA (50)  */
#define IND_RED_MA     20          /* 50 % of LED_RED_MAX_MA (40) */
/* Disconnected, the optical LEDs BREATHE: one smooth dark -> bright ->
 * dark cycle per ind_breath_ms (default 1.5 s, a little slower than the
 * old 1 Hz blink, per the user), repainted every IND_TICK_US. */
#define IND_TICK_US    (30LL * 1000)
#if NARBIS_BENCH_BUILD
#define OB_HALF_US     (250LL * 1000)   /* bare-module spec: 2 Hz pulse */
#else
#define OB_HALF_US     (500LL * 1000)   /* functest: 1 Hz pulse         */
#endif

/* One breath, Q8: ((1 - cos(2 pi i / 64)) / 2)^2 * 255 — a raised cosine
 * (round at both ends, no corners) squared as a gamma so the eye sees an
 * even fade instead of a long bright plateau. */
static const uint8_t k_breath[64] = {
      0,   0,   0,   0,   0,   1,   2,   3,   5,   9,  13,  18,  24,  32,  41,  52,
     64,  77,  91, 106, 122, 138, 154, 170, 186, 200, 214, 226, 236, 244, 250, 254,
    255, 254, 250, 244, 236, 226, 214, 200, 186, 170, 154, 138, 122, 106,  91,  77,
     64,  52,  41,  32,  24,  18,  13,   9,   5,   3,   2,   1,   0,   0,   0,   0,
};

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
static esp_timer_handle_t s_tmr;
static atomic_bool s_tick_pending;   /* a SYS_TEST_IND is in sys_q */

/* esp_timer task: hop into sys context — every LED write must
 * serialize with test ops / AGC actuation there. */
static void ind_tick_cb(void *arg)
{
    (void)arg;
    if (atomic_exchange(&s_tick_pending, true)) {
        return;                    /* previous tick not consumed yet */
    }
    const sys_msg_t m = { .type = SYS_TEST_IND };
    if (!sys_post(&m)) {
        atomic_store(&s_tick_pending, false);   /* full queue: skip a beat */
    }
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
    esp_timer_start_periodic(s_tmr, IND_TICK_US);
    ind_tick_cb(NULL);             /* first paint ASAP via sys_task */
    ESP_LOGI(TAG, "connect indicator armed (breathing, %d ms)",
             (int)nc_knob_get(KNOB_IND_BREATH_MS));
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
    atomic_store(&s_tick_pending, false);
    if (s_ind == IND_OFF) {
        return;
    }
    const bool conn = ble_is_connected();
    const bool session = sys_app_session();   /* not an idle OS link */
    const int64_t now_us = esp_timer_get_time();

    /* Onboard module LED first — never handed over, tracks link state
     * through every ownership transition below: steady = connected,
     * 1 Hz pulse = powered + advertising. */
    const bool ob_phase = ((now_us / OB_HALF_US) & 1) == 0;
    gpio_set_level(ONBOARD_LED_GPIO,
                   (conn || ob_phase) ? ONBOARD_ON : ONBOARD_OFF);

    /* (a) acquisition running: the AGC owns the optical LEDs. */
    if (acq_ppg_running()) {
        if (s_ind == IND_OWNING) {
            s_ind = IND_RELEASED;
            ESP_LOGI(TAG, "LEDs handed over (acquisition)");
        }
        return;
    }

    if (s_ind == IND_RELEASED) {
        if (session) {
            return;                /* re-arm needs the app session to end */
        }
        s_ind = IND_OWNING;
        ESP_LOGI(TAG, "re-armed (idle + no app session)");
    }

    /* OWNING: always breathing — a steady optical light means SENSING
     * (the AGC owns the LEDs), never just "connected" (user, v0.3.0-31:
     * the old steady-while-connected looked exactly like "worn"). The
     * breath phase comes from the clock, so a skipped tick (full sys_q)
     * only holds one level a little longer. */
    const int64_t period_us = (int64_t)nc_knob_get(KNOB_IND_BREATH_MS) * 1000;
    const uint32_t level = k_breath[((now_us % period_us) * 64) / period_us];
    const uint8_t ir = (uint8_t)((IND_IR_MA * level + 127) / 255);
    const uint8_t red = (uint8_t)((IND_RED_MA * level + 127) / 255);

    /* On a bare module (BENCH) there is no AFE: latch its absence after
     * the first failed bring-up so the optical path goes quiet instead
     * of warning at every tick. The onboard GPIO15 LED above is the
     * whole indicator in that case. A power cycle re-probes (soldering
     * to the mainboard implies one). */
    static bool s_afe_absent;
    if (s_afe_absent) {
        return;
    }
    if (test_ops_ensure_bench_afe() != ESP_OK) {
#if NARBIS_BENCH_BUILD
        s_afe_absent = true;
        ESP_LOGW(TAG, "no AFE (bare module) — onboard LED only");
#else
        ESP_LOGW(TAG, "bench AFE unavailable");
#endif
        return;
    }
    if (afe4404_cur_ir_ma() != ir || afe4404_cur_red_ma() != red) {
        (void)afe4404_set_led_ma(ir, red);
    }
}

#endif /* NARBIS_TEST_MODE */
