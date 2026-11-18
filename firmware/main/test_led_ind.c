/*
 * test_led_ind.c — the red-LED indicator. See test_led_ind.h for what it
 * shows; this file is empty in the gated profile.
 *
 * Design notes:
 *  - The periodic timer runs for the life of the boot (~33 ticks/s for
 *    the fades); ticks landing while RELEASED just re-check the re-arm
 *    condition. At most ONE tick is ever queued (pending flag), so the
 *    indicator can never crowd AGC / wear / CONTROL traffic out of
 *    sys_q; a busy sys_task just repaints less often.
 *  - The AFE is brought up on demand (ind_afe_up): TEST builds share the
 *    test ops' bench bring-up, production inits it directly at 100 sps.
 *    The desired-vs-actual compare uses the driver's own post-clamp
 *    caches (afe4404_cur_*_ma), which an afe4404_init zeroes — a re-init
 *    therefore repaints automatically instead of trusting a stale shadow.
 *  - Re-arm needs BOTH "acquisition stopped" and "no app session": an
 *    operator's manual LED setup must survive until their session ends.
 *  - The charge fade keeps its own phase accumulator, so the period can
 *    change with the battery level without the fade jumping.
 */
#include "test_led_ind.h"

#if !NARBIS_GATED_PROFILE

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

#define IND_RED_MA     20          /* breathing: 50 % of LED_RED_MAX_MA (40) */
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
 * LOW). TEST builds: power/link state for the vendor bench. Production:
 * held dark. */
#define ONBOARD_LED_GPIO   15
#define ONBOARD_ON         0       /* active low */
#define ONBOARD_OFF        1

typedef enum { IND_OFF = 0, IND_OWNING, IND_RELEASED } ind_state_t;
typedef enum { LOOK_NONE = 0, LOOK_BREATH, LOOK_CHARGING, LOOK_FULL } ind_look_t;

static volatile ind_state_t s_ind = IND_OFF;
static esp_timer_handle_t s_tmr;
static atomic_bool s_tick_pending;   /* a SYS_TEST_IND is in sys_q */

static ind_look_t s_look;
static bool s_width_trimmed;         /* red drive window shortened by us */
static uint32_t s_phase_q16;         /* charge fade: 65536 = one cycle */
static int64_t s_phase_t_us;

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

static esp_err_t ind_afe_up(void)
{
#if NARBIS_TEST_MODE
    return test_ops_ensure_bench_afe();   /* shares the bench bring-up */
#else
    if (acq_ppg_running()) {
        return ESP_OK;             /* pipeline owns a live AFE */
    }
    return afe4404_is_up() ? ESP_OK : afe4404_init(NC_RATE_100);
#endif
}

/* Full red window again (the breathing and the solid "charged" light use
 * the table's width; only the charge fade trims it). */
static void width_restore(void)
{
    if (s_width_trimmed && afe4404_is_up()) {
        (void)afe4404_set_red_drive_q16(0xFFFF);
    }
    s_width_trimmed = false;
}

/* k_breath, linearly interpolated: Q16 level for a Q16 phase. */
static uint32_t breath_q16(uint32_t phase_q16)
{
    const uint32_t pos = phase_q16 & 0xFFFFu;
    const uint32_t i = pos >> 10, f = pos & 1023u;
    const uint32_t a = k_breath[i], b = k_breath[(i + 1) & 63u];
    return ((a * (1024u - f) + b * f) * 257u) / 1024u;   /* 255 -> 65535 */
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
    ESP_LOGI(TAG, "red indicator armed (breath %d ms, charge %d mA %d..%d ms)",
             (int)nc_knob_get(KNOB_IND_BREATH_MS),
             (int)nc_knob_get(KNOB_IND_CHG_MA),
             (int)nc_knob_get(KNOB_IND_CHG_SLOW_MS),
             (int)nc_knob_get(KNOB_IND_CHG_FAST_MS));
}

void test_led_ind_release(void)
{
    if (s_ind == IND_OWNING) {
        width_restore();
        s_look = LOOK_NONE;
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
    const bool session = sys_app_session();   /* not an idle OS link */
    const int64_t now_us = esp_timer_get_time();

#if NARBIS_TEST_MODE
    /* Onboard module LED first — never handed over, tracks link state
     * through every ownership transition below: steady = connected,
     * 1 Hz pulse = powered + advertising. */
    {
        const bool conn = ble_is_connected();
        const bool ob_phase = ((now_us / OB_HALF_US) & 1) == 0;
        gpio_set_level(ONBOARD_LED_GPIO,
                       (conn || ob_phase) ? ONBOARD_ON : ONBOARD_OFF);
    }
#endif

    /* (a) acquisition running: the AGC owns the optical LEDs (its
     * afe4404_init restored the red window). */
    if (acq_ppg_running()) {
        if (s_ind == IND_OWNING) {
            s_ind = IND_RELEASED;
            s_width_trimmed = false;
            s_look = LOOK_NONE;
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

    /* On a bare module (BENCH) there is no AFE: latch its absence after
     * the first failed bring-up so the optical path goes quiet instead
     * of warning at every tick. The onboard GPIO15 LED above is the
     * whole indicator in that case. A power cycle re-probes (soldering
     * to the mainboard implies one). */
    static bool s_afe_absent;
    if (s_afe_absent) {
        return;
    }
    if (ind_afe_up() != ESP_OK) {
#if NARBIS_BENCH_BUILD
        s_afe_absent = true;
        ESP_LOGW(TAG, "no AFE (bare module) — onboard LED only");
#else
        ESP_LOGW(TAG, "indicator AFE unavailable");
#endif
        return;
    }

    uint8_t pct = 0;
    const nc_charger_state_t chg = sys_charger_state(&pct);
    const ind_look_t look = (chg == NC_CHG_CHARGING) ? LOOK_CHARGING
                          : (chg == NC_CHG_COMPLETE) ? LOOK_FULL
                                                     : LOOK_BREATH;
    if (look != s_look) {
        ESP_LOGI(TAG, "%s", look == LOOK_CHARGING ? "charging: fade"
                            : look == LOOK_FULL   ? "charged: solid"
                                                  : "on battery: breathe");
        s_look = look;
        s_phase_q16 = 0;           /* every fade starts from dark */
        s_phase_t_us = now_us;
    }

    if (look == LOOK_BREATH) {
        /* The breath phase comes from the clock, so a skipped tick (full
         * sys_q) only holds one level a little longer. */
        width_restore();
        const int64_t period_us = (int64_t)nc_knob_get(KNOB_IND_BREATH_MS) * 1000;
        const uint32_t level = k_breath[((now_us % period_us) * 64) / period_us];
        const uint8_t red = (uint8_t)((IND_RED_MA * level + 127) / 255);
        if (afe4404_cur_ir_ma() != 0 || afe4404_cur_red_ma() != red) {
            (void)afe4404_set_led_ma(0, red);
        }
        return;
    }

    const uint8_t ma = (uint8_t)nc_knob_get(KNOB_IND_CHG_MA);
    if (afe4404_cur_ir_ma() != 0 || afe4404_cur_red_ma() != ma) {
        (void)afe4404_set_led_ma(0, ma);
    }
    if (look == LOOK_FULL) {
        width_restore();           /* solid at ind_chg_ma */
        return;
    }

    /* Charging: period from the battery level, slow when empty, faster as
     * it fills. The phase advances by elapsed time (capped, so a long
     * sys_task stall does not skip half a breath). */
    const int64_t slow_us = (int64_t)nc_knob_get(KNOB_IND_CHG_SLOW_MS) * 1000;
    const int64_t fast_us = (int64_t)nc_knob_get(KNOB_IND_CHG_FAST_MS) * 1000;
    const int64_t p = (pct > 100) ? 100 : pct;
    int64_t period_us = slow_us - ((slow_us - fast_us) * p) / 100;
    if (period_us < 100000) {
        period_us = 100000;
    }
    int64_t dt = now_us - s_phase_t_us;
    s_phase_t_us = now_us;
    if (dt < 0) {
        dt = 0;
    } else if (dt > 200000) {
        dt = 200000;
    }
    s_phase_q16 += (uint32_t)((dt * 65536) / period_us);
    const uint32_t q = breath_q16(s_phase_q16);
    if (afe4404_set_red_drive_q16((uint16_t)(q > 0xFFFFu ? 0xFFFFu : q)) == ESP_OK) {
        s_width_trimmed = true;
    }
}

#endif /* !NARBIS_GATED_PROFILE */
