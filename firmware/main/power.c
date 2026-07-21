/*
 * power.c — power MECHANISMS: DFS/tickless config, pm locks, deep-sleep
 * entry, wake decode. Policy (when to transition) lives in sys_task.c.
 *
 * Deep-sleep facts this file encodes (board.h / Addendum 1):
 *  - GPIO21 (AFE RESETZ) is NOT an LP pin. It must be driven low and
 *    HELD (gpio_hold_en + gpio_deep_sleep_hold_en) across deep sleep,
 *    or R1 (100 k pull-up on V2.1) re-enables the AFE. Hold-low burns
 *    33 uA through R1 — accepted, part of the <= 80 uA OFF floor.
 *  - GPIO2 (button) has NO external pull-up. The internal LP pull-up
 *    must be enabled AND survive the sleep or ext1 never fires.
 *    Mechanism chosen for esp32c6 @ IDF v5.5.1 (verified in-tree):
 *    rtc_gpio_pullup_en() programs the LP_IO pad pull (rtc_io.h is
 *    available on C6: SOC_RTCIO_INPUT_OUTPUT_SUPPORTED=1), and
 *    esp_deep_sleep_start()'s ext1_wakeup_prepare() (sleep_modes.c)
 *    routes the pad to the LP mux and — because we leave the
 *    RTC_PERIPH domain in its default AUTO(off) state — latches the
 *    pad with the RTCIO HOLD feature, which retains the pull-up at
 *    lower power than forcing ESP_PD_DOMAIN_RTC_PERIPH on. This is the
 *    recipe of the IDF deep_sleep example (ext_wakeup.c, C6 branch).
 *
 * Brownout: no custom handler. The LED-kill requirement ("brownout
 * turns LED currents off first") is satisfied in hardware: a collapsing
 * 3.3 V rail is TX_SUP itself, and the C6 brownout ISR on IDF v5.5 is
 * internal (bootloader + esp_system register it; there is no public
 * hook on the C6 port). The ROM/bootloader BOD resets the chip; the
 * AFE loses its registers (LEDs off) at the same time.
 */
#include "power.h"

#include "freertos/FreeRTOS.h"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"

#include "ble_iface.h"
#include "board.h"
#include "lis2dh12.h"

static const char *TAG = "power";

/* Build-time escape hatch: if NimBLE + auto light sleep misbehave on
 * bench (missed connection events, scheduling overruns), flip to 0 to
 * ship DFS-only while it is investigated. Test-mode builds always run
 * without light sleep so USB console + timing measurements stay live. */
#define POWER_ALLOW_LIGHT_SLEEP 1

/* Cross-boot-path guards, set by main.c as subsystems come up. The
 * ghost-wake and timer-wake paths call power_enter_off() before the
 * sensor stack / BLE ever initialize; these gate the teardown calls
 * that would touch uninitialized drivers. */
bool power_sensors_started = false;
bool power_ble_started = false;

static esp_pm_lock_handle_t s_lock_no_ls;   /* ESP_PM_NO_LIGHT_SLEEP */
static esp_pm_lock_handle_t s_lock_cpu_max; /* ESP_PM_CPU_FREQ_MAX   */
static bool s_held_no_ls, s_held_cpu_max;

/* Early ext1 evidence captured at app_main entry (short-tap catch). */
static bool s_early_saw_low;

/* Why the last OFF happened — survives deep sleep in LP SRAM so a
 * ghost re-sleep keeps the battery-forced 12 h recheck policy. */
static RTC_DATA_ATTR bool s_off_batt_forced;

esp_err_t power_init(void)
{
    /* 160 -> 80 MHz DFS. 80 is a valid C6 DFS point (PLL 480/6 —
     * rtc_clk_cpu_freq_mhz_to_config, esp32c6/rtc_clk.c); it keeps the
     * PLL up so NimBLE's own pm locks see cheap freq transitions.
     * 40 (XTAL) is also legal and would save a little more in APB_MIN —
     * revisit with bench numbers if IDLE misses its 1.5 mA target. */
    const esp_pm_config_t pm = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 80,
#if POWER_ALLOW_LIGHT_SLEEP && !NARBIS_TEST_MODE
        .light_sleep_enable = true,
#else
        .light_sleep_enable = false,
#endif
    };
    esp_err_t err = esp_pm_configure(&pm);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_pm_configure: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "nb_no_ls", &s_lock_no_ls);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "nb_cpu_max", &s_lock_cpu_max);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG, "DFS 160/80 MHz, light sleep %s",
             pm.light_sleep_enable ? "enabled" : "disabled");
    return ESP_OK;
}

pwr_boot_cause_t power_boot_cause(void)
{
    const esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
    if (wc == ESP_SLEEP_WAKEUP_UNDEFINED) {
        return PWR_BOOT_COLD;   /* power-on / flash / panic reboot */
    }

    /* --- any deep-sleep wake: release the sleep holds BEFORE drivers
     * init (they must be free to reconfigure the pads). ------------- */

    /* GPIO21 first: latch an output-low configuration while the pad is
     * still held, so the release hands over seamlessly and the AFE
     * stays in hardware PWDN (no pull-up glitch through R1). The AFE
     * driver runs its own rev-proof RESETZ sequence later. */
    const gpio_config_t afe_rst = {
        .pin_bit_mask = 1ULL << PIN_AFE_RESET,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&afe_rst);
    gpio_set_level(PIN_AFE_RESET, 0);
    gpio_hold_dis(PIN_AFE_RESET);
#if SOC_GPIO_SUPPORT_HOLD_IO_IN_DSLP && !SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP
    /* Older chips (ESP32/S2/S3/C2/C3) need the global digital-pad
     * deep-sleep hold on top of gpio_hold_en. The C6 holds single IOs
     * through deep sleep directly (SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP),
     * so this API does not exist there. */
    gpio_deep_sleep_hold_dis();
#endif

    /* GPIO2: ext1 prep routed it to the LP mux and HOLD-latched it
     * (sleep_modes.c ext1_wakeup_prepare, RTC_PERIPH auto-off branch).
     * Release + route back to the digital matrix for the button driver. */
    rtc_gpio_hold_dis(PIN_BUTTON);
    rtc_gpio_deinit(PIN_BUTTON);

    if (wc == ESP_SLEEP_WAKEUP_TIMER) {
        return PWR_BOOT_TIMER;   /* battery-protection recheck */
    }
    if (wc != ESP_SLEEP_WAKEUP_EXT1) {
        ESP_LOGW(TAG, "unexpected wakeup cause %d", (int)wc);
        return PWR_BOOT_OTHER;
    }

    /* EXT1: confirm a human press. TVS/C22 ringing on the button net
     * can fire ext1 without a real press (ghost). Two chances to prove
     * a human: (a) the early capture taken at the very top of app_main
     * (catches short taps released before drivers init — boot latency
     * is ~150-250 ms and a quick tap is 50-300 ms); (b) a re-sample
     * over BUTTON_DEBOUNCE_MS now: >= 80 % low AND still low at the
     * end. Ghost only when NEITHER saw the button. Residual limit:
     * taps shorter than the boot latency that ALSO missed the early
     * capture are undetectable without an RTC wake stub — bench item. */
    const gpio_config_t btn = {
        .pin_bit_mask = 1ULL << PIN_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn);

    int low = 0;
    for (int i = 0; i < BUTTON_DEBOUNCE_MS; i++) {
        esp_rom_delay_us(1000);
        low += (gpio_get_level(PIN_BUTTON) == 0);
    }
    const bool held = (low >= (BUTTON_DEBOUNCE_MS * 8) / 10) &&
                      (gpio_get_level(PIN_BUTTON) == 0);
    ESP_LOGI(TAG, "ext1 wake: early=%d, %d/%d samples low -> %s",
             (int)s_early_saw_low, low, BUTTON_DEBOUNCE_MS,
             (held || s_early_saw_low) ? "button" : "ghost");
    return (held || s_early_saw_low) ? PWR_BOOT_BUTTON : PWR_BOOT_GHOST;
}

void power_early_wake_capture(void)
{
    /* FIRST statement of app_main. On an ext1 wake the pad still
     * carries the HOLD-latched LP pull-up, so a plain level read works
     * before any driver init (digital IO mux reset state reads the pad
     * through the input path; no gpio_config needed for a read). 64
     * samples / 50 us apart ≈ 3.2 ms: TVS/C22 ringing decays in well
     * under 1 ms, so >= 60 % low means a finger, not a transient. */
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT1) {
        return;
    }
    int low = 0;
    for (int i = 0; i < 64; i++) {
        low += (gpio_get_level(PIN_BUTTON) == 0);
        esp_rom_delay_us(50);
    }
    s_early_saw_low = (low >= 38);   /* 60 % of 64 */
}

bool power_last_off_battery_forced(void)
{
    return s_off_batt_forced;
}

void power_apply_state(nc_sys_state_t state, bool usb_present)
{
    if (s_lock_no_ls == NULL || s_lock_cpu_max == NULL) {
        return;   /* power_init not run / failed — locks are policy-only */
    }

    /* STREAMING holds both locks per the header contract. OTA and
     * SELFTEST are folded into the same rule: OTA wants flash-write +
     * BLE throughput, SELFTEST runs timed AFE captures that must not be
     * stretched by sleep entry/exit latency. USB present holds
     * NO_LIGHT_SLEEP alone (console liveness; power is free on USB). */
    const bool busy = (state == NC_STATE_STREAMING) ||
                      (state == NC_STATE_OTA) ||
                      (state == NC_STATE_SELFTEST);
    const bool want_no_ls = busy || usb_present;
    const bool want_cpu = busy;

    /* pm locks are recursive — track our own held bits so each lock is
     * acquired exactly once regardless of transition order. */
    if (want_no_ls != s_held_no_ls) {
        (want_no_ls ? esp_pm_lock_acquire : esp_pm_lock_release)(s_lock_no_ls);
        s_held_no_ls = want_no_ls;
    }
    if (want_cpu != s_held_cpu_max) {
        (want_cpu ? esp_pm_lock_acquire : esp_pm_lock_release)(s_lock_cpu_max);
        s_held_cpu_max = want_cpu;
    }
}

void power_enter_off(bool battery_forced)
{
    ESP_LOGI(TAG, "OFF (battery_forced=%d)", (int)battery_forced);

    /* 1. LIS2DH12 -> power-down (0.5 uA) while I2C still works. Skipped
     *    on boot paths where the sensor stack never came up. */
    if (power_sensors_started) {
        esp_err_t err = lis2dh12_powerdown();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "lis2dh12_powerdown: %s", esp_err_to_name(err));
        }
    }

    /* 2. AFE hardware PWDN: RESETZ low > 200 us (~8 uA, all registers
     *    lost — by design, wake does a full reprogram). Driven directly
     *    rather than through afe4404.c so this also works on the ghost/
     *    timer paths where the driver never initialized. */
    const gpio_config_t afe_rst = {
        .pin_bit_mask = 1ULL << PIN_AFE_RESET,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&afe_rst);
    gpio_set_level(PIN_AFE_RESET, 0);
    esp_rom_delay_us(AFE_T_PWDN_MIN_US);

    /* 3. BLE down before the radio domain powers off. */
    if (power_ble_started) {
        ble_shutdown();
    }

    /* 4. GPIO21 is not an LP pin: pad hold keeps RESETZ clamped low
     *    against R1 through the sleep. On the C6, gpio_hold_en alone
     *    persists through deep sleep (SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP);
     *    the global gpio_deep_sleep_hold_en only exists on older chips.
     *    BENCH: verify RESETZ stays low across sleep (README checklist). */
    gpio_hold_en(PIN_AFE_RESET);
#if SOC_GPIO_SUPPORT_HOLD_IO_IN_DSLP && !SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP
    gpio_deep_sleep_hold_en();
#endif

    /* 5. CRITICAL (Addendum 1 item 3): the button net has no external
     *    pull-up. Program the LP_IO pull-up on GPIO2; with RTC_PERIPH
     *    left AUTO(off), sleep entry HOLD-latches the pad so the pull
     *    survives — see file header for the verified v5.5.1 mechanism. */
    rtc_gpio_pulldown_dis(PIN_BUTTON);
    rtc_gpio_pullup_en(PIN_BUTTON);

    /* 5b. Disarm any stale light-sleep GPIO wake left by button.c
     *     (gpio_wakeup_enable): a latched RTC_GPIO_TRIG_EN would decode
     *     the wake as WAKEUP_GPIO instead of EXT1 (bypassing the ghost
     *     check) AND force RTC_PERIPH on, breaking the HOLD-latch
     *     low-power recipe. Must precede esp_deep_sleep_start. */
    gpio_wakeup_disable(PIN_BUTTON);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);

    /* Remember why we slept: a ghost wake must re-sleep with the SAME
     * policy, or a battery-forced OFF loses its 12 h recheck forever. */
    s_off_batt_forced = battery_forced;

    /* 6. Wake sources: button low via ext1 (GPIO2 is LP IO 0..7);
     *    battery-forced OFF adds a 12 h recheck so a recovering cell is
     *    noticed (charging cannot wake us — VUSB is not an LP pin). */
    esp_err_t err = esp_sleep_enable_ext1_wakeup_io(1ULL << PIN_BUTTON,
                                                    ESP_EXT1_WAKEUP_ANY_LOW);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ext1 arm failed: %s", esp_err_to_name(err));
    }
    if (battery_forced) {
        esp_sleep_enable_timer_wakeup(12ULL * 3600ULL * 1000000ULL);
    }

    /* 7. */
    esp_deep_sleep_start();
    while (true) { }   /* not reached — esp_deep_sleep_start is noreturn */
}
