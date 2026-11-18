/*
 * power.h — power MECHANISMS (locks, sleep entry, wake decode).
 * Power POLICY (when to transition) lives in sys_task.c.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "narbis/nc_types.h"

esp_err_t power_init(void);   /* pm config (DFS + tickless), wake cause decode */

/* Why did we boot? (wraps esp_sleep_get_wakeup_cause + reset reason) */
/* Call as the VERY FIRST statement of app_main: on an ext1 wake it
 * burst-samples the button for ~3 ms so short taps released before the
 * (150-250 ms later) full recheck still classify as BUTTON, not GHOST. */
void power_early_wake_capture(void);

/* Policy of the previous power_enter_off (RTC-retained): a ghost wake
 * re-sleeps with the SAME battery_forced flag so the 12 h recheck
 * timer of a battery-forced OFF is never dropped. */
bool power_last_off_battery_forced(void);

typedef enum {
    PWR_BOOT_COLD,        /* power-on / flash                          */
    PWR_BOOT_BUTTON,      /* ext1 wake, press confirmed after debounce */
    PWR_BOOT_GHOST,       /* ext1 wake, button NOT held at recheck —
                             caller should go straight back to sleep   */
    PWR_BOOT_TIMER,       /* RTC timer (battery-protection recheck)    */
    PWR_BOOT_OTHER,
} pwr_boot_cause_t;
pwr_boot_cause_t power_boot_cause(void);

/* PM lock management per system state:
 * STREAMING -> NO_LIGHT_SLEEP + CPU_MAX held; USB present -> NO_LIGHT_SLEEP
 * (console liveness; power is free on USB). */
void power_apply_state(nc_sys_state_t state, bool usb_present);

/* Deep-sleep entry, EXACT order (never returns):
 * caller has already stopped acquisition + saved NVS.
 * 1. LIS2DH12 power-down over I2C
 * 2. AFE hardware PWDN: GPIO21 low
 * 3. BLE shutdown
 * 4. gpio_hold_en(21) + gpio_deep_sleep_hold_en()  [21 is not an LP pin]
 * 5. GPIO2: enable + RETAIN internal LP pull-up (no external pull-up on
 *    the button net — ext1 wake never fires without this)
 * 6. ext1 wake on GPIO2 low; optional RTC timer (battery recheck)
 * 7. esp_deep_sleep_start()                                            */
void power_enter_off(bool battery_forced) __attribute__((noreturn));
