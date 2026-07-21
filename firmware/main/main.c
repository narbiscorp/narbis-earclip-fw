/*
 * main.c — boot orchestration (handoff §3 + §8, Addendum 1).
 *
 * Boot decision tree (power_boot_cause decodes the wake):
 *   GHOST  — ext1 fired but the button is not held after a 30 ms
 *            recheck (TVS/C22 ringing): straight back to deep sleep,
 *            no BLE, no sensor init.
 *   TIMER  — 12 h battery-protection recheck: measure the cell; still
 *            below vbatt_off -> sleep another 12 h; recovered -> fall
 *            through to a full boot.
 *   BUTTON/COLD/OTHER — full boot.
 *
 * Full-boot order matters:
 *   1. NVS (knobs + bonds live there; NimBLE needs it),
 *   2. knob registry -> NVS overlay (everything after reads live knobs),
 *   3. diag (logs reset reason before anything can fail),
 *   4. power (DFS config + wake decode; must precede driver GPIO work
 *      so the deep-sleep pad holds are released first),
 *   5. sys_q, then buses/monitors, then acq_init (which spawns the sys
 *      task LAST among its tasks — mailbox must exist first),
 *   6. BLE, console, OTA engine; ota_boot_validate runs after all of
 *      the above came up, so a broken OTA image that dies in init
 *      never marks itself valid and auto-rolls back.
 */
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_app_desc.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "acq.h"
#include "app_msgs.h"
#include "battery.h"
#include "ble_iface.h"
#include "board.h"
#include "button.h"
#include "charger.h"
#include "console.h"
#include "diag.h"
#include "i2c_bus.h"
#include "knobs_nvs.h"
#include "ota.h"
#include "power.h"
#include "sensor_types.h"

#include "narbis/nc_knobs.h"

static const char *TAG = "main";

/* power.c teardown guards (see their definitions for semantics). */
extern bool power_sensors_started;
extern bool power_ble_started;
/* sys_task.c: LOWBATT boot-refusal flag (set pre-scheduling). */
extern void sys_task_set_boot_lowbatt(void);

static const char *boot_cause_str(pwr_boot_cause_t c)
{
    switch (c) {
    case PWR_BOOT_COLD:   return "cold";
    case PWR_BOOT_BUTTON: return "button";
    case PWR_BOOT_GHOST:  return "ghost";
    case PWR_BOOT_TIMER:  return "timer";
    default:              return "other";
    }
}

void app_main(void)
{
    /* NVS with the standard erase-retry idiom (page format changes /
     * truncated partitions after reflash). */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nc_knobs_init();          /* defaults ... */
    knobs_nvs_load();         /* ... overlaid with persisted deltas */
    diag_init();
    ESP_ERROR_CHECK(power_init());

    const pwr_boot_cause_t cause = power_boot_cause();
    bool batt_ready = false;

    switch (cause) {
    case PWR_BOOT_GHOST:
        /* TVS/cap ringing, not a person. No BLE, no sensors. */
        ESP_LOGW(TAG, "ghost ext1 wake — returning to sleep");
        power_enter_off(false);   /* noreturn */
        break;

    case PWR_BOOT_TIMER: {
        /* Battery-protection recheck: the ONLY question is whether the
         * cell recovered above vbatt_off. */
        ESP_ERROR_CHECK(battery_init());
        batt_ready = true;
        uint16_t mv = 0;
        battery_read_mv(&mv, NULL);
        const uint16_t off_mv = (uint16_t)nc_knob_get(KNOB_VBATT_OFF_MV);
        ESP_LOGI(TAG, "timer recheck: %u mV (off threshold %u)", mv, off_mv);
        if (mv < off_mv) {
            power_enter_off(true);   /* noreturn — next recheck in 12 h */
        }
        break;   /* recovered — continue into a full boot */
    }

    default:
        break;   /* BUTTON / COLD / OTHER — full boot */
    }

    /* --------------------------- full boot --------------------------- */

    /* sys_q (and every other queue) is created inside acq_init before
     * it spawns any task; BLE/button/console producers all init after
     * acq_init, so no post can race the handle. */
    ESP_ERROR_CHECK(i2c_bus_init());
    if (!batt_ready) {
        ESP_ERROR_CHECK(battery_init());
    }
    ESP_ERROR_CHECK(charger_init());

    /* LOWBATT boot refusal (handoff §3): below vbatt_off on battery we
     * still bring BLE up so a host can see WHY (STATUS state LOWBATT,
     * 10 s advertise handled by sys_task), then sleep with the 12 h
     * recheck. On USB the cell is charging — no refusal. */
    {
        bool vusb = false;
        (void)charger_poll(&vusb);
        uint16_t mv = 0;
        battery_read_mv(&mv, NULL);
        if (!vusb && mv < (uint16_t)nc_knob_get(KNOB_VBATT_OFF_MV)) {
            ESP_LOGW(TAG, "boot at %u mV — battery-critical mode", mv);
            sys_task_set_boot_lowbatt();
        }
    }

    ESP_ERROR_CHECK(acq_init());        /* queues + afe/imu/dsp/sys tasks */
    power_sensors_started = true;       /* power_enter_off may now touch I2C */

    ESP_ERROR_CHECK(button_init());

    ESP_ERROR_CHECK(ble_iface_init());
    power_ble_started = true;           /* power_enter_off calls ble_shutdown */

    console_init();
    ota_engine_init();
    ota_boot_validate();                /* mark-valid / rollback decision */

    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG,
             "Narbis Edge Earclip fw %s | hw V2.1 | reset: %s | boot: %s | "
             "test_mode=%d",
             app->version, diag_reset_reason_str(), boot_cause_str(cause),
             NARBIS_TEST_MODE);
}
