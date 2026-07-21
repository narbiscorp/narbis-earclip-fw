/*
 * ota.h — BLE OTA engine (esp_ota A/B) per proto.h OTA section.
 *
 * Transport: ble_ota_iface.h (OTA_CTRL writes -> control cb, OTA_DATA
 * write-no-response -> data cb, responses/progress via
 * ble_ota_submit_ind). Both callbacks run in the NimBLE host task; the
 * engine is internally mutexed, so sys_task may call ota_active()/
 * ota_deadline_check() concurrently.
 */
#pragma once
#include <stdbool.h>
#include "esp_err.h"

/* Create the engine (mutex + timers) and register the OTA GATT
 * callbacks via ble_ota_register(). Call once at boot, after BLE init. */
esp_err_t ota_engine_init(void);

/* Post-OTA rollback gate. If the running image is PENDING_VERIFY, arms
 * a 10 s one-shot self-check (NVS mounted, WHO_AM_I, AFE bring-up, heap
 * sanity — surviving 10 s of BLE+WDT is part of the test) that ends in
 * esp_ota_mark_app_valid_cancel_rollback() or
 * esp_ota_mark_app_invalid_rollback_and_reboot(). Call once at boot. */
void ota_boot_validate(void);

/* True while a transfer is in flight or validated-awaiting-restart
 * (RECEIVING/VALIDATING/READY). sys_task uses it to refuse sleep. */
bool ota_active(void);

/* Call at 1 Hz from sys_task: RECEIVING with no OTA_DATA for 60 s ->
 * esp_ota_abort + FAILED (also bounds the resume window: a BEGIN with
 * the same {size,crc} resumes only while the session is still open). */
void ota_deadline_check(void);

/* Post-OTA hardware self-check: the 10 s esp_timer only marks it due;
 * sys_task calls this at 1 Hz and the probes (I2C/GPIO) run in sys
 * context, serialized against acquisition start/stop. No-op unless a
 * PENDING_VERIFY image armed it. May not return (rollback reboots). */
void ota_self_check_poll(void);
