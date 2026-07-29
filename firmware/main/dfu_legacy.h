/*
 * dfu_legacy.h — legacy Narbis DFU (Edge-glasses wire protocol) served
 * device-side so the existing OTA hub / web updater can flash earclips.
 *
 * Wire contract: docs/legacy_dfu_protocol.md (service 0x00FF, chars
 * FF01 CONTROL / FF02 DATA / FF03 STATUS; opcodes 0xA8 BEGIN, 0xAD
 * PAGE_CONFIRM, 0xA9 FINISH, 0xAA CANCEL; 4096-byte pages accumulated
 * by offset with a CRC32 handshake per full page).
 *
 * Coexistence with the modern engine (ota.c): the two are mutually
 * exclusive — a legacy BEGIN answers ERROR 0x08 while ota_active(),
 * and the modern NC_OTA_BEGIN answers NC_ST_BUSY while
 * dfu_legacy_active() (checked in ota.c).
 *
 * Threading: every handler (GATT access callbacks + the disconnect
 * hook) runs on the NimBLE host task, so the engine needs no locking
 * of its own. Only the 500 ms restart timer runs elsewhere (esp_timer
 * task) and it does nothing but esp_restart().
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/* Register the 0x00FF service (own svc-def table, bleprph service-module
 * idiom: ble_gatts_count_cfg + ble_gatts_add_svcs). Call from
 * ble_iface_init after the main table's ble_gatts_add_svcs and before
 * the host starts. encrypted_only mirrors ble_gatt.c's chr_harden
 * policy: production builds get *_ENC attribute flags, TEST builds /
 * open_pairing leave the service open (the legacy hub never pairs). */
esp_err_t dfu_legacy_gatt_register(bool encrypted_only);

/* NimBLE host task, from ble_gatt.c's BLE_GAP_EVENT_DISCONNECT path:
 * abort a mid-flight session silently (protocol §3: a disconnect kills
 * the session; the central retries BEGIN after reconnecting). */
void dfu_legacy_on_disconnect(void);

/* True while a legacy session is open (BEGIN accepted, not yet
 * finished/cancelled/failed). Read by ota.c (mutual exclusion) and by
 * sys_task (holds NC_STATE_OTA so acquisition stays stopped). */
bool dfu_legacy_active(void);

/* Shared §7 image-identity check (both OTA directions use it):
 * hdr = first bytes of the image; needs at least 0x5E bytes.
 *   [0x00] u8    ESP image magic          == 0xE9
 *   [0x0C] u16LE chip_id                  == 0x000D (ESP32-C6)
 *   [0x20] u32LE esp_app_desc magic       == 0xABCD5432
 *   [0x50] char  project_name             starts "narbis_earclip"
 */
bool dfu_image_hdr_ok(const uint8_t *hdr, size_t len);
