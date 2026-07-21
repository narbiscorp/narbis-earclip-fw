/*
 * ble_ota_iface.h — boundary between the BLE layer (ble_gatt.c/ble_tx.c)
 * and the OTA engine. The OTA module registers callbacks here; the BLE
 * layer never interprets OTA payloads.
 *
 * Wire contract (proto.h): OTA_CTRL uses the CONTROL envelope
 * [u8 op][u8 tid][payload] with responses as INDICATIONS on the same
 * characteristic; OTA_DATA is write-no-response [u32 offset][data<=240].
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* OTA_CTRL write received. The OTA module parses the request and builds
 * its own response, submitting it via ble_ota_submit_ind(). */
typedef void (*ble_ota_ctrl_cb_t)(const uint8_t *req, uint16_t len);

/* OTA_DATA write-no-response received ([u32 offset][chunk]). */
typedef void (*ble_ota_data_cb_t)(const uint8_t *data, uint16_t len);

/* Register the OTA engine's receive hooks. Pass NULL to unregister.
 *
 * CONTEXT WARNING: both callbacks run on the NimBLE host task. They must
 * be FAST — memcpy into the OTA engine's own buffer / flash-write queue
 * and return. No flash erase, no blocking queue posts, no I2C. Anything
 * slow stalls every GATT transaction on the link (including the OTA
 * data stream itself). */
void ble_ota_register(ble_ota_ctrl_cb_t ctrl_cb, ble_ota_data_cb_t data_cb);

/* Submit an OTA_CTRL response indication. Copies pkt into the dedicated
 * 2-slot OTA staging ring in ble_tx (drained right after BLE_CH_CTRL_IND,
 * ahead of all streams); never blocks. False = len invalid, no central
 * connected, or OTA_CTRL indications not subscribed (packet discarded).
 * len <= 244 (NC_ATT_PAYLOAD_MAX). */
bool ble_ota_submit_ind(const uint8_t *pkt, uint16_t len);
