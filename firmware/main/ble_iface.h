/*
 * ble_iface.h — boundary between the BLE stack (ble_gatt.c/ble_tx.c) and
 * the rest of the firmware. Producers never call NimBLE directly.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* Drain priority order (ble_tx services lower enum values first):
 * control responses and events are small and semantically critical;
 * raw streams are seq-numbered so loss is recoverable. All channels
 * are notifications — indications are unusable in this peripheral-only
 * build (BLE_GATTC=0: acks compiled out, procs leak; see ble_gatt.c). */
typedef enum {
    BLE_CH_CTRL_RESP = 0,  /* CONTROL response (notification) */
    BLE_CH_EVENT,
    BLE_CH_IBI,
    BLE_CH_STATUS,
    BLE_CH_PPG,
    BLE_CH_ACCEL,
    BLE_CH_COUNT
} ble_chan_t;

esp_err_t ble_iface_init(void);   /* NimBLE up, GATT registered, advertising */
void      ble_shutdown(void);     /* orderly stop for deep-sleep entry       */

/* Copies pkt into the static staging pool; never blocks. On a full ring
 * the OLDEST staged packet of that channel is dropped (g_diag.notify_drop
 * incremented) and the new one staged. False = not connected/nothing
 * subscribed (packet discarded silently). */
bool ble_tx_submit(ble_chan_t ch, const uint8_t *pkt, uint16_t len);

bool ble_is_connected(void);
void ble_get_conn_stats(uint16_t *mtu, uint8_t *phy, uint16_t *interval_1_25ms);

/* Current ATT notification payload ceiling = min(negotiated MTU - 3,
 * NC_ATT_PAYLOAD_MAX). Producers of large stream packets (PPG/ACCEL
 * batchers) clamp their fill to this after every batch reset — the
 * stack silently truncates oversized notifications. */
uint16_t ble_att_payload_budget(void);

/* Bonding window: advertise connectable+bondable and accept a new bond
 * for the given duration (button double-press or open_pairing knob). */
void ble_open_pairing_window(uint32_t seconds);

void ble_update_battery(uint8_t pct);
/* HRS 0x2A37: flags+bpm+RR intervals (1/1024 s units). n_rr <= 7. */
void ble_notify_hrs(uint8_t bpm, const uint16_t *rr_1024, int n_rr);

/* ------------------------------------------------------------------ */
/* Appended (BLE layer, 2026-07): connection-interval control          */
/* ------------------------------------------------------------------ */
/* fast=true: 7.5-15 ms (streaming); fast=false: 30-50 ms (idle).
 * Called by sys_task on stream start/stop. Remembered and re-applied
 * on the next connection if called while disconnected. */
void ble_request_conn_speed(bool fast);

/* ------------------------------------------------------------------ */
/* Appended: PRIVATE surface between ble_gatt.c and ble_tx.c. Nothing  */
/* outside those two files may call these.                             */
/* ------------------------------------------------------------------ */
/* Extra ble_tx staging channel for OTA_CTRL responses (not part of
 * the public producer enum; drained right after BLE_CH_CTRL_RESP). */
#define BLE_TX_CH_OTA_RESP ((int)BLE_CH_COUNT)
#define BLE_TX_CH_TOTAL    ((int)BLE_CH_COUNT + 1)

/* True iff a central is connected, the channel's CCCD is enabled and —
 * for these all-custom channels — the encrypted-link gate passes.
 * Out-params (any may be NULL): conn handle, char value handle. */
bool ble_gatt_tx_chan_ready(int tx_ch, uint16_t *conn,
                            uint16_t *val_handle);

/* ble_tx pushes every STATUS submission here so STATUS reads return
 * the latest snapshot even between notifications. */
void ble_gatt_status_cache_set(const uint8_t *pkt, uint16_t len);

/* Battery 0x2A19 read-value cache + notify-readiness (std service:
 * never gated on encryption). Same pattern for HRS 0x2A37. */
void ble_gatt_set_battery(uint8_t pct);
bool ble_gatt_batt_notify_ready(uint16_t *conn, uint16_t *val_handle);
bool ble_gatt_hrs_notify_ready(uint16_t *conn, uint16_t *val_handle);

esp_err_t ble_tx_start(void);    /* create notifier task (after GATT reg) */
void ble_tx_flush_all(void);     /* drop all staged packets (disconnect)  */
void ble_tx_on_notify_tx(void);  /* BLE_GAP_EVENT_NOTIFY_TX -> unblock tx */
