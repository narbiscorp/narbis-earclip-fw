/*
 * ble_tx.c — the single notifier task (handoff §5.4).
 *
 * Every outbound notification funnels through per-channel staging
 * rings here; producers (sys/dsp/afe/imu tasks) only ever memcpy into
 * a ring under a short critical section and never touch NimBLE. One
 * task (prio 15) drains the rings in priority order:
 *
 *   CTRL_RESP, OTA_RESP, EVENT, IBI, STATUS, PPG, ACCEL
 *
 * Everything is a NOTIFICATION — including both response channels.
 * Indications are banned in this peripheral-only build: BLE_GATTC=0
 * compiles out NimBLE's CFM/timeout handlers, so an indication's ack
 * never surfaces and each ble_gatts_indicate_custom() leaks one of
 * the CONFIG_BT_NIMBLE_GATT_MAX_PROCS (4) procs; after four sends
 * every send ENOMEMs and the drain livelocks (V2.1 first functional
 * test, 2026-07-31: one control response per 35 s, then total TX
 * starvation). Response delivery assurance is protocol-level (tid
 * echo + client retry), not transport-level.
 *
 * Flow control:
 *  - BLE_HS_ENOMEM/EAGAIN/EBUSY/EALREADY from a send are transient
 *    (msys mbuf pool exhausted): the packet goes back to the head of
 *    its ring, we wait on the binary semaphore that
 *    ble_tx_on_notify_tx() gives (every NOTIFY_TX event frees
 *    resources), and retry; after two failed retries the drain pass
 *    ends and the outer 100 ms wait takes over.
 *  - Other send errors drop the packet (g_diag.notify_drop).
 *
 * Ring policy: on a full ring the OLDEST staged packet is dropped
 * (counted, except STATUS which is a 1-slot latest-wins snapshot).
 * Staged packets are discarded wholesale only on disconnect
 * (ble_tx_flush_all from ble_gatt.c); while connected-but-unsubscribed
 * the rings self-limit via drop-oldest.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "host/ble_hs.h"          /* ble_gatts_notify_custom,
                                     ble_hs_mbuf_from_flat, BLE_HS_E* */

#include "ble_iface.h"
#include "ble_ota_iface.h"
#include "diag.h"

#include "narbis/proto.h"

static const char *TAG = "ble_tx";

/* ------------------------------------------------------------------ */
/* Staging rings                                                       */
/* ------------------------------------------------------------------ */
#define SLOT_SZ NC_ATT_PAYLOAD_MAX     /* 244 */

typedef struct {
    uint8_t (*buf)[SLOT_SZ];
    uint16_t *len;
    uint8_t n_slots;
    uint8_t head;                  /* index of the oldest staged packet */
    uint8_t count;
    bool count_drops;              /* STATUS overwrites are not "drops" */
    portMUX_TYPE mux;
} tx_ring_t;

#define DEF_RING_STORAGE(name, slots) \
    static uint8_t name##_buf[slots][SLOT_SZ]; \
    static uint16_t name##_len[slots]

DEF_RING_STORAGE(s_ctrl, 4);       /* BLE_CH_CTRL_RESP */
DEF_RING_STORAGE(s_event, 8);      /* BLE_CH_EVENT     */
DEF_RING_STORAGE(s_ibi, 8);        /* BLE_CH_IBI       */
DEF_RING_STORAGE(s_status, 1);     /* BLE_CH_STATUS — latest wins */
DEF_RING_STORAGE(s_ppg, 8);        /* BLE_CH_PPG       */
DEF_RING_STORAGE(s_accel, 4);      /* BLE_CH_ACCEL     */
DEF_RING_STORAGE(s_ota, 2);        /* BLE_TX_CH_OTA_RESP */

#define RING_INIT(name, slots, drops) \
    { .buf = name##_buf, .len = name##_len, .n_slots = (slots), \
      .head = 0, .count = 0, .count_drops = (drops), \
      .mux = portMUX_INITIALIZER_UNLOCKED }

static tx_ring_t s_rings[BLE_TX_CH_TOTAL] = {
    [BLE_CH_CTRL_RESP] = RING_INIT(s_ctrl, 4, true),
    [BLE_CH_EVENT]     = RING_INIT(s_event, 8, true),
    [BLE_CH_IBI]       = RING_INIT(s_ibi, 8, true),
    [BLE_CH_STATUS]    = RING_INIT(s_status, 1, false),
    [BLE_CH_PPG]       = RING_INIT(s_ppg, 8, true),
    [BLE_CH_ACCEL]     = RING_INIT(s_accel, 4, true),
    [BLE_TX_CH_OTA_RESP] = RING_INIT(s_ota, 2, true),
};

/* Drain priority (design contract: control-plane first, streams last). */
static const uint8_t s_drain_order[BLE_TX_CH_TOTAL] = {
    BLE_CH_CTRL_RESP, BLE_TX_CH_OTA_RESP, BLE_CH_EVENT, BLE_CH_IBI,
    BLE_CH_STATUS, BLE_CH_PPG, BLE_CH_ACCEL,
};

static TaskHandle_t s_task;
static SemaphoreHandle_t s_tx_sem;

/* ------------------------------------------------------------------ */
/* Ring primitives (all under the ring's own critical section)         */
/* ------------------------------------------------------------------ */
static bool ring_stage(int ch, const uint8_t *pkt, uint16_t len)
{
    tx_ring_t *r = &s_rings[ch];

    portENTER_CRITICAL(&r->mux);
    if (r->count == r->n_slots) {
        /* Full: drop the OLDEST staged packet of this channel. */
        r->head = (uint8_t)((r->head + 1) % r->n_slots);
        r->count--;
        if (r->count_drops) {
            g_diag.notify_drop++;      /* serialized by the crit section */
        }
    }
    uint8_t slot = (uint8_t)((r->head + r->count) % r->n_slots);
    memcpy(r->buf[slot], pkt, len);
    r->len[slot] = len;
    r->count++;
    if (r->count > (uint8_t)g_diag.queue_hw_ble) {
        g_diag.queue_hw_ble = r->count;    /* per-ring high-water mark */
    }
    portEXIT_CRITICAL(&r->mux);
    return true;
}

static bool ring_pop(int ch, uint8_t *out, uint16_t *out_len)
{
    tx_ring_t *r = &s_rings[ch];
    bool got = false;

    portENTER_CRITICAL(&r->mux);
    if (r->count > 0) {
        memcpy(out, r->buf[r->head], r->len[r->head]);
        *out_len = r->len[r->head];
        r->head = (uint8_t)((r->head + 1) % r->n_slots);
        r->count--;
        got = true;
    }
    portEXIT_CRITICAL(&r->mux);
    return got;
}

/* Un-pop after a transient send failure. If the ring refilled to full
 * meanwhile, STATUS discards the stale snapshot (a newer one is
 * staged); other channels evict their current oldest to preserve the
 * popped packet's ordering (counted as a drop). */
static void ring_push_front(int ch, const uint8_t *pkt, uint16_t len)
{
    tx_ring_t *r = &s_rings[ch];

    portENTER_CRITICAL(&r->mux);
    if (r->count == r->n_slots) {
        if (!r->count_drops) {         /* STATUS: newer snapshot wins */
            portEXIT_CRITICAL(&r->mux);
            return;
        }
        r->head = (uint8_t)((r->head + 1) % r->n_slots);
        r->count--;
        g_diag.notify_drop++;
    }
    r->head = (uint8_t)((r->head + r->n_slots - 1) % r->n_slots);
    memcpy(r->buf[r->head], pkt, len);
    r->len[r->head] = len;
    r->count++;
    portEXIT_CRITICAL(&r->mux);
}

/* ------------------------------------------------------------------ */
/* Submission (producer context: sys/dsp/afe/imu tasks, host task)     */
/* ------------------------------------------------------------------ */
static bool tx_stage(int ch, const uint8_t *pkt, uint16_t len)
{
    if (pkt == NULL || len == 0 || len > NC_ATT_PAYLOAD_MAX) {
        return false;
    }

    /* STATUS doubles as the GATT read cache — keep it fresh even while
     * disconnected so the first read on a new connection is current. */
    if (ch == BLE_CH_STATUS) {
        ble_gatt_status_cache_set(pkt, len);
    }

    if (s_task == NULL || !ble_is_connected()) {
        return false;               /* discarded silently by contract */
    }

    ring_stage(ch, pkt, len);
    xTaskNotifyGive(s_task);
    return true;
}

bool ble_tx_submit(ble_chan_t ch, const uint8_t *pkt, uint16_t len)
{
    if ((int)ch < 0 || ch >= BLE_CH_COUNT) {
        return false;
    }
    return tx_stage((int)ch, pkt, len);
}

/* OTA_CTRL responses ride the dedicated 2-slot ring drained right
 * after BLE_CH_CTRL_RESP (ble_ota_iface.h contract). */
bool ble_ota_submit_resp(const uint8_t *pkt, uint16_t len)
{
    return tx_stage(BLE_TX_CH_OTA_RESP, pkt, len);
}

/* ------------------------------------------------------------------ */
/* Drain                                                               */
/* ------------------------------------------------------------------ */
static bool ring_has_data(int ch)
{
    tx_ring_t *r = &s_rings[ch];
    portENTER_CRITICAL(&r->mux);
    bool has = r->count > 0;
    portEXIT_CRITICAL(&r->mux);
    return has;
}

static bool rc_is_transient(int rc)
{
    return rc == BLE_HS_ENOMEM || rc == BLE_HS_EAGAIN ||
           rc == BLE_HS_EBUSY || rc == BLE_HS_EALREADY;
}

/* Returns false to abort the whole drain pass (transient exhaustion or
 * link going down) — the outer loop re-enters after notify_tx/timeout. */
static bool drain_channel(int ch)
{
    uint8_t pkt[SLOT_SZ];
    uint16_t len;
    int retries = 0;

    while (ring_has_data(ch)) {
        uint16_t conn = 0, vh = 0;
        if (!ble_gatt_tx_chan_ready(ch, &conn, &vh)) {
            /* Disconnected / unsubscribed / enc-gated: leave the
             * packets staged (drop-oldest bounds them; disconnect
             * flushes them). */
            return true;
        }
        if (!ring_pop(ch, pkt, &len)) {
            return true;
        }

        struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt, len);
        if (om == NULL) {
            ring_push_front(ch, pkt, len);
            (void)xSemaphoreTake(s_tx_sem, pdMS_TO_TICKS(100));
            if (++retries > 2) {
                return false;
            }
            continue;
        }

        /* Consumes om on every path (verified in ble_gattc.c). */
        int rc = ble_gatts_notify_custom(conn, vh, om);

        if (rc == 0) {
            retries = 0;
            continue;
        }

        if (rc == BLE_HS_ENOTCONN) {
            /* Link gone mid-drain: this packet belongs to the dead
             * session. Do NOT re-stage it — that races the disconnect
             * flush and could resurrect it into the next central's
             * session. Drop it and end the pass. */
            return false;
        }
        if (rc_is_transient(rc)) {
            ring_push_front(ch, pkt, len);
            (void)xSemaphoreTake(s_tx_sem, pdMS_TO_TICKS(100));
            if (++retries > 2) {
                return false;
            }
            continue;
        }

        /* Permanent failure for this packet: drop and move on. */
        ESP_LOGW(TAG, "send failed on ch %d: rc=%d — packet dropped",
                 ch, rc);
        portENTER_CRITICAL(&s_rings[ch].mux);   /* counter discipline */
        g_diag.notify_drop++;
        portEXIT_CRITICAL(&s_rings[ch].mux);
    }
    return true;
}

static void ble_tx_task_run(void *arg)
{
    (void)arg;
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        for (int i = 0; i < BLE_TX_CH_TOTAL; i++) {
            if (!drain_channel(s_drain_order[i])) {
                break;              /* back to the 100 ms wait */
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Lifecycle (called by ble_gatt.c)                                    */
/* ------------------------------------------------------------------ */
esp_err_t ble_tx_start(void)
{
    if (s_task != NULL) {
        return ESP_OK;              /* idempotent */
    }
    s_tx_sem = xSemaphoreCreateBinary();
    if (s_tx_sem == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(ble_tx_task_run, "ble_tx", 3072, NULL, 15,
                    &s_task) != pdPASS) {
        vSemaphoreDelete(s_tx_sem);
        s_tx_sem = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void ble_tx_flush_all(void)
{
    for (int ch = 0; ch < BLE_TX_CH_TOTAL; ch++) {
        tx_ring_t *r = &s_rings[ch];
        portENTER_CRITICAL(&r->mux);
        r->head = 0;
        r->count = 0;
        portEXIT_CRITICAL(&r->mux);
    }
}

/* BLE_GAP_EVENT_NOTIFY_TX (any status, notify or indicate): resources
 * freed / indication settled — wake the drain. May be invoked from the
 * NimBLE host task or re-entrantly from ble_tx_task_run itself (the
 * notify path fires the event synchronously in the caller's context). */
void ble_tx_on_notify_tx(void)
{
    if (s_tx_sem != NULL) {
        (void)xSemaphoreGive(s_tx_sem);
    }
    if (s_task != NULL) {
        xTaskNotifyGive(s_task);
    }
}
