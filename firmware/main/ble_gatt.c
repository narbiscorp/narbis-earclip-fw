/*
 * ble_gatt.c — NimBLE peripheral: stack bring-up, GATT services,
 * advertising, security policy, connection management (handoff §5.4).
 *
 * Split of responsibilities: this file owns everything that runs on
 * the NimBLE host task (GAP events, access callbacks) plus the public
 * ble_iface.h surface; ble_tx.c owns the single notifier task that
 * drains the staging rings (appended section of ble_iface.h).
 *
 * Every subscribable characteristic is NOTIFY — indications are banned
 * in this build: BLE_GATTC=0 compiles out NimBLE's CFM/timeout
 * handlers, so an indication's ack never surfaces and each send leaks
 * a GATT proc until the pool (4) is gone and all sends ENOMEM (found
 * on V2.1 first functional test, 2026-07-31: one control response per
 * 35 s, then total TX starvation). Delivery assurance is protocol-
 * level: tid echo + per-step retry in the clients.
 *
 * Security model (handoff §5.4 + Devon decision 3):
 *  - LE Secure Connections, Just Works (no I/O), bonding on.
 *  - New bonds are accepted only while the pairing window is open
 *    (button double-press -> ble_open_pairing_window), or when the
 *    open_pairing knob / NARBIS_TEST_MODE build forces open pairing.
 *    NimBLE has no pre-SMP veto hook, so the policy is enforced at
 *    BLE_GAP_EVENT_ENC_CHANGE: a link that encrypts via a NEW pairing
 *    outside the window gets its just-stored keys deleted and the
 *    connection terminated with BLE_ERR_AUTH_FAIL.
 *  - BLE_GAP_EVENT_REPEAT_PAIRING (bonded peer lost its keys): the
 *    stale bond is deleted and pairing retried only inside the window,
 *    otherwise the request is ignored.
 *  - Custom (Narbis + OTA) characteristics require an encrypted link:
 *    enforced statically with *_ENC attribute flags (drives the
 *    central to pair) and dynamically in the access callbacks /
 *    ble_gatt_tx_chan_ready. Standard services (BAS/DIS/HRS) stay
 *    open. open_pairing is a reboot-class knob, so deciding the
 *    static flags once at init is exact.
 *
 * Advertising: legacy PDUs. ADV payload = flags + the 128-bit Narbis
 * sensor-service UUID (21 bytes); the 19..24-char name lives in the
 * scan response (§5.4 adv-payload note). 100 ms interval for 30 s
 * after boot/disconnect/pairing-window, then 1000-1500 ms, always
 * connectable.
 *
 * NOTE (deviation, verified against the v5.5.1 tree): the design
 * called for ble_gattc_exchange_mtu() on connect, but with
 * CONFIG_BT_NIMBLE_ROLE_CENTRAL=n the GATT client is compiled out
 * (BT_NIMBLE_GATT_CLIENT depends on ROLE_CENTRAL), so the symbol does
 * not link. MTU 247 is still reached: sdkconfig sets
 * CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU=247 and every phone/host central
 * initiates the exchange itself; BLE_GAP_EVENT_MTU caches the result.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_app_desc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "app_msgs.h"
#include "ble_iface.h"
#include "ble_ota_iface.h"
#include "board.h"
#include "dfu_legacy.h"

#include "narbis/proto.h"
#include "narbis/nc_knobs.h"

/* store/config has no public init prototype; this is the canonical
 * bleprph idiom for the NVS-backed bond store. */
void ble_store_config_init(void);

static const char *TAG = "ble";

/* ------------------------------------------------------------------ */
/* Tunables (fixed by the design)                                      */
/* ------------------------------------------------------------------ */
#define ADV_FAST_ITVL        BLE_GAP_ADV_ITVL_MS(100)   /* 160 x 0.625 ms */
#define ADV_SLOW_ITVL_MIN    BLE_GAP_ADV_ITVL_MS(1000)
#define ADV_SLOW_ITVL_MAX    BLE_GAP_ADV_ITVL_MS(1500)
#define ADV_FAST_WINDOW_US   (30LL * 1000 * 1000)

#define CONN_ITVL_FAST_MIN   6      /* 7.5 ms  (1.25 ms units) */
#define CONN_ITVL_FAST_MAX   12     /* 15 ms   */
#define CONN_ITVL_SLOW_MIN   24     /* 30 ms   */
#define CONN_ITVL_SLOW_MAX   40     /* 50 ms   */
#define CONN_TIMEOUT_10MS    400    /* 4 s supervision timeout */

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static volatile uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static volatile uint16_t s_mtu = 23;   /* BLE_ATT_MTU_DFLT */
static volatile bool s_encrypted;
static bool s_peer_was_bonded;         /* host task only */

static volatile bool s_sub_ppg, s_sub_accel, s_sub_ibi, s_sub_event;
static volatile bool s_sub_status, s_sub_ctrl, s_sub_hrs, s_sub_batt;
static volatile bool s_sub_ota;

static volatile bool s_window_open;
static bool s_sec_open;                /* open_pairing knob || TEST_MODE */
static bool s_hrs_en;
static bool s_inited;
static volatile bool s_shutdown;
static volatile bool s_synced;
static volatile bool s_fast_pref;      /* ble_request_conn_speed memory */
static bool s_adv_fast;
static uint8_t s_own_addr_type;

static esp_timer_handle_t s_adv_tmr;   /* fast -> slow one-shot */
static esp_timer_handle_t s_win_tmr;   /* pairing window one-shot */

static char s_name[32];
static char s_fw_ver[34];

/* Protocol version string for DIS 0x2A28, assembled at compile time
 * from the proto.h numbers. */
#define BLE_STR2_(x) #x
#define BLE_STR_(x) BLE_STR2_(x)
static const char s_sw_rev[] =
    BLE_STR_(NC_PROTO_VER_MAJOR) "." BLE_STR_(NC_PROTO_VER_MINOR);

static volatile uint8_t s_batt_pct = 100;

static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_status_cache[NC_ATT_PAYLOAD_MAX];
static uint16_t s_status_len;
static uint8_t s_status_rdbuf[NC_ATT_PAYLOAD_MAX];  /* host-task scratch */

static volatile ble_ota_ctrl_cb_t s_ota_ctrl_cb;
static volatile ble_ota_data_cb_t s_ota_data_cb;

/* host-task scratch for flattened writes (the host task serializes all
 * GATT access callbacks, so one buffer per characteristic suffices). */
static uint8_t s_ctrl_wbuf[NC_ATT_PAYLOAD_MAX];
static uint8_t s_ota_cbuf[NC_ATT_PAYLOAD_MAX];
static uint8_t s_ota_dbuf[NC_ATT_PAYLOAD_MAX];

/* ------------------------------------------------------------------ */
/* UUIDs                                                               */
/* ------------------------------------------------------------------ */
#define UUID_SVC_BAS    0x180F
#define UUID_CHR_BATT   0x2A19
#define UUID_SVC_DIS    0x180A
#define UUID_CHR_MFG    0x2A29
#define UUID_CHR_MODEL  0x2A24
#define UUID_CHR_HWREV  0x2A27
#define UUID_CHR_FWREV  0x2A26
#define UUID_CHR_SWREV  0x2A28
#define UUID_SVC_HRS    0x180D
#define UUID_CHR_HRMEAS 0x2A37

/* 128-bit Narbis UUIDs, built at init from the proto.h base so the
 * wire contract has exactly one authoritative byte table. */
static ble_uuid128_t s_uuid_svc, s_uuid_ppg, s_uuid_accel, s_uuid_ibi,
                     s_uuid_event, s_uuid_status, s_uuid_ctrl,
                     s_uuid_pver, s_uuid_ota_svc, s_uuid_ota_ctrl,
                     s_uuid_ota_data;

static void uuid_make(ble_uuid128_t *u, uint16_t alias)
{
    static const uint8_t base[16] = NC_UUID_BASE_BYTES;
    u->u.type = BLE_UUID_TYPE_128;
    memcpy(u->value, base, sizeof(base));
    u->value[12] = (uint8_t)(alias & 0xFF);   /* alias at [12..13], LE */
    u->value[13] = (uint8_t)(alias >> 8);
}

/* ------------------------------------------------------------------ */
/* GATT table                                                          */
/* ------------------------------------------------------------------ */
enum {
    CHR_ID_BATT, CHR_ID_MFG, CHR_ID_MODEL, CHR_ID_HWREV, CHR_ID_FWREV,
    CHR_ID_SWREV, CHR_ID_HRS, CHR_ID_PPG, CHR_ID_ACCEL, CHR_ID_IBI,
    CHR_ID_EVENT, CHR_ID_STATUS, CHR_ID_CTRL, CHR_ID_PVER,
    CHR_ID_OTA_CTRL, CHR_ID_OTA_DATA,
};
#define CHR_ARG(id) ((void *)(uintptr_t)(id))

static uint16_t s_vh_batt, s_vh_hrs, s_vh_ppg, s_vh_accel, s_vh_ibi,
                s_vh_event, s_vh_status, s_vh_ctrl, s_vh_pver,
                s_vh_ota_ctrl, s_vh_ota_data;

static int chr_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg);

static struct ble_gatt_chr_def s_chr_bas[] = {
    { .uuid = BLE_UUID16_DECLARE(UUID_CHR_BATT), .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_BATT),
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
      .val_handle = &s_vh_batt },
    { 0 },
};

static struct ble_gatt_chr_def s_chr_dis[] = {
    { .uuid = BLE_UUID16_DECLARE(UUID_CHR_MFG), .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_MFG), .flags = BLE_GATT_CHR_F_READ },
    { .uuid = BLE_UUID16_DECLARE(UUID_CHR_MODEL), .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_MODEL), .flags = BLE_GATT_CHR_F_READ },
    { .uuid = BLE_UUID16_DECLARE(UUID_CHR_HWREV), .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_HWREV), .flags = BLE_GATT_CHR_F_READ },
    { .uuid = BLE_UUID16_DECLARE(UUID_CHR_FWREV), .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_FWREV), .flags = BLE_GATT_CHR_F_READ },
    { .uuid = BLE_UUID16_DECLARE(UUID_CHR_SWREV), .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_SWREV), .flags = BLE_GATT_CHR_F_READ },
    { 0 },
};

static struct ble_gatt_chr_def s_chr_hrs[] = {
    { .uuid = BLE_UUID16_DECLARE(UUID_CHR_HRMEAS), .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_HRS), .flags = BLE_GATT_CHR_F_NOTIFY,
      .val_handle = &s_vh_hrs },
    { 0 },
};

static struct ble_gatt_chr_def s_chr_narbis[] = {
    { .uuid = &s_uuid_ppg.u, .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_PPG), .flags = BLE_GATT_CHR_F_NOTIFY,
      .val_handle = &s_vh_ppg },
    { .uuid = &s_uuid_accel.u, .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_ACCEL), .flags = BLE_GATT_CHR_F_NOTIFY,
      .val_handle = &s_vh_accel },
    { .uuid = &s_uuid_ibi.u, .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_IBI), .flags = BLE_GATT_CHR_F_NOTIFY,
      .val_handle = &s_vh_ibi },
    { .uuid = &s_uuid_event.u, .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_EVENT), .flags = BLE_GATT_CHR_F_NOTIFY,
      .val_handle = &s_vh_event },
    { .uuid = &s_uuid_status.u, .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_STATUS),
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
      .val_handle = &s_vh_status },
    { .uuid = &s_uuid_ctrl.u, .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_CTRL),
      /* NOTIFY, deliberately not INDICATE: this build is peripheral-only
       * (no CONFIG_BT_NIMBLE_GATT_CLIENT), and NimBLE compiles the
       * indication CFM/timeout handlers out under BLE_GATTC=0 — the ack
       * never surfaces and every indicate leaks one of the 4 GATT procs
       * until sends ENOMEM forever (V2.1 first functional test,
       * 2026-07-31). Delivery assurance lives in the protocol: tid
       * echo + per-step retry. */
      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
      .val_handle = &s_vh_ctrl },
    { .uuid = &s_uuid_pver.u, .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_PVER), .flags = BLE_GATT_CHR_F_READ,
      .val_handle = &s_vh_pver },
    { 0 },
};

static struct ble_gatt_chr_def s_chr_ota[] = {
    { .uuid = &s_uuid_ota_ctrl.u, .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_OTA_CTRL),
      /* NOTIFY not INDICATE — same BLE_GATTC=0 trap as the CTRL char. */
      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
      .val_handle = &s_vh_ota_ctrl },
    { .uuid = &s_uuid_ota_data.u, .access_cb = chr_access,
      .arg = CHR_ARG(CHR_ID_OTA_DATA),
      .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
      .val_handle = &s_vh_ota_data },
    { 0 },
};

/* HRS is deliberately LAST: hrs_en is a reboot-class knob, and when it
 * is 0 the entry is overwritten with the end-of-table marker at init
 * (service invisible => no subscription => no HRS-driven acquisition). */
static struct ble_gatt_svc_def s_svcs[] = {
    { .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = BLE_UUID16_DECLARE(UUID_SVC_BAS),
      .characteristics = s_chr_bas },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = BLE_UUID16_DECLARE(UUID_SVC_DIS),
      .characteristics = s_chr_dis },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &s_uuid_svc.u,
      .characteristics = s_chr_narbis },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &s_uuid_ota_svc.u,
      .characteristics = s_chr_ota },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = BLE_UUID16_DECLARE(UUID_SVC_HRS),
      .characteristics = s_chr_hrs },
    { 0 },
};
#define SVC_IDX_HRS 4

/* Custom chars get encrypted-link attribute flags unless the build /
 * open_pairing knob (reboot-class) leaves the device open. */
static void chr_harden(struct ble_gatt_chr_def *c)
{
    for (; c->uuid != NULL; c++) {
        if (c->flags & BLE_GATT_CHR_F_READ) {
            c->flags |= BLE_GATT_CHR_F_READ_ENC;
        }
        if (c->flags & (BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP)) {
            c->flags |= BLE_GATT_CHR_F_WRITE_ENC;
        }
        if (c->flags & (BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE)) {
            c->flags |= BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC;
        }
    }
}

/* ------------------------------------------------------------------ */
/* sys_task plumbing                                                   */
/* ------------------------------------------------------------------ */
static void post_conn_change(bool up)
{
    sys_msg_t m = { .type = SYS_CONN_CHANGE, .u.flag = up };
    (void)sys_post(&m);
}

static void post_sub_change(void)
{
    sys_msg_t m = { .type = SYS_SUB_CHANGE };
    m.u.subs.ppg = s_sub_ppg;
    m.u.subs.accel = s_sub_accel;
    m.u.subs.ibi = s_sub_ibi;
    m.u.subs.event = s_sub_event;
    m.u.subs.hrs = s_sub_hrs;
    (void)sys_post(&m);
}

/* CONTROL protocol-level error, staged as a notification. */
static void ctrl_err_resp(uint8_t op, uint8_t tid, uint8_t st)
{
    uint8_t r[3] = { (uint8_t)(op | NC_OP_RESP_FLAG), tid, st };
    (void)ble_tx_submit(BLE_CH_CTRL_RESP, r, sizeof(r));
}

/* ------------------------------------------------------------------ */
/* Access callbacks (NimBLE host task)                                 */
/* ------------------------------------------------------------------ */
static bool custom_access_ok(uint16_t conn_handle)
{
    if (s_sec_open) {
        return true;
    }
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return true;               /* stack-internal read */
    }
    struct ble_gap_conn_desc d;
    if (ble_gap_conn_find(conn_handle, &d) != 0) {
        return false;
    }
    return d.sec_state.encrypted;
}

static int om_append(struct ble_gatt_access_ctxt *ctxt, const void *data,
                     uint16_t len)
{
    return os_mbuf_append(ctxt->om, data, len) == 0
               ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int om_append_str(struct ble_gatt_access_ctxt *ctxt, const char *s)
{
    return om_append(ctxt, s, (uint16_t)strlen(s));
}

/* Flatten a write mbuf into dst. Returns an ATT error code or 0 with
 * *out_len set. */
static int flat_write(struct os_mbuf *om, uint8_t *dst, uint16_t max,
                      uint16_t *out_len)
{
    if (OS_MBUF_PKTLEN(om) > max) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    if (ble_hs_mbuf_to_flat(om, dst, max, out_len) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

static int ctrl_write(uint16_t conn_handle, struct ble_gatt_access_ctxt *ctxt)
{
    if (!custom_access_ok(conn_handle)) {
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }
    uint16_t len = 0;
    int rc = flat_write(ctxt->om, s_ctrl_wbuf, sizeof(s_ctrl_wbuf), &len);
    if (rc != 0) {
        return rc;
    }
    if (len < 2) {
        ctrl_err_resp(0x00, 0x00, NC_ST_BAD_LEN);   /* [0x80][0][BAD_LEN] */
        return 0;
    }
    if (len > SYS_CTRL_REQ_MAX) {
        ctrl_err_resp(s_ctrl_wbuf[0], s_ctrl_wbuf[1], NC_ST_BAD_LEN);
        return 0;
    }
    sys_msg_t m = { .type = SYS_CTRL_REQ };
    m.u.ctrl.len = (uint8_t)len;
    memcpy(m.u.ctrl.buf, s_ctrl_wbuf, len);
    if (!sys_post(&m)) {
        ctrl_err_resp(s_ctrl_wbuf[0], s_ctrl_wbuf[1], NC_ST_BUSY);
    }
    return 0;
}

static int ota_ctrl_write(uint16_t conn_handle,
                          struct ble_gatt_access_ctxt *ctxt)
{
    if (!custom_access_ok(conn_handle)) {
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }
    uint16_t len = 0;
    int rc = flat_write(ctxt->om, s_ota_cbuf, sizeof(s_ota_cbuf), &len);
    if (rc != 0) {
        return rc;
    }
    if (len >= 1 && s_ota_cbuf[0] == NC_OTA_BEGIN) {
        /* A BEGIN may arrive without a prior CONTROL ENTER_OTA — tell
         * sys_task to stop acquisition for the flash writes. */
        sys_msg_t m = { .type = SYS_ENTER_OTA };
        (void)sys_post(&m);
    }
    ble_ota_ctrl_cb_t cb = s_ota_ctrl_cb;
    if (cb != NULL) {
        cb(s_ota_cbuf, len);       /* host-task context per ble_ota_iface.h */
    }
    return 0;
}

static int ota_data_write(uint16_t conn_handle,
                          struct ble_gatt_access_ctxt *ctxt)
{
    if (!custom_access_ok(conn_handle)) {
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }
    uint16_t len = 0;
    int rc = flat_write(ctxt->om, s_ota_dbuf, sizeof(s_ota_dbuf), &len);
    if (rc != 0) {
        return rc;
    }
    ble_ota_data_cb_t cb = s_ota_data_cb;
    if (cb != NULL) {
        cb(s_ota_dbuf, len);
    }
    return 0;
}

static int chr_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)attr_handle;
    const int id = (int)(uintptr_t)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        switch (id) {
        case CHR_ID_BATT: {
            uint8_t pct = s_batt_pct;
            return om_append(ctxt, &pct, 1);
        }
        case CHR_ID_MFG:
            return om_append_str(ctxt, "Narbis");
        case CHR_ID_MODEL:
            return om_append_str(ctxt, "Edge Earclip");
        case CHR_ID_HWREV:
            return om_append_str(ctxt, "V2.1");
        case CHR_ID_FWREV:
            return om_append_str(ctxt, s_fw_ver);
        case CHR_ID_SWREV:
            return om_append_str(ctxt, s_sw_rev);
        case CHR_ID_STATUS: {
            if (!custom_access_ok(conn_handle)) {
                return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
            }
            uint16_t len;
            portENTER_CRITICAL(&s_status_mux);
            len = s_status_len;
            memcpy(s_status_rdbuf, s_status_cache, len);
            portEXIT_CRITICAL(&s_status_mux);
            return om_append(ctxt, s_status_rdbuf, len);
        }
        case CHR_ID_PVER: {
            if (!custom_access_ok(conn_handle)) {
                return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
            }
            const uint8_t v[2] = { (uint8_t)(NC_PROTO_VER & 0xFF),
                                   (uint8_t)(NC_PROTO_VER >> 8) };
            return om_append(ctxt, v, sizeof(v));
        }
        default:
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        switch (id) {
        case CHR_ID_CTRL:
            return ctrl_write(conn_handle, ctxt);
        case CHR_ID_OTA_CTRL:
            return ota_ctrl_write(conn_handle, ctxt);
        case CHR_ID_OTA_DATA:
            return ota_data_write(conn_handle, ctxt);
        default:
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* ------------------------------------------------------------------ */
/* Advertising                                                         */
/* ------------------------------------------------------------------ */
static int gap_event_cb(struct ble_gap_event *event, void *arg);

static void adv_start(bool fast)
{
#if NARBIS_TEST_MODE
    /* Bench builds never drop to slow advertising: at 1-1.5 s intervals
     * Chrome's chooser can take many seconds to list the device, which
     * reads as "board randomly missing". The idle-power cost does not
     * matter on the bench and this build never ships in an enclosure. */
    fast = true;
#endif
    if (!s_synced || s_shutdown || s_conn != BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    if (ble_gap_adv_active()) {
        int src = ble_gap_adv_stop();
        if (src != 0 && src != BLE_HS_EALREADY) {
            ESP_LOGW(TAG, "adv_stop: rc=%d", src);
        }
    }

    struct ble_hs_adv_fields f;
    memset(&f, 0, sizeof(f));
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    f.uuids128 = &s_uuid_svc;      /* 3 + 18 = 21 B, fits legacy ADV */
    f.num_uuids128 = 1;
    f.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&f);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields: rc=%d", rc);
        return;
    }

    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.name = (const uint8_t *)s_name;
    rsp.name_len = (uint8_t)strlen(s_name);
    rsp.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_rsp_set_fields: rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params p;
    memset(&p, 0, sizeof(p));
    p.conn_mode = BLE_GAP_CONN_MODE_UND;     /* always connectable */
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    p.itvl_min = fast ? ADV_FAST_ITVL : ADV_SLOW_ITVL_MIN;
    p.itvl_max = fast ? ADV_FAST_ITVL : ADV_SLOW_ITVL_MAX;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &p,
                           gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "adv_start: rc=%d", rc);
        return;
    }
    s_adv_fast = fast;
    ESP_LOGI(TAG, "advertising (%s)", fast ? "fast 100 ms" : "slow 1-1.5 s");

    esp_timer_stop(s_adv_tmr);     /* INVALID_STATE when idle — ignored */
    if (fast) {
        esp_timer_start_once(s_adv_tmr, ADV_FAST_WINDOW_US);
    }
}

static void adv_switch_cb(void *a)
{
    (void)a;
    if (s_conn == BLE_HS_CONN_HANDLE_NONE) {
        adv_start(false);
    }
}

static void window_cb(void *a)
{
    (void)a;
    s_window_open = false;
    ESP_LOGI(TAG, "pairing window closed");
}

/* ------------------------------------------------------------------ */
/* Security helpers                                                    */
/* ------------------------------------------------------------------ */
static bool peer_is_bonded(uint16_t conn_handle)
{
    struct ble_gap_conn_desc d;
    if (ble_gap_conn_find(conn_handle, &d) != 0) {
        return false;
    }
    ble_addr_t peers[8];
    int n = 0;
    if (ble_store_util_bonded_peers(peers, &n, 8) != 0) {
        return false;
    }
    for (int i = 0; i < n; i++) {
        if (ble_addr_cmp(&peers[i], &d.peer_id_addr) == 0) {
            return true;
        }
    }
    return false;
}

static bool bond_window_ok(void)
{
    return s_sec_open || s_window_open;
}

/* ------------------------------------------------------------------ */
/* Connection parameter policy                                         */
/* ------------------------------------------------------------------ */
static void apply_conn_params(void)
{
    uint16_t c = s_conn;
    if (c == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    const bool fast = s_fast_pref;
    struct ble_gap_upd_params up = {
        .itvl_min = fast ? CONN_ITVL_FAST_MIN : CONN_ITVL_SLOW_MIN,
        .itvl_max = fast ? CONN_ITVL_FAST_MAX : CONN_ITVL_SLOW_MAX,
        .latency = 0,
        .supervision_timeout = CONN_TIMEOUT_10MS,
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    int rc = ble_gap_update_params(c, &up);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGD(TAG, "update_params(%s): rc=%d", fast ? "fast" : "slow", rc);
    }
}

/* ------------------------------------------------------------------ */
/* GAP events (NimBLE host task)                                       */
/* ------------------------------------------------------------------ */
static void clear_link_state(void)
{
    s_sub_ppg = s_sub_accel = s_sub_ibi = s_sub_event = false;
    s_sub_status = s_sub_ctrl = s_sub_hrs = s_sub_batt = false;
    s_sub_ota = false;
    s_encrypted = false;
    s_peer_was_bonded = false;
    s_mtu = 23;
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "connect failed (%d) — re-advertising",
                     event->connect.status);
            adv_start(true);
            return 0;
        }
        clear_link_state();
        /* Defense-in-depth against the drain/disconnect race: any
         * packet still staged from the previous session must not leak
         * into this one. Flush BEFORE the handle goes live so the
         * drain task cannot start serving the new connection first. */
        ble_tx_flush_all();
        s_conn = event->connect.conn_handle;
        s_peer_was_bonded = peer_is_bonded(s_conn);
        s_mtu = ble_att_mtu(s_conn);
        esp_timer_stop(s_adv_tmr);
        ESP_LOGI(TAG, "connected (handle %u, peer %s)", (unsigned)s_conn,
                 s_peer_was_bonded ? "bonded" : "new");

        /* Link tuning: 2M PHY + DLE + interval policy. All best-effort;
         * the central may override or reject any of them. */
        (void)ble_gap_set_prefered_le_phy(s_conn, BLE_GAP_LE_PHY_2M_MASK,
                                          BLE_GAP_LE_PHY_2M_MASK, 0);
        (void)ble_hs_hci_util_set_data_len(s_conn, 251, 2120);
        apply_conn_params();

        post_conn_change(true);
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected (reason %d)", event->disconnect.reason);
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        clear_link_state();
        ble_tx_flush_all();
        dfu_legacy_on_disconnect();   /* abort a mid-flight legacy DFU */
        post_conn_change(false);
        adv_start(true);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE: {
        const uint16_t ah = event->subscribe.attr_handle;
        const bool cn = event->subscribe.cur_notify;
        /* Every subscribable char here is notify-only. A central whose
         * GATT cache remembers the pre-2026-07-31 indicate CTRL char
         * will fail its CCCD write and must re-discover (forget/re-pair
         * or toggle Bluetooth) — acceptable: nothing shipped. */
        if (ah == s_vh_ppg)            s_sub_ppg = cn;
        else if (ah == s_vh_accel)     s_sub_accel = cn;
        else if (ah == s_vh_ibi)       s_sub_ibi = cn;
        else if (ah == s_vh_event)     s_sub_event = cn;
        else if (ah == s_vh_status)    s_sub_status = cn;
        else if (ah == s_vh_ctrl)      s_sub_ctrl = cn;
        else if (ah == s_vh_hrs)       s_sub_hrs = cn;
        else if (ah == s_vh_batt)      s_sub_batt = cn;
        else if (ah == s_vh_ota_ctrl)  s_sub_ota = cn;
        post_sub_change();
        ble_tx_on_notify_tx();     /* fresh CCCD: kick the drain */
        return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_TX:
        ble_tx_on_notify_tx();     /* resources freed: kick the drain */
        return 0;

    case BLE_GAP_EVENT_MTU:
        s_mtu = event->mtu.value;
        ESP_LOGI(TAG, "MTU %u", (unsigned)s_mtu);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status != 0) {
            ESP_LOGW(TAG, "encryption failed (%d)", event->enc_change.status);
            return 0;
        }
        s_encrypted = true;
        if (!s_peer_was_bonded) {
            /* The link just encrypted via a fresh pairing. */
            if (bond_window_ok()) {
                s_peer_was_bonded = true;
                ESP_LOGI(TAG, "new bond accepted");
            } else {
                ESP_LOGW(TAG, "pairing outside window — rejecting");
                struct ble_gap_conn_desc d;
                if (ble_gap_conn_find(event->enc_change.conn_handle, &d) == 0) {
                    (void)ble_store_util_delete_peer(&d.peer_id_addr);
                }
                (void)ble_gap_terminate(event->enc_change.conn_handle,
                                        BLE_ERR_AUTH_FAIL);
            }
        }
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* Bonded peer lost its keys. Deleting the stale bond is itself
         * a bond-creating act — window-gated like any new bond. */
        if (bond_window_ok()) {
            struct ble_gap_conn_desc d;
            if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &d) == 0) {
                (void)ble_store_util_delete_peer(&d.peer_id_addr);
            }
            return BLE_GAP_REPEAT_PAIRING_RETRY;
        }
        ESP_LOGW(TAG, "re-pairing outside window — ignored");
        return BLE_GAP_REPEAT_PAIRING_IGNORE;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        if (s_conn == BLE_HS_CONN_HANDLE_NONE) {
            adv_start(s_adv_fast);
        }
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGD(TAG, "conn update status=%d", event->conn_update.status);
        return 0;

    default:
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* Host lifecycle                                                      */
/* ------------------------------------------------------------------ */
static void on_reset(int reason)
{
    ESP_LOGE(TAG, "host reset, reason %d", reason);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ensure_addr: rc=%d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "infer_auto: rc=%d", rc);
        return;
    }
    s_synced = true;
    adv_start(true);               /* fast for 30 s after boot */
}

static void host_task_fn(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();             /* returns at nimble_port_stop() */
    nimble_port_freertos_deinit(); /* self-delete (bleprph idiom) */
}

/* ------------------------------------------------------------------ */
/* Public API — ble_iface.h                                            */
/* ------------------------------------------------------------------ */
esp_err_t ble_iface_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }
    s_shutdown = false;

    /* Reboot-class knobs are latched once here (NVS overlay was loaded
     * by main before we run). */
    s_sec_open = (NARBIS_TEST_MODE != 0) ||
                 (nc_knob_get(KNOB_OPEN_PAIRING) != 0);
    s_hrs_en = nc_knob_get(KNOB_HRS_EN) != 0;

    snprintf(s_name, sizeof(s_name), "Narbis Edge Earclip%s",
             (NARBIS_TEST_MODE != 0) ? " TEST" : "");
    const esp_app_desc_t *app = esp_app_get_description();
    size_t vlen = strnlen(app->version, sizeof(app->version));
    if (vlen >= sizeof(s_fw_ver)) {
        vlen = sizeof(s_fw_ver) - 1;
    }
    memcpy(s_fw_ver, app->version, vlen);
    s_fw_ver[vlen] = '\0';

    uuid_make(&s_uuid_svc, NC_ALIAS_SENSOR_SVC);
    uuid_make(&s_uuid_ppg, NC_ALIAS_PPG);
    uuid_make(&s_uuid_accel, NC_ALIAS_ACCEL);
    uuid_make(&s_uuid_ibi, NC_ALIAS_IBI);
    uuid_make(&s_uuid_event, NC_ALIAS_EVENT);
    uuid_make(&s_uuid_status, NC_ALIAS_STATUS);
    uuid_make(&s_uuid_ctrl, NC_ALIAS_CONTROL);
    uuid_make(&s_uuid_pver, NC_ALIAS_PROTO_VER);
    uuid_make(&s_uuid_ota_svc, NC_ALIAS_OTA_SVC);
    uuid_make(&s_uuid_ota_ctrl, NC_ALIAS_OTA_CTRL);
    uuid_make(&s_uuid_ota_data, NC_ALIAS_OTA_DATA);

    if (!s_sec_open) {
        chr_harden(s_chr_narbis);
        chr_harden(s_chr_ota);
    }
    if (!s_hrs_en) {
        memset(&s_svcs[SVC_IDX_HRS], 0, sizeof(s_svcs[SVC_IDX_HRS]));
        ESP_LOGI(TAG, "HRS disabled (hrs_en=0)");
    }

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(err));
        return err;
    }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;   /* Just Works */
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;                               /* LE Secure Conn */
    ble_hs_cfg.sm_our_key_dist |=
        BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist |=
        BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "gatts_count_cfg: rc=%d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(s_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "gatts_add_svcs: rc=%d", rc);
        return ESP_FAIL;
    }
    /* Legacy DFU service 0x00FF (dfu_legacy.c owns table + callbacks;
     * service-module registration idiom, same as ble_svc_gap_init).
     * Hardened to encrypted-only exactly when the custom tables are. */
    err = dfu_legacy_gatt_register(!s_sec_open);
    if (err != ESP_OK) {
        return err;
    }
    rc = ble_svc_gap_device_name_set(s_name);
    if (rc != 0) {
        ESP_LOGW(TAG, "device_name_set: rc=%d", rc);
    }

    ble_store_config_init();       /* NVS-backed bond store */

    err = ble_tx_start();          /* notifier task, after GATT reg */
    if (err != ESP_OK) {
        return err;
    }

    /* Timers survive a shutdown/re-init cycle — create once. */
    if (s_adv_tmr == NULL) {
        const esp_timer_create_args_t adv_args = { .callback = adv_switch_cb,
                                                   .name = "ble_advsw" };
        err = esp_timer_create(&adv_args, &s_adv_tmr);
        if (err != ESP_OK) {
            return err;
        }
    }
    if (s_win_tmr == NULL) {
        const esp_timer_create_args_t win_args = { .callback = window_cb,
                                                   .name = "ble_pairwin" };
        err = esp_timer_create(&win_args, &s_win_tmr);
        if (err != ESP_OK) {
            return err;
        }
    }

    s_inited = true;
    nimble_port_freertos_init(host_task_fn);
    ESP_LOGI(TAG, "up as \"%s\" (open_pairing=%d, hrs_en=%d)", s_name,
             (int)s_sec_open, (int)s_hrs_en);
    return ESP_OK;
}

void ble_shutdown(void)
{
    if (!s_inited) {
        return;
    }
    s_shutdown = true;

    esp_timer_stop(s_adv_tmr);     /* INVALID_STATE when idle — ignored */
    esp_timer_stop(s_win_tmr);
    s_window_open = false;

    if (ble_gap_adv_active()) {
        (void)ble_gap_adv_stop();
    }
    uint16_t c = s_conn;
    if (c != BLE_HS_CONN_HANDLE_NONE) {
        (void)ble_gap_terminate(c, BLE_ERR_REM_USER_CONN_TERM);
        vTaskDelay(pdMS_TO_TICKS(50));      /* let the terminate go out */
    }

    int rc = nimble_port_stop();
    if (rc == 0) {
        /* The host task exits nimble_port_run() and self-deletes via
         * nimble_port_freertos_deinit() (bleprph idiom); give it a
         * beat before tearing the port down. */
        vTaskDelay(pdMS_TO_TICKS(20));
        nimble_port_deinit();
    } else {
        ESP_LOGW(TAG, "nimble_port_stop: rc=%d", rc);
    }
    s_conn = BLE_HS_CONN_HANDLE_NONE;
    clear_link_state();
    ble_tx_flush_all();
    s_synced = false;
    s_inited = false;
    ESP_LOGI(TAG, "shut down");
}

bool ble_is_connected(void)
{
    return s_conn != BLE_HS_CONN_HANDLE_NONE;
}

uint16_t ble_att_payload_budget(void)
{
    /* Producers of large stream packets clamp their batch fill to this.
     * MTU-3 is the notification payload ceiling; NimBLE silently
     * truncates anything bigger. Disconnected -> full budget (batches
     * are rebuilt per-connection and the MTU event lands before any
     * subscription). If the peer never exchanges MTU, 23-3 = 20 holds. */
    uint16_t c = s_conn;
    if (c == BLE_HS_CONN_HANDLE_NONE) {
        return NC_ATT_PAYLOAD_MAX;
    }
    uint16_t m = s_mtu;
    uint16_t budget = (m > 3) ? (uint16_t)(m - 3) : 20;
    if (budget < 20) budget = 20;
    return (budget < NC_ATT_PAYLOAD_MAX) ? budget : NC_ATT_PAYLOAD_MAX;
}

void ble_get_conn_stats(uint16_t *mtu, uint8_t *phy, uint16_t *interval_1_25ms)
{
    uint16_t c = s_conn;
    if (mtu != NULL) {
        *mtu = (c != BLE_HS_CONN_HANDLE_NONE) ? s_mtu : 0;
    }
    if (phy != NULL) {
        *phy = 0;
        if (c != BLE_HS_CONN_HANDLE_NONE) {
            uint8_t tx = 0, rx = 0;
            if (ble_gap_read_le_phy(c, &tx, &rx) == 0) {
                *phy = tx;         /* 1 = 1M, 2 = 2M, 3 = coded */
            }
        }
    }
    if (interval_1_25ms != NULL) {
        *interval_1_25ms = 0;
        struct ble_gap_conn_desc d;
        if (c != BLE_HS_CONN_HANDLE_NONE &&
            ble_gap_conn_find(c, &d) == 0) {
            *interval_1_25ms = d.conn_itvl;
        }
    }
}

void ble_open_pairing_window(uint32_t seconds)
{
    if (!s_inited || seconds == 0) {
        return;
    }
    s_window_open = true;
    esp_timer_stop(s_win_tmr);
    esp_timer_start_once(s_win_tmr, (uint64_t)seconds * 1000000ULL);
    ESP_LOGI(TAG, "pairing window open for %lu s", (unsigned long)seconds);

    /* Make the device promptly findable for the phone doing the bond. */
    if (s_conn == BLE_HS_CONN_HANDLE_NONE) {
        adv_start(true);
    }
}

void ble_update_battery(uint8_t pct)
{
    if (pct > 100) {
        pct = 100;
    }
    s_batt_pct = pct;
    uint16_t conn, vh;
    if (ble_gatt_batt_notify_ready(&conn, &vh)) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&pct, 1);
        if (om != NULL) {
            int rc = ble_gatts_notify_custom(conn, vh, om);
            if (rc != 0) {
                ESP_LOGD(TAG, "battery notify: rc=%d", rc);
            }
        }
    }
}

void ble_notify_hrs(uint8_t bpm, const uint16_t *rr_1024, int n_rr)
{
    uint16_t conn, vh;
    if (!ble_gatt_hrs_notify_ready(&conn, &vh)) {
        return;                    /* nobody listening — free by design */
    }
    if (n_rr < 0 || rr_1024 == NULL) {
        n_rr = 0;
    }
    if (n_rr > 7) {
        n_rr = 7;                  /* 2 + 7*2 = 16 B, fits MTU 23 too */
    }
    uint8_t buf[2 + 7 * 2];
    /* Flags: HR u8 (bit0=0), RR intervals present when n_rr > 0. */
    buf[0] = (n_rr > 0) ? 0x10 : 0x00;
    buf[1] = bpm;
    uint16_t len = 2;
    for (int i = 0; i < n_rr; i++) {
        buf[len++] = (uint8_t)(rr_1024[i] & 0xFF);
        buf[len++] = (uint8_t)(rr_1024[i] >> 8);
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
    if (om != NULL) {
        int rc = ble_gatts_notify_custom(conn, vh, om);
        if (rc != 0) {
            ESP_LOGD(TAG, "hrs notify: rc=%d", rc);
        }
    }
}

void ble_request_conn_speed(bool fast)
{
    s_fast_pref = fast;            /* remembered; re-applied on connect */
    if (s_conn != BLE_HS_CONN_HANDLE_NONE) {
        apply_conn_params();
    }
}

/* ------------------------------------------------------------------ */
/* OTA registration — ble_ota_iface.h                                  */
/* ------------------------------------------------------------------ */
void ble_ota_register(ble_ota_ctrl_cb_t ctrl_cb, ble_ota_data_cb_t data_cb)
{
    s_ota_ctrl_cb = ctrl_cb;
    s_ota_data_cb = data_cb;
}

/* ------------------------------------------------------------------ */
/* Private surface for ble_tx.c — ble_iface.h appended section         */
/* ------------------------------------------------------------------ */
bool ble_gatt_tx_chan_ready(int tx_ch, uint16_t *conn, uint16_t *val_handle)
{
    const uint16_t c = s_conn;
    if (c == BLE_HS_CONN_HANDLE_NONE) {
        return false;
    }

    bool sub = false;
    uint16_t vh = 0;
    switch (tx_ch) {
    case BLE_CH_CTRL_RESP:
        sub = s_sub_ctrl;  vh = s_vh_ctrl;     break;
    case BLE_CH_EVENT:
        sub = s_sub_event; vh = s_vh_event;    break;
    case BLE_CH_IBI:
        sub = s_sub_ibi;   vh = s_vh_ibi;      break;
    case BLE_CH_STATUS:
        sub = s_sub_status; vh = s_vh_status;  break;
    case BLE_CH_PPG:
        sub = s_sub_ppg;   vh = s_vh_ppg;      break;
    case BLE_CH_ACCEL:
        sub = s_sub_accel; vh = s_vh_accel;    break;
    case BLE_TX_CH_OTA_RESP:
        sub = s_sub_ota;   vh = s_vh_ota_ctrl; break;
    default:
        return false;
    }

    if (!sub || vh == 0) {
        return false;
    }
    /* All ble_tx channels are custom characteristics: encrypted-link
     * gate unless the device runs open (knob / test build). */
    if (!s_sec_open && !s_encrypted) {
        return false;
    }

    if (conn != NULL) {
        *conn = c;
    }
    if (val_handle != NULL) {
        *val_handle = vh;
    }
    return true;
}

void ble_gatt_status_cache_set(const uint8_t *pkt, uint16_t len)
{
    if (pkt == NULL || len == 0 || len > NC_ATT_PAYLOAD_MAX) {
        return;
    }
    portENTER_CRITICAL(&s_status_mux);
    memcpy(s_status_cache, pkt, len);
    s_status_len = len;
    portEXIT_CRITICAL(&s_status_mux);
}

void ble_gatt_set_battery(uint8_t pct)
{
    s_batt_pct = (pct > 100) ? 100 : pct;
}

bool ble_gatt_batt_notify_ready(uint16_t *conn, uint16_t *val_handle)
{
    const uint16_t c = s_conn;
    if (c == BLE_HS_CONN_HANDLE_NONE || !s_sub_batt || s_vh_batt == 0) {
        return false;              /* std service: no encryption gate */
    }
    if (conn != NULL) {
        *conn = c;
    }
    if (val_handle != NULL) {
        *val_handle = s_vh_batt;
    }
    return true;
}

bool ble_gatt_hrs_notify_ready(uint16_t *conn, uint16_t *val_handle)
{
    const uint16_t c = s_conn;
    if (c == BLE_HS_CONN_HANDLE_NONE || !s_hrs_en || !s_sub_hrs ||
        s_vh_hrs == 0) {
        return false;              /* std service: no encryption gate */
    }
    if (conn != NULL) {
        *conn = c;
    }
    if (val_handle != NULL) {
        *val_handle = s_vh_hrs;
    }
    return true;
}

