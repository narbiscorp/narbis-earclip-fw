/*
 * dfu_legacy.c — device side of the legacy Narbis DFU protocol
 * (docs/legacy_dfu_protocol.md), byte-compatible with the Edge-glasses
 * firmware so the existing narbiscorp/edge-OTA web updater and the OTA
 * hub can flash earclips unchanged.
 *
 * Protocol mapping (doc section -> here):
 *  §1  service 0x00FF, FF01 CONTROL (write/WNR/read), FF02 DATA
 *      (write/WNR), FF03 STATUS (notify/read; last frame cached for
 *      reads).
 *  §2  0xA8 BEGIN [+size_LE32]: guards, erase, READY. The 4 size bytes
 *      are optional — absent/zero falls back to OTA_SIZE_UNKNOWN
 *      (whole-partition erase), matching "legacy firmware reads only
 *      byte 0".
 *  §5  DATA bytes accumulate into ONE static 4096-byte page buffer by
 *      offset (no per-chunk framing). Full page -> PAGE_CRC handshake
 *      (crc32 big-endian on the wire) -> 0xAD commit/reject ->
 *      PAGE_OK / PAGE_RESEND (max 3 resends, then ERROR 0x02). The
 *      final partial page is NOT handshaked: its bytes are written
 *      during FINISH (0xA9), then esp_ota_end + set_boot_partition +
 *      SUCCESS + reboot 500 ms later. 0xAA CANCEL aborts.
 *  §7  page-0 identity check BEFORE the first flash write (magic 0xE9,
 *      chip_id 0x000D, app-desc magic, project_name "narbis_earclip"),
 *      else ERROR 0x07. Runs after the CRC handshake confirmed the
 *      bytes, so a corrupted transfer resends instead of failing 0x07.
 *  §10 the peripheral requests a 32 s supervision timeout for the
 *      session (the erase stall) and restores the normal 4 s policy on
 *      session end.
 *
 * Threading: GATT access callbacks and the GAP disconnect event all run
 * on the NimBLE host task — the engine is single-threaded by
 * construction, no mutex. esp_ota_begin's erase (seconds) and the 4 KB
 * esp_ota_write per page block the host task deliberately: this is DFU
 * mode, acquisition is stopped (SYS_ENTER_OTA), the central is waiting
 * on our status frame anyway, and LL keep-alives come from the
 * controller, not the host task.
 *
 * STATUS frames are notified straight from the handler via
 * ble_gatts_notify_custom (host-task context, same as ble_update_battery)
 * without a CCCD check — the protocol requires the central to subscribe
 * before doing anything (§5), and frames only ever target the writer's
 * connection.
 */
#include "dfu_legacy.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "host/ble_hs.h"

#include "app_msgs.h"
#include "battery.h"
#include "charger.h"
#include "ota.h"

#include "narbis/nc_crc32.h"
#include "narbis/proto.h"      /* NC_ATT_PAYLOAD_MAX */

static const char *TAG = "dfu_leg";

/* ------------------------------------------------------------------ */
/* Wire constants (docs/legacy_dfu_protocol.md §2-§4)                  */
/* ------------------------------------------------------------------ */
#define OP_BEGIN            0xA8
#define OP_FINISH           0xA9
#define OP_CANCEL           0xAA
#define OP_PAGE_CONFIRM     0xAD

#define ST_READY            0x01
#define ST_SUCCESS          0x03
#define ST_ERROR            0x04
#define ST_CANCELLED        0x05
#define ST_PAGE_CRC         0x06
#define ST_PAGE_OK          0x07
#define ST_PAGE_RESEND      0x08

#define ERR_BEGIN_FAILED    0x01
#define ERR_WRITE_FAILED    0x02
#define ERR_FINALIZE_FAILED 0x03
#define ERR_NO_SESSION      0x04
#define ERR_NO_PARTITION    0x05
#define ERR_LOW_BATTERY     0x06   /* earclip: battery < 30 % */
#define ERR_CHIP_MISMATCH   0x07   /* earclip: §7 identity failed */
#define ERR_ALREADY_IN_OTA  0x08   /* earclip: modern or legacy busy */

#define DFU_PAGE_SIZE       4096
#define DFU_MAX_RESENDS     3
#define DFU_BATT_MIN_PCT    30
#define DFU_RESTART_DELAY_US (500LL * 1000)

/* Session conn params (§10): slow interval, 32 s supervision timeout so
 * the begin-erase stall cannot kill the link. Normal = ble_gatt.c's
 * slow/idle policy (30-50 ms, 4 s) reapplied on session end. */
#define DFU_ITVL_MIN        24      /* 30 ms, 1.25 ms units */
#define DFU_ITVL_MAX        40      /* 50 ms */
#define DFU_TIMEOUT_10MS    3200    /* 32 s */
#define NORM_TIMEOUT_10MS   400     /* 4 s  */

/* ------------------------------------------------------------------ */
/* Engine state — NimBLE host task only (see file header)              */
/* ------------------------------------------------------------------ */
static volatile bool           s_active;
static uint16_t                s_sess_conn = BLE_HS_CONN_HANDLE_NONE;
static const esp_partition_t  *s_part;
static esp_ota_handle_t        s_ota;
static bool                    s_ota_open;
static uint32_t                s_img_size;       /* 0 = unknown */
static uint32_t                s_written;
static uint8_t                 s_page[DFU_PAGE_SIZE];
static uint32_t                s_fill;           /* bytes in s_page */
static uint16_t                s_page_idx;       /* 0-based */
static uint8_t                 s_resends;
static bool                    s_wait_confirm;   /* PAGE_CRC sent, 0xAD due */
static bool                    s_stray_notified; /* one ERROR 0x04 per burst */

static uint8_t                 s_last_ctrl;      /* FF01 read value */
static uint8_t                 s_status[7] = { 0, 0, 0, 0 };
static uint8_t                 s_status_len = 4; /* FF03 read cache */

static uint16_t                s_vh_ctrl, s_vh_data, s_vh_status;
static esp_timer_handle_t      s_restart_tmr;

/* Host-task scratch for flattened writes (host task serializes all
 * access callbacks — one buffer per characteristic suffices). */
static uint8_t s_ctrl_buf[16];
static uint8_t s_data_buf[NC_ATT_PAYLOAD_MAX];

/* ------------------------------------------------------------------ */
/* §7 image identity — shared with ota.c's model lock                  */
/* ------------------------------------------------------------------ */
bool dfu_image_hdr_ok(const uint8_t *h, size_t len)
{
    if (h == NULL || len < 0x50 + 14) {
        return false;
    }
    if (h[0x00] != 0xE9) {
        return false;                          /* ESP image magic */
    }
    const uint16_t chip = (uint16_t)h[0x0C] | ((uint16_t)h[0x0D] << 8);
    if (chip != 0x000D) {
        return false;                          /* ESP32-C6 only */
    }
    const uint32_t am = (uint32_t)h[0x20] | ((uint32_t)h[0x21] << 8) |
                        ((uint32_t)h[0x22] << 16) | ((uint32_t)h[0x23] << 24);
    if (am != 0xABCD5432u) {
        return false;                          /* esp_app_desc magic */
    }
    return memcmp(h + 0x50, "narbis_earclip", 14) == 0;
}

/* ------------------------------------------------------------------ */
/* Status notification (also feeds the FF03 read cache)                */
/* ------------------------------------------------------------------ */
static void dfu_notify(uint16_t conn, const uint8_t *frame, uint8_t len)
{
    memcpy(s_status, frame, len);
    s_status_len = len;
    if (conn == BLE_HS_CONN_HANDLE_NONE || s_vh_status == 0) {
        return;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(frame, len);
    if (om == NULL) {
        ESP_LOGW(TAG, "status 0x%02x: no mbuf", frame[0]);
        return;
    }
    int rc = ble_gatts_notify_custom(conn, s_vh_status, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "status 0x%02x: notify rc=%d", frame[0], rc);
    }
}

static void notify_code(uint16_t conn, uint8_t code)   /* [code,0,0,0] */
{
    const uint8_t f[4] = { code, 0, 0, 0 };
    dfu_notify(conn, f, sizeof(f));
}

static void notify_error(uint16_t conn, uint8_t err)   /* [0x04,err,0,0] */
{
    const uint8_t f[4] = { ST_ERROR, err, 0, 0 };
    ESP_LOGE(TAG, "ERROR 0x%02x", err);
    dfu_notify(conn, f, sizeof(f));
}

static void notify_page(uint16_t conn, uint8_t code, uint16_t page)
{
    const uint8_t f[3] = { code, (uint8_t)(page >> 8), (uint8_t)page };
    dfu_notify(conn, f, sizeof(f));            /* page index big-endian */
}

static void notify_page_crc(uint16_t conn, uint16_t page, uint32_t crc)
{
    const uint8_t f[7] = {
        ST_PAGE_CRC, (uint8_t)(page >> 8), (uint8_t)page,
        (uint8_t)(crc >> 24), (uint8_t)(crc >> 16),   /* CRC32 BE (§3) */
        (uint8_t)(crc >> 8), (uint8_t)crc,
    };
    dfu_notify(conn, f, sizeof(f));
}

/* ------------------------------------------------------------------ */
/* Session helpers                                                     */
/* ------------------------------------------------------------------ */
static void dfu_conn_params(uint16_t conn, bool dfu_session)
{
    if (conn == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    const struct ble_gap_upd_params up = {
        .itvl_min = DFU_ITVL_MIN,
        .itvl_max = DFU_ITVL_MAX,
        .latency = 0,
        .supervision_timeout = dfu_session ? DFU_TIMEOUT_10MS
                                           : NORM_TIMEOUT_10MS,
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    int rc = ble_gap_update_params(conn, &up);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "conn params (%s): rc=%d",
                 dfu_session ? "dfu" : "normal", rc);
    }
}

static void session_reset(void)
{
    if (s_ota_open) {
        esp_ota_abort(s_ota);
        s_ota_open = false;
    }
    s_active = false;
    s_sess_conn = BLE_HS_CONN_HANDLE_NONE;
    s_fill = 0;
    s_page_idx = 0;
    s_resends = 0;
    s_wait_confirm = false;
    s_written = 0;
}

/* Error that terminates the session: notify, abort, restore params. */
static void session_fail(uint16_t conn, uint8_t err)
{
    notify_error(conn, err);
    session_reset();
    dfu_conn_params(conn, false);
}

static void restart_cb(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "restarting into new image");
    esp_restart();
}

/* ------------------------------------------------------------------ */
/* Opcode handlers (NimBLE host task)                                  */
/* ------------------------------------------------------------------ */
static void op_begin(uint16_t conn, const uint8_t *pl, uint16_t len)
{
    s_stray_notified = false;

    if (s_active || ota_active()) {
        /* §4 0x08: some OTA (either engine) already owns the partition. */
        notify_error(conn, ERR_ALREADY_IN_OTA);
        return;
    }

    /* Battery guard: >= 30 % unless USB-powered (charger_poll reads the
     * VUSB divider directly — cheap GPIO). battery_status can lose the
     * ADC try-lock to sys_task's 1 Hz read: retry briefly, and refuse
     * (fail-closed) if the cell really cannot be measured. */
    bool vusb = false;
    (void)charger_poll(&vusb);
    if (!vusb) {
        uint8_t pct = 0;
        esp_err_t err = ESP_FAIL;
        for (int i = 0; i < 3 && err != ESP_OK; i++) {
            err = battery_status(NULL, &pct);
            if (err != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        if (err != ESP_OK || pct < DFU_BATT_MIN_PCT) {
            ESP_LOGW(TAG, "BEGIN refused: battery %u%% (err %d)", pct, err);
            notify_error(conn, ERR_LOW_BATTERY);
            return;
        }
    }

    s_part = esp_ota_get_next_update_partition(NULL);
    if (s_part == NULL) {
        notify_error(conn, ERR_NO_PARTITION);
        return;
    }

    /* Optional size_LE32 after the opcode (§2). Absent/zero -> unknown
     * (whole-partition erase, slower READY). Bigger than the slot ->
     * begin would fail anyway; report it as begin-failed. */
    uint32_t size = 0;
    if (len >= 5) {
        size = (uint32_t)pl[1] | ((uint32_t)pl[2] << 8) |
               ((uint32_t)pl[3] << 16) | ((uint32_t)pl[4] << 24);
    }
    if (size > s_part->size) {
        ESP_LOGE(TAG, "BEGIN size %lu > slot %lu", (unsigned long)size,
                 (unsigned long)s_part->size);
        notify_error(conn, ERR_BEGIN_FAILED);
        return;
    }

    /* Stop acquisition for the flash session (sys_task enters
     * NC_STATE_OTA; it stays there while dfu_legacy_active()). */
    const sys_msg_t m = { .type = SYS_ENTER_OTA };
    (void)sys_post(&m);

    /* Long supervision timeout BEFORE the erase stall (§10). */
    dfu_conn_params(conn, true);

    ESP_LOGI(TAG, "BEGIN size=%lu -> %s (erasing...)",
             (unsigned long)size, s_part->label);
    esp_err_t err = esp_ota_begin(s_part, size ? size : OTA_SIZE_UNKNOWN,
                                  &s_ota);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        dfu_conn_params(conn, false);
        notify_error(conn, ERR_BEGIN_FAILED);
        return;
    }

    s_ota_open = true;
    s_img_size = size;
    s_fill = 0;
    s_page_idx = 0;
    s_resends = 0;
    s_wait_confirm = false;
    s_written = 0;
    s_sess_conn = conn;
    s_active = true;
    notify_code(conn, ST_READY);
}

static void op_confirm(uint16_t conn, uint8_t commit)
{
    if (!s_active || !s_wait_confirm) {
        notify_error(conn, ERR_NO_SESSION);    /* §4 0x04 */
        return;
    }

    if (commit == 0x01) {
        /* §7: validate identity on the CONFIRMED page-0 bytes before
         * anything reaches flash. */
        if (s_page_idx == 0 && !dfu_image_hdr_ok(s_page, s_fill)) {
            ESP_LOGE(TAG, "page 0: not a narbis_earclip C6 image");
            session_fail(conn, ERR_CHIP_MISMATCH);
            return;
        }
        esp_err_t err = esp_ota_write(s_ota, s_page, DFU_PAGE_SIZE);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write page %u: %s", s_page_idx,
                     esp_err_to_name(err));
            session_fail(conn, ERR_WRITE_FAILED);
            return;
        }
        s_written += DFU_PAGE_SIZE;
        notify_page(conn, ST_PAGE_OK, s_page_idx);
        s_page_idx++;
        s_fill = 0;
        s_resends = 0;
        s_wait_confirm = false;
    } else {
        /* Central's CRC disagreed — grant up to 3 resends per page. */
        if (s_resends >= DFU_MAX_RESENDS) {
            ESP_LOGE(TAG, "page %u: resend limit", s_page_idx);
            session_fail(conn, ERR_WRITE_FAILED);
            return;
        }
        s_resends++;
        s_fill = 0;
        s_wait_confirm = false;
        notify_page(conn, ST_PAGE_RESEND, s_page_idx);
    }
}

static void op_finish(uint16_t conn)
{
    if (!s_active || s_wait_confirm) {
        /* No session, or a full page's handshake was skipped. */
        notify_error(conn, ERR_NO_SESSION);
        return;
    }

    /* §5: the final partial page is not CRC-handshaked — flush it now.
     * A sub-4096 image means the partial page IS page 0: identity-check
     * it here instead. */
    if (s_fill > 0) {
        if (s_page_idx == 0 && !dfu_image_hdr_ok(s_page, s_fill)) {
            ESP_LOGE(TAG, "final page 0: not a narbis_earclip C6 image");
            session_fail(conn, ERR_CHIP_MISMATCH);
            return;
        }
        esp_err_t werr = esp_ota_write(s_ota, s_page, s_fill);
        if (werr != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write final %lu B: %s",
                     (unsigned long)s_fill, esp_err_to_name(werr));
            session_fail(conn, ERR_WRITE_FAILED);
            return;
        }
        s_written += s_fill;
        s_fill = 0;
    }

    esp_err_t err = esp_ota_end(s_ota);        /* image validation incl. */
    s_ota_open = false;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        session_fail(conn, ERR_FINALIZE_FAILED);
        return;
    }
    err = esp_ota_set_boot_partition(s_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_boot_partition: %s", esp_err_to_name(err));
        session_fail(conn, ERR_FINALIZE_FAILED);
        return;
    }

    ESP_LOGI(TAG, "FINISH: %lu B -> %s, rebooting in 500 ms",
             (unsigned long)s_written, s_part->label);
    session_reset();
    /* Conn params left as-is: the link dies at the restart anyway. */
    notify_code(conn, ST_SUCCESS);
    if (s_restart_tmr != NULL) {
        esp_timer_stop(s_restart_tmr);         /* idle-stop err ignored */
        esp_timer_start_once(s_restart_tmr, DFU_RESTART_DELAY_US);
    } else {
        esp_restart();                         /* timer alloc failed */
    }
}

static void op_cancel(uint16_t conn)
{
    ESP_LOGW(TAG, "CANCEL (active=%d)", (int)s_active);
    session_reset();                           /* idempotent when idle */
    dfu_conn_params(conn, false);
    notify_code(conn, ST_CANCELLED);
}

/* ------------------------------------------------------------------ */
/* Characteristic access (NimBLE host task)                            */
/* ------------------------------------------------------------------ */
static void ctrl_write(uint16_t conn, const uint8_t *d, uint16_t len)
{
    if (len < 1) {
        return;
    }
    s_last_ctrl = d[0];
    switch (d[0]) {
    case OP_BEGIN:
        op_begin(conn, d, len);
        break;
    case OP_PAGE_CONFIRM:
        if (len < 2) {
            ESP_LOGW(TAG, "0xAD without argument — ignored");
        } else {
            op_confirm(conn, d[1]);
        }
        break;
    case OP_FINISH:
        op_finish(conn);
        break;
    case OP_CANCEL:
        op_cancel(conn);
        break;
    default:
        ESP_LOGW(TAG, "unknown opcode 0x%02x", d[0]);
        break;
    }
}

static void data_write(uint16_t conn, const uint8_t *d, uint16_t len)
{
    if (!s_active) {
        /* §4 0x04 — but only once per burst, or a page's worth of stray
         * chunks floods the link with 17 error frames. */
        if (!s_stray_notified) {
            s_stray_notified = true;
            notify_error(conn, ERR_NO_SESSION);
        }
        return;
    }
    if (s_wait_confirm) {
        /* Host must await the CRC handshake before more data. */
        ESP_LOGW(TAG, "DATA during PAGE_CRC handshake — dropped");
        return;
    }

    uint32_t room = DFU_PAGE_SIZE - s_fill;
    uint32_t n = (len < room) ? len : room;
    memcpy(s_page + s_fill, d, n);
    s_fill += n;
    if (n < len) {
        ESP_LOGW(TAG, "page %u overflow: %u B dropped", s_page_idx,
                 (unsigned)(len - n));
    }

    if (s_fill == DFU_PAGE_SIZE) {
        const uint32_t crc = nc_crc32(0, s_page, DFU_PAGE_SIZE);
        s_wait_confirm = true;
        notify_page_crc(conn, s_page_idx, crc);
    }
}

enum { DFU_ID_CTRL, DFU_ID_DATA, DFU_ID_STATUS };

static int dfu_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)attr_handle;
    const int id = (int)(uintptr_t)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        switch (id) {
        case DFU_ID_CTRL:
            return os_mbuf_append(ctxt->om, &s_last_ctrl, 1) == 0
                       ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        case DFU_ID_STATUS:
            return os_mbuf_append(ctxt->om, s_status, s_status_len) == 0
                       ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t *buf = (id == DFU_ID_DATA) ? s_data_buf : s_ctrl_buf;
        const uint16_t cap = (id == DFU_ID_DATA) ? sizeof(s_data_buf)
                                                 : sizeof(s_ctrl_buf);
        if (OS_MBUF_PKTLEN(ctxt->om) > cap) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        uint16_t len = 0;
        if (ble_hs_mbuf_to_flat(ctxt->om, buf, cap, &len) != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        switch (id) {
        case DFU_ID_CTRL:
            ctrl_write(conn_handle, buf, len);
            return 0;
        case DFU_ID_DATA:
            data_write(conn_handle, buf, len);
            return 0;
        default:
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* ------------------------------------------------------------------ */
/* GATT table (§1)                                                     */
/* ------------------------------------------------------------------ */
static struct ble_gatt_chr_def s_dfu_chrs[] = {
    { .uuid = BLE_UUID16_DECLARE(0xFF01), .access_cb = dfu_access,
      .arg = (void *)(uintptr_t)DFU_ID_CTRL,
      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP |
               BLE_GATT_CHR_F_READ,
      .val_handle = &s_vh_ctrl },
    { .uuid = BLE_UUID16_DECLARE(0xFF02), .access_cb = dfu_access,
      .arg = (void *)(uintptr_t)DFU_ID_DATA,
      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
      .val_handle = &s_vh_data },
    { .uuid = BLE_UUID16_DECLARE(0xFF03), .access_cb = dfu_access,
      .arg = (void *)(uintptr_t)DFU_ID_STATUS,
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
      .val_handle = &s_vh_status },
    { 0 },
};

static const struct ble_gatt_svc_def s_dfu_svcs[] = {
    { .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = BLE_UUID16_DECLARE(0x00FF),
      .characteristics = s_dfu_chrs },
    { 0 },
};

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
esp_err_t dfu_legacy_gatt_register(bool encrypted_only)
{
    if (encrypted_only) {
        /* Same hardening as ble_gatt.c chr_harden: production builds
         * demand an encrypted link on every custom characteristic. */
        for (struct ble_gatt_chr_def *c = s_dfu_chrs; c->uuid != NULL; c++) {
            if (c->flags & BLE_GATT_CHR_F_READ) {
                c->flags |= BLE_GATT_CHR_F_READ_ENC;
            }
            if (c->flags & (BLE_GATT_CHR_F_WRITE |
                            BLE_GATT_CHR_F_WRITE_NO_RSP)) {
                c->flags |= BLE_GATT_CHR_F_WRITE_ENC;
            }
            if (c->flags & BLE_GATT_CHR_F_NOTIFY) {
                c->flags |= BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC;
            }
        }
    }

    int rc = ble_gatts_count_cfg(s_dfu_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "count_cfg: rc=%d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(s_dfu_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "add_svcs: rc=%d", rc);
        return ESP_FAIL;
    }

    if (s_restart_tmr == NULL) {
        const esp_timer_create_args_t a = { .callback = restart_cb,
                                            .name = "dfu_rst" };
        esp_err_t err = esp_timer_create(&a, &s_restart_tmr);
        if (err != ESP_OK) {
            return err;                        /* engine still functional */
        }
    }
    ESP_LOGI(TAG, "legacy DFU service 0x00FF registered (%s)",
             encrypted_only ? "encrypted-only" : "open");
    return ESP_OK;
}

void dfu_legacy_on_disconnect(void)
{
    if (s_active) {
        ESP_LOGW(TAG, "disconnect mid-session at page %u — aborting",
                 s_page_idx);
        session_reset();                       /* silent (§3 note) */
    }
    s_stray_notified = false;
}

bool dfu_legacy_active(void)
{
    return s_active;
}
