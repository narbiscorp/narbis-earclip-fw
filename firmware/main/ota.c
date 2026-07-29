/*
 * ota.c — BLE OTA engine: esp_ota A/B partitions, resumable chunked
 * transfer, whole-image CRC32 readback, rollback self-check.
 *
 * Wire contract (proto.h OTA section):
 *   OTA_CTRL uses the CONTROL envelope: req [op][tid][payload],
 *   resp indication [op|0x80][tid][status][payload].
 *   OTA_DATA (write-no-response): [u32 offset][chunk <= 240].
 *
 * Status byte mapping (generic nc_ctrl_status_t on the envelope, the
 * OTA-specific nc_ota_err_t latched in last_err and readable via
 * NC_OTA_STATUS):
 *   size rejected        -> NC_ST_OUT_OF_RANGE + NC_OTAERR_SIZE
 *   image/CRC invalid    -> NC_ST_CRC_ERR      + NC_OTAERR_IMAGE/_CRC
 *   flash/esp_ota errors -> NC_ST_NVS_ERR      + NC_OTAERR_FLASH
 *   wrong state          -> NC_ST_WRONG_STATE  + NC_OTAERR_STATE
 *
 * Threading: ctrl/data callbacks run in the NimBLE host task,
 * ota_deadline_check() in sys_task, timers in the esp_timer task; a
 * single mutex serializes them. esp_ota_write() of a <=240 B chunk in
 * host-task context is the standard IDF pattern (flash writes are
 * buffered and short); FINISH blocks the host task ~100-200 ms for the
 * 1.6 MB CRC readback — acceptable, the host is waiting on us anyway.
 *
 * Disconnect handling: the engine never hears about disconnects
 * directly. "Interrupted transfer resumable for 60 s" falls out of the
 * deadline check: the esp_ota handle stays open while RECEIVING, and a
 * BEGIN carrying the same {size, crc32} resumes at bytes_rx as long as
 * the 60 s no-data deadline has not yet closed the session.
 */
#include "ota.h"

#include <string.h>
#include <stdlib.h>
#include <stdatomic.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_partition.h"
#include "nvs.h"

#include "narbis/proto.h"
#include "narbis/nc_proto_encode.h"
#include "narbis/nc_crc32.h"

#include "ble_ota_iface.h"
#include "acq.h"
#include "afe4404.h"
#include "dfu_legacy.h"
#include "lis2dh12.h"

static const char *TAG = "ota";

/* ------------------------------------------------------------------ */
/* Engine state (all fields guarded by s_mtx except s_state, which is  */
/* a single volatile byte so ota_active() can read it lock-free).      */
/* ------------------------------------------------------------------ */
static volatile uint8_t        s_state = NC_OTA_IDLE;   /* nc_ota_state_t */
static uint16_t                s_last_err = NC_OTAERR_NONE;
static const esp_partition_t  *s_part;
static esp_ota_handle_t        s_handle;
static bool                    s_handle_open;
static uint32_t                s_img_size;
static uint32_t                s_img_crc;
static uint32_t                s_bytes_rx;
static bool                    s_off_latch;    /* EXPECTED_OFFSET latched:
                                                  drop DATA until GET_STATUS */
static int64_t                 s_last_data_us;
static uint8_t                 s_last_ind_pct;
static uint32_t                s_data_drop;    /* malformed/out-of-state DATA */

static SemaphoreHandle_t       s_mtx;
static esp_timer_handle_t      s_restart_tmr;
static esp_timer_handle_t      s_validate_tmr;

#define OTA_DEADLINE_US        (60LL * 1000 * 1000)
#define OTA_RESTART_DELAY_US   (500LL * 1000)
#define OTA_VALIDATE_DELAY_US  (10LL * 1000 * 1000)
#define OTA_CRC_CHUNK          4096

static void lock(void)   { xSemaphoreTake(s_mtx, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(s_mtx); }

/* ------------------------------------------------------------------ */
/* Response / progress plumbing                                        */
/* ------------------------------------------------------------------ */
static void ota_respond(uint8_t op, uint8_t tid, uint8_t st,
                        const uint8_t *pl, size_t pl_len)
{
    uint8_t buf[3 + 16];
    buf[0] = op | NC_OP_RESP_FLAG;
    buf[1] = tid;
    buf[2] = st;
    if (pl_len) {
        memcpy(buf + 3, pl, pl_len);
    }
    ble_ota_submit_ind(buf, (uint16_t)(3 + pl_len));
}

/* Unsolicited progress: an NC_OTA_STATUS response with tid 0 (tid 0 is
 * reserved for device-originated indications; hosts correlate solicited
 * replies by their own nonzero tid). Payload {u8 state, u32 bytes_rx,
 * u16 last_err} — same shape as a polled GET_STATUS. */
static void ota_progress_ind(void)
{
    uint8_t pl[7];
    pl[0] = s_state;
    nc_wr_u32(pl + 1, s_bytes_rx);
    nc_wr_u16(pl + 5, s_last_err);
    ota_respond(NC_OTA_STATUS, 0, NC_ST_OK, pl, sizeof pl);
}

static void abort_session(uint16_t err)
{
    if (s_handle_open) {
        esp_ota_abort(s_handle);
        s_handle_open = false;
    }
    s_last_err = err;
    s_state = NC_OTA_FAILED;
}

/* ------------------------------------------------------------------ */
/* OTA_CTRL ops                                                        */
/* ------------------------------------------------------------------ */
static void op_begin(uint8_t tid, const uint8_t *p, size_t n)
{
    if (n < 9) {
        ota_respond(NC_OTA_BEGIN, tid, NC_ST_BAD_LEN, NULL, 0);
        return;
    }
    uint32_t size = nc_rd_u32(p);
    uint32_t crc  = nc_rd_u32(p + 4);
    uint8_t  vlen = p[8];
    if (n != (size_t)9 + vlen) {
        ota_respond(NC_OTA_BEGIN, tid, NC_ST_BAD_LEN, NULL, 0);
        return;
    }
    char ver[33] = { 0 };
    memcpy(ver, p + 9, vlen < 32 ? vlen : 32);

    /* Mutual exclusion with the legacy DFU engine (dfu_legacy.c): both
     * run on this same NimBLE host task, so this check cannot race. */
    if (dfu_legacy_active()) {
        ota_respond(NC_OTA_BEGIN, tid, NC_ST_BUSY, NULL, 0);
        return;
    }

    uint8_t st = NC_ST_OK;
    uint8_t pl[4];

    lock();
    if (s_state == NC_OTA_VALIDATING || s_state == NC_OTA_READY) {
        /* Mid-validation / boot-partition already switched: refuse. */
        s_last_err = NC_OTAERR_STATE;
        st = NC_ST_WRONG_STATE;
    } else if (s_state == NC_OTA_RECEIVING &&
               size == s_img_size && crc == s_img_crc) {
        /* Resume: same image, session still open (deadline not fired,
         * so we are within 60 s of the last data — see header note). */
        s_off_latch = false;
        s_last_data_us = esp_timer_get_time();
        nc_wr_u32(pl, s_bytes_rx);
        ESP_LOGI(TAG, "BEGIN resume '%s' at %" PRIu32 "/%" PRIu32,
                 ver, s_bytes_rx, size);
    } else {
        /* Fresh transfer (also the RECEIVING-but-different-image path). */
        if (s_handle_open) {
            esp_ota_abort(s_handle);
            s_handle_open = false;
        }
        s_part = esp_ota_get_next_update_partition(NULL);
        if (s_part == NULL) {
            s_last_err = NC_OTAERR_FLASH;
            s_state = NC_OTA_FAILED;
            st = NC_ST_NVS_ERR;
        } else if (size == 0 || size > s_part->size) {
            s_last_err = NC_OTAERR_SIZE;
            st = NC_ST_OUT_OF_RANGE;
            ESP_LOGE(TAG, "BEGIN size %" PRIu32 " vs partition %" PRIu32,
                     size, (uint32_t)s_part->size);
        } else {
            esp_err_t err = esp_ota_begin(s_part, size, &s_handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
                s_last_err = NC_OTAERR_FLASH;
                s_state = NC_OTA_FAILED;
                st = NC_ST_NVS_ERR;
            } else {
                s_handle_open = true;
                s_img_size = size;
                s_img_crc = crc;
                s_bytes_rx = 0;
                s_off_latch = false;
                s_last_err = NC_OTAERR_NONE;
                s_last_ind_pct = 0;
                s_last_data_us = esp_timer_get_time();
                s_state = NC_OTA_RECEIVING;
                nc_wr_u32(pl, 0);
                ESP_LOGI(TAG, "BEGIN '%s' %" PRIu32 " B crc 0x%08" PRIx32
                         " -> %s", ver, size, crc, s_part->label);
            }
        }
    }
    unlock();
    ota_respond(NC_OTA_BEGIN, tid, st, (st == NC_ST_OK) ? pl : NULL,
                (st == NC_ST_OK) ? 4 : 0);
}

static void op_status(uint8_t tid)
{
    uint8_t pl[7];
    lock();
    s_off_latch = false;              /* host reseeks after reading status */
    pl[0] = s_state;
    nc_wr_u32(pl + 1, s_bytes_rx);
    nc_wr_u16(pl + 5, s_last_err);
    unlock();
    ota_respond(NC_OTA_STATUS, tid, NC_ST_OK, pl, sizeof pl);
}

static void restart_tmr_cb(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "restarting into new image");
    esp_restart();
}

static void op_finish(uint8_t tid)
{
    uint8_t st = NC_ST_OK;

    lock();
    if (s_state != NC_OTA_RECEIVING) {
        s_last_err = NC_OTAERR_STATE;
        st = NC_ST_WRONG_STATE;
        goto out;
    }
    if (s_bytes_rx != s_img_size) {
        s_last_err = NC_OTAERR_SIZE;
        st = NC_ST_BAD_PARAM;
        ESP_LOGE(TAG, "FINISH at %" PRIu32 "/%" PRIu32, s_bytes_rx, s_img_size);
        goto out;
    }
    s_state = NC_OTA_VALIDATING;

    /* 1. esp_ota_end: image header/magic + completeness validation. */
    esp_err_t err = esp_ota_end(s_handle);
    s_handle_open = false;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        s_last_err = NC_OTAERR_IMAGE;
        s_state = NC_OTA_FAILED;
        st = NC_ST_CRC_ERR;
        goto out;
    }

    /* 2. Whole-image CRC32 readback from flash vs the BEGIN crc — this
     * verifies what actually landed in the partition, not what passed
     * through RAM. 4 KB chunks; ~1.6 MB reads back in well under 1 s. */
    uint8_t *buf = malloc(OTA_CRC_CHUNK);
    if (buf == NULL) {
        s_last_err = NC_OTAERR_FLASH;
        s_state = NC_OTA_FAILED;
        st = NC_ST_NVS_ERR;
        goto out;
    }
    uint32_t crc = 0;
    for (uint32_t off = 0; off < s_img_size && err == ESP_OK;
         off += OTA_CRC_CHUNK) {
        uint32_t chunk = s_img_size - off;
        if (chunk > OTA_CRC_CHUNK) {
            chunk = OTA_CRC_CHUNK;
        }
        err = esp_partition_read(s_part, off, buf, chunk);
        if (err == ESP_OK) {
            crc = nc_crc32(crc, buf, chunk);
        }
    }

    /* 2b. Model lock (legacy DFU doc §7, shared dfu_image_hdr_ok):
     * re-read the WRITTEN first 512 B and require magic 0xE9, chip_id
     * 0x000D and project_name "narbis_earclip". A CRC-correct transfer
     * of the wrong build (e.g. an Edge-glasses image) fails here before
     * the boot partition ever switches. */
    bool id_ok = false;
    if (err == ESP_OK) {
        uint32_t hlen = (s_img_size < 512) ? s_img_size : 512;
        err = esp_partition_read(s_part, 0, buf, hlen);
        if (err == ESP_OK) {
            id_ok = dfu_image_hdr_ok(buf, hlen);
        }
    }
    free(buf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "partition read: %s", esp_err_to_name(err));
        s_last_err = NC_OTAERR_FLASH;
        s_state = NC_OTA_FAILED;
        st = NC_ST_NVS_ERR;
        goto out;
    }
    if (crc != s_img_crc) {
        ESP_LOGE(TAG, "CRC 0x%08" PRIx32 " != expected 0x%08" PRIx32,
                 crc, s_img_crc);
        s_last_err = NC_OTAERR_CRC;
        s_state = NC_OTA_FAILED;
        st = NC_ST_CRC_ERR;
        goto out;
    }
    if (!id_ok) {
        ESP_LOGE(TAG, "image identity: not a narbis_earclip ESP32-C6 build");
        s_last_err = NC_OTAERR_IMAGE;
        s_state = NC_OTA_FAILED;
        st = NC_ST_BAD_PARAM;
        goto out;
    }

    /* 3. App-descriptor sanity (magic word checked inside; logs version). */
    esp_app_desc_t desc;
    err = esp_ota_get_partition_description(s_part, &desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no valid app descriptor: %s", esp_err_to_name(err));
        s_last_err = NC_OTAERR_IMAGE;
        s_state = NC_OTA_FAILED;
        st = NC_ST_CRC_ERR;
        goto out;
    }
    ESP_LOGI(TAG, "image valid: %s (%s %s)", desc.version, desc.date,
             desc.time);

    err = esp_ota_set_boot_partition(s_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_boot_partition: %s", esp_err_to_name(err));
        s_last_err = NC_OTAERR_FLASH;
        s_state = NC_OTA_FAILED;
        st = NC_ST_NVS_ERR;
        goto out;
    }
    s_last_err = NC_OTAERR_NONE;
    s_state = NC_OTA_READY;

out:
    unlock();
    /* Indicate first, restart after 500 ms so the indication drains. */
    ota_respond(NC_OTA_FINISH, tid, st, NULL, 0);
    if (st == NC_ST_OK) {
        esp_timer_start_once(s_restart_tmr, OTA_RESTART_DELAY_US);
    }
}

static void op_abort(uint8_t tid)
{
    lock();
    if (s_handle_open) {
        esp_ota_abort(s_handle);
        s_handle_open = false;
    }
    s_state = NC_OTA_IDLE;
    s_last_err = NC_OTAERR_NONE;
    s_bytes_rx = 0;
    s_off_latch = false;
    unlock();
    ESP_LOGW(TAG, "transfer aborted by host");
    ota_respond(NC_OTA_ABORT, tid, NC_ST_OK, NULL, 0);
}

/* ------------------------------------------------------------------ */
/* GATT callbacks (NimBLE host task context)                           */
/* ------------------------------------------------------------------ */
static void ota_ctrl_cb(const uint8_t *req, uint16_t len)
{
    if (len < 2) {
        return;                       /* not even [op][tid] — drop */
    }
    uint8_t op = req[0];
    uint8_t tid = req[1];
    const uint8_t *p = req + 2;
    size_t n = len - 2;

    switch (op) {
    case NC_OTA_BEGIN:
        op_begin(tid, p, n);
        break;
    case NC_OTA_STATUS:
        if (n != 0) {
            ota_respond(op, tid, NC_ST_BAD_LEN, NULL, 0);
        } else {
            op_status(tid);
        }
        break;
    case NC_OTA_FINISH:
        if (n != 0) {
            ota_respond(op, tid, NC_ST_BAD_LEN, NULL, 0);
        } else {
            op_finish(tid);
        }
        break;
    case NC_OTA_ABORT:
        op_abort(tid);
        break;
    default:
        ota_respond(op, tid, NC_ST_UNKNOWN_OP, NULL, 0);
        break;
    }
}

static void ota_data_cb(const uint8_t *data, uint16_t len)
{
    /* [u32 offset][chunk 1..240]; write-no-response, so errors are only
     * latched (host discovers them via GET_STATUS). */
    if (len < 5 || (size_t)len - 4 > NC_OTA_CHUNK_MAX) {
        s_data_drop++;
        return;
    }
    uint32_t off = nc_rd_u32(data);
    uint16_t n = len - 4;

    lock();
    if (s_state != NC_OTA_RECEIVING) {
        s_data_drop++;
        unlock();
        return;
    }
    s_last_data_us = esp_timer_get_time();   /* link is alive */
    if (s_off_latch) {
        unlock();
        return;
    }
    if (off != s_bytes_rx) {
        /* Idempotent reseek protocol: latch, drop everything until the
         * host polls GET_STATUS and restarts from bytes_rx. */
        s_off_latch = true;
        s_last_err = NC_OTAERR_EXPECTED_OFFSET;
        ESP_LOGW(TAG, "offset %" PRIu32 " != expected %" PRIu32,
                 off, s_bytes_rx);
        unlock();
        return;
    }
    if (s_bytes_rx + n > s_img_size) {
        s_off_latch = true;
        s_last_err = NC_OTAERR_SIZE;
        unlock();
        return;
    }
    esp_err_t err = esp_ota_write(s_handle, data + 4, n);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write @%" PRIu32 ": %s", off,
                 esp_err_to_name(err));
        abort_session(NC_OTAERR_FLASH);
        unlock();
        return;
    }
    s_bytes_rx += n;

    uint8_t pct = (uint8_t)(((uint64_t)s_bytes_rx * 100) / s_img_size);
    bool ind = (pct >= s_last_ind_pct + 5) || (pct == 100);
    if (ind) {
        s_last_ind_pct = pct - (pct % 5);
        ota_progress_ind();
    }
    unlock();
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
esp_err_t ota_engine_init(void)
{
    s_mtx = xSemaphoreCreateMutex();
    if (s_mtx == NULL) {
        return ESP_ERR_NO_MEM;
    }
    const esp_timer_create_args_t rst = {
        .callback = restart_tmr_cb,
        .name = "ota_rst",
    };
    esp_err_t err = esp_timer_create(&rst, &s_restart_tmr);
    if (err != ESP_OK) {
        return err;
    }
    /* Return value deliberately unused: works whether ble_ota_iface.h
     * declares this void or esp_err_t. */
    ble_ota_register(ota_ctrl_cb, ota_data_cb);
    const esp_partition_t *run = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "engine up, running from %s", run ? run->label : "?");
    return ESP_OK;
}

bool ota_active(void)
{
    uint8_t st = s_state;
    return st == NC_OTA_RECEIVING || st == NC_OTA_VALIDATING ||
           st == NC_OTA_READY;
}

void ota_deadline_check(void)
{
    lock();
    if (s_state == NC_OTA_RECEIVING &&
        esp_timer_get_time() - s_last_data_us > OTA_DEADLINE_US) {
        ESP_LOGE(TAG, "no data for 60 s at %" PRIu32 "/%" PRIu32
                 " — aborting", s_bytes_rx, s_img_size);
        abort_session(NC_OTAERR_STATE);
    }
    unlock();
}

/* ------------------------------------------------------------------ */
/* Post-boot rollback gate                                             */
/* ------------------------------------------------------------------ */
/*
 * Runs 10 s after boot in the esp_timer task so that surviving BLE
 * init, advertising and the task WDT for 10 s is itself part of the
 * check (a boot-looping image never reaches this callback with a
 * healthy system).
 *
 * Checks (each must pass, else rollback):
 *  - NVS mounted (nvs_get_stats on the default partition);
 *  - heap sanity (> 16 KB free — a leaking/corrupt image fails this);
 *  - LIS2DH12 alive: if the accel pipeline is running, frames flowing
 *    is the proof; otherwise lis2dh12_init() re-probes WHO_AM_I==0x33
 *    (idempotent: re-applies CTRL_REG0 and leaves power-down, which IS
 *    the not-running state);
 *  - AFE4404 alive: if PPG is running, live frames are the proof;
 *    otherwise a full rev-proof afe4404_init() (dozens of acked config
 *    writes) followed by afe4404_powerdown_hw() to return to the idle
 *    state. The CONTROL2 readback suggested by the plan is skipped
 *    deliberately: afe4404_reg_read() is contract-limited to TIMEREN=0
 *    and init leaves the timer running — init success is the stronger
 *    probe anyway.
 *
 * CONTEXT SPLIT: the esp_timer callback only marks the check due; the
 * I2C/GPIO probes run in ota_self_check_poll() from sys_task's 1 Hz
 * tick — sys context serializes them against acq_ppg_start/stop (an
 * esp_timer-context afe4404_init could interleave with a live stream
 * start and then power the AFE down under it).
 */
static _Atomic bool s_selfcheck_due;

static void validate_tmr_cb(void *arg)
{
    (void)arg;
    atomic_store(&s_selfcheck_due, true);
}

void ota_self_check_poll(void)
{
    if (!atomic_exchange(&s_selfcheck_due, false)) {
        return;
    }
    bool ok = true;

    nvs_stats_t stats;
    if (nvs_get_stats(NULL, &stats) != ESP_OK) {
        ESP_LOGE(TAG, "self-check: NVS not mounted");
        ok = false;
    }
    if (esp_get_free_heap_size() < 16384) {
        ESP_LOGE(TAG, "self-check: heap exhausted (%" PRIu32 ")",
                 esp_get_free_heap_size());
        ok = false;
    }
    if (ok && !acq_accel_running()) {
        if (lis2dh12_init() != ESP_OK) {
            ESP_LOGE(TAG, "self-check: LIS2DH12 probe failed");
            ok = false;
        }
    }
    if (ok && !acq_ppg_running()) {
        if (afe4404_init(NC_RATE_100) != ESP_OK) {
            ESP_LOGE(TAG, "self-check: AFE4404 bring-up failed");
            ok = false;
        }
        afe4404_powerdown_hw();
    }

    if (ok) {
        ESP_LOGI(TAG, "post-OTA self-check PASS — image marked valid");
        esp_ota_mark_app_valid_cancel_rollback();
    } else {
        ESP_LOGE(TAG, "post-OTA self-check FAIL — rolling back");
        esp_ota_mark_app_invalid_rollback_and_reboot();
        /* not reached */
    }
}

void ota_boot_validate(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (run == NULL || esp_ota_get_state_partition(run, &st) != ESP_OK) {
        return;
    }
    if (st != ESP_OTA_IMG_PENDING_VERIFY) {
        return;
    }
    ESP_LOGW(TAG, "image PENDING_VERIFY — self-check in 10 s");
    const esp_timer_create_args_t val = {
        .callback = validate_tmr_cb,
        .name = "ota_val",
    };
    if (esp_timer_create(&val, &s_validate_tmr) == ESP_OK) {
        esp_timer_start_once(s_validate_tmr, OTA_VALIDATE_DELAY_US);
    } else {
        /* Can't schedule: mark due immediately — the sys 1 Hz poll
         * runs the check on its next tick. */
        atomic_store(&s_selfcheck_due, true);
    }
}
