/*
 * nc_control.h — CONTROL characteristic dispatcher (pure C11).
 *
 * Parses [u8 op][u8 tid][payload] requests and produces
 * [u8 op|0x80][u8 tid][u8 status][payload] responses. All hardware /
 * OS side effects go through the callback table below, so the full
 * opcode matrix is host-testable against a mock context; knob
 * get/set/discover run entirely in-process via nc_knobs.
 *
 * Serialization (one outstanding request -> NC_ST_BUSY) is the BLE
 * layer's job in main/, not this module's.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "narbis/proto.h"

/* Room left for response payload after the 3-byte envelope. */
#define NC_CTRL_RESP_PAYLOAD_MAX  (NC_ATT_PAYLOAD_MAX - 3)

/*
 * Callback contract:
 *  - A NULL callback means the op is unsupported in this build/state:
 *    the dispatcher answers NC_ST_WRONG_STATE (exceptions: time_sync_seen
 *    is an optional observer, silently skipped when NULL).
 *  - Status-returning callbacks' values are passed to the host verbatim,
 *    so main/ can surface NC_ST_BUSY / NC_ST_NVS_ERR / NC_ST_LOWBATT etc.
 *  - Destructive ops (power_off/reboot/enter_ota/factory_reset) must only
 *    LATCH the action: main/ sends the response first, then acts.
 */
typedef struct nc_ctrl_ctx {
    bool  test_mode;   /* unlocks the 0xE0..0xEF block (NARBIS_TEST_MODE) */
    void *user;        /* opaque, first argument of every callback        */

    nc_ctrl_status_t (*stream_start)(void *u, uint8_t mask);   /* NC_STREAM_MASK_* */
    nc_ctrl_status_t (*stream_stop)(void *u, uint8_t mask);
    nc_ctrl_status_t (*set_rate)(void *u, uint8_t rate_code);  /* nc_rate_t, pre-validated */
    nc_ctrl_status_t (*knob_save)(void *u);                    /* RAM -> NVS         */
    nc_ctrl_status_t (*knob_reset)(void *u, uint8_t scope);    /* called AFTER the RAM
                                    reset; scope 1 additionally erases NVS. Required
                                    for scope 1, optional (re-apply hook) for scope 0. */
    uint64_t (*get_time_us)(void *u);
    void (*time_sync_seen)(void *u, uint64_t host_us, uint64_t dev_us); /* optional */
    nc_ctrl_status_t (*marker)(void *u, uint16_t marker_id);
    nc_ctrl_status_t (*agc_freeze)(void *u, bool freeze);
    nc_ctrl_status_t (*agc_manual)(void *u, uint8_t ir_ma, uint8_t red_ma,
                                   uint8_t rf_code, uint8_t apply_mask);
    nc_ctrl_status_t (*selftest_run)(void *u, uint32_t test_mask);
    /* Already-encoded result blob for chunked replies (0x51, and 0xEA in
     * test mode — main/ decides what the blob holds at any moment).
     * NULL return = no result available yet -> NC_ST_WRONG_STATE. */
    const uint8_t *(*selftest_blob)(void *u, size_t *len);
    nc_ctrl_status_t (*enter_ota)(void *u);
    nc_ctrl_status_t (*power_off)(void *u);
    nc_ctrl_status_t (*reboot)(void *u);
    nc_ctrl_status_t (*factory_reset)(void *u);
    /* 0xE0..0xEF (except 0xEA) when test_mode; resp holds at most
     * NC_CTRL_RESP_PAYLOAD_MAX bytes, *resp_len starts at 0. */
    nc_ctrl_status_t (*test_op)(void *u, uint8_t op, const uint8_t *payload,
                                size_t len, uint8_t *resp, size_t *resp_len);
} nc_ctrl_ctx_t;

/* resp must hold NC_ATT_PAYLOAD_MAX bytes. Returns the response length
 * (>= 3), or 0 only for req_len == 0 (nothing to echo — drop). */
size_t nc_ctrl_dispatch(nc_ctrl_ctx_t *ctx, const uint8_t *req, size_t req_len,
                        uint8_t *resp);
