/*
 * control_dispatch.c — CONTROL opcode dispatcher. Pure C11.
 *
 * Check order per op: exact length -> parameter validity -> callback
 * presence -> invoke. Payload lengths are EXACT-match (a long payload is
 * as much a framing bug as a short one). Responses always carry the
 * [op|0x80][tid][status] envelope so the host can correlate by tid even
 * on errors.
 */
#include "narbis/nc_control.h"
#include "narbis/nc_proto_encode.h"
#include "narbis/nc_knobs.h"

/* Chunk payload: [u16 total][u16 offset][u8 n][n bytes] */
#define CHUNK_HDR   5u
#define CHUNK_DATA_MAX (NC_CTRL_RESP_PAYLOAD_MAX - CHUNK_HDR)

/* Shared by SELFTEST_RESULT and TEST_REPORT. Returns payload bytes
 * written at dst, or sets *st and returns 0 on error. */
static size_t chunk_from_blob(const nc_ctrl_ctx_t *ctx, uint16_t offset,
                              uint8_t *dst, uint8_t *st)
{
    if (!ctx->selftest_blob) { *st = NC_ST_WRONG_STATE; return 0; }
    size_t blob_len = 0;
    const uint8_t *blob = ctx->selftest_blob(ctx->user, &blob_len);
    if (!blob || blob_len > 0xFFFF) { *st = NC_ST_WRONG_STATE; return 0; }
    if (offset > blob_len) { *st = NC_ST_BAD_PARAM; return 0; }

    size_t n = blob_len - offset;
    if (n > CHUNK_DATA_MAX) n = CHUNK_DATA_MAX;   /* CHUNK_DATA_MAX=236 < 256 */
    nc_wr_u16(dst, (uint16_t)blob_len);
    nc_wr_u16(dst + 2, offset);
    dst[4] = (uint8_t)n;
    if (n) memcpy(dst + CHUNK_HDR, blob + offset, n);
    return CHUNK_HDR + n;
}

size_t nc_ctrl_dispatch(nc_ctrl_ctx_t *ctx, const uint8_t *req, size_t req_len,
                        uint8_t *resp)
{
    if (req_len == 0) return 0;

    uint8_t op = req[0];
    resp[0] = op | NC_OP_RESP_FLAG;
    resp[1] = (req_len >= 2) ? req[1] : 0;

    uint8_t st = NC_ST_OK;
    size_t pl = 0;                       /* response payload at resp+3 */

    if (req_len < 2) { st = NC_ST_BAD_LEN; goto out; }

    const uint8_t *p = req + 2;
    size_t n = req_len - 2;

#define WANT_LEN(want) do { if (n != (size_t)(want)) { st = NC_ST_BAD_LEN; goto out; } } while (0)
#define WANT_CB(cb)    do { if (!ctx->cb) { st = NC_ST_WRONG_STATE; goto out; } } while (0)

    /* TEST block: gate BEFORE any parsing so nothing leaks when locked. */
    if (op >= 0xE0 && op <= 0xEF) {
        if (!ctx->test_mode) { st = NC_ST_UNAUTHORIZED; goto out; }
        if (op == NC_OP_TEST_REPORT) {
            WANT_LEN(2);
            pl = chunk_from_blob(ctx, nc_rd_u16(p), resp + 3, &st);
            goto out;
        }
        WANT_CB(test_op);
        size_t rl = 0;
        st = ctx->test_op(ctx->user, op, p, n, resp + 3, &rl);
        pl = (rl > NC_CTRL_RESP_PAYLOAD_MAX) ? NC_CTRL_RESP_PAYLOAD_MAX : rl;
        goto out;
    }

    switch (op) {
    case NC_OP_STREAM_START:
    case NC_OP_STREAM_STOP: {
        WANT_LEN(1);
        uint8_t mask = p[0];
        if (mask & (uint8_t)~(NC_STREAM_MASK_PPG | NC_STREAM_MASK_ACCEL |
                              NC_STREAM_MASK_IBI | NC_STREAM_MASK_EVENT)) {
            st = NC_ST_BAD_PARAM;
            break;
        }
        if (op == NC_OP_STREAM_START) {
            WANT_CB(stream_start);
            st = ctx->stream_start(ctx->user, mask);
        } else {
            WANT_CB(stream_stop);
            st = ctx->stream_stop(ctx->user, mask);
        }
        break;
    }

    case NC_OP_SET_RATE:
        WANT_LEN(1);
        if (p[0] >= NC_RATE_COUNT) { st = NC_ST_BAD_PARAM; break; }
        WANT_CB(set_rate);
        st = ctx->set_rate(ctx->user, p[0]);
        break;

    case NC_OP_KNOB_GET: {
        WANT_LEN(2);
        uint16_t id = nc_rd_u16(p);
        if (nc_knob_index_of(id) < 0) { st = NC_ST_BAD_PARAM; break; }
        nc_wr_u16(resp + 3, id);
        nc_wr_i32(resp + 5, nc_knob_get_id(id));
        pl = 6;
        break;
    }

    case NC_OP_KNOB_SET:
        WANT_LEN(6);
        /* nc_knobs status verbatim: OK / NEEDS_RESTART / OUT_OF_RANGE /
         * BAD_PARAM (unknown id). */
        st = nc_knob_set_id(nc_rd_u16(p), nc_rd_i32(p + 2));
        break;

    case NC_OP_KNOB_SAVE:
        WANT_LEN(0);
        WANT_CB(knob_save);
        st = ctx->knob_save(ctx->user);
        break;

    case NC_OP_KNOB_RESET: {
        WANT_LEN(1);
        uint8_t scope = p[0];
        if (scope > 1) { st = NC_ST_BAD_PARAM; break; }
        /* Scope 1 needs the NVS-erase callback; refuse BEFORE mutating
         * RAM so a failed request leaves state untouched. */
        if (scope == 1 && !ctx->knob_reset) { st = NC_ST_WRONG_STATE; break; }
        nc_knobs_init();
        if (ctx->knob_reset) st = ctx->knob_reset(ctx->user, scope);
        break;
    }

    case NC_OP_KNOB_DISCOVER: {
        WANT_LEN(2);
        uint16_t next;
        pl = nc_enc_knob_discover(resp + 3, NC_CTRL_RESP_PAYLOAD_MAX,
                                  nc_rd_u16(p), &next);
        break;
    }

    case NC_OP_TIME_SYNC: {
        WANT_LEN(8);
        WANT_CB(get_time_us);
        uint64_t host_us = nc_rd_u64(p);
        uint64_t dev_us = ctx->get_time_us(ctx->user);
        nc_wr_u64(resp + 3, host_us);      /* echo for host-side RTT pairing */
        nc_wr_u64(resp + 11, dev_us);
        pl = 16;
        if (ctx->time_sync_seen) ctx->time_sync_seen(ctx->user, host_us, dev_us);
        break;
    }

    case NC_OP_GET_TIME:
        WANT_LEN(0);
        WANT_CB(get_time_us);
        nc_wr_u64(resp + 3, ctx->get_time_us(ctx->user));
        pl = 8;
        break;

    case NC_OP_MARKER:
        WANT_LEN(2);
        WANT_CB(marker);
        st = ctx->marker(ctx->user, nc_rd_u16(p));
        break;

    case NC_OP_AGC_FREEZE:
        WANT_LEN(1);
        WANT_CB(agc_freeze);
        st = ctx->agc_freeze(ctx->user, p[0] != 0);
        break;

    case NC_OP_AGC_MANUAL: {
        WANT_LEN(4);
        uint8_t mask = p[3];
        if (mask & (uint8_t)~0x07u) { st = NC_ST_BAD_PARAM; break; }
        if ((mask & 0x04u) && p[2] > 7) { st = NC_ST_BAD_PARAM; break; } /* RF code 0..7 */
        WANT_CB(agc_manual);
        st = ctx->agc_manual(ctx->user, p[0], p[1], p[2], mask);
        break;
    }

    case NC_OP_SELFTEST_RUN:
        WANT_LEN(4);
        WANT_CB(selftest_run);
        st = ctx->selftest_run(ctx->user, nc_rd_u32(p));
        break;

    case NC_OP_SELFTEST_RESULT:
        WANT_LEN(2);
        pl = chunk_from_blob(ctx, nc_rd_u16(p), resp + 3, &st);
        break;

    case NC_OP_ENTER_OTA:
        WANT_LEN(0);
        WANT_CB(enter_ota);
        st = ctx->enter_ota(ctx->user);
        break;

    case NC_OP_POWER_OFF:
        WANT_LEN(0);
        WANT_CB(power_off);
        st = ctx->power_off(ctx->user);
        break;

    case NC_OP_REBOOT:
        WANT_LEN(0);
        WANT_CB(reboot);
        st = ctx->reboot(ctx->user);
        break;

    case NC_OP_FACTORY_RESET:
        WANT_LEN(4);
        if (nc_rd_u32(p) != NC_FACTORY_MAGIC) { st = NC_ST_BAD_PARAM; break; }
        WANT_CB(factory_reset);
        st = ctx->factory_reset(ctx->user);
        break;

    default:
        st = NC_ST_UNKNOWN_OP;
        break;
    }

#undef WANT_LEN
#undef WANT_CB

out:
    resp[2] = st;
    return 3 + pl;
}
