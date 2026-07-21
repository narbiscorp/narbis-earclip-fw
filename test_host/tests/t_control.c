/*
 * t_control.c — nc_ctrl_dispatch against a mock context: envelope/tid
 * echo, exact-length policy, knob flows, discovery chunk walking,
 * chunked blob replies, factory-reset magic, and 0xE0-block gating.
 */
#include "tst.h"
#include "narbis/nc_control.h"
#include "narbis/nc_proto_encode.h"
#include "narbis/nc_knobs.h"

TST_SUITE("control");

/* ------------------------------------------------------------------ */
/* Mock context: counts invocations, records last args, returns M.rc.  */
/* ------------------------------------------------------------------ */
typedef struct {
    int start_n, stop_n, rate_n, save_n, reset_n, marker_n, freeze_n,
        manual_n, strun_n, blob_n, ota_n, off_n, boot_n, fact_n,
        testop_n, tsync_n, gettime_n;
    uint8_t  last_start_mask, last_stop_mask, last_rate, last_scope;
    uint8_t  last_freeze, last_ir, last_red, last_rf, last_amask, last_testop;
    uint16_t last_marker;
    uint32_t last_strun_mask;
    uint64_t ts_host, ts_dev, now;
    nc_ctrl_status_t rc;
    const uint8_t *blob;
    size_t blob_len;
} mock_t;

static mock_t M;

static nc_ctrl_status_t cb_start(void *u, uint8_t m)   { (void)u; M.start_n++; M.last_start_mask = m; return M.rc; }
static nc_ctrl_status_t cb_stop(void *u, uint8_t m)    { (void)u; M.stop_n++;  M.last_stop_mask = m;  return M.rc; }
static nc_ctrl_status_t cb_rate(void *u, uint8_t c)    { (void)u; M.rate_n++;  M.last_rate = c;       return M.rc; }
static nc_ctrl_status_t cb_save(void *u)               { (void)u; M.save_n++;                         return M.rc; }
static nc_ctrl_status_t cb_reset(void *u, uint8_t s)   { (void)u; M.reset_n++; M.last_scope = s;      return M.rc; }
static uint64_t cb_gettime(void *u)                    { (void)u; M.gettime_n++; return M.now; }
static void cb_tsync(void *u, uint64_t h, uint64_t d)  { (void)u; M.tsync_n++; M.ts_host = h; M.ts_dev = d; }
static nc_ctrl_status_t cb_marker(void *u, uint16_t id){ (void)u; M.marker_n++; M.last_marker = id;   return M.rc; }
static nc_ctrl_status_t cb_freeze(void *u, bool f)     { (void)u; M.freeze_n++; M.last_freeze = f ? 1 : 0; return M.rc; }
static nc_ctrl_status_t cb_manual(void *u, uint8_t ir, uint8_t red, uint8_t rf, uint8_t am)
{ (void)u; M.manual_n++; M.last_ir = ir; M.last_red = red; M.last_rf = rf; M.last_amask = am; return M.rc; }
static nc_ctrl_status_t cb_strun(void *u, uint32_t m)  { (void)u; M.strun_n++; M.last_strun_mask = m; return M.rc; }
static const uint8_t *cb_blob(void *u, size_t *len)    { (void)u; M.blob_n++; *len = M.blob_len; return M.blob; }
static nc_ctrl_status_t cb_ota(void *u)                { (void)u; M.ota_n++;  return M.rc; }
static nc_ctrl_status_t cb_off(void *u)                { (void)u; M.off_n++;  return M.rc; }
static nc_ctrl_status_t cb_boot(void *u)               { (void)u; M.boot_n++; return M.rc; }
static nc_ctrl_status_t cb_fact(void *u)               { (void)u; M.fact_n++; return M.rc; }
static nc_ctrl_status_t cb_testop(void *u, uint8_t op, const uint8_t *pl, size_t len,
                                  uint8_t *resp, size_t *rl)
{
    (void)u;
    M.testop_n++;
    M.last_testop = op;
    if (len) memcpy(resp, pl, len);   /* echo the payload back */
    *rl = len;
    return M.rc;
}

static nc_ctrl_ctx_t CTX;

static nc_ctrl_ctx_t *ctx_full(bool test_mode)
{
    memset(&M, 0, sizeof M);
    M.rc = NC_ST_OK;
    memset(&CTX, 0, sizeof CTX);
    CTX.test_mode = test_mode;
    CTX.stream_start = cb_start;
    CTX.stream_stop = cb_stop;
    CTX.set_rate = cb_rate;
    CTX.knob_save = cb_save;
    CTX.knob_reset = cb_reset;
    CTX.get_time_us = cb_gettime;
    CTX.time_sync_seen = cb_tsync;
    CTX.marker = cb_marker;
    CTX.agc_freeze = cb_freeze;
    CTX.agc_manual = cb_manual;
    CTX.selftest_run = cb_strun;
    CTX.selftest_blob = cb_blob;
    CTX.enter_ota = cb_ota;
    CTX.power_off = cb_off;
    CTX.reboot = cb_boot;
    CTX.factory_reset = cb_fact;
    CTX.test_op = cb_testop;
    return &CTX;
}

static uint8_t RESP[NC_ATT_PAYLOAD_MAX];

static size_t dsp(nc_ctrl_ctx_t *c, uint8_t op, uint8_t tid, const void *pl, size_t n)
{
    uint8_t req[NC_ATT_PAYLOAD_MAX];
    req[0] = op;
    req[1] = tid;
    if (n) memcpy(req + 2, pl, n);
    memset(RESP, 0xEE, sizeof RESP);
    return nc_ctrl_dispatch(c, req, 2 + n, RESP);
}

/* ------------------------------------------------------------------ */
static void test_envelope_and_tid(void)
{
    nc_ctrl_ctx_t *c = ctx_full(false);
    uint8_t mask = 0x0F;
    size_t rl = dsp(c, NC_OP_STREAM_START, 0xA5, &mask, 1);
    CHECK_EQ(rl, 3);
    CHECK_EQ(RESP[0], NC_OP_STREAM_START | NC_OP_RESP_FLAG);
    CHECK_EQ(RESP[1], 0xA5);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(M.start_n, 1);
    CHECK_EQ(M.last_start_mask, 0x0F);

    /* callback status passes through verbatim */
    M.rc = NC_ST_BUSY;
    mask = 0x01;
    dsp(c, NC_OP_STREAM_STOP, 0x11, &mask, 1);
    CHECK_EQ(RESP[0], NC_OP_STREAM_STOP | NC_OP_RESP_FLAG);
    CHECK_EQ(RESP[1], 0x11);
    CHECK_EQ(RESP[2], NC_ST_BUSY);
    CHECK_EQ(M.stop_n, 1);

    /* unknown stream mask bits -> BAD_PARAM before the callback */
    M.rc = NC_ST_OK;
    mask = 0x10;
    dsp(c, NC_OP_STREAM_START, 1, &mask, 1);
    CHECK_EQ(RESP[2], NC_ST_BAD_PARAM);
    CHECK_EQ(M.start_n, 1);
}

static void test_unknown_op(void)
{
    nc_ctrl_ctx_t *c = ctx_full(false);
    CHECK_EQ(dsp(c, 0x7F, 9, NULL, 0), 3);
    CHECK_EQ(RESP[0], 0xFF);
    CHECK_EQ(RESP[2], NC_ST_UNKNOWN_OP);
    dsp(c, 0x05, 9, NULL, 0);
    CHECK_EQ(RESP[2], NC_ST_UNKNOWN_OP);
    /* a "response" opcode is not a valid request */
    dsp(c, 0x81, 9, NULL, 0);
    CHECK_EQ(RESP[2], NC_ST_UNKNOWN_OP);
}

static void test_short_frames(void)
{
    nc_ctrl_ctx_t *c = ctx_full(false);
    uint8_t req[2] = { NC_OP_GET_TIME, 0x33 };
    CHECK_EQ(nc_ctrl_dispatch(c, req, 0, RESP), 0);      /* nothing to echo */
    CHECK_EQ(nc_ctrl_dispatch(c, req, 1, RESP), 3);      /* no tid byte */
    CHECK_EQ(RESP[0], NC_OP_GET_TIME | NC_OP_RESP_FLAG);
    CHECK_EQ(RESP[1], 0);
    CHECK_EQ(RESP[2], NC_ST_BAD_LEN);
}

static void test_truncated_every_op(void)
{
    /* {op, exact payload length} for the whole non-test matrix */
    static const struct { uint8_t op; uint8_t len; } tab[] = {
        { NC_OP_STREAM_START, 1 },   { NC_OP_STREAM_STOP, 1 },
        { NC_OP_SET_RATE, 1 },       { NC_OP_KNOB_GET, 2 },
        { NC_OP_KNOB_SET, 6 },       { NC_OP_KNOB_SAVE, 0 },
        { NC_OP_KNOB_RESET, 1 },     { NC_OP_KNOB_DISCOVER, 2 },
        { NC_OP_TIME_SYNC, 8 },      { NC_OP_GET_TIME, 0 },
        { NC_OP_MARKER, 2 },         { NC_OP_AGC_FREEZE, 1 },
        { NC_OP_AGC_MANUAL, 4 },     { NC_OP_SELFTEST_RUN, 4 },
        { NC_OP_SELFTEST_RESULT, 2 },{ NC_OP_ENTER_OTA, 0 },
        { NC_OP_POWER_OFF, 0 },      { NC_OP_REBOOT, 0 },
        { NC_OP_FACTORY_RESET, 4 },
    };
    nc_ctrl_ctx_t *c = ctx_full(false);
    uint8_t zeros[16] = { 0 };

    for (size_t i = 0; i < sizeof tab / sizeof tab[0]; i++) {
        if (tab[i].len > 0) {
            dsp(c, tab[i].op, 7, zeros, (size_t)tab[i].len - 1);
            CHECK_EQ(RESP[2], NC_ST_BAD_LEN);
        }
        /* over-long is a framing bug too */
        dsp(c, tab[i].op, 7, zeros, (size_t)tab[i].len + 1);
        CHECK_EQ(RESP[2], NC_ST_BAD_LEN);
    }
    /* length policing happens before any side effect */
    CHECK_EQ(M.start_n + M.stop_n + M.rate_n + M.save_n + M.reset_n +
             M.marker_n + M.freeze_n + M.manual_n + M.strun_n + M.blob_n +
             M.ota_n + M.off_n + M.boot_n + M.fact_n + M.gettime_n, 0);

    /* 0xEA in test mode obeys the same rule */
    c = ctx_full(true);
    dsp(c, NC_OP_TEST_REPORT, 7, zeros, 1);
    CHECK_EQ(RESP[2], NC_ST_BAD_LEN);
    CHECK_EQ(M.blob_n, 0);
}

/* ------------------------------------------------------------------ */
static void test_knob_get_set(void)
{
    nc_ctrl_ctx_t *c = ctx_full(false);
    nc_knobs_init();
    uint8_t pl[6];

    /* GET ppg_rate (0x0301, default 1) */
    nc_wr_u16(pl, 0x0301);
    size_t rl = dsp(c, NC_OP_KNOB_GET, 1, pl, 2);
    CHECK_EQ(rl, 3 + 6);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(nc_rd_u16(RESP + 3), 0x0301);
    CHECK_EQ(nc_rd_i32(RESP + 5), 1);

    /* GET unknown id */
    nc_wr_u16(pl, 0x7777);
    CHECK_EQ(dsp(c, NC_OP_KNOB_GET, 1, pl, 2), 3);
    CHECK_EQ(RESP[2], NC_ST_BAD_PARAM);

    /* SET valid */
    nc_wr_u16(pl, 0x0301);
    nc_wr_i32(pl + 2, 4);
    dsp(c, NC_OP_KNOB_SET, 2, pl, 6);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(nc_knob_get_id(0x0301), 4);

    /* SET out of range (ppg_rate max 4): value untouched */
    nc_wr_i32(pl + 2, 5);
    dsp(c, NC_OP_KNOB_SET, 2, pl, 6);
    CHECK_EQ(RESP[2], NC_ST_OUT_OF_RANGE);
    CHECK_EQ(nc_knob_get_id(0x0301), 4);

    /* SET a NC_KF_REBOOT knob (open_pairing 0x0202) -> NEEDS_RESTART, stored */
    nc_wr_u16(pl, 0x0202);
    nc_wr_i32(pl + 2, 1);
    dsp(c, NC_OP_KNOB_SET, 2, pl, 6);
    CHECK_EQ(RESP[2], NC_ST_NEEDS_RESTART);
    CHECK_EQ(nc_knob_get_id(0x0202), 1);

    /* SET unknown id */
    nc_wr_u16(pl, 0x7777);
    dsp(c, NC_OP_KNOB_SET, 2, pl, 6);
    CHECK_EQ(RESP[2], NC_ST_BAD_PARAM);

    nc_knobs_init();
}

static void test_knob_save_reset(void)
{
    nc_ctrl_ctx_t *c = ctx_full(false);
    nc_knobs_init();
    uint8_t pl[6];

    dsp(c, NC_OP_KNOB_SAVE, 3, NULL, 0);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(M.save_n, 1);

    M.rc = NC_ST_NVS_ERR;
    dsp(c, NC_OP_KNOB_SAVE, 3, NULL, 0);
    CHECK_EQ(RESP[2], NC_ST_NVS_ERR);
    CHECK_EQ(M.save_n, 2);
    M.rc = NC_ST_OK;

    /* RESET scope 0 restores defaults and notifies the callback */
    nc_wr_u16(pl, 0x0301);
    nc_wr_i32(pl + 2, 3);
    dsp(c, NC_OP_KNOB_SET, 3, pl, 6);
    CHECK_EQ(nc_knob_get_id(0x0301), 3);
    uint8_t scope = 0;
    dsp(c, NC_OP_KNOB_RESET, 3, &scope, 1);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(M.reset_n, 1);
    CHECK_EQ(M.last_scope, 0);
    CHECK_EQ(nc_knob_get_id(0x0301), 1);

    scope = 1;
    dsp(c, NC_OP_KNOB_RESET, 3, &scope, 1);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(M.last_scope, 1);

    scope = 2;
    dsp(c, NC_OP_KNOB_RESET, 3, &scope, 1);
    CHECK_EQ(RESP[2], NC_ST_BAD_PARAM);
    CHECK_EQ(M.reset_n, 2);

    /* scope 1 without a persistence callback: refused, RAM untouched */
    c->knob_reset = NULL;
    nc_wr_u16(pl, 0x0301);
    nc_wr_i32(pl + 2, 3);
    dsp(c, NC_OP_KNOB_SET, 3, pl, 6);
    scope = 1;
    dsp(c, NC_OP_KNOB_RESET, 3, &scope, 1);
    CHECK_EQ(RESP[2], NC_ST_WRONG_STATE);
    CHECK_EQ(nc_knob_get_id(0x0301), 3);

    /* scope 0 without the callback is still a pure-RAM reset */
    scope = 0;
    dsp(c, NC_OP_KNOB_RESET, 3, &scope, 1);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(nc_knob_get_id(0x0301), 1);
}

/* ------------------------------------------------------------------ */
/* Walk the discovery chunking at a given cap; returns knobs covered.  */
static int walk_discover(size_t cap)
{
    static uint8_t buf[NC_ATT_PAYLOAD_MAX];
    uint16_t start = 0, next = 0;
    int covered = 0;
    int32_t last_id = -1;

    while (start < NC_KNOB_COUNT) {
        size_t len = nc_enc_knob_discover(buf, cap, start, &next);
        if (len < NC_KNOB_DISC_HDR_SIZE) return -1;
        CHECK_EQ(nc_rd_u16(buf), NC_KNOB_COUNT);
        CHECK_EQ(nc_rd_u16(buf + 2), start);
        uint8_t n = buf[4];
        CHECK_EQ(next, start + n);
        if (n == 0) return covered;      /* stall: cap can't fit a record */

        size_t off = NC_KNOB_DISC_HDR_SIZE;
        for (uint8_t i = 0; i < n; i++) {
            /* no record split: full record must lie inside this chunk */
            CHECK(off + 21 <= len);
            int32_t id = nc_rd_u16(buf + off);
            CHECK(id > last_id);         /* ids strictly ascending */
            last_id = id;
            uint8_t nl = buf[off + 20];
            CHECK(off + 21u + nl + 1u <= len);
            uint8_t ul = buf[off + 21 + nl];
            off += 21u + nl + 1u + ul;
            CHECK(off <= len);
        }
        CHECK_EQ(off, len);              /* chunk holds whole records only */
        covered += n;
        start = next;
    }
    return covered;
}

static void test_discover_chunking(void)
{
    nc_knobs_init();

    CHECK_EQ(walk_discover(244), NC_KNOB_COUNT);
    CHECK_EQ(walk_discover(100), NC_KNOB_COUNT);

    /* cap 23: 23 - 5 hdr = 18 < the 21-byte fixed record part, so no
     * record can EVER fit -> encoder must signal a clean stall (n = 0,
     * next == start) instead of splitting a record or looping. */
    CHECK_EQ(walk_discover(23), 0);
    static uint8_t buf[64];
    uint16_t next = 0xBEEF;
    size_t len = nc_enc_knob_discover(buf, 23, 0, &next);
    CHECK_EQ(len, NC_KNOB_DISC_HDR_SIZE);
    CHECK_EQ(buf[4], 0);
    CHECK_EQ(next, 0);

    /* cap below even the chunk header */
    CHECK_EQ(nc_enc_knob_discover(buf, 4, 0, &next), 0);

    /* start beyond the table clamps to an empty terminal chunk */
    len = nc_enc_knob_discover(buf, 64, 9999, &next);
    CHECK_EQ(len, NC_KNOB_DISC_HDR_SIZE);
    CHECK_EQ(nc_rd_u16(buf + 2), NC_KNOB_COUNT);
    CHECK_EQ(buf[4], 0);
    CHECK_EQ(next, NC_KNOB_COUNT);

    /* and through the dispatcher envelope */
    nc_ctrl_ctx_t *c = ctx_full(false);
    uint8_t pl[2];
    nc_wr_u16(pl, 0);
    size_t rl = dsp(c, NC_OP_KNOB_DISCOVER, 4, pl, 2);
    CHECK(rl > 3 + NC_KNOB_DISC_HDR_SIZE);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(nc_rd_u16(RESP + 3), NC_KNOB_COUNT);
    CHECK_EQ(nc_rd_u16(RESP + 5), 0);
    CHECK(RESP[7] > 0);
}

/* ------------------------------------------------------------------ */
static void test_time_ops(void)
{
    nc_ctrl_ctx_t *c = ctx_full(false);
    M.now = 0x0000123456789ABCull;

    uint8_t pl[8];
    nc_wr_u64(pl, 0x0011223344556677ull);
    size_t rl = dsp(c, NC_OP_TIME_SYNC, 5, pl, 8);
    CHECK_EQ(rl, 3 + 16);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(nc_rd_u64(RESP + 3), 0x0011223344556677ull);   /* echo */
    CHECK_EQ(nc_rd_u64(RESP + 11), M.now);
    CHECK_EQ(M.tsync_n, 1);
    CHECK_EQ(M.ts_host, 0x0011223344556677ull);
    CHECK_EQ(M.ts_dev, M.now);

    /* time_sync_seen is optional */
    c->time_sync_seen = NULL;
    rl = dsp(c, NC_OP_TIME_SYNC, 5, pl, 8);
    CHECK_EQ(rl, 3 + 16);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(M.tsync_n, 1);

    rl = dsp(c, NC_OP_GET_TIME, 6, NULL, 0);
    CHECK_EQ(rl, 3 + 8);
    CHECK_EQ(nc_rd_u64(RESP + 3), M.now);

    /* no clock source -> WRONG_STATE */
    c->get_time_us = NULL;
    dsp(c, NC_OP_TIME_SYNC, 5, pl, 8);
    CHECK_EQ(RESP[2], NC_ST_WRONG_STATE);
    dsp(c, NC_OP_GET_TIME, 6, NULL, 0);
    CHECK_EQ(RESP[2], NC_ST_WRONG_STATE);
}

static void test_marker_agc(void)
{
    nc_ctrl_ctx_t *c = ctx_full(false);
    uint8_t pl[4];

    nc_wr_u16(pl, 0x0102);
    dsp(c, NC_OP_MARKER, 1, pl, 2);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(M.last_marker, 0x0102);

    pl[0] = 1;
    dsp(c, NC_OP_AGC_FREEZE, 2, pl, 1);
    CHECK_EQ(M.freeze_n, 1);
    CHECK_EQ(M.last_freeze, 1);
    pl[0] = 0;
    dsp(c, NC_OP_AGC_FREEZE, 2, pl, 1);
    CHECK_EQ(M.last_freeze, 0);

    pl[0] = 10; pl[1] = 8; pl[2] = 3; pl[3] = 0x07;
    dsp(c, NC_OP_AGC_MANUAL, 3, pl, 4);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(M.last_ir, 10);
    CHECK_EQ(M.last_red, 8);
    CHECK_EQ(M.last_rf, 3);
    CHECK_EQ(M.last_amask, 0x07);

    /* undefined apply bits */
    pl[3] = 0x09;
    dsp(c, NC_OP_AGC_MANUAL, 3, pl, 4);
    CHECK_EQ(RESP[2], NC_ST_BAD_PARAM);
    CHECK_EQ(M.manual_n, 1);

    /* RF code out of the 0..7 register range, gain bit set */
    pl[2] = 8; pl[3] = 0x04;
    dsp(c, NC_OP_AGC_MANUAL, 3, pl, 4);
    CHECK_EQ(RESP[2], NC_ST_BAD_PARAM);
    /* ...but ignored when the gain bit is clear */
    pl[3] = 0x03;
    dsp(c, NC_OP_AGC_MANUAL, 3, pl, 4);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(M.manual_n, 2);

    /* rate: validated before the callback */
    pl[0] = NC_RATE_COUNT;
    dsp(c, NC_OP_SET_RATE, 4, pl, 1);
    CHECK_EQ(RESP[2], NC_ST_BAD_PARAM);
    CHECK_EQ(M.rate_n, 0);
    pl[0] = NC_RATE_250;
    dsp(c, NC_OP_SET_RATE, 4, pl, 1);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(M.last_rate, NC_RATE_250);
}

/* ------------------------------------------------------------------ */
static void test_selftest_chunks(void)
{
    nc_ctrl_ctx_t *c = ctx_full(false);
    uint8_t pl[4];

    nc_wr_u32(pl, 0x12345678);
    dsp(c, NC_OP_SELFTEST_RUN, 1, pl, 4);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(M.last_strun_mask, 0x12345678);

    /* 600-byte blob forces 3 chunks at the 236-byte chunk data cap */
    static uint8_t blob[600], rebuilt[600];
    for (int i = 0; i < 600; i++) blob[i] = (uint8_t)((i * 7 + 3) & 0xFF);
    M.blob = blob;
    M.blob_len = sizeof blob;

    static const uint16_t offs[] = { 0, 236, 472 };
    static const int lens[] = { 236, 236, 128 };
    size_t got = 0;
    for (int k = 0; k < 3; k++) {
        nc_wr_u16(pl, offs[k]);
        size_t rl = dsp(c, NC_OP_SELFTEST_RESULT, 2, pl, 2);
        CHECK_EQ(RESP[2], NC_ST_OK);
        CHECK_EQ(nc_rd_u16(RESP + 3), 600);         /* total */
        CHECK_EQ(nc_rd_u16(RESP + 5), offs[k]);     /* offset echo */
        CHECK_EQ(RESP[7], lens[k]);                 /* n */
        CHECK_EQ(rl, 3u + 5u + (size_t)lens[k]);
        memcpy(rebuilt + got, RESP + 8, (size_t)lens[k]);
        got += (size_t)lens[k];
    }
    CHECK_EQ(got, 600);
    CHECK_MEMEQ(rebuilt, blob, sizeof blob);

    /* offset == total: legal empty terminal chunk */
    nc_wr_u16(pl, 600);
    dsp(c, NC_OP_SELFTEST_RESULT, 2, pl, 2);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(RESP[7], 0);

    /* offset past the blob */
    nc_wr_u16(pl, 601);
    dsp(c, NC_OP_SELFTEST_RESULT, 2, pl, 2);
    CHECK_EQ(RESP[2], NC_ST_BAD_PARAM);

    /* no result yet (cb returns NULL) and no cb at all */
    M.blob = NULL;
    M.blob_len = 0;
    nc_wr_u16(pl, 0);
    dsp(c, NC_OP_SELFTEST_RESULT, 2, pl, 2);
    CHECK_EQ(RESP[2], NC_ST_WRONG_STATE);
    c->selftest_blob = NULL;
    dsp(c, NC_OP_SELFTEST_RESULT, 2, pl, 2);
    CHECK_EQ(RESP[2], NC_ST_WRONG_STATE);
}

static void test_power_ops(void)
{
    nc_ctrl_ctx_t *c = ctx_full(false);
    dsp(c, NC_OP_ENTER_OTA, 1, NULL, 0);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(M.ota_n, 1);

    M.rc = NC_ST_LOWBATT;   /* e.g. refuse OTA below the safety floor */
    dsp(c, NC_OP_ENTER_OTA, 1, NULL, 0);
    CHECK_EQ(RESP[2], NC_ST_LOWBATT);
    M.rc = NC_ST_OK;

    dsp(c, NC_OP_POWER_OFF, 2, NULL, 0);
    CHECK_EQ(M.off_n, 1);
    dsp(c, NC_OP_REBOOT, 3, NULL, 0);
    CHECK_EQ(M.boot_n, 1);
}

static void test_factory_reset_magic(void)
{
    nc_ctrl_ctx_t *c = ctx_full(false);
    uint8_t pl[4];

    nc_wr_u32(pl, 0x12345678);
    dsp(c, NC_OP_FACTORY_RESET, 1, pl, 4);
    CHECK_EQ(RESP[2], NC_ST_BAD_PARAM);
    CHECK_EQ(M.fact_n, 0);

    nc_wr_u32(pl, NC_FACTORY_MAGIC);
    dsp(c, NC_OP_FACTORY_RESET, 1, pl, 4);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(M.fact_n, 1);
}

/* ------------------------------------------------------------------ */
static void test_test_mode_gating(void)
{
    /* locked: the whole 0xE0..0xEF block answers UNAUTHORIZED, nothing
     * is parsed, no callback runs */
    nc_ctrl_ctx_t *c = ctx_full(false);
    uint8_t pl[4] = { 5, 0, 0, 0 };
    for (int op = 0xE0; op <= 0xEF; op++) {
        CHECK_EQ(dsp(c, (uint8_t)op, 1, pl, 1), 3);
        CHECK_EQ(RESP[2], NC_ST_UNAUTHORIZED);
    }
    CHECK_EQ(M.testop_n, 0);
    CHECK_EQ(M.blob_n, 0);

    /* unlocked: delegated to test_op, payload echoed back */
    c = ctx_full(true);
    size_t rl = dsp(c, NC_OP_TEST_SELFTEST_ONE, 2, pl, 1);
    CHECK_EQ(rl, 3 + 1);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(RESP[3], 5);
    CHECK_EQ(M.last_testop, NC_OP_TEST_SELFTEST_ONE);

    pl[0] = 0; pl[1] = 25; nc_wr_u16(pl + 2, 500);
    rl = dsp(c, NC_OP_TEST_LED_DRIVE, 2, pl, 4);
    CHECK_EQ(rl, 3 + 4);
    CHECK_MEMEQ(RESP + 3, pl, 4);
    CHECK_EQ(M.testop_n, 2);

    /* undefined test op still goes to test_op (it owns the sub-decode) */
    dsp(c, 0xEB, 2, NULL, 0);
    CHECK_EQ(M.last_testop, 0xEB);

    /* 0xEA = TEST_REPORT: chunked from the blob, NOT via test_op */
    static uint8_t blob[40];
    for (int i = 0; i < 40; i++) blob[i] = (uint8_t)(0xC0 + i);
    M.blob = blob;
    M.blob_len = sizeof blob;
    int before = M.testop_n;
    nc_wr_u16(pl, 0);
    rl = dsp(c, NC_OP_TEST_REPORT, 3, pl, 2);
    CHECK_EQ(RESP[2], NC_ST_OK);
    CHECK_EQ(nc_rd_u16(RESP + 3), 40);
    CHECK_EQ(RESP[7], 40);
    CHECK_MEMEQ(RESP + 8, blob, 40);
    CHECK_EQ(rl, 3u + 5u + 40u);
    CHECK_EQ(M.testop_n, before);

    /* unlocked but no test_op wired -> WRONG_STATE */
    c->test_op = NULL;
    dsp(c, NC_OP_TEST_SELFTEST_ONE, 2, pl, 1);
    CHECK_EQ(RESP[2], NC_ST_WRONG_STATE);
}

static void test_null_callbacks(void)
{
    /* bare context: everything hardware-backed answers WRONG_STATE,
     * knob get/set/discover still work (pure nc_knobs) */
    memset(&M, 0, sizeof M);
    memset(&CTX, 0, sizeof CTX);
    nc_knobs_init();
    uint8_t pl[8] = { 0 };

    pl[0] = 0x01;
    dsp(&CTX, NC_OP_STREAM_START, 1, pl, 1);
    CHECK_EQ(RESP[2], NC_ST_WRONG_STATE);
    dsp(&CTX, NC_OP_KNOB_SAVE, 1, NULL, 0);
    CHECK_EQ(RESP[2], NC_ST_WRONG_STATE);
    dsp(&CTX, NC_OP_POWER_OFF, 1, NULL, 0);
    CHECK_EQ(RESP[2], NC_ST_WRONG_STATE);

    /* param check outranks the missing callback */
    pl[0] = 9;
    dsp(&CTX, NC_OP_SET_RATE, 1, pl, 1);
    CHECK_EQ(RESP[2], NC_ST_BAD_PARAM);

    nc_wr_u16(pl, 0x0301);
    size_t rl = dsp(&CTX, NC_OP_KNOB_GET, 1, pl, 2);
    CHECK_EQ(rl, 9);
    CHECK_EQ(RESP[2], NC_ST_OK);

    nc_wr_u16(pl, 0);
    rl = dsp(&CTX, NC_OP_KNOB_DISCOVER, 1, pl, 2);
    CHECK(rl > 8);
    CHECK_EQ(RESP[2], NC_ST_OK);
}

/* ------------------------------------------------------------------ */
int main(void)
{
    TST_RUN(test_envelope_and_tid);
    TST_RUN(test_unknown_op);
    TST_RUN(test_short_frames);
    TST_RUN(test_truncated_every_op);
    TST_RUN(test_knob_get_set);
    TST_RUN(test_knob_save_reset);
    TST_RUN(test_discover_chunking);
    TST_RUN(test_time_ops);
    TST_RUN(test_marker_agc);
    TST_RUN(test_selftest_chunks);
    TST_RUN(test_power_ops);
    TST_RUN(test_factory_reset_magic);
    TST_RUN(test_test_mode_gating);
    TST_RUN(test_null_callbacks);
    TST_REPORT();
}
