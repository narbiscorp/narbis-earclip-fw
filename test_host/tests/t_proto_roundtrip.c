/*
 * t_proto_roundtrip.c — encode->decode identity for every proto.h packet
 * type, budget arithmetic, CRC vector, and shared binary fixtures.
 *
 * Fixtures are written to out/fixtures/c_<name>.bin with the fixed seeds
 * below so the python mirror (tools/tests) can cross-parse them; if
 * out/fixtures/py_<name>.bin exist they are byte-compared here.
 *
 * FIXTURE SEEDS (keep in lockstep with the python generator):
 *   ppg_noamb: seq=1 rate=1 amb=0 n=28  t_us=1000000+10000*i
 *              ir=100000+1111*i  red=-100000-2222*i  finish flags=0x41
 *   ppg_amb:   seq=2 rate=4 amb=1 n=19  t_us=2000000+2000*i
 *              ir=200000+333*i  red=-200000+444*i  amb=40000+555*i
 *              finish flags=0x00 (wire flags = 0x10, AMB forced)
 *   accel:     seq=3 odr=2 n=38  t_us=3000000+20000*i
 *              x=-16000+900*i  y=16000-900*i  z=-100*i  flags=0x01 (FS ±4g)
 *   ibi:       seq=4 n=19  t_beat_us=4000000+800000*i  ibi_ms=800+7*i
 *              confidence=(5*i)%101  flags=i&0x0F
 *   event_all: seq=5, one record per type in enum order, t_us=5000000+1000*i,
 *              data bytes as in fix_events[] below
 *   status:    field values as in fix_status() below
 *   knob_disc_chunk0: nc_enc_knob_discover(cap=244, start=0) over defaults
 */
#include "tst.h"
#include "narbis/nc_proto_encode.h"
#include "narbis/nc_crc32.h"
#include "narbis/nc_knobs.h"

TST_SUITE("proto_roundtrip");

/* ------------------------------------------------------------------ */
static void write_fixture(const char *name, const void *data, size_t len)
{
    char path[128];
    snprintf(path, sizeof path, "out/fixtures/c_%s.bin", name);
    FILE *f = fopen(path, "wb");
    CHECK(f != NULL);
    if (f) {
        CHECK_EQ(fwrite(data, 1, len, f), len);
        fclose(f);
    }
}

/* Load out/fixtures/<file>; returns length or 0 if absent (skip). */
static size_t load_fixture(const char *file, uint8_t *buf, size_t cap)
{
    char path[128];
    snprintf(path, sizeof path, "out/fixtures/%s", file);
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    size_t rd = fread(buf, 1, cap, f);
    fclose(f);
    return rd;
}

/* Byte-compare against the python twin ONLY if it is the same length
 * (i.e. built from the same seeds). The python agent also ships twins
 * built from its own seeds — those are cross-PARSED in
 * test_py_cross_parse instead. Silently skip when absent. */
static void compare_py(const char *name, const void *data, size_t len)
{
    static uint8_t buf[4096];
    char file[96];
    snprintf(file, sizeof file, "py_%s.bin", name);
    size_t rd = load_fixture(file, buf, sizeof buf);
    if (rd == 0) return;
    if (rd == len) CHECK_MEMEQ(buf, data, len);
}

/* ------------------------------------------------------------------ */
static void test_crc32_vector(void)
{
    /* Canonical CRC-32/ISO-HDLC check value (matches python zlib.crc32). */
    CHECK_EQ(nc_crc32(0, "123456789", 9), 0xCBF43926u);
    /* Incremental == one-shot */
    uint32_t c = nc_crc32(0, "1234", 4);
    c = nc_crc32(c, "56789", 5);
    CHECK_EQ(c, 0xCBF43926u);
    CHECK_EQ(nc_crc32(0, "", 0), 0u);
}

static void test_budget_arithmetic(void)
{
    CHECK(NC_PPG_HDR_SIZE + NC_PPG_MAX_N_NOAMB * 8 <= NC_ATT_PAYLOAD_MAX);
    CHECK(NC_PPG_HDR_SIZE + (NC_PPG_MAX_N_NOAMB + 1) * 8 > NC_ATT_PAYLOAD_MAX);
    CHECK(NC_PPG_HDR_SIZE + NC_PPG_MAX_N_AMB * 12 <= NC_ATT_PAYLOAD_MAX);
    CHECK(NC_PPG_HDR_SIZE + (NC_PPG_MAX_N_AMB + 1) * 12 > NC_ATT_PAYLOAD_MAX);
    CHECK(NC_ACCEL_HDR_SIZE + NC_ACCEL_MAX_N * 6 <= NC_ATT_PAYLOAD_MAX);
    CHECK(NC_ACCEL_HDR_SIZE + (NC_ACCEL_MAX_N + 1) * 6 > NC_ATT_PAYLOAD_MAX);
    CHECK(NC_IBI_HDR_SIZE + NC_IBI_MAX_N * NC_IBI_REC_SIZE <= NC_ATT_PAYLOAD_MAX);
    CHECK(NC_IBI_HDR_SIZE + (NC_IBI_MAX_N + 1) * NC_IBI_REC_SIZE > NC_ATT_PAYLOAD_MAX);
    CHECK_EQ(sizeof(nc_status_t), 32);
}

/* ------------------------------------------------------------------ */
static void test_ppg_noamb(void)
{
    static nc_ppg_batch_t b;
    nc_ppg_batch_reset(&b, 1, 1, false);
    CHECK_EQ(nc_ppg_batch_count(&b), 0);

    for (int i = 0; i < NC_PPG_MAX_N_NOAMB; i++) {
        CHECK(nc_ppg_batch_add(&b, 1000000ull + 10000ull * (uint64_t)i,
                               100000 + 1111 * i, -100000 - 2222 * i, 0));
    }
    /* full: 29th add refused, sample not stored */
    CHECK(!nc_ppg_batch_add(&b, 0, 0, 0, 0));
    CHECK_EQ(nc_ppg_batch_count(&b), NC_PPG_MAX_N_NOAMB);

    uint8_t *out;
    size_t len = nc_ppg_batch_finish(&b, 0x41, &out);
    CHECK_EQ(len, NC_PPG_HDR_SIZE + NC_PPG_MAX_N_NOAMB * 8);

    nc_ppg_hdr_t h;
    CHECK(nc_dec_ppg_hdr(out, len, &h));
    CHECK_EQ(h.seq, 1);
    CHECK_EQ(h.t0_us, 1000000);
    CHECK_EQ(h.rate_code, 1);
    CHECK_EQ(h.n, NC_PPG_MAX_N_NOAMB);
    CHECK_EQ(h.flags, 0x41);              /* AMB bit must be clear */

    for (int i = 0; i < h.n; i++) {
        const uint8_t *s = out + NC_PPG_HDR_SIZE + i * 8;
        CHECK_EQ(nc_rd_i32(s), 100000 + 1111 * i);
        CHECK_EQ(nc_rd_i32(s + 4), -100000 - 2222 * i);
    }

    write_fixture("ppg_noamb", out, len);
    compare_py("ppg_noamb", out, len);

    /* decoder rejects length/n mismatch and truncation */
    CHECK(!nc_dec_ppg_hdr(out, len - 1, &h));
    CHECK(!nc_dec_ppg_hdr(out, NC_PPG_HDR_SIZE - 1, &h));
}

static void test_ppg_amb(void)
{
    static nc_ppg_batch_t b;
    nc_ppg_batch_reset(&b, 2, 4, true);
    for (int i = 0; i < NC_PPG_MAX_N_AMB; i++) {
        CHECK(nc_ppg_batch_add(&b, 2000000ull + 2000ull * (uint64_t)i,
                               200000 + 333 * i, -200000 + 444 * i,
                               40000 + 555 * i));
    }
    CHECK(!nc_ppg_batch_add(&b, 0, 0, 0, 0));

    uint8_t *out;
    size_t len = nc_ppg_batch_finish(&b, 0x00, &out);
    CHECK_EQ(len, NC_PPG_HDR_SIZE + NC_PPG_MAX_N_AMB * 12);

    nc_ppg_hdr_t h;
    CHECK(nc_dec_ppg_hdr(out, len, &h));
    CHECK_EQ(h.seq, 2);
    CHECK_EQ(h.t0_us, 2000000);
    CHECK_EQ(h.rate_code, 4);
    CHECK_EQ(h.n, NC_PPG_MAX_N_AMB);
    CHECK_EQ(h.flags, NC_PPGF_AMB);       /* AMB bit forced on */

    for (int i = 0; i < h.n; i++) {
        const uint8_t *s = out + NC_PPG_HDR_SIZE + i * 12;
        CHECK_EQ(nc_rd_i32(s), 200000 + 333 * i);
        CHECK_EQ(nc_rd_i32(s + 4), -200000 + 444 * i);
        CHECK_EQ(nc_rd_i32(s + 8), 40000 + 555 * i);
    }

    write_fixture("ppg_amb", out, len);
    compare_py("ppg_amb", out, len);
}

static void test_ppg_edges(void)
{
    static nc_ppg_batch_t b;
    uint8_t *out;

    /* n = 0: header-only batch, t0 = 0 */
    nc_ppg_batch_reset(&b, 7, 0, false);
    size_t len = nc_ppg_batch_finish(&b, 0, &out);
    CHECK_EQ(len, NC_PPG_HDR_SIZE);
    nc_ppg_hdr_t h;
    CHECK(nc_dec_ppg_hdr(out, len, &h));
    CHECK_EQ(h.n, 0);
    CHECK_EQ(h.t0_us, 0);

    /* t0 comes from the FIRST sample; reset rearms it */
    nc_ppg_batch_reset(&b, 8, 0, false);
    CHECK(nc_ppg_batch_add(&b, 42, 1, 2, 0));
    CHECK(nc_ppg_batch_add(&b, 99, 3, 4, 0));
    len = nc_ppg_batch_finish(&b, 0, &out);
    CHECK(nc_dec_ppg_hdr(out, len, &h));
    CHECK_EQ(h.t0_us, 42);
    CHECK_EQ(h.n, 2);

    nc_ppg_batch_reset(&b, 9, 0, false);
    CHECK_EQ(nc_ppg_batch_count(&b), 0);
    CHECK(nc_ppg_batch_add(&b, 1234, 5, 6, 0));
    len = nc_ppg_batch_finish(&b, 0, &out);
    CHECK(nc_dec_ppg_hdr(out, len, &h));
    CHECK_EQ(h.t0_us, 1234);

    /* an AMB flag passed for a no-amb batch must be scrubbed */
    nc_ppg_batch_reset(&b, 10, 0, false);
    CHECK(nc_ppg_batch_add(&b, 1, 1, 1, 0));
    len = nc_ppg_batch_finish(&b, NC_PPGF_AMB | NC_PPGF_GATE, &out);
    CHECK(nc_dec_ppg_hdr(out, len, &h));
    CHECK_EQ(h.flags, NC_PPGF_GATE);
}

/* ------------------------------------------------------------------ */
static void test_accel(void)
{
    static nc_accel_batch_t b;
    nc_accel_batch_reset(&b, 3, 2);
    for (int i = 0; i < NC_ACCEL_MAX_N; i++) {
        CHECK(nc_accel_batch_add(&b, 3000000ull + 20000ull * (uint64_t)i,
                                 (int16_t)(-16000 + 900 * i),
                                 (int16_t)(16000 - 900 * i),
                                 (int16_t)(-100 * i)));
    }
    CHECK(!nc_accel_batch_add(&b, 0, 0, 0, 0));

    uint8_t *out;
    size_t len = nc_accel_batch_finish(&b, 0x01, &out);   /* FS = ±4g */
    CHECK_EQ(len, NC_ACCEL_HDR_SIZE + NC_ACCEL_MAX_N * 6);

    nc_accel_hdr_t h;
    CHECK(nc_dec_accel_hdr(out, len, &h));
    CHECK_EQ(h.seq, 3);
    CHECK_EQ(h.t0_us, 3000000);
    CHECK_EQ(h.odr_code, 2);
    CHECK_EQ(h.n, NC_ACCEL_MAX_N);
    CHECK_EQ(h.flags & NC_ACCF_FS_MASK, 0x01);

    for (int i = 0; i < h.n; i++) {
        const uint8_t *s = out + NC_ACCEL_HDR_SIZE + i * 6;
        CHECK_EQ(nc_rd_i16(s), (int16_t)(-16000 + 900 * i));
        CHECK_EQ(nc_rd_i16(s + 2), (int16_t)(16000 - 900 * i));
        CHECK_EQ(nc_rd_i16(s + 4), (int16_t)(-100 * i));
    }

    write_fixture("accel", out, len);
    compare_py("accel", out, len);

    /* n = 0 edge + malformed length */
    nc_accel_batch_reset(&b, 11, 5);
    len = nc_accel_batch_finish(&b, 0, &out);
    CHECK_EQ(len, NC_ACCEL_HDR_SIZE);
    CHECK(nc_dec_accel_hdr(out, len, &h));
    CHECK_EQ(h.n, 0);
    CHECK(!nc_dec_accel_hdr(out, len + 1, &h));
}

/* ------------------------------------------------------------------ */
static void test_ibi(void)
{
    static uint8_t buf[NC_ATT_PAYLOAD_MAX];
    nc_ibi_rec_t recs[NC_IBI_MAX_N], back[NC_IBI_MAX_N];

    for (int i = 0; i < NC_IBI_MAX_N; i++) {
        recs[i].t_beat_us = 4000000ull + 800000ull * (uint64_t)i;
        recs[i].ibi_ms = (uint16_t)(800 + 7 * i);
        recs[i].confidence = (uint8_t)((5 * i) % 101);
        recs[i].flags = (uint8_t)(i & 0x0F);
    }

    size_t len = nc_enc_ibi(buf, 4, recs, NC_IBI_MAX_N);
    CHECK_EQ(len, NC_IBI_HDR_SIZE + NC_IBI_MAX_N * NC_IBI_REC_SIZE);

    uint32_t seq = 0;
    int n = nc_dec_ibi(buf, len, &seq, back, NC_IBI_MAX_N);
    CHECK_EQ(n, NC_IBI_MAX_N);
    CHECK_EQ(seq, 4);
    CHECK_MEMEQ(recs, back, sizeof recs);

    write_fixture("ibi", buf, len);
    compare_py("ibi", buf, len);

    /* n = 0, oversize refusal, malformed length, undersized dest */
    CHECK_EQ(nc_enc_ibi(buf, 5, recs, 0), NC_IBI_HDR_SIZE);
    CHECK_EQ(nc_dec_ibi(buf, NC_IBI_HDR_SIZE, &seq, back, 0), 0);
    CHECK_EQ(seq, 5);
    CHECK_EQ(nc_enc_ibi(buf, 6, recs, NC_IBI_MAX_N + 1), 0);
    CHECK_EQ(nc_dec_ibi(buf, 4, &seq, back, NC_IBI_MAX_N), -1);
    len = nc_enc_ibi(buf, 7, recs, 3);
    CHECK_EQ(nc_dec_ibi(buf, len - 1, &seq, back, NC_IBI_MAX_N), -1);
    CHECK_EQ(nc_dec_ibi(buf, len, &seq, back, 2), -1);
}

/* ------------------------------------------------------------------ */
/* One event of every type; data bytes are the fixture seed.           */
typedef struct { uint8_t type, wire_len, dlen; uint8_t data[8]; } ev_fix_t;

static const ev_fix_t fix_events[] = {
    { NC_EV_AGC_STEP,      NC_EVLEN_AGC_STEP,      5, { 0, 10, 12, 5, 5 } },
    { NC_EV_GATE,          NC_EVLEN_GATE,          2, { 1, 0x05 } },
    { NC_EV_WEAR,          NC_EVLEN_WEAR,          1, { 1 } },
    { NC_EV_MARKER,        NC_EVLEN_MARKER,        3, { 1, 0x02, 0x01 } },
    { NC_EV_ERROR,         NC_EVLEN_ERROR,         6, { 0x06, 0x00, 0xEF, 0xBE, 0xAD, 0xDE } },
    { NC_EV_RATE_CHANGE,   NC_EVLEN_RATE_CHANGE,   2, { 1, 4 } },
    { NC_EV_AGC_OFFDAC,    NC_EVLEN_AGC_OFFDAC,    3, { 2, 3, 7 } },
    { NC_EV_SELFTEST_DONE, NC_EVLEN_SELFTEST_DONE, 2, { 7, 1 } },
};
#define N_EV_FIX (sizeof fix_events / sizeof fix_events[0])

static void test_event_all(void)
{
    static uint8_t buf[NC_ATT_PAYLOAD_MAX];
    nc_event_t evs[N_EV_FIX];

    for (size_t i = 0; i < N_EV_FIX; i++) {
        /* NC_EVLEN_* include the 8-byte t_us: internal len = wire - 8 */
        CHECK_EQ(fix_events[i].dlen, fix_events[i].wire_len - 8);
        evs[i].t_us = 5000000ull + 1000ull * (uint64_t)i;
        evs[i].type = fix_events[i].type;
        evs[i].len = fix_events[i].dlen;
        memset(evs[i].data, 0, sizeof evs[i].data);
        memcpy(evs[i].data, fix_events[i].data, fix_events[i].dlen);
    }

    size_t len = nc_enc_event_batch(buf, 5, evs, (uint8_t)N_EV_FIX);
    CHECK(len > 0);
    CHECK_EQ(nc_rd_u32(buf), 5);
    CHECK_EQ(buf[4], N_EV_FIX);

    size_t off = NC_EVENT_HDR_SIZE;
    for (size_t i = 0; i < N_EV_FIX; i++) {
        CHECK_EQ(buf[off], fix_events[i].type);
        CHECK_EQ(buf[off + 1], fix_events[i].wire_len);
        CHECK_EQ(nc_rd_u64(buf + off + 2), 5000000ull + 1000ull * (uint64_t)i);
        CHECK_MEMEQ(buf + off + 10, fix_events[i].data, fix_events[i].dlen);
        off += 2u + buf[off + 1];
    }
    CHECK_EQ(off, len);

    write_fixture("event_all", buf, len);
    compare_py("event_all", buf, len);

    /* n = 0 */
    CHECK_EQ(nc_enc_event_batch(buf, 6, evs, 0), NC_EVENT_HDR_SIZE);
    CHECK_EQ(buf[4], 0);

    /* corrupt internal len -> refused */
    nc_event_t bad = { .t_us = 1, .type = NC_EV_WEAR, .len = 15, .data = { 0 } };
    CHECK_EQ(nc_enc_event_batch(buf, 7, &bad, 1), 0);

    /* overflow of the ATT budget -> refused (11 x max-size = 264 > 244) */
    nc_event_t big[11];
    for (int i = 0; i < 11; i++) {
        big[i].t_us = (uint64_t)i;
        big[i].type = NC_EV_ERROR;
        big[i].len = 14;
        memset(big[i].data, (int)i, 14);
    }
    CHECK_EQ(nc_enc_event_batch(buf, 8, big, 11), 0);
    CHECK(nc_enc_event_batch(buf, 8, big, 9) > 0);  /* 5 + 9*24 = 221 fits */
}

/* ------------------------------------------------------------------ */
static void test_status(void)
{
    static uint8_t buf[64];
    nc_status_t s = {
        .sys_state = 2,               /* STREAMING */
        .flags = 0x4D,                /* CHARGING|USB|WORN|HRS_ACTIVE */
        .batt_mv = 3987,
        .batt_pct = 76,
        .ppg_rate_code = 1,
        .led_ir_ma = 12,
        .led_red_ma = 8,
        .tia_gain_code = 4,
        .tia_cf_code = 2,
        .gate_duty_x100 = 123,
        .notif_drop_count = 42,
        .i2c_err_count = 7,
        .clock_drift_ppm_x10 = -153,
        .uptime_s = 3600,
        .ibi_last_ms = 812,
        .hr_bpm = 74,
        .reserved = { 0 },
    };

    size_t len = nc_enc_status(buf, &s);
    CHECK_EQ(len, 32);

    nc_status_t back;
    CHECK(nc_dec_status(buf, len, &back));
    CHECK_MEMEQ(&s, &back, sizeof s);

    /* append-only: longer accepted, shorter refused */
    buf[32] = 0xAA;
    CHECK(nc_dec_status(buf, 33, &back));
    CHECK_MEMEQ(&s, &back, sizeof s);
    CHECK(!nc_dec_status(buf, 31, &back));

    write_fixture("status", buf, 32);
    compare_py("status", buf, 32);
}

/* ------------------------------------------------------------------ */
static void test_knob_disc_chunk0(void)
{
    static uint8_t buf[NC_ATT_PAYLOAD_MAX];
    nc_knobs_init();   /* fixture is over defaults */

    uint16_t next = 0;
    size_t len = nc_enc_knob_discover(buf, sizeof buf, 0, &next);
    CHECK(len > NC_KNOB_DISC_HDR_SIZE);
    CHECK(len <= sizeof buf);

    CHECK_EQ(nc_rd_u16(buf), NC_KNOB_COUNT);
    CHECK_EQ(nc_rd_u16(buf + 2), 0);
    uint8_t n = buf[4];
    CHECK(n > 0);
    CHECK_EQ(next, n);

    /* every record parses, matches its descriptor, and the cursor lands
     * exactly on len (whole records only) */
    size_t off = NC_KNOB_DISC_HDR_SIZE;
    for (uint8_t i = 0; i < n; i++) {
        const nc_knob_desc_t *d = nc_knob_desc(i);
        CHECK_EQ(nc_rd_u16(buf + off), d->id);
        CHECK_EQ(buf[off + 2], d->type);
        CHECK_EQ(buf[off + 3], d->flags);
        CHECK_EQ(nc_rd_i32(buf + off + 4), d->min);
        CHECK_EQ(nc_rd_i32(buf + off + 8), d->max);
        CHECK_EQ(nc_rd_i32(buf + off + 12), d->def);
        CHECK_EQ(nc_rd_i32(buf + off + 16), d->def);   /* current == default */
        uint8_t nl = buf[off + 20];
        CHECK_EQ(nl, strlen(d->name));
        CHECK_MEMEQ(buf + off + 21, d->name, nl);
        uint8_t ul = buf[off + 21 + nl];
        CHECK_EQ(ul, strlen(d->unit));
        CHECK_MEMEQ(buf + off + 22 + nl, d->unit, ul);
        off += 21u + nl + 1u + ul;
    }
    CHECK_EQ(off, len);

    write_fixture("knob_disc_chunk0", buf, len);
    compare_py("knob_disc_chunk0", buf, len);
}

/* ------------------------------------------------------------------ */
/* Decode whatever the python encoder emitted (its fixtures use its own
 * seeds, incl. 24-bit rail extremes and an unknown event type): every
 * py_* stream fixture must parse cleanly with the C decoders, and the
 * knob-discovery fixture must agree with OUR descriptor table — that is
 * the actual C<->python contract, independent of seed choice.          */
static void test_py_cross_parse(void)
{
    static uint8_t buf[4096];
    size_t len;

    if ((len = load_fixture("py_ppg_noamb.bin", buf, sizeof buf)) > 0) {
        nc_ppg_hdr_t h;
        CHECK(nc_dec_ppg_hdr(buf, len, &h));
        CHECK(!(h.flags & NC_PPGF_AMB));
        CHECK(h.rate_code < NC_RATE_COUNT);
        CHECK(h.n <= NC_PPG_MAX_N_NOAMB);
    }
    if ((len = load_fixture("py_ppg_amb.bin", buf, sizeof buf)) > 0) {
        nc_ppg_hdr_t h;
        CHECK(nc_dec_ppg_hdr(buf, len, &h));
        CHECK(h.flags & NC_PPGF_AMB);
        CHECK(h.n <= NC_PPG_MAX_N_AMB);
    }
    if ((len = load_fixture("py_ppg_empty.bin", buf, sizeof buf)) > 0) {
        nc_ppg_hdr_t h;
        CHECK(nc_dec_ppg_hdr(buf, len, &h));
        CHECK_EQ(h.n, 0);
    }
    if ((len = load_fixture("py_accel.bin", buf, sizeof buf)) > 0) {
        nc_accel_hdr_t h;
        CHECK(nc_dec_accel_hdr(buf, len, &h));
        CHECK(h.odr_code < NC_ODR_COUNT);
    }
    if ((len = load_fixture("py_ibi.bin", buf, sizeof buf)) > 0) {
        nc_ibi_rec_t recs[NC_IBI_MAX_N];
        uint32_t seq;
        int n = nc_dec_ibi(buf, len, &seq, recs, NC_IBI_MAX_N);
        CHECK(n >= 0);
        for (int i = 0; i < n; i++) CHECK(recs[i].confidence <= 100);
    }
    if ((len = load_fixture("py_status.bin", buf, sizeof buf)) > 0) {
        nc_status_t s;
        CHECK(nc_dec_status(buf, len, &s));
        CHECK(s.batt_pct <= 100);
    }
    if ((len = load_fixture("py_event.bin", buf, sizeof buf)) > 0) {
        /* records must be walkable purely via len — including types we
         * do not know (forward compatibility is part of the contract) */
        CHECK(len >= NC_EVENT_HDR_SIZE);
        uint8_t n = buf[4];
        size_t off = NC_EVENT_HDR_SIZE;
        uint8_t seen = 0;
        while (off + 2 <= len && seen < n) {
            uint8_t rl = buf[off + 1];
            CHECK(rl >= 8);                    /* t_us always leads */
            CHECK(off + 2u + rl <= len);
            off += 2u + rl;
            seen++;
        }
        CHECK_EQ(seen, n);
        CHECK_EQ(off, len);
    }
    if ((len = load_fixture("py_knob_disc.bin", buf, sizeof buf)) > 0) {
        CHECK(len >= NC_KNOB_DISC_HDR_SIZE);
        CHECK_EQ(nc_rd_u16(buf), NC_KNOB_COUNT);   /* mirrored table size */
        uint8_t n = buf[4];
        size_t off = NC_KNOB_DISC_HDR_SIZE;
        for (uint8_t i = 0; i < n; i++) {
            CHECK(off + 21 <= len);
            int idx = nc_knob_index_of(nc_rd_u16(buf + off));
            CHECK(idx >= 0);                   /* id exists in our table */
            if (idx >= 0) {
                const nc_knob_desc_t *d = nc_knob_desc((size_t)idx);
                CHECK_EQ(buf[off + 2], d->type);
                CHECK_EQ(buf[off + 3], d->flags);
                CHECK_EQ(nc_rd_i32(buf + off + 4), d->min);
                CHECK_EQ(nc_rd_i32(buf + off + 8), d->max);
                CHECK_EQ(nc_rd_i32(buf + off + 12), d->def);
                uint8_t nl = buf[off + 20];
                CHECK_EQ(nl, strlen(d->name));
                CHECK_MEMEQ(buf + off + 21, d->name, nl);
                uint8_t ul = buf[off + 21 + nl];
                CHECK_EQ(ul, strlen(d->unit));
                CHECK_MEMEQ(buf + off + 22 + nl, d->unit, ul);
                off += 21u + nl + 1u + ul;
            }
        }
        CHECK_EQ(off, len);
    }
}

/* ------------------------------------------------------------------ */
int main(void)
{
    TST_RUN(test_crc32_vector);
    TST_RUN(test_budget_arithmetic);
    TST_RUN(test_ppg_noamb);
    TST_RUN(test_ppg_amb);
    TST_RUN(test_ppg_edges);
    TST_RUN(test_accel);
    TST_RUN(test_ibi);
    TST_RUN(test_event_all);
    TST_RUN(test_status);
    TST_RUN(test_knob_disc_chunk0);
    TST_RUN(test_py_cross_parse);
    TST_REPORT();
}
