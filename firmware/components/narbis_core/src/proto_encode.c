/*
 * proto_encode.c — wire codecs for proto.h. Pure C11, no allocation.
 *
 * Layout invariants live in proto.h (_Static_assert-locked); this file
 * only turns structs/samples into bytes and back. Fixture agreement
 * with the python mirror is proven by t_proto_roundtrip.c.
 */
#include "narbis/nc_proto_encode.h"
#include "narbis/nc_knobs.h"

/* ------------------------------------------------------------------ */
/* PPG batch                                                           */
/* ------------------------------------------------------------------ */
void nc_ppg_batch_reset(nc_ppg_batch_t *b, uint32_t seq, uint8_t rate_code, bool amb)
{
    b->seq = seq;
    b->rate_code = rate_code;
    b->amb = amb;
    b->stride = amb ? 12 : 8;
    b->max_n = amb ? NC_PPG_MAX_N_AMB : NC_PPG_MAX_N_NOAMB;
    b->n = 0;
    b->t0_us = 0;
}

bool nc_ppg_batch_add(nc_ppg_batch_t *b, uint64_t t_us, int32_t ir, int32_t red, int32_t amb)
{
    if (b->n >= b->max_n) return false;   /* full: sample NOT stored */
    if (b->n == 0) b->t0_us = t_us;
    uint8_t *p = b->buf + NC_PPG_HDR_SIZE + (size_t)b->n * b->stride;
    nc_wr_i32(p, ir);
    nc_wr_i32(p + 4, red);
    if (b->amb) nc_wr_i32(p + 8, amb);
    b->n++;
    return true;
}

size_t nc_ppg_batch_finish(nc_ppg_batch_t *b, uint8_t flags, uint8_t **out)
{
    /* AMB bit must reflect the actual sample stride or clients misparse. */
    flags = (uint8_t)((flags & (uint8_t)~NC_PPGF_AMB) | (b->amb ? NC_PPGF_AMB : 0));
    nc_wr_u32(b->buf, b->seq);
    nc_wr_u64(b->buf + 4, b->t0_us);
    b->buf[12] = b->rate_code;
    b->buf[13] = b->n;
    b->buf[14] = flags;
    *out = b->buf;
    return NC_PPG_HDR_SIZE + (size_t)b->n * b->stride;
}

/* ------------------------------------------------------------------ */
/* Accel batch                                                         */
/* ------------------------------------------------------------------ */
void nc_accel_batch_reset(nc_accel_batch_t *b, uint32_t seq, uint8_t odr_code)
{
    b->seq = seq;
    b->odr_code = odr_code;
    b->n = 0;
    b->t0_us = 0;
}

bool nc_accel_batch_add(nc_accel_batch_t *b, uint64_t t_us, int16_t x, int16_t y, int16_t z)
{
    if (b->n >= NC_ACCEL_MAX_N) return false;
    if (b->n == 0) b->t0_us = t_us;
    uint8_t *p = b->buf + NC_ACCEL_HDR_SIZE + (size_t)b->n * 6;
    nc_wr_i16(p, x);
    nc_wr_i16(p + 2, y);
    nc_wr_i16(p + 4, z);
    b->n++;
    return true;
}

size_t nc_accel_batch_finish(nc_accel_batch_t *b, uint8_t flags, uint8_t **out)
{
    nc_wr_u32(b->buf, b->seq);
    nc_wr_u64(b->buf + 4, b->t0_us);
    b->buf[12] = b->odr_code;
    b->buf[13] = b->n;
    b->buf[14] = flags;
    *out = b->buf;
    return NC_ACCEL_HDR_SIZE + (size_t)b->n * 6;
}

/* ------------------------------------------------------------------ */
/* IBI / EVENT / STATUS one-shots                                      */
/* ------------------------------------------------------------------ */
size_t nc_enc_ibi(uint8_t *buf, uint32_t seq, const nc_ibi_rec_t *recs, uint8_t n)
{
    if (n > NC_IBI_MAX_N) return 0;
    nc_wr_u32(buf, seq);
    buf[4] = n;
    /* nc_ibi_rec_t is packed+LE: its in-memory image IS the wire image. */
    if (n) memcpy(buf + NC_IBI_HDR_SIZE, recs, (size_t)n * NC_IBI_REC_SIZE);
    return NC_IBI_HDR_SIZE + (size_t)n * NC_IBI_REC_SIZE;
}

size_t nc_enc_event_batch(uint8_t *buf, uint32_t seq, const nc_event_t *evs, uint8_t n)
{
    size_t off = NC_EVENT_HDR_SIZE;
    for (uint8_t i = 0; i < n; i++) {
        const nc_event_t *e = &evs[i];
        if (e->len > sizeof e->data) return 0;
        size_t rec = 2u + 8u + e->len;      /* type + len + t_us + data */
        if (off + rec > NC_ATT_PAYLOAD_MAX) return 0;
        buf[off] = e->type;
        buf[off + 1] = (uint8_t)(8u + e->len);
        nc_wr_u64(buf + off + 2, e->t_us);
        if (e->len) memcpy(buf + off + 10, e->data, e->len);
        off += rec;
    }
    nc_wr_u32(buf, seq);
    buf[4] = n;
    return off;
}

size_t nc_enc_status(uint8_t *buf, const nc_status_t *s)
{
    memcpy(buf, s, sizeof *s);   /* packed 32B, _Static_assert-locked */
    return sizeof *s;
}

/* ------------------------------------------------------------------ */
/* Knob discovery                                                      */
/* ------------------------------------------------------------------ */
/* On-wire fixed part of one record: id(2)+type(1)+flags(1)+4*i32(16)
 * +name_len(1) = 21, then name, then unit_len(1)+unit.
 * NB: proto.h's NC_KNOB_REC_FIXED (19) does not add up for the record
 * layout its own comment documents; the layout is normative, so sizes
 * here are computed from it. Flagged upstream. */
#define KNOB_REC_FIXED_WIRE 21u

size_t nc_enc_knob_discover(uint8_t *buf, size_t buf_cap, uint16_t start_index,
                            uint16_t *next_index)
{
    const uint16_t total = NC_KNOB_COUNT;
    uint16_t first = (start_index > total) ? total : start_index;

    if (buf_cap < NC_KNOB_DISC_HDR_SIZE) {
        *next_index = first;
        return 0;
    }

    size_t off = NC_KNOB_DISC_HDR_SIZE;
    uint8_t n = 0;
    for (uint16_t i = first; i < total && n < 0xFF; i++) {
        const nc_knob_desc_t *d = nc_knob_desc(i);
        size_t name_len = strlen(d->name);   /* <= 22, enforced by t_knobs */
        size_t unit_len = strlen(d->unit);
        size_t rec = KNOB_REC_FIXED_WIRE + name_len + 1u + unit_len;
        if (off + rec > buf_cap) break;      /* whole records only */

        uint8_t *p = buf + off;
        nc_wr_u16(p, d->id);
        p[2] = d->type;
        p[3] = d->flags;
        nc_wr_i32(p + 4, d->min);
        nc_wr_i32(p + 8, d->max);
        nc_wr_i32(p + 12, d->def);
        nc_wr_i32(p + 16, nc_knob_get((int)i));
        p[20] = (uint8_t)name_len;
        memcpy(p + 21, d->name, name_len);
        p[21 + name_len] = (uint8_t)unit_len;
        if (unit_len) memcpy(p + 22 + name_len, d->unit, unit_len);

        off += rec;
        n++;
    }

    nc_wr_u16(buf, total);
    nc_wr_u16(buf + 2, first);
    buf[4] = n;
    *next_index = (uint16_t)(first + n);
    return off;
}

/* ------------------------------------------------------------------ */
/* Decoders                                                            */
/* ------------------------------------------------------------------ */
bool nc_dec_ppg_hdr(const uint8_t *buf, size_t len, nc_ppg_hdr_t *h)
{
    if (len < NC_PPG_HDR_SIZE) return false;
    memcpy(h, buf, NC_PPG_HDR_SIZE);
    size_t stride = (h->flags & NC_PPGF_AMB) ? 12 : 8;
    return len == NC_PPG_HDR_SIZE + (size_t)h->n * stride;
}

bool nc_dec_accel_hdr(const uint8_t *buf, size_t len, nc_accel_hdr_t *h)
{
    if (len < NC_ACCEL_HDR_SIZE) return false;
    memcpy(h, buf, NC_ACCEL_HDR_SIZE);
    return len == NC_ACCEL_HDR_SIZE + (size_t)h->n * 6u;
}

int nc_dec_ibi(const uint8_t *buf, size_t len, uint32_t *seq,
               nc_ibi_rec_t *recs, int max_recs)
{
    if (len < NC_IBI_HDR_SIZE) return -1;
    uint8_t n = buf[4];
    if (len != NC_IBI_HDR_SIZE + (size_t)n * NC_IBI_REC_SIZE) return -1;
    if ((int)n > max_recs) return -1;
    *seq = nc_rd_u32(buf);
    if (n) memcpy(recs, buf + NC_IBI_HDR_SIZE, (size_t)n * NC_IBI_REC_SIZE);
    return (int)n;
}

bool nc_dec_status(const uint8_t *buf, size_t len, nc_status_t *s)
{
    /* STATUS is append-only: accept any length >= the fields we know. */
    if (len < sizeof *s) return false;
    memcpy(s, buf, sizeof *s);
    return true;
}
