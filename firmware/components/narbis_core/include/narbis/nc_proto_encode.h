/*
 * nc_proto_encode.h — wire codecs for the proto.h packet layouts.
 *
 * Pure C11 (host-testable). Both RV32 target and x86-64 host are
 * little-endian, so the scalar accessors below are memcpy-based: they
 * compile to plain loads/stores yet stay legal on unaligned wire offsets.
 *
 * Batch builders write samples directly into an ATT-sized buffer and
 * back-fill the header at finish time, so the notify path is a single
 * pointer hand-off with zero copies.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "narbis/proto.h"

/* ------------------------------------------------------------------ */
/* Little-endian scalar accessors (valid at any alignment)             */
/* ------------------------------------------------------------------ */
static inline void nc_wr_u16(uint8_t *p, uint16_t v) { memcpy(p, &v, 2); }
static inline void nc_wr_u32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }
static inline void nc_wr_u64(uint8_t *p, uint64_t v) { memcpy(p, &v, 8); }
static inline void nc_wr_i16(uint8_t *p, int16_t v)  { memcpy(p, &v, 2); }
static inline void nc_wr_i32(uint8_t *p, int32_t v)  { memcpy(p, &v, 4); }

static inline uint16_t nc_rd_u16(const uint8_t *p) { uint16_t v; memcpy(&v, p, 2); return v; }
static inline uint32_t nc_rd_u32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static inline uint64_t nc_rd_u64(const uint8_t *p) { uint64_t v; memcpy(&v, p, 8); return v; }
static inline int16_t  nc_rd_i16(const uint8_t *p) { int16_t v;  memcpy(&v, p, 2); return v; }
static inline int32_t  nc_rd_i32(const uint8_t *p) { int32_t v;  memcpy(&v, p, 4); return v; }

/* ------------------------------------------------------------------ */
/* PPG batch builder                                                   */
/* Lifecycle: reset -> add xN (false = full, sample NOT stored: flush  */
/* via finish, reset, re-add the rejected sample) -> finish -> notify. */
/* After finish the batch must be reset before further adds.           */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t  buf[NC_ATT_PAYLOAD_MAX];
    uint64_t t0_us;      /* captured from the FIRST add after reset    */
    uint32_t seq;
    uint8_t  rate_code;
    uint8_t  n;
    uint8_t  max_n;      /* NC_PPG_MAX_N_AMB or _NOAMB                 */
    uint8_t  stride;     /* 12 with ambient, 8 without                 */
    bool     amb;
} nc_ppg_batch_t;

void   nc_ppg_batch_reset(nc_ppg_batch_t *b, uint32_t seq, uint8_t rate_code, bool amb);
bool   nc_ppg_batch_add(nc_ppg_batch_t *b, uint64_t t_us, int32_t ir, int32_t red, int32_t amb);
/* Writes the 15-byte header; the NC_PPGF_AMB bit of `flags` is forced
 * to match the batch's ambient mode (a mismatch would corrupt parsing). */
size_t nc_ppg_batch_finish(nc_ppg_batch_t *b, uint8_t flags, uint8_t **out);

static inline uint8_t nc_ppg_batch_count(const nc_ppg_batch_t *b) { return b->n; }

/* Clamp the batch fill to an ATT payload budget (negotiated MTU - 3,
 * from ble_att_payload_budget()). Without this, a peer that negotiated
 * less than MTU 247 gets every oversized notification silently
 * truncated by the stack. Call after reset; keeps at least 1 sample. */
void nc_ppg_batch_set_cap(nc_ppg_batch_t *b, uint16_t payload_budget);

/* ------------------------------------------------------------------ */
/* Accel batch builder (same lifecycle; fixed 6-byte stride)           */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t  buf[NC_ATT_PAYLOAD_MAX];
    uint64_t t0_us;
    uint32_t seq;
    uint8_t  odr_code;
    uint8_t  n;
    uint8_t  max_n;      /* NC_ACCEL_MAX_N, or lower via set_cap        */
} nc_accel_batch_t;

void   nc_accel_batch_reset(nc_accel_batch_t *b, uint32_t seq, uint8_t odr_code);
bool   nc_accel_batch_add(nc_accel_batch_t *b, uint64_t t_us, int16_t x, int16_t y, int16_t z);
/* `flags` must carry the FS code in NC_ACCF_FS_MASK (caller owns it). */
size_t nc_accel_batch_finish(nc_accel_batch_t *b, uint8_t flags, uint8_t **out);

static inline uint8_t nc_accel_batch_count(const nc_accel_batch_t *b) { return b->n; }

/* Accel twin of nc_ppg_batch_set_cap. */
void nc_accel_batch_set_cap(nc_accel_batch_t *b, uint16_t payload_budget);

/* ------------------------------------------------------------------ */
/* One-shot encoders. All return the total byte count written, or 0    */
/* if the input cannot legally fit NC_ATT_PAYLOAD_MAX (caller bug).    */
/* buf must hold NC_ATT_PAYLOAD_MAX bytes unless noted.                */
/* ------------------------------------------------------------------ */
size_t nc_enc_ibi(uint8_t *buf, uint32_t seq, const nc_ibi_rec_t *recs, uint8_t n);
size_t nc_enc_event_batch(uint8_t *buf, uint32_t seq, const nc_event_t *evs, uint8_t n);
size_t nc_enc_status(uint8_t *buf, const nc_status_t *s);   /* always 32 */

/* Knob discovery chunk: [u16 total][u16 first_idx][u8 n][records...].
 * Packs whole records only, starting at table index start_index, until
 * buf_cap runs out; *next_index = first_idx + n (the client's next
 * start_index).  A chunk with n == 0 while first_idx < total means
 * buf_cap cannot hold even one record — the caller must treat that as
 * an error, not retry.  Returns bytes written (>= 5), or 0 only when
 * buf_cap < the 5-byte chunk header. */
size_t nc_enc_knob_discover(uint8_t *buf, size_t buf_cap, uint16_t start_index,
                            uint16_t *next_index);

/* ------------------------------------------------------------------ */
/* Decoders (tests + host tools; tolerant of trailing extensions only  */
/* where the contract allows: STATUS is append-only).                  */
/* ------------------------------------------------------------------ */
/* Header decoders also validate that len matches n * stride exactly.  */
bool nc_dec_ppg_hdr(const uint8_t *buf, size_t len, nc_ppg_hdr_t *h);
bool nc_dec_accel_hdr(const uint8_t *buf, size_t len, nc_accel_hdr_t *h);
/* Returns record count (>= 0) or -1 on malformed/oversized input.     */
int  nc_dec_ibi(const uint8_t *buf, size_t len, uint32_t *seq,
                nc_ibi_rec_t *recs, int max_recs);
bool nc_dec_status(const uint8_t *buf, size_t len, nc_status_t *s);
