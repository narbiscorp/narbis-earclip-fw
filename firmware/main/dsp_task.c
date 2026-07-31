/*
 * dsp_task.c — per-sample DSP/IBI/gate/wear pipeline + slow AGC tick
 * (handoff §5.7–§5.9). Consumes ppg_queue; everything downstream of
 * the raw tee lives here.
 *
 * Per sample: ambient subtract (knob) -> nc_dsp_run per channel ->
 * gate input assembly (accel energy atomic from imu_task, sat vs
 * sat_pct·2^21, DC-step via nc_gate helpers, AGC settle blanking,
 * wear) -> nc_gate_update -> nc_ibi_update on the selected channel ->
 * IBI packet + HRS notify per beat.
 *
 * Slow paths by SAMPLE COUNT (no extra timers; the sample clock is the
 * only clock this pipeline trusts): AGC every agc_period_ms, wear +
 * aggregates + gate-duty ring every 1 s.
 *
 * Concurrency:
 *  - blank_until is a u64 written by sys_task, read per sample: RV32
 *    has no 64-bit atomic load, so both sides take s_blank_mux
 *    (portMUX critical section, two-word copy — nanoseconds).
 *  - aggregates + tap ring are portMUX-guarded snapshots (sys/console
 *    readers); all sections are plain memory copies, no calls inside.
 *  - the AGC offset-DAC codes have no driver readback getter, so this
 *    file keeps a shadow (s_offdac) updated optimistically when
 *    actions are POSTED; sys_task is the sole applier and drops are
 *    self-healing (next evaluation re-derives from DC reality).
 *  - reset requests (start / rate switch) arrive via one atomic word;
 *    the FULL bit is sticky across a not-yet-consumed rate request so
 *    a start immediately followed by a set_rate can't lose the full
 *    re-init.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "acq.h"
#include "app_msgs.h"
#include "afe4404.h"
#include "ble_iface.h"
#include "diag.h"
#include "sensor_types.h"
#include "acq_internal.h"

#include "narbis/nc_dsp.h"
#include "narbis/nc_ibi.h"
#include "narbis/nc_gate.h"
#include "narbis/nc_wear.h"
#include "narbis/nc_agc.h"
#include "narbis/nc_proto_encode.h"

static const char *TAG = "acq_dsp";

/* Reset request encoding: bits[3:0] = rate code + 1 (0 == no request),
 * bit 8 = full reset. See acq_internal.h for full-vs-rate semantics. */
#define DSP_RESET_FULL 0x100u

#define DUTY_RING_S 60
#define TAP_RING_N  32

/* ------------------------------------------------------------------ */
/* Shared atomics (extern'd in acq_internal.h)                         */
/* ------------------------------------------------------------------ */
_Atomic uint8_t g_acq_batch_bits;
_Atomic bool g_acq_agc_frozen;

/* ------------------------------------------------------------------ */
/* Pipeline state (dsp_task-owned unless noted)                        */
/* ------------------------------------------------------------------ */
static nc_chan_dsp_t s_ch[2];          /* [0] IR, [1] RED               */
static nc_ibi_t s_ibi;
static nc_gate_t s_gate;
static nc_gate_dcstep_t s_dcstep;      /* fed with the IBI channel's DC */
static nc_wear_t s_wear;
static nc_agc_t s_agc;
static int8_t s_offdac[2];             /* shadow, see file header       */
static bool s_prev_gated;
static bool s_prev_worn;
static bool s_wear_reported;   /* one-shot: never-worn case reported     */
static uint32_t s_notworn_s;   /* consecutive not-worn 1 Hz ticks        */

static nc_rate_t s_rate = NC_RATE_100;
static uint16_t s_sps = 100;

static _Atomic uint32_t s_reset_req;

/* Measured sample period: EMA of inter-frame dt, alpha 1/16, kept at
 * x256 (Q8) so 0.1 % drift at 500 sps (2 µs) stays representable. */
static uint32_t s_ts_ema_q8 = 10000u << 8;
static uint32_t s_ibi_ts_cur = 10000;  /* period last handed to nc_ibi  */
static uint64_t s_t_prev;
static _Atomic uint32_t s_ts_pub = 10000;   /* acq_measured_ts_us()     */

/* AGC settle blanking (sys_task writes, sample path reads). */
static portMUX_TYPE s_blank_mux = portMUX_INITIALIZER_UNLOCKED;
static uint64_t s_blank_until_us;

/* AGC slow tick. */
static uint32_t s_agc_cnt;
static uint32_t s_agc_period_smp = 50; /* recomputed at reset + tick    */
static bool s_sat_win[2];

/* 1 Hz tick + gate duty over a rolling 60 s. */
static uint32_t s_sec_cnt, s_sec_gated;
static int64_t s_bp_pow_acc, s_bp_pow_last;
static uint16_t s_duty_gated[DUTY_RING_S], s_duty_total[DUTY_RING_S];
static uint32_t s_duty_sum_g, s_duty_sum_t;
static uint8_t s_duty_idx;

/* HR: 3-beat median -> EMA (alpha 1/4), x16 fixed point. */
static uint16_t s_ibi_ring[3];
static uint8_t s_ibi_rn, s_ibi_ri;
static uint16_t s_hr_ema_x16;
static uint32_t s_ibi_seq;
static uint8_t s_ibi_pkt[NC_ATT_PAYLOAD_MAX];

/* Aggregates snapshot (acq_get_aggregates). */
static portMUX_TYPE s_agg_mux = portMUX_INITIALIZER_UNLOCKED;
static acq_aggregates_t s_agg;

/* Console raw tap. */
static portMUX_TYPE s_tap_mux = portMUX_INITIALIZER_UNLOCKED;
static nc_ppg_sample_t s_tap_ring[TAP_RING_N];
static uint8_t s_tap_head, s_tap_cnt;  /* head = next write slot        */
static uint16_t s_tap_arm;

/* ------------------------------------------------------------------ */
/* Public setters/getters (acq.h; callable from sys_task/console)      */
/* ------------------------------------------------------------------ */
void acq_set_blank_until(uint64_t t_us)
{
    portENTER_CRITICAL(&s_blank_mux);
    s_blank_until_us = t_us;
    portEXIT_CRITICAL(&s_blank_mux);
}

void acq_set_agc_frozen(bool frozen)
{
    atomic_store(&g_acq_agc_frozen, frozen);
}

uint32_t acq_measured_ts_us(void)
{
    return atomic_load(&s_ts_pub);
}

void acq_get_aggregates(acq_aggregates_t *out)
{
    if (out == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_agg_mux);
    *out = s_agg;
    portEXIT_CRITICAL(&s_agg_mux);
}

void acq_tap_arm(uint16_t n)
{
    portENTER_CRITICAL(&s_tap_mux);
    s_tap_arm = n;
    s_tap_head = 0;
    s_tap_cnt = 0;
    portEXIT_CRITICAL(&s_tap_mux);
}

int acq_tap_read(nc_ppg_sample_t *out, int max)
{
    if (out == NULL || max <= 0) {
        return 0;
    }
    portENTER_CRITICAL(&s_tap_mux);
    int n = s_tap_cnt;
    if (n > max) {
        n = max;
    }
    /* head points at the next write slot; oldest = head - cnt (mod N) */
    uint8_t idx = (uint8_t)((s_tap_head + TAP_RING_N - s_tap_cnt) % TAP_RING_N);
    for (int i = 0; i < n; i++) {
        out[i] = s_tap_ring[idx];
        idx = (uint8_t)((idx + 1) % TAP_RING_N);
    }
    s_tap_cnt = (uint8_t)(s_tap_cnt - n);
    portEXIT_CRITICAL(&s_tap_mux);
    return n;
}

void acq_dsp_request_reset(nc_rate_t rate, bool full)
{
    uint32_t enc = ((uint32_t)rate + 1u) | (full ? DSP_RESET_FULL : 0u);
    uint32_t old = atomic_load(&s_reset_req);
    uint32_t nv;
    do {   /* FULL bit is sticky over an unconsumed pending request */
        nv = enc | (old & DSP_RESET_FULL);
    } while (!atomic_compare_exchange_weak(&s_reset_req, &old, nv));
}

/* ------------------------------------------------------------------ */
/* Internals                                                           */
/* ------------------------------------------------------------------ */
static uint32_t agc_period_samples(void)
{
    uint32_t n = (uint32_t)s_sps *
                 (uint32_t)nc_knob_get(KNOB_AGC_PERIOD_MS) / 1000u;
    return (n > 0) ? n : 1;
}

static void dsp_apply_reset(uint32_t req)
{
    nc_rate_t rate = (nc_rate_t)((req & 0xFu) - 1u);
    bool full = (req & DSP_RESET_FULL) != 0;

    s_rate = rate;
    s_sps = nc_rate_sps(rate);
    if (s_sps == 0) {           /* defensive; encoder validated rate */
        s_sps = 100;
        s_rate = NC_RATE_100;
    }

    nc_dsp_init(&s_ch[0], s_rate);
    nc_dsp_init(&s_ch[1], s_rate);

    uint32_t nom = 1000000u / s_sps;
    s_ts_ema_q8 = nom << 8;
    s_ibi_ts_cur = nom;
    atomic_store(&s_ts_pub, nom);
    s_t_prev = 0;

    nc_ibi_init(&s_ibi, nom);          /* re-reads the IBI knob block  */
    nc_gate_dcstep_init(&s_dcstep);

    s_agc_cnt = 0;
    s_agc_period_smp = agc_period_samples();
    s_sat_win[0] = s_sat_win[1] = false;
    s_sec_cnt = 0;
    s_sec_gated = 0;
    s_bp_pow_acc = 0;

    if (full) {
        nc_gate_init(&s_gate);
        s_prev_gated = false;
        atomic_store(&g_acq_batch_bits, 0);

        nc_agc_init(&s_agc);
        s_offdac[0] = s_offdac[1] = 0;  /* afe4404_init zeroed the DACs */

        nc_wear_init(&s_wear);
        s_prev_worn = nc_wear_is_worn(&s_wear);
        s_wear_reported = false;
        s_notworn_s = 0;

        memset(s_duty_gated, 0, sizeof(s_duty_gated));
        memset(s_duty_total, 0, sizeof(s_duty_total));
        s_duty_sum_g = s_duty_sum_t = 0;
        s_duty_idx = 0;

        s_ibi_rn = s_ibi_ri = 0;
        s_hr_ema_x16 = 0;
        s_ibi_seq = 0;
        s_bp_pow_last = 0;

        portENTER_CRITICAL(&s_agg_mux);
        memset(&s_agg, 0, sizeof(s_agg));
        portEXIT_CRITICAL(&s_agg_mux);
    }
    ESP_LOGI(TAG, "reset (%s) @%u sps", full ? "full" : "rate", s_sps);
}

static uint16_t med3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { b = c; }
    return (a > b) ? a : b;
}

static void on_beat(const nc_ibi_rec_t *rec)
{
    size_t len = nc_enc_ibi(s_ibi_pkt, s_ibi_seq, rec, 1);
    if (len > 0) {
        (void)ble_tx_submit(BLE_CH_IBI, s_ibi_pkt, (uint16_t)len);
        s_ibi_seq++;
    }

    if (rec->ibi_ms == 0) {
        /* FIRST_AFTER_GAP: valid timestamp, no interval — restart the
         * median context, keep the displayed HR (EMA) for continuity. */
        s_ibi_rn = s_ibi_ri = 0;
        return;
    }

    s_ibi_ring[s_ibi_ri] = rec->ibi_ms;
    s_ibi_ri = (uint8_t)((s_ibi_ri + 1) % 3);
    if (s_ibi_rn < 3) {
        s_ibi_rn++;
    }
    uint16_t med = (s_ibi_rn >= 3)
        ? med3(s_ibi_ring[0], s_ibi_ring[1], s_ibi_ring[2])
        : rec->ibi_ms;

    uint32_t bpm = 60000u / med;       /* med >= ibi_min_ms >= 200      */
    if (bpm > 250) {
        bpm = 250;
    }
    int32_t x16 = (int32_t)(bpm << 4);
    if (s_hr_ema_x16 == 0) {
        s_hr_ema_x16 = (uint16_t)x16;
    } else {
        int32_t ema = (int32_t)s_hr_ema_x16;
        ema += (x16 - ema) / 4;        /* alpha 1/4 */
        s_hr_ema_x16 = (uint16_t)ema;
    }
    uint8_t hr = (uint8_t)((s_hr_ema_x16 + 8) >> 4);

    portENTER_CRITICAL(&s_agg_mux);
    s_agg.ibi_last_ms = rec->ibi_ms;
    s_agg.hr_bpm = hr;
    portEXIT_CRITICAL(&s_agg_mux);

    /* HRS: RR in 1/1024 s units, one interval per beat. The BLE layer
     * discards it when nobody subscribed. */
    uint16_t rr = (uint16_t)((uint32_t)rec->ibi_ms * 1024u / 1000u);
    ble_notify_hrs(hr, &rr, 1);
}

static void agc_tick(uint64_t t_us)
{
    s_agc_cnt = 0;
    s_agc_period_smp = agc_period_samples();   /* LIVE knob pickup */

    nc_agc_in_t in = {
        .dc_raw  = { s_ch[0].dc, s_ch[1].dc },
        .sat     = { s_sat_win[0], s_sat_win[1] },
        .led_ma  = { afe4404_cur_ir_ma(), afe4404_cur_red_ma() },
        .rf_code = afe4404_cur_rf(),
        .offdac  = { s_offdac[0], s_offdac[1] },
        .now_ms  = t_us / 1000u,
        .frozen  = atomic_load(&g_acq_agc_frozen),
    };
    s_sat_win[0] = s_sat_win[1] = false;

    nc_agc_act_t acts[4];
    int n = nc_agc_evaluate(&s_agc, &in, acts);
    if (n <= 0) {
        return;
    }
    if (n > 4) {
        n = 4;
    }

    sys_msg_t m = { .type = SYS_AGC_ACTIONS };
    for (int i = 0; i < n; i++) {
        m.u.agc.acts[i] = acts[i];
        if (acts[i].kind == NC_AGC_OFFDAC && acts[i].chan < 2) {
            s_offdac[acts[i].chan] = acts[i].offdac;   /* shadow */
        }
    }
    m.u.agc.count = (uint8_t)n;
    /* Drop-on-full is self-healing: the next period re-evaluates from
     * measured DC. (Shadow may run ahead of hardware for one period.) */
    (void)sys_post(&m);
}

static void one_hz_tick(void)
{
    s_bp_pow_last = s_bp_pow_acc;
    s_bp_pow_acc = 0;

    /* Wear runs on the IR channel regardless of the IBI channel knob. */
    nc_wear_tick_1hz(&s_wear, s_ch[0].dc, s_bp_pow_last);
    bool worn = nc_wear_is_worn(&s_wear);
    if (worn != s_prev_worn) {
        s_prev_worn = worn;
        s_wear_reported = true;
        sys_msg_t m = { .type = SYS_WEAR_CHANGED, .u.flag = worn };
        (void)sys_post(&m);   /* sys_task owns the EVENT + policy side */
    } else if (!s_wear_reported && !worn &&
               ++s_notworn_s >= (uint32_t)nc_knob_get(KNOB_WEAR_OFF_S)) {
        /* Never-worn handshake: sys_task boots with an optimistic
         * s_worn=true; without a transition it would stream LEDs
         * forever and never arm off-ear auto-sleep. Report the
         * sustained not-worn state once. */
        s_wear_reported = true;
        sys_msg_t m = { .type = SYS_WEAR_CHANGED, .u.flag = false };
        (void)sys_post(&m);
    }

    /* Gate duty, rolling 60 s. */
    s_duty_sum_g -= s_duty_gated[s_duty_idx];
    s_duty_sum_t -= s_duty_total[s_duty_idx];
    s_duty_gated[s_duty_idx] = (uint16_t)s_sec_gated;
    s_duty_total[s_duty_idx] = (uint16_t)s_sec_cnt;
    s_duty_sum_g += s_sec_gated;
    s_duty_sum_t += s_sec_cnt;
    s_duty_idx = (uint8_t)((s_duty_idx + 1) % DUTY_RING_S);
    uint16_t duty = (s_duty_sum_t > 0)
        ? (uint16_t)((uint64_t)s_duty_sum_g * 10000u / s_duty_sum_t)
        : 0;
    s_sec_cnt = 0;
    s_sec_gated = 0;

    portENTER_CRITICAL(&s_agg_mux);
    s_agg.dc_ir = s_ch[0].dc;
    s_agg.dc_red = s_ch[1].dc;
    s_agg.bp_power_1s = s_bp_pow_last;
    s_agg.gate_duty_x100 = duty;
    s_agg.gated = s_prev_gated;
    s_agg.worn = worn;
    /* ibi_last_ms / hr_bpm maintained at beat time */
    portEXIT_CRITICAL(&s_agg_mux);
}

static void dsp_process(const nc_ppg_sample_t *s)
{
    bool amb_sub = nc_knob_get(KNOB_AMB_SUBTRACT) != 0;
    int32_t xi = amb_sub ? s->ir - s->amb : s->ir;
    int32_t xr = amb_sub ? s->red - s->amb : s->red;

    nc_dsp_out_t o[2];
    nc_dsp_run(&s_ch[0], xi, &o[0]);
    nc_dsp_run(&s_ch[1], xr, &o[1]);

    int sel = (nc_knob_get(KNOB_IBI_CHANNEL) != 0) ? 1 : 0;

    /* Saturation on RAW values (pre-subtract): the rail lives there. */
    int32_t thr = acq_sat_thr_counts();
    bool sat_ir = (s->ir >= thr) || (s->ir <= -thr);
    bool sat_red = (s->red >= thr) || (s->red <= -thr);
    s_sat_win[0] |= sat_ir;
    s_sat_win[1] |= sat_red;

    portENTER_CRITICAL(&s_blank_mux);
    uint64_t blank = s_blank_until_us;   /* u64: no atomic load on RV32 */
    portEXIT_CRITICAL(&s_blank_mux);
    bool settling = s->t_us < blank;

    bool wear_off = !nc_wear_is_worn(&s_wear);

    /* amp_collapse is an IBI-side input not wired in v1 (task scope);
     * gate reason COLLAPSE stays available on the wire. */
    nc_gate_in_t gin;
    nc_gate_build_input(&gin, &s_dcstep, atomic_load(&g_acq_acc_energy),
                        o[sel].dc, sat_ir || sat_red, false, settling,
                        wear_off);
    uint8_t reason = 0;
    bool gated = nc_gate_update(&s_gate, &gin, (uint32_t)(s->t_us / 1000u),
                                &reason);
    if (gated != s_prev_gated) {
        s_prev_gated = gated;
        /* sys_task is the single NC_EV_GATE emitter and the sole writer
         * of g_diag.gate_transitions (app_msgs.h SYS_GATE_CHANGED
         * contract + diag.h single-writer rule) — no local emission. */
        sys_msg_t m = { .type = SYS_GATE_CHANGED };
        m.u.gate.on = gated;
        m.u.gate.reason = reason;
        (void)sys_post(&m);
    }
    atomic_store(&g_acq_batch_bits,
                 (uint8_t)((gated ? NC_PPGF_GATE : 0u) |
                           (settling ? NC_PPGF_AGC_SETTLING : 0u)));

    /* Console tap: raw frames, as queued. */
    if (s_tap_arm > 0) {
        portENTER_CRITICAL(&s_tap_mux);
        if (s_tap_arm > 0) {
            s_tap_arm--;
            s_tap_ring[s_tap_head] = *s;
            s_tap_head = (uint8_t)((s_tap_head + 1) % TAP_RING_N);
            if (s_tap_cnt < TAP_RING_N) {
                s_tap_cnt++;      /* full ring: oldest overwritten */
            }
        }
        portEXIT_CRITICAL(&s_tap_mux);
    }

    nc_ibi_rec_t rec;
    if (nc_ibi_update(&s_ibi, o[sel].bp, s->t_us, gated, &rec)) {
        on_beat(&rec);
    }

    /* Wear plausibility power: IR band-pass, 1 s sum. */
    s_bp_pow_acc += (int64_t)o[0].bp * o[0].bp;

    /* Measured sample period EMA (alpha 1/16). Outliers beyond
     * [ema/2, 2*ema] are gaps (stop/start, dropped frames), not
     * timing truth — excluded. */
    if (s_t_prev != 0) {
        uint32_t dt = (uint32_t)(s->t_us - s_t_prev);
        uint32_t ema = s_ts_ema_q8 >> 8;
        if (dt > ema / 2 && dt < ema * 2) {
            int32_t d = (int32_t)(dt << 8) - (int32_t)s_ts_ema_q8;
            s_ts_ema_q8 = (uint32_t)((int32_t)s_ts_ema_q8 + (d >> 4));
            uint32_t cur = s_ts_ema_q8 >> 8;
            atomic_store(&s_ts_pub, cur);
            uint32_t drift = (cur > s_ibi_ts_cur) ? cur - s_ibi_ts_cur
                                                  : s_ibi_ts_cur - cur;
            if (drift * 1000u > s_ibi_ts_cur) {   /* > 0.1 % */
                nc_ibi_set_ts(&s_ibi, cur);
                s_ibi_ts_cur = cur;
            }
        }
    }
    s_t_prev = s->t_us;

    /* Slow ticks, sample-clocked. */
    s_sec_cnt++;
    if (gated) {
        s_sec_gated++;
    }
    if (++s_agc_cnt >= s_agc_period_smp) {
        agc_tick(s->t_us);
    }
    if (s_sec_cnt >= s_sps) {
        one_hz_tick();
    }
}

/* ------------------------------------------------------------------ */
/* Task                                                                */
/* ------------------------------------------------------------------ */
void acq_dsp_task_run(void *arg)
{
    (void)arg;
    /* Permanent TWDT subscription — resetting while UNsubscribed makes
     * IDF v5.5 log an E-level "task not found" every loop (10 Hz idle
     * console flood, found on V2.1 first boot). */
    (void)esp_task_wdt_add(NULL);
    for (;;) {
        /* 100 ms receive timeout keeps the WDT fed across idle spans. */
        (void)esp_task_wdt_reset();

        uint32_t req = atomic_exchange(&s_reset_req, 0);
        if (req != 0) {
            dsp_apply_reset(req);
        }

        nc_ppg_sample_t s;
        if (xQueueReceive(ppg_queue, &s, pdMS_TO_TICKS(100)) == pdTRUE) {
            /* Re-check after the wake: start/set_rate post the reset
             * BEFORE re-enabling the ISR, so if this sample is from the
             * new configuration the request is already visible here —
             * no sample is ever run through stale-rate filters. */
            req = atomic_exchange(&s_reset_req, 0);
            if (req != 0) {
                dsp_apply_reset(req);
            }
            dsp_process(&s);
        }
    }
}
