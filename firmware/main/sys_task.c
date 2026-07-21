/*
 * sys_task.c — the single serialization point (handoff §3, §5.1–5.3,
 * §8.1/8.4). Everything that mutates system state funnels through
 * sys_q: CONTROL requests, subscription snapshots, AGC decisions,
 * button edges, wear/gate changes, the 1 Hz housekeeping tick. All
 * AFE4404 config I2C happens here and only here.
 *
 * Demand model: this task owns an nc_acq_in_t snapshot (subscriptions,
 * latched CONTROL start/stop masks, wear pause, USB/battery gates) and
 * re-runs nc_acq_eval() after every input change, applying only the
 * diffs via acq_ppg_start/stop + acq_accel_start/stop.
 *
 * Stream override latching (documented per task contract): a
 * STREAM_STOP bit suppresses that source until cleared; STREAM_START
 * of a source clears its stop bit, and a fresh CCCD subscribe of the
 * source clears it too (nc_acq_policy.h contract — the policy itself
 * is stateless). Symmetrically a STREAM_STOP clears the same source's
 * latched start bit, so stop always wins over an older manual start.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "host/ble_store.h"   /* ble_store_clear() — NimBLE bond store */

#include "acq.h"
#include "afe4404.h"
#include "app_msgs.h"
#include "battery.h"
#include "ble_iface.h"
#include "board.h"
#include "charger.h"
#include "diag.h"
#include "knobs_nvs.h"
#include "ota.h"
#include "power.h"
#include "selftest.h"
#include "sensor_types.h"
#include "test_ops.h"

#include "narbis/nc_acq_policy.h"
#include "narbis/nc_button.h"
#include "narbis/nc_control.h"
#include "narbis/nc_knobs.h"
#include "narbis/nc_proto_encode.h"
#include "narbis/nc_timesync.h"
#include "narbis/nc_types.h"
#include "narbis/proto.h"

static const char *TAG = "sys";

/* Provided by other modules against their declared interfaces (the
 * owning agents add the prototypes to their own headers; the symbols
 * are link-time contracts):
 *  - ble_request_conn_speed: BLE conn-interval policy, fast while data
 *    is flowing (7.5–15 ms) vs idle (150–300 ms);
 *  - acq_set_agc_frozen: freeze flag consumed by dsp_task's
 *    nc_agc_evaluate() input (we also skip actuation here — both ends
 *    are gated so a freeze that races an in-flight decision is safe). */
extern void ble_request_conn_speed(bool fast);
extern void acq_set_agc_frozen(bool frozen);

/* Mailbox (contract: app_msgs.h): sys_q + the sys_post helpers are
 * DEFINED in app_tasks.c, which creates every queue before it spawns
 * any task (including this one) — producers never race the handle. */

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static nc_sys_state_t s_state = NC_STATE_IDLE;
static nc_acq_in_t s_in;              /* latched demand inputs          */
static bool s_connected;
static bool s_worn = true;            /* optimistic until dsp says else */
static bool s_vusb;
static nc_charger_state_t s_chg = NC_CHG_ON_BATTERY;
static uint16_t s_batt_mv = 4200;
static uint8_t s_batt_pct = 100;
static uint8_t s_batt_pct_sent = 0xFF;

static bool s_agc_frozen;
static int8_t s_offdac[2];            /* cached codes; sys is sole writer */
static nc_rate_t s_rate_live = NC_RATE_100;

static bool s_boot_lowbatt;           /* LOWBATT boot-refusal mode      */
static bool s_lowbatt_warn, s_lowbatt_stop;
static int64_t s_warn_since = -1, s_stop_since = -1, s_off_since = -1;
#define LOWBATT_HYST_US   (30LL * 1000000)
#define LOWBATT_CLEAR_MV  100         /* de-assert margin above threshold */

static int64_t s_last_activity_us;
static int64_t s_offear_since = -1;   /* off-ear auto-sleep countdown   */
static int64_t s_ota_enter_us;
#define OTA_NO_BEGIN_TIMEOUT_US  (60LL * 1000000)
#define BOOT_LOWBATT_ADV_US      (10LL * 1000000)

static nc_ts_t s_ts;
static nc_btn_fsm_t s_btn;
static esp_timer_handle_t s_tick_tmr, s_btn_tmr;

static uint32_t s_event_seq;
static bool s_fast_conn;              /* last value sent to the BLE layer */
static bool s_save_pending;           /* an explicit SAVE failed — retry at OFF */

static uint8_t s_status_state_last = 0xFF, s_status_flags_last;

/* Destructive CONTROL ops latch here; the response is sent first, then
 * the action runs (nc_control.h contract). */
static enum {
    LATCH_NONE, LATCH_ENTER_OTA, LATCH_POWER_OFF, LATCH_REBOOT, LATCH_FACTORY
} s_latch;

static void orderly_off(bool battery_forced) __attribute__((noreturn));

/* ------------------------------------------------------------------ */
/* Events                                                              */
/* ------------------------------------------------------------------ */
static void ev_emit(uint8_t type, const uint8_t *data, uint8_t len)
{
    nc_event_t ev = {
        .t_us = (uint64_t)esp_timer_get_time(),
        .type = type,
        .len = len,
    };
    memcpy(ev.data, data, len);
    if (event_q == NULL || xQueueSend(event_q, &ev, 0) != pdTRUE) {
        ESP_LOGD(TAG, "event_q full, type 0x%02x dropped", type);
    }
}

/* Drain up to 4 queued events per wakeup into one EVENT notification.
 * seq increments once per notification: a gap tells the host that
 * ble_tx dropped a staged batch. */
static void pump_events(void)
{
    nc_event_t evs[4];
    uint8_t n = 0;
    while (n < 4 && event_q != NULL &&
           xQueueReceive(event_q, &evs[n], 0) == pdTRUE) {
        n++;
    }
    if (n == 0) {
        return;
    }
    uint8_t buf[NC_ATT_PAYLOAD_MAX];
    size_t len = nc_enc_event_batch(buf, s_event_seq, evs, n);
    if (len > 0) {
        s_event_seq++;
        ble_tx_submit(BLE_CH_EVENT, buf, (uint16_t)len);
    }
}

/* ------------------------------------------------------------------ */
/* STATUS                                                              */
/* ------------------------------------------------------------------ */
static void status_sync(bool force)
{
    acq_aggregates_t ag;
    acq_get_aggregates(&ag);

    nc_status_t st = { 0 };
    st.sys_state = (uint8_t)s_state;
    uint8_t f = 0;
    if (s_chg == NC_CHG_CHARGING) f |= NC_STF_CHARGING;
    if (s_chg == NC_CHG_COMPLETE) f |= NC_STF_CHARGE_DONE;
    if (s_vusb)                   f |= NC_STF_USB;
    if (s_worn)                   f |= NC_STF_WORN;
    if (ag.gated)                 f |= NC_STF_GATE;
    if (s_agc_frozen)             f |= NC_STF_AGC_FROZEN;
    if (s_in.sub_hrs)             f |= NC_STF_HRS_ACTIVE;
    if (s_lowbatt_warn)           f |= NC_STF_LOWBATT_WARN;
    st.flags = f;

    if (!force && st.sys_state == s_status_state_last &&
        f == s_status_flags_last) {
        return;   /* "on change" pushes key off state+flags edges */
    }

    st.batt_mv = s_batt_mv;
    st.batt_pct = s_batt_pct;
    st.ppg_rate_code = (uint8_t)nc_knob_get(KNOB_PPG_RATE);
    st.led_ir_ma = afe4404_cur_ir_ma();
    st.led_red_ma = afe4404_cur_red_ma();
    st.tia_gain_code = afe4404_cur_rf();
    st.tia_cf_code = afe4404_cur_cf();
    st.gate_duty_x100 = ag.gate_duty_x100;
    st.notif_drop_count = g_diag.notify_drop;
    st.i2c_err_count =
        (g_diag.i2c_err > 0xFFFF) ? 0xFFFF : (uint16_t)g_diag.i2c_err;
    st.clock_drift_ppm_x10 = nc_ts_drift_ppm_x10(&s_ts);
    st.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    st.ibi_last_ms = ag.ibi_last_ms;
    st.hr_bpm = ag.hr_bpm;

    uint8_t buf[NC_ATT_PAYLOAD_MAX];
    size_t len = nc_enc_status(buf, &st);
    if (len > 0) {
        ble_tx_submit(BLE_CH_STATUS, buf, (uint16_t)len);
    }
    s_status_state_last = st.sys_state;
    s_status_flags_last = f;
}

static void set_state(nc_sys_state_t ns)
{
    if (ns == s_state) {
        return;
    }
    ESP_LOGI(TAG, "state %u -> %u", (unsigned)s_state, (unsigned)ns);
    s_state = ns;
    power_apply_state(ns, s_vusb);   /* pm locks track every transition */
    status_sync(true);               /* STATUS immediate on transition  */
}

/* ------------------------------------------------------------------ */
/* Demand evaluation                                                   */
/* ------------------------------------------------------------------ */
static void demand(void)
{
    s_in.gate_en = nc_knob_get(KNOB_GATE_EN) != 0;
#if NARBIS_TEST_MODE
    s_in.wear_paused = false;        /* test builds never wear-gate */
#else
    s_in.wear_paused = !s_worn;
#endif
    /* usb_stream_ok doubles as the unconditional all-off input for the
     * low-battery stop level, the LOWBATT boot refusal and the OTA /
     * SELFTEST states (nc_acq_eval forces everything off when false). */
    const bool usb_ok = !(s_vusb && nc_knob_get(KNOB_STREAM_ON_USB) == 0);
    s_in.usb_stream_ok = usb_ok && !s_lowbatt_stop && !s_boot_lowbatt &&
                         s_state != NC_STATE_OTA &&
                         s_state != NC_STATE_SELFTEST;

    nc_acq_out_t want;
    nc_acq_eval(&s_in, &want);

    if (want.ppg_on && !acq_ppg_running()) {
        const nc_rate_t rate = (nc_rate_t)nc_knob_get(KNOB_PPG_RATE);
        esp_err_t err = acq_ppg_start(rate);
        if (err == ESP_OK) {
            s_rate_live = rate;
            s_offdac[0] = s_offdac[1] = 0;   /* afe init zeroes the DACs */
        } else {
            ESP_LOGE(TAG, "acq_ppg_start: %s", esp_err_to_name(err));
        }
    } else if (!want.ppg_on && acq_ppg_running()) {
        acq_ppg_stop();
    }

    if (want.accel_on && !acq_accel_running()) {
        esp_err_t err = acq_accel_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "acq_accel_start: %s", esp_err_to_name(err));
        }
    } else if (!want.accel_on && acq_accel_running()) {
        acq_accel_stop();
    }

    /* Connection-interval policy follows actual data flow. */
    const bool fast = acq_ppg_running() || acq_accel_running();
    if (fast != s_fast_conn) {
        s_fast_conn = fast;
        ble_request_conn_speed(fast);
    }

    /* Presentation state (OTA/SELFTEST/boot-LOWBATT hold themselves). */
    if (s_state != NC_STATE_OTA && s_state != NC_STATE_SELFTEST &&
        !s_boot_lowbatt) {
        set_state(acq_ppg_running() ? NC_STATE_STREAMING
                  : s_connected     ? NC_STATE_CONNECTED
                                    : NC_STATE_IDLE);
    }
    status_sync(false);
}

/* ------------------------------------------------------------------ */
/* CONTROL callbacks (nc_ctrl_ctx_t; dispatcher pre-validates ranges)  */
/* ------------------------------------------------------------------ */
static nc_ctrl_status_t cb_stream_start(void *u, uint8_t mask)
{
    (void)u;
    if (s_state == NC_STATE_OTA || s_state == NC_STATE_SELFTEST) {
        return NC_ST_WRONG_STATE;
    }
    if (s_lowbatt_stop || s_boot_lowbatt) {
        return NC_ST_LOWBATT;
    }
    s_in.ctrl_start_mask |= mask;
    s_in.ctrl_stop_mask &= (uint8_t)~mask;   /* start clears opposing stop */
    return NC_ST_OK;
}

static nc_ctrl_status_t cb_stream_stop(void *u, uint8_t mask)
{
    (void)u;
    s_in.ctrl_stop_mask |= mask;
    s_in.ctrl_start_mask &= (uint8_t)~mask;  /* stop clears opposing start */
    return NC_ST_OK;
}

static nc_ctrl_status_t cb_set_rate(void *u, uint8_t rate_code)
{
    (void)u;
    /* Keep the knob authoritative (persist candidate + STATUS source). */
    nc_ctrl_status_t st =
        nc_knob_set_id(nc_knob_desc(KNOB_PPG_RATE)->id, (int32_t)rate_code);
    if (st != NC_ST_OK) {
        return st;
    }
    if (acq_ppg_running() && (nc_rate_t)rate_code != s_rate_live) {
        const uint8_t old = (uint8_t)s_rate_live;
        esp_err_t err = acq_ppg_set_rate((nc_rate_t)rate_code);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "rate switch: %s", esp_err_to_name(err));
            return NC_ST_BUSY;
        }
        s_rate_live = (nc_rate_t)rate_code;
        const uint8_t d[2] = { old, rate_code };
        ev_emit(NC_EV_RATE_CHANGE, d, 2);
    }
    return NC_ST_OK;
}

static nc_ctrl_status_t cb_knob_save(void *u)
{
    (void)u;
    esp_err_t err = knobs_nvs_save();
    if (err != ESP_OK) {
        s_save_pending = true;   /* retried once during orderly OFF */
        return NC_ST_NVS_ERR;
    }
    s_save_pending = false;
    return NC_ST_OK;
}

static nc_ctrl_status_t cb_knob_reset(void *u, uint8_t scope)
{
    (void)u;
    /* Dispatcher has already reset RAM (nc_knobs_init). Re-apply the
     * live side effects we own: a running stream keeps its programmed
     * rate only if it still matches the (now default) knob. Demand-side
     * knobs (gate_en, stream_on_usb) re-latch on the post-dispatch
     * demand() pass. */
    if (acq_ppg_running() &&
        (nc_rate_t)nc_knob_get(KNOB_PPG_RATE) != s_rate_live) {
        if (acq_ppg_set_rate((nc_rate_t)nc_knob_get(KNOB_PPG_RATE)) == ESP_OK) {
            const uint8_t d[2] = { (uint8_t)s_rate_live,
                                   (uint8_t)nc_knob_get(KNOB_PPG_RATE) };
            s_rate_live = (nc_rate_t)nc_knob_get(KNOB_PPG_RATE);
            ev_emit(NC_EV_RATE_CHANGE, d, 2);
        }
    }
    if (scope == 1 && knobs_nvs_reset() != ESP_OK) {
        return NC_ST_NVS_ERR;
    }
    return NC_ST_OK;
}

static uint64_t cb_get_time_us(void *u)
{
    (void)u;
    return (uint64_t)esp_timer_get_time();
}

static void cb_time_sync_seen(void *u, uint64_t host_us, uint64_t dev_us)
{
    (void)u;
    nc_ts_add_pair(&s_ts, host_us, dev_us);   /* drift -> STATUS field */
}

static nc_ctrl_status_t cb_marker(void *u, uint16_t marker_id)
{
    (void)u;
    const uint8_t d[3] = { NC_MARKER_SRC_HOST,
                           (uint8_t)(marker_id & 0xFF),
                           (uint8_t)(marker_id >> 8) };
    ev_emit(NC_EV_MARKER, d, 3);
    return NC_ST_OK;
}

static nc_ctrl_status_t cb_agc_freeze(void *u, bool freeze)
{
    (void)u;
    s_agc_frozen = freeze;
    acq_set_agc_frozen(freeze);
    return NC_ST_OK;
}

/* Every actuation blanks the sample stream for agc_settle_ms. */
static void settle_blank(void)
{
    const uint64_t settle_us =
        (uint64_t)nc_knob_get(KNOB_AGC_SETTLE_MS) * 1000ULL;
    acq_set_blank_until((uint64_t)esp_timer_get_time() + settle_us);
}

static nc_ctrl_status_t cb_agc_manual(void *u, uint8_t ir_ma, uint8_t red_ma,
                                      uint8_t rf_code, uint8_t apply_mask)
{
    (void)u;
    if (apply_mask == 0 || (apply_mask & (uint8_t)~0x07)) {
        return NC_ST_BAD_PARAM;
    }
    /* Hardware PWDN wipes AFE registers — manual settings only make
     * sense (and only stick) while the PPG engine is running. */
    if (!acq_ppg_running()) {
        return NC_ST_WRONG_STATE;
    }
    const uint8_t old_ir = afe4404_cur_ir_ma();
    const uint8_t old_red = afe4404_cur_red_ma();
    const uint8_t old_rf = afe4404_cur_rf();

    /* SFH 7016 DC abs-max clamps (board.h) — enforced here in addition
     * to the driver's own clamp. */
    uint8_t new_ir = old_ir, new_red = old_red, new_rf = old_rf;
    if (apply_mask & 0x01) {
        new_ir = (ir_ma > LED_IR_MAX_MA) ? LED_IR_MAX_MA : ir_ma;
    }
    if (apply_mask & 0x02) {
        new_red = (red_ma > LED_RED_MAX_MA) ? LED_RED_MAX_MA : red_ma;
    }
    if (apply_mask & 0x04) {
        if (rf_code > 7) {
            return NC_ST_OUT_OF_RANGE;
        }
        new_rf = rf_code;
    }

    if (apply_mask & 0x03) {
        if (afe4404_set_led_ma(new_ir, new_red) != ESP_OK) {
            return NC_ST_BUSY;
        }
    }
    if (apply_mask & 0x04) {
        if (afe4404_set_tia(new_rf, afe4404_cur_cf()) != ESP_OK) {
            return NC_ST_BUSY;
        }
    }
    settle_blank();

    /* One AGC_STEP per touched LED so downstream amplitude reconcilers
     * see per-channel discontinuities; gain-only manual moves use the
     * led=0xFF convention shared with the automatic gain step. */
    if (apply_mask & 0x01) {
        const uint8_t d[5] = { 0, old_ir, new_ir, old_rf, new_rf };
        ev_emit(NC_EV_AGC_STEP, d, 5);
    }
    if (apply_mask & 0x02) {
        const uint8_t d[5] = { 1, old_red, new_red, old_rf, new_rf };
        ev_emit(NC_EV_AGC_STEP, d, 5);
    }
    if (apply_mask == 0x04) {
        const uint8_t d[5] = { 0xFF, 0, 0, old_rf, new_rf };
        ev_emit(NC_EV_AGC_STEP, d, 5);
    }
    return NC_ST_OK;
}

static nc_ctrl_status_t cb_selftest_run(void *u, uint32_t test_mask)
{
    (void)u;
    if (s_state == NC_STATE_OTA) {
        return NC_ST_WRONG_STATE;
    }
    if (s_lowbatt_stop || s_boot_lowbatt) {
        return NC_ST_LOWBATT;   /* LED-on tests on a critical cell: no */
    }

    set_state(NC_STATE_SELFTEST);
    demand();                   /* SELFTEST state forces acquisition off */
    selftest_execute(test_mask);   /* blocking, sys ctx — I2C serialized */

    /* SELFTEST_DONE pass/fail counts come from the result blob
     * ([u8 ver][u64 t_run][u8 n][n x {id,status,i32 val,i32 thr}]). */
    uint16_t blen = 0;
    const uint8_t *blob = selftest_blob(&blen);
    uint8_t pass = 0, fail = 0;
    if (blob != NULL && blen >= 10) {
        const uint8_t n = blob[9];
        for (uint8_t i = 0; i < n && (10u + 10u * i + 1u) < blen; i++) {
            const uint8_t r = blob[10 + 10 * i + 1];
            if (r == NC_TR_PASS) pass++;
            else if (r == NC_TR_FAIL) fail++;
        }
    }
    const uint8_t d[2] = { pass, fail };
    ev_emit(NC_EV_SELFTEST_DONE, d, 2);

    set_state(s_connected ? NC_STATE_CONNECTED : NC_STATE_IDLE);
    demand();                   /* restore whatever was streaming */
    return NC_ST_OK;
}

static const uint8_t *cb_selftest_blob(void *u, size_t *len)
{
    (void)u;
    uint16_t l16 = 0;
    const uint8_t *b = selftest_blob(&l16);
    *len = l16;
    return b;
}

static nc_ctrl_status_t cb_enter_ota(void *u)
{
    (void)u;
    if (s_state == NC_STATE_OTA) {
        return NC_ST_OK;   /* idempotent re-enter, deadline restarts */
    }
    if (s_state == NC_STATE_SELFTEST) {
        return NC_ST_WRONG_STATE;
    }
    if (s_lowbatt_stop || s_boot_lowbatt) {
        return NC_ST_LOWBATT;   /* don't start a flash write near cutoff */
    }
    s_latch = LATCH_ENTER_OTA;
    return NC_ST_OK;
}

static nc_ctrl_status_t cb_power_off(void *u)
{
    (void)u;
    s_latch = LATCH_POWER_OFF;
    return NC_ST_OK;
}

static nc_ctrl_status_t cb_reboot(void *u)
{
    (void)u;
    s_latch = LATCH_REBOOT;
    return NC_ST_OK;
}

static nc_ctrl_status_t cb_factory_reset(void *u)
{
    (void)u;
    s_latch = LATCH_FACTORY;   /* dispatcher already checked the magic */
    return NC_ST_OK;
}

static nc_ctrl_status_t cb_test_op(void *u, uint8_t op, const uint8_t *payload,
                                   size_t len, uint8_t *resp, size_t *resp_len)
{
    (void)u;
    /* services/test_ops.c answers UNAUTHORIZED when NARBIS_TEST_MODE=0 */
    return test_ops_dispatch(op, payload, len, resp, resp_len);
}

static nc_ctrl_ctx_t s_ctrl = {
    .test_mode = (NARBIS_TEST_MODE != 0),
    .user = NULL,
    .stream_start = cb_stream_start,
    .stream_stop = cb_stream_stop,
    .set_rate = cb_set_rate,
    .knob_save = cb_knob_save,
    .knob_reset = cb_knob_reset,
    .get_time_us = cb_get_time_us,
    .time_sync_seen = cb_time_sync_seen,
    .marker = cb_marker,
    .agc_freeze = cb_agc_freeze,
    .agc_manual = cb_agc_manual,
    .selftest_run = cb_selftest_run,
    .selftest_blob = cb_selftest_blob,
    .enter_ota = cb_enter_ota,
    .power_off = cb_power_off,
    .reboot = cb_reboot,
    .factory_reset = cb_factory_reset,
    .test_op = cb_test_op,
};

static void handle_ctrl(const sys_msg_t *m)
{
    uint8_t resp[NC_ATT_PAYLOAD_MAX];
    size_t rlen = nc_ctrl_dispatch(&s_ctrl, m->u.ctrl.buf, m->u.ctrl.len, resp);
    if (rlen > 0) {
        ble_tx_submit(BLE_CH_CTRL_IND, resp, (uint16_t)rlen);
    }
    s_last_activity_us = esp_timer_get_time();

    /* Response staged first, THEN the destructive action (contract). */
    switch (s_latch) {
    case LATCH_NONE:
        break;
    case LATCH_ENTER_OTA:
        s_latch = LATCH_NONE;
        s_ota_enter_us = esp_timer_get_time();
        set_state(NC_STATE_OTA);
        break;
    case LATCH_POWER_OFF:
        s_latch = LATCH_NONE;
        orderly_off(false);          /* includes its own 300 ms drain */
        break;
    case LATCH_REBOOT:
        s_latch = LATCH_NONE;
        vTaskDelay(pdMS_TO_TICKS(200));   /* let the indication drain */
        esp_restart();
        break;
    case LATCH_FACTORY:
        s_latch = LATCH_NONE;
        vTaskDelay(pdMS_TO_TICKS(200));
        knobs_nvs_reset();
        ble_store_clear();           /* forget every bond */
        esp_restart();
        break;
    }
    demand();
}

/* ------------------------------------------------------------------ */
/* AGC actuation (decisions computed in dsp_task's slow tick)          */
/* ------------------------------------------------------------------ */
static void handle_agc(const sys_msg_t *m)
{
    if (s_agc_frozen || !acq_ppg_running()) {
        return;   /* frozen raced a decision in flight, or stream gone */
    }
    for (uint8_t i = 0; i < m->u.agc.count && i < 4; i++) {
        const nc_agc_act_t *a = &m->u.agc.acts[i];
        bool ok = false;

        switch (a->kind) {
        case NC_AGC_LED: {
            const uint8_t old_ir = afe4404_cur_ir_ma();
            const uint8_t old_red = afe4404_cur_red_ma();
            const uint8_t rf = afe4404_cur_rf();
            const uint8_t tgt_ir = (a->chan == NC_AGC_CH_IR) ? a->led_ma : old_ir;
            const uint8_t tgt_red = (a->chan == NC_AGC_CH_RED) ? a->led_ma : old_red;
            ok = afe4404_set_led_ma(tgt_ir, tgt_red) == ESP_OK;
            if (ok) {
                const uint8_t d[5] = {
                    a->chan,
                    (a->chan == NC_AGC_CH_IR) ? old_ir : old_red,
                    a->led_ma, rf, rf,
                };
                ev_emit(NC_EV_AGC_STEP, d, 5);
            }
            break;
        }
        case NC_AGC_GAIN: {
            const uint8_t old_rf = afe4404_cur_rf();
            ok = afe4404_set_tia(a->rf_code, afe4404_cur_cf()) == ESP_OK;
            if (ok) {
                const uint8_t d[5] = { 0xFF, 0, 0, old_rf, a->rf_code };
                ev_emit(NC_EV_AGC_STEP, d, 5);
            }
            break;
        }
        case NC_AGC_OFFDAC: {
            /* chan 0 (IR) samples in phase LED1, chan 1 (RED) in phase
             * LED2 — afe4404.h phase codes 0/1 in the same order. */
            const uint8_t phase = a->chan;
            const int8_t old = s_offdac[a->chan & 1];
            ok = afe4404_set_offdac(phase, a->offdac) == ESP_OK;
            if (ok) {
                s_offdac[a->chan & 1] = a->offdac;
                const uint8_t d[3] = { phase, (uint8_t)old, (uint8_t)a->offdac };
                ev_emit(NC_EV_AGC_OFFDAC, d, 3);
            }
            break;
        }
        default:
            break;
        }

        if (ok) {
            settle_blank();
            g_diag.agc_steps++;
        } else if (a->kind != NC_AGC_NONE) {
            ESP_LOGW(TAG, "AGC actuation failed (kind %d)", (int)a->kind);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Button gestures                                                     */
/* ------------------------------------------------------------------ */
static void btn_run(nc_btn_ev_t ev, uint32_t t_ms)
{
    uint32_t arm = 0;
    const nc_btn_act_t act = nc_btn_step(&s_btn, ev, t_ms, &arm);

    /* Exactly one one-shot: 0 cancels, nonzero re-arms at t_ms + arm. */
    esp_timer_stop(s_btn_tmr);   /* INVALID_STATE when idle — ignored */
    if (arm != 0) {
        const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        uint32_t delay = t_ms + arm - now;   /* wrap-safe unsigned math */
        if (delay == 0 || delay > 0x80000000u) {
            delay = 1;   /* deadline already passed — fire ASAP */
        }
        esp_timer_start_once(s_btn_tmr, (uint64_t)delay * 1000);
    }

    switch (act) {
    case NC_BTN_ACT_MARKER: {
        const uint8_t d[3] = { NC_MARKER_SRC_BUTTON, 0, 0 };
        ev_emit(NC_EV_MARKER, d, 3);
        break;
    }
    case NC_BTN_ACT_PAIRING:
        ble_open_pairing_window((uint32_t)nc_knob_get(KNOB_PAIRING_WINDOW_S));
        break;
    case NC_BTN_ACT_OFF:
        orderly_off(false);
        break;
    case NC_BTN_ACT_REBOOT:
        esp_restart();   /* escape hatch: immediate, no drain */
        break;
    default:
        break;
    }
    s_last_activity_us = esp_timer_get_time();
}

/* ------------------------------------------------------------------ */
/* 1 Hz housekeeping                                                   */
/* ------------------------------------------------------------------ */
static void lowbatt_policy(int64_t now)
{
    if (s_vusb) {
        /* ON BATTERY only: charging clears every latch and timer. */
        s_warn_since = s_stop_since = s_off_since = -1;
        if (s_lowbatt_warn || s_lowbatt_stop) {
            s_lowbatt_warn = false;
            s_lowbatt_stop = false;
        }
        return;
    }

    const uint16_t warn_mv = (uint16_t)nc_knob_get(KNOB_VBATT_WARN_MV);
    const uint16_t stop_mv = (uint16_t)nc_knob_get(KNOB_VBATT_STOP_MV);
    const uint16_t off_mv = (uint16_t)nc_knob_get(KNOB_VBATT_OFF_MV);

    /* 30 s of continuous violation before each level acts (load sag on
     * the 62 mAh cell can dip readings transiently at stream start). */
    if (s_batt_mv < warn_mv) {
        if (s_warn_since < 0) s_warn_since = now;
        if (!s_lowbatt_warn && now - s_warn_since >= LOWBATT_HYST_US) {
            s_lowbatt_warn = true;
            /* Advisory on EVENT_STREAM: code NONE = notice (not a
             * fault), arg = cell mV. STATUS carries the sticky flag. */
            const uint8_t d[6] = {
                (uint8_t)NC_ERR_NONE, 0,
                (uint8_t)(s_batt_mv & 0xFF), (uint8_t)(s_batt_mv >> 8), 0, 0,
            };
            ev_emit(NC_EV_ERROR, d, 6);
            ESP_LOGW(TAG, "battery warn: %u mV", s_batt_mv);
        }
    } else {
        s_warn_since = -1;
        if (s_lowbatt_warn && s_batt_mv >= warn_mv + LOWBATT_CLEAR_MV) {
            s_lowbatt_warn = false;
        }
    }

    if (s_batt_mv < stop_mv) {
        if (s_stop_since < 0) s_stop_since = now;
        if (!s_lowbatt_stop && now - s_stop_since >= LOWBATT_HYST_US) {
            s_lowbatt_stop = true;   /* demand() forces streaming off */
            ESP_LOGW(TAG, "battery stop: %u mV — streaming blocked", s_batt_mv);
        }
    } else {
        s_stop_since = -1;
        if (s_lowbatt_stop && s_batt_mv >= stop_mv + LOWBATT_CLEAR_MV) {
            s_lowbatt_stop = false;
        }
    }

    if (s_batt_mv < off_mv) {
        if (s_off_since < 0) s_off_since = now;
        if (now - s_off_since >= LOWBATT_HYST_US) {
            ESP_LOGW(TAG, "battery off: %u mV — forced OFF", s_batt_mv);
            orderly_off(true);   /* 12 h recheck timer armed */
        }
    } else {
        s_off_since = -1;
    }
}

static void handle_tick(void)
{
    const int64_t now = esp_timer_get_time();

    uint16_t mv;
    uint8_t pct;
    if (battery_status(&mv, &pct) == ESP_OK) {
        s_batt_mv = mv;
        s_batt_pct = pct;
    }
    bool vusb;
    s_chg = charger_poll(&vusb);
    if (vusb != s_vusb) {
        s_vusb = vusb;
        power_apply_state(s_state, s_vusb);      /* console-liveness lock */
        acq_set_extra_ppg_flags(vusb ? NC_PPGF_USB_PRESENT : 0,
                                vusb ? 0 : NC_PPGF_USB_PRESENT);
    }
    if (s_vusb && s_boot_lowbatt) {
        /* Charging during the boot-refusal window makes the refusal
         * moot — rejoin normal life. Level-checked (not edge-checked)
         * so USB present since before the first tick still cancels. */
        s_boot_lowbatt = false;
        set_state(s_connected ? NC_STATE_CONNECTED : NC_STATE_IDLE);
    }
    if (s_batt_pct != s_batt_pct_sent) {
        s_batt_pct_sent = s_batt_pct;
        ble_update_battery(s_batt_pct);
    }

    lowbatt_policy(now);   /* may not return (forced OFF) */

    if (s_boot_lowbatt && now >= BOOT_LOWBATT_ADV_US) {
        /* Battery-critical advertise window over (handoff §3: refuse
         * streaming wake below vbatt_off; we advertise LOWBATT for 10 s
         * so a nearby host learns why, then sleep with 12 h recheck). */
        orderly_off(true);
    }

    if (s_state == NC_STATE_OTA) {
        (void)ota_deadline_check();   /* engine-side bookkeeping */
        if (!ota_active() && now - s_ota_enter_us >= OTA_NO_BEGIN_TIMEOUT_US) {
            ESP_LOGW(TAG, "OTA: no BEGIN within 60 s — leaving OTA state");
            set_state(s_connected ? NC_STATE_CONNECTED : NC_STATE_IDLE);
        }
    }

#if !NARBIS_TEST_MODE
    /* Auto-sleep policies (compiled out in design-verification builds). */
    if (s_connected || acq_ppg_running()) {
        s_last_activity_us = now;
    }
    const int64_t idle_us = (int64_t)nc_knob_get(KNOB_IDLE_TIMEOUT_S) * 1000000;
    if (!s_connected && now - s_last_activity_us >= idle_us) {
        ESP_LOGI(TAG, "idle timeout — OFF");
        orderly_off(false);
    }
    if (!s_worn && s_offear_since >= 0) {
        const int64_t offear_us =
            (int64_t)nc_knob_get(KNOB_OFFEAR_SLEEP_S) * 1000000;
        if (now - s_offear_since >= offear_us) {
            ESP_LOGI(TAG, "off-ear timeout — OFF");
            orderly_off(false);
        }
    }
#endif

    demand();
    status_sync(true);   /* the 1 Hz STATUS heartbeat */
}

/* ------------------------------------------------------------------ */
/* Orderly OFF                                                         */
/* ------------------------------------------------------------------ */
static void orderly_off(bool battery_forced)
{
    ESP_LOGI(TAG, "orderly OFF");
    if (acq_ppg_running()) {
        acq_ppg_stop();
    }
    if (acq_accel_running()) {
        acq_accel_stop();
    }
    /* Explicit-save semantics: dirty-but-unsaved knob edits are dropped
     * by design; only a previously REQUESTED save that failed (NVS
     * error) gets one retry here. */
    if (s_save_pending && knobs_nvs_save() == ESP_OK) {
        s_save_pending = false;
    }
    status_sync(true);                    /* final snapshot for the host */
    vTaskDelay(pdMS_TO_TICKS(300));       /* ble_tx drain window */
    power_enter_off(battery_forced);
}

/* main.c calls this (pre-scheduling of the sys task) when the boot
 * battery reading is below vbatt_off: 10 s battery-critical advertise,
 * then OFF with the 12 h recheck. */
void sys_task_set_boot_lowbatt(void)
{
    s_boot_lowbatt = true;
}

/* ------------------------------------------------------------------ */
/* Timer callbacks (esp_timer task context — plain sys_post)           */
/* ------------------------------------------------------------------ */
static void tick_cb(void *arg)
{
    (void)arg;
    const sys_msg_t m = { .type = SYS_TICK_1HZ };
    sys_post(&m);   /* overflow: skip a beat, the next tick catches up */
}

static void btn_timeout_cb(void *arg)
{
    (void)arg;
    const sys_msg_t m = { .type = SYS_BTN_TIMEOUT };
    sys_post(&m);
}

/* ------------------------------------------------------------------ */
/* Task body (task itself is created by acq_init)                      */
/* ------------------------------------------------------------------ */
void sys_task_run(void *arg)
{
    (void)arg;
    configASSERT(sys_q != NULL);

    nc_btn_init(&s_btn);
    nc_ts_init(&s_ts);
    s_in.usb_stream_ok = true;
    s_last_activity_us = esp_timer_get_time();

    const esp_timer_create_args_t tick_args = { .callback = tick_cb,
                                                .name = "sys_1hz" };
    const esp_timer_create_args_t btn_args = { .callback = btn_timeout_cb,
                                               .name = "sys_btn" };
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &s_tick_tmr));
    ESP_ERROR_CHECK(esp_timer_create(&btn_args, &s_btn_tmr));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_tick_tmr, 1000000));

    /* Baseline snapshot before the first tick. */
    s_chg = charger_poll(&s_vusb);
    battery_status(&s_batt_mv, &s_batt_pct);
    acq_set_extra_ppg_flags(s_vusb ? NC_PPGF_USB_PRESENT : 0,
                            s_vusb ? 0 : NC_PPGF_USB_PRESENT);
    if (s_boot_lowbatt) {
        s_state = NC_STATE_LOWBATT;
    }
    power_apply_state(s_state, s_vusb);

    for (;;) {
        sys_msg_t m;
        if (xQueueReceive(sys_q, &m, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        switch (m.type) {
        case SYS_BTN_EDGE:
            btn_run(m.u.btn.pressed ? NC_BTN_EV_DOWN : NC_BTN_EV_UP,
                    m.u.btn.t_ms);
            break;
        case SYS_BTN_TIMEOUT:
            btn_run(NC_BTN_EV_TIMEOUT,
                    (uint32_t)(esp_timer_get_time() / 1000));
            break;
        case SYS_AGC_ACTIONS:
            handle_agc(&m);
            break;
        case SYS_SUB_CHANGE:
            /* Re-subscribe of a source clears its latched stop bit
             * (nc_acq_policy contract). IBI and HRS share the IBI
             * demand path. */
            if (m.u.subs.ppg && !s_in.sub_ppg) {
                s_in.ctrl_stop_mask &= (uint8_t)~NC_STREAM_MASK_PPG;
            }
            if (m.u.subs.accel && !s_in.sub_accel) {
                s_in.ctrl_stop_mask &= (uint8_t)~NC_STREAM_MASK_ACCEL;
            }
            if ((m.u.subs.ibi && !s_in.sub_ibi) ||
                (m.u.subs.hrs && !s_in.sub_hrs)) {
                s_in.ctrl_stop_mask &= (uint8_t)~NC_STREAM_MASK_IBI;
            }
            s_in.sub_ppg = m.u.subs.ppg;
            s_in.sub_accel = m.u.subs.accel;
            s_in.sub_ibi = m.u.subs.ibi;
            s_in.sub_event = m.u.subs.event;
            s_in.sub_hrs = m.u.subs.hrs;
            s_last_activity_us = esp_timer_get_time();
            demand();
            break;
        case SYS_CTRL_REQ:
            handle_ctrl(&m);
            break;
        case SYS_TICK_1HZ:
            handle_tick();
            break;
        case SYS_WEAR_CHANGED:
            if (m.u.flag != s_worn) {
                s_worn = m.u.flag;
                const uint8_t d[1] = { (uint8_t)s_worn };
                ev_emit(NC_EV_WEAR, d, 1);
                acq_set_extra_ppg_flags(s_worn ? 0 : NC_PPGF_WEAR_OFF,
                                        s_worn ? NC_PPGF_WEAR_OFF : 0);
                s_offear_since = s_worn ? -1 : esp_timer_get_time();
                demand();          /* auto-pause via wear_paused input */
                status_sync(false);
            }
            break;
        case SYS_GATE_CHANGED: {
            const uint8_t d[2] = { (uint8_t)m.u.gate.on, m.u.gate.reason };
            ev_emit(NC_EV_GATE, d, 2);
            g_diag.gate_transitions++;
            status_sync(false);
            break;
        }
        case SYS_POWER_OFF_REQ:
            orderly_off(false);
            break;
        case SYS_CONN_CHANGE:
            s_connected = m.u.flag;
            if (!s_connected) {
                /* Session state dies with the link: subscriptions,
                 * stream overrides and the AGC freeze are per-central. */
                s_in.sub_ppg = s_in.sub_accel = s_in.sub_ibi = false;
                s_in.sub_event = s_in.sub_hrs = false;
                s_in.ctrl_start_mask = 0;
                s_in.ctrl_stop_mask = 0;
                if (s_agc_frozen) {
                    s_agc_frozen = false;
                    acq_set_agc_frozen(false);
                }
                if (s_state == NC_STATE_OTA) {
                    /* Engine keeps the partial image for resume; the
                     * mode itself does not survive the link. */
                    set_state(NC_STATE_IDLE);
                }
            }
            s_last_activity_us = esp_timer_get_time();
            demand();
            break;
        case SYS_ENTER_OTA:
            /* OTA service saw a BEGIN without a prior CONTROL enter. */
            s_ota_enter_us = esp_timer_get_time();
            set_state(NC_STATE_OTA);
            demand();              /* stop acquisition for flash writes */
            break;
        default:
            ESP_LOGW(TAG, "unknown msg type %d", (int)m.type);
            break;
        }
        pump_events();
    }
}
