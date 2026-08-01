/*
 * test_ops.c — TEST opcode block 0xE0..0xE9 (+ defensive 0xEA), only
 * compiled when NARBIS_TEST_MODE=1 (board.h). Production builds get a
 * two-line stub: zero test surface in flash, matching the dispatcher's
 * unconditional NC_ST_UNAUTHORIZED gate on 0xE0..0xEF.
 *
 * Execution contract (see test_ops.h): sys_task calls this INLINE, so
 * every op already runs at the single config-I2C serialization point.
 * Ops that need exclusive AFE frame capture (0xE2/0xE3/0xE4) refuse
 * while the PPG pipeline runs; they bring the AFE up themselves via the
 * bench helper below and BLOCK sys_task for their duration (up to ~15 s
 * sweep / 30 s rate count) — bench-only behavior by design.
 *
 * Deferred actions that must NOT run in timer context re-enter through
 * sys_q: the LED auto-off timer posts a synthetic CONTROL request
 * [0xE1][tid=0][{0,0,0,0}] so the actual LEDCNTRL write happens in
 * sys_task like every other mutation (tid 0 = device-internal; the
 * resulting unsolicited response indication is documented protocol
 * noise). Only if sys_q is full does the timer write the LEDs off
 * directly — safety beats architecture for an emitter left on.
 */
#include "board.h"
#include "test_ops.h"

#if !NARBIS_TEST_MODE

nc_ctrl_status_t test_ops_dispatch(uint8_t op, const uint8_t *pl,
                                   size_t len, uint8_t *resp,
                                   size_t *resp_len)
{
    (void)op; (void)pl; (void)len; (void)resp; (void)resp_len;
    return NC_ST_UNAUTHORIZED;
}

void test_ops_on_disconnect(void)
{
}

void test_ops_btn_edge(bool pressed, uint32_t t_ms)
{
    (void)pressed; (void)t_ms;
}

#else /* NARBIS_TEST_MODE */

#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "narbis/nc_types.h"
#include "narbis/nc_control.h"        /* NC_CTRL_RESP_PAYLOAD_MAX */
#include "narbis/nc_knobs.h"
#include "narbis/nc_proto_encode.h"

#include "afe4404.h"
#include "afe4404_regs.h"
#include "lis2dh12.h"
#include "battery.h"
#include "charger.h"
#include "acq.h"
#include "app_msgs.h"
#include "diag.h"
#include "selftest.h"
#include "sensor_types.h"
#include "test_led_ind.h"

static const char *TAG = "test_ops";

/* sys_task.c (we run inline in sys context): freeze AGC exactly like
 * NC_OP_AGC_FREEZE so STATUS + conn-teardown semantics stay coherent
 * (link-time contract, same idiom as ble_request_conn_speed). */
extern void sys_agc_freeze_set(bool freeze);

/* ------------------------------------------------------------------ */
/* Sweep report blob (layout documented in test_ops.h). Served through */
/* selftest_set_external_blob -> CONTROL 0xEA / 0x51 chunking.         */
/* ------------------------------------------------------------------ */
#define REPORT_HDR   4
#define REPORT_REC   13
#define REPORT_MAX_POINTS 58              /* 4 + 58*13 = 758 <= 768 */

static uint8_t  s_report[768];
static uint16_t s_report_len;

/* ------------------------------------------------------------------ */
/* Bench AFE: ops that run without the PPG pipeline bring the AFE up   */
/* themselves. sys_task tears acquisition state down/up around test    */
/* mode sessions; a later acq_ppg_start() simply re-inits over this.   */
/* ------------------------------------------------------------------ */
static bool      s_bench_up;
static nc_rate_t s_bench_rate;

static esp_err_t ensure_bench_afe(nc_rate_t rate)
{
    /* The cache alone goes stale when anything else powers the AFE
     * down (sys selftest, acq stop, OTA validator) — consult the
     * driver's live up-flag so bench ops self-heal with a re-init. */
    if (!s_bench_up || !afe4404_is_up()) {
        esp_err_t err = afe4404_init(rate);
        if (err != ESP_OK) {
            return err;
        }
        s_bench_up = true;
        s_bench_rate = rate;
    } else if (rate != s_bench_rate) {
        esp_err_t err = afe4404_apply_rate(rate);
        if (err != ESP_OK) {
            return err;
        }
        s_bench_rate = rate;
    }
    return ESP_OK;
}

/* Exported flavor for test_led_ind.c (sys ctx): the indicator runs at
 * the same 100 sps the bench ops default to, and must not re-init an
 * AFE the PPG pipeline owns. */
esp_err_t test_ops_ensure_bench_afe(void)
{
    if (acq_ppg_running()) {
        return ESP_OK;             /* pipeline owns a live AFE */
    }
    return ensure_bench_afe(NC_RATE_100);
}

/* ------------------------------------------------------------------ */
/* Timers (created lazily; all cbs run in the esp_timer task)          */
/* ------------------------------------------------------------------ */
static esp_timer_handle_t s_led_off_tmr;
static esp_timer_handle_t s_sleep_tmr;

static void led_off_cb(void *arg)
{
    (void)arg;
    /* Re-enter through sys_q (see file header); payload {led,ma,dur}=0. */
    sys_msg_t m = { .type = SYS_CTRL_REQ };
    m.u.ctrl.len = 6;
    memset(m.u.ctrl.buf, 0, 6);
    m.u.ctrl.buf[0] = NC_OP_TEST_LED_DRIVE;
    if (!sys_post(&m)) {
        ESP_LOGW(TAG, "sys_q full — LED off written directly");
        afe4404_set_led_ma(0, 0);
    }
}

static void sleep_now_cb(void *arg)
{
    (void)arg;
    sys_msg_t m = { .type = SYS_POWER_OFF_REQ };
    sys_post(&m);
}

/* ------------------------------------------------------------------ */
/* 0xEB — continuous LED triangle sweep engine                         */
/* ------------------------------------------------------------------ */
/* 100 ms esp_timer steps compute the drive from elapsed phase time:
 * ramp 0->max over phase_s, hold max phase_s, ramp ->0 phase_s, hold 0
 * phase_s, repeat. Direct afe4404_set_led_ma from the esp_timer task is
 * deliberate and safe: the driver mutex serializes the ~10 Hz register
 * writes against sys-context config I2C (spec'd in proto.h 0xEB). */
static esp_timer_handle_t s_sweep_tmr;
static volatile bool      s_sweep_on;
static uint8_t            s_sweep_mask;      /* b0 IR, b1 RED */
static uint32_t           s_sweep_phase_us;
static int64_t            s_sweep_t0;

static uint8_t tri_ma(int64_t t_us, uint32_t phase_us, uint8_t max)
{
    const uint64_t cyc = (uint64_t)phase_us * 4u;
    const uint64_t t = (uint64_t)t_us % cyc;
    const uint32_t seg = (uint32_t)(t / phase_us);
    const uint32_t frac = (uint32_t)(t % phase_us);
    switch (seg) {
    case 0:  return (uint8_t)(((uint64_t)max * frac) / phase_us);
    case 1:  return max;
    case 2:  return (uint8_t)(((uint64_t)max * (phase_us - frac)) / phase_us);
    default: return 0;
    }
}

static void sweep_tick_cb(void *arg)
{
    (void)arg;
    if (!s_sweep_on) {
        return;                   /* stop raced an in-flight dispatch */
    }
    const int64_t t = esp_timer_get_time() - s_sweep_t0;
    /* Non-swept LED keeps its current (driver-cached) value. */
    const uint8_t ir = (s_sweep_mask & 0x01)
                           ? tri_ma(t, s_sweep_phase_us, LED_IR_MAX_MA)
                           : afe4404_cur_ir_ma();
    const uint8_t red = (s_sweep_mask & 0x02)
                            ? tri_ma(t, s_sweep_phase_us, LED_RED_MAX_MA)
                            : afe4404_cur_red_ma();
    (void)afe4404_set_led_ma(ir, red);
}

/* sys ctx. Swept LEDs end at 0; the connect indicator re-arms (and
 * repaints) only once the device is also disconnected. */
static void sweep_stop(void)
{
    if (!s_sweep_on) {
        return;
    }
    s_sweep_on = false;
    esp_timer_stop(s_sweep_tmr);
    /* An already-dispatched callback may still be running past its
     * s_sweep_on check; give the (high-priority) esp_timer task a beat
     * so our final write below lands last. */
    vTaskDelay(pdMS_TO_TICKS(20));
    if (afe4404_is_up()) {
        const uint8_t ir = (s_sweep_mask & 0x01) ? 0 : afe4404_cur_ir_ma();
        const uint8_t red = (s_sweep_mask & 0x02) ? 0 : afe4404_cur_red_ma();
        (void)afe4404_set_led_ma(ir, red);
    }
    ESP_LOGI(TAG, "continuous sweep stopped");
}

static esp_err_t tmr_lazy(esp_timer_handle_t *h, esp_timer_cb_t cb,
                          const char *name)
{
    if (*h != NULL) {
        return ESP_OK;
    }
    const esp_timer_create_args_t a = { .callback = cb, .name = name };
    return esp_timer_create(&a, h);
}

/* ------------------------------------------------------------------ */
/* Sweep engine shared by 0xE2/0xE3                                    */
/* ------------------------------------------------------------------ */
static void report_begin(uint8_t kind, uint8_t param)
{
    s_report[0] = 2;                  /* blob ver 2 = sweep report */
    s_report[1] = kind;
    s_report[2] = param;
    s_report[3] = 0;                  /* n_points, patched at end */
    s_report_len = REPORT_HDR;
}

static bool report_point(uint8_t setting)
{
    if (s_report[3] >= REPORT_MAX_POINTS) {
        return false;
    }
    int32_t ir, red, amb;
    vTaskDelay(pdMS_TO_TICKS(100));   /* analog settle after each step */
    /* 32 frames (320 ms @100 sps): first hardware showed mains-flicker
     * residue ~ one 2 mA step with mean-of-10 — LED and ambient phases
     * sample the room light at different instants, so the subtraction
     * can't cancel the beat; only averaging over whole beat cycles
     * does. 320 ms spans 6+ cycles of the 20 Hz (120 Hz vs 100 sps)
     * beat. Sweep wall time ~11 s worst case; app timeout is 38 s. */
    if (selftest_capture_mean(32, &ir, &red, &amb) != ESP_OK) {
        return false;
    }
    uint8_t *p = s_report + s_report_len;
    p[0] = setting;
    nc_wr_i32(p + 1, ir);
    nc_wr_i32(p + 5, red);
    nc_wr_i32(p + 9, amb);
    s_report_len += REPORT_REC;
    s_report[3]++;
    return true;
}

static void report_publish(void)
{
    selftest_set_external_blob(s_report, s_report_len);
}

/* ------------------------------------------------------------------ */
/* Op handlers                                                         */
/* ------------------------------------------------------------------ */

/* 0xE0 — run one self-test by id; response = its 10-byte blob record. */
static nc_ctrl_status_t op_selftest_one(const uint8_t *pl, size_t len,
                                        uint8_t *resp, size_t *resp_len)
{
    if (len != 1) {
        return NC_ST_BAD_LEN;
    }
    uint8_t id = pl[0];
    if (id < 1 || id > NC_TEST_COUNT_) {
        return NC_ST_BAD_PARAM;
    }
    esp_err_t err = selftest_execute(1u << (id - 1));
    if (err != ESP_OK) {
        return NC_ST_WRONG_STATE;     /* acquisition still running */
    }
    uint16_t blen = 0;
    const uint8_t *blob = selftest_blob(&blen);
    size_t off = 10 + (size_t)(id - 1) * NC_ST_REC_SIZE;
    if (blob == NULL || blen < off + NC_ST_REC_SIZE) {
        return NC_ST_WRONG_STATE;
    }
    memcpy(resp, blob + off, NC_ST_REC_SIZE);
    *resp_len = NC_ST_REC_SIZE;
    return NC_ST_OK;
}

/* 0xE1 — direct LED drive with auto-off. Works on a live stream (the
 * point: watch the photocurrent) or brings the bench AFE up. Response
 * = post-clamp actuals so the operator sees the 40/50 mA ceilings. */
static nc_ctrl_status_t op_led_drive(const uint8_t *pl, size_t len,
                                     uint8_t *resp, size_t *resp_len)
{
    if (len != 4) {
        return NC_ST_BAD_LEN;
    }
    uint8_t led = pl[0], ma = pl[1];
    uint16_t dur = nc_rd_u16(pl + 2);
    if (led > 1) {
        return NC_ST_BAD_PARAM;
    }
    test_led_ind_release();        /* LED-affecting op: indicator off */
    if (!acq_ppg_running() && ensure_bench_afe(NC_RATE_100) != ESP_OK) {
        return NC_ST_WRONG_STATE;
    }
    if (tmr_lazy(&s_led_off_tmr, led_off_cb, "to_ledoff") != ESP_OK) {
        return NC_ST_BUSY;
    }
    esp_timer_stop(s_led_off_tmr);    /* re-arm below; err if idle = fine */

    esp_err_t err;
    if (ma == 0 || dur == 0) {
        err = afe4404_set_led_ma(0, 0);
    } else {
        if (dur > 60000) {
            dur = 60000;
        }
        err = afe4404_set_led_ma(led == 0 ? ma : 0, led == 1 ? ma : 0);
        if (err == ESP_OK) {
            esp_timer_start_once(s_led_off_tmr, (uint64_t)dur * 1000);
        }
    }
    if (err != ESP_OK) {
        return NC_ST_WRONG_STATE;
    }
    resp[0] = afe4404_cur_ir_ma();
    resp[1] = afe4404_cur_red_ma();
    *resp_len = 2;
    return NC_ST_OK;
}

/* 0xE2 — LED I-V sweep 0..clamp-max in ma_step. Flat DC response =
 * open LED / disconnected FFC; compression at high mA = supply sag. */
static nc_ctrl_status_t op_led_sweep(const uint8_t *pl, size_t len,
                                     uint8_t *resp, size_t *resp_len)
{
    if (len != 2) {
        return NC_ST_BAD_LEN;
    }
    uint8_t led = pl[0], step = pl[1];
    if (led > 1 || step < 1 || step > 20) {
        return NC_ST_BAD_PARAM;
    }
    if (acq_ppg_running()) {
        return NC_ST_WRONG_STATE;     /* needs the capture ISR */
    }
    test_led_ind_release();           /* LED-affecting op: indicator off */
    if (ensure_bench_afe(NC_RATE_100) != ESP_OK) {
        return NC_ST_WRONG_STATE;
    }
    uint8_t max = (led == 1) ? LED_RED_MAX_MA : LED_IR_MAX_MA;

    report_begin(1, led);
    bool ok = true;
    for (uint16_t ma = 0; ma <= max && ok; ma += step) {
        ok = (afe4404_set_led_ma(led == 0 ? (uint8_t)ma : 0,
                                 led == 1 ? (uint8_t)ma : 0) == ESP_OK) &&
             report_point((uint8_t)ma);
    }
    afe4404_set_led_ma(0, 0);
    if (!ok) {
        s_report_len = 0;
        selftest_set_external_blob(NULL, 0);
        return NC_ST_WRONG_STATE;
    }
    report_publish();
    resp[0] = s_report[3];
    nc_wr_u16(resp + 1, s_report_len);
    *resp_len = 3;
    ESP_LOGI(TAG, "LED%u sweep: %u points", led, s_report[3]);
    return NC_ST_OK;
}

/* 0xE3 — RX sweep at fixed IR drive (20 mA): TIA RF codes 0..7, or
 * offset-DAC codes -15..+15 on the IR phase. Restores RF=100k / DAC=0. */
static nc_ctrl_status_t op_rx_sweep(const uint8_t *pl, size_t len,
                                    uint8_t *resp, size_t *resp_len)
{
    if (len != 1) {
        return NC_ST_BAD_LEN;
    }
    uint8_t what = pl[0];
    if (what > 1) {
        return NC_ST_BAD_PARAM;
    }
    if (acq_ppg_running()) {
        return NC_ST_WRONG_STATE;
    }
    test_led_ind_release();           /* LED-affecting op: indicator off */
    if (ensure_bench_afe(NC_RATE_100) != ESP_OK ||
        afe4404_set_led_ma(20, 0) != ESP_OK) {
        return NC_ST_WRONG_STATE;
    }

    report_begin(2, what);
    bool ok = true;
    if (what == 0) {
        for (uint8_t rf = 0; rf <= 7 && ok; rf++) {
            ok = (afe4404_set_tia(rf, AFE_CF_5PF) == ESP_OK) &&
                 report_point(rf);
        }
        afe4404_set_tia(AFE_RF_100K, AFE_CF_5PF);
    } else {
        for (int8_t code = -15; code <= 15 && ok; code++) {
            ok = (afe4404_set_offdac(0, code) == ESP_OK) &&
                 report_point((uint8_t)code);
        }
        afe4404_set_offdac(0, 0);
    }
    afe4404_set_led_ma(0, 0);
    if (!ok) {
        s_report_len = 0;
        selftest_set_external_blob(NULL, 0);
        return NC_ST_WRONG_STATE;
    }
    report_publish();
    resp[0] = s_report[3];
    nc_wr_u16(resp + 1, s_report_len);
    *resp_len = 3;
    return NC_ST_OK;
}

/* 0xE4 — ADC_RDY pulse count over N s at the configured ppg_rate knob.
 * proto.h notes "async done event"; implemented synchronously instead —
 * sys_task blocks (bench-only) and the response IS the completion. */
static nc_ctrl_status_t op_rate_count(const uint8_t *pl, size_t len,
                                      uint8_t *resp, size_t *resp_len)
{
    if (len != 1) {
        return NC_ST_BAD_LEN;
    }
    uint32_t secs = pl[0];
    if (secs < 1) {
        secs = 1;
    }
    if (secs > 30) {
        secs = 30;
    }
    if (acq_ppg_running()) {
        return NC_ST_WRONG_STATE;
    }
    nc_rate_t rate = (nc_rate_t)nc_knob_get(KNOB_PPG_RATE);
    if (rate >= NC_RATE_COUNT || ensure_bench_afe(rate) != ESP_OK) {
        return NC_ST_WRONG_STATE;
    }
    uint32_t pulses = 0, elapsed = 0;
    if (selftest_count_pulses(secs * 1000, &pulses, &elapsed) != ESP_OK) {
        return NC_ST_WRONG_STATE;
    }
    nc_wr_u32(resp, pulses);
    nc_wr_u32(resp + 4, elapsed);
    *resp_len = 8;
    ESP_LOGI(TAG, "rate count: %" PRIu32 " pulses / %" PRIu32 " ms "
             "(expect %u sps)", pulses, elapsed, nc_rate_sps(rate));
    return NC_ST_OK;
}

/* 0xE5 — button echo on/off. Armed = the debounced FSM edges are
 * mirrored to EVENT_STREAM by test_ops_btn_edge() (sys_task's
 * SYS_BTN_EDGE handler). The first design polled the raw GPIO at 50 Hz
 * from an esp_timer; on real hardware the poll went silent after the
 * first press (power-managed idle) while the ISR->debounce->FSM path
 * kept delivering — so the echo now rides that proven path. */
static bool s_btn_echo_on;

static nc_ctrl_status_t op_button_echo(const uint8_t *pl, size_t len)
{
    if (len != 1) {
        return NC_ST_BAD_LEN;
    }
    s_btn_echo_on = (pl[0] != 0);
    return NC_ST_OK;
}

void test_ops_btn_edge(bool pressed, uint32_t t_ms)
{
    (void)t_ms;
    if (!s_btn_echo_on || event_q == NULL) {
        return;
    }
    uint16_t id = (uint16_t)(1000 + (pressed ? 0 : 1)); /* level after edge */
    nc_event_t ev = {
        .t_us = (uint64_t)esp_timer_get_time(),
        .type = NC_EV_MARKER,
        .len = 3,
        .data = { NC_MARKER_SRC_BUTTON, (uint8_t)id, (uint8_t)(id >> 8) },
    };
    (void)xQueueSend(event_q, &ev, 0);
}

/* 0xE6 — charger snapshot. Polled-response design (stateless): the
 * host polls at whatever cadence it likes while the operator plugs and
 * unplugs; the legacy "enable" byte from proto.h is accepted and
 * ignored. Response {u8 vusb, u8 stat_raw, u8 decoded_state}. */
static nc_ctrl_status_t op_charger_live(size_t len, uint8_t *resp,
                                        size_t *resp_len)
{
    if (len > 1) {
        return NC_ST_BAD_LEN;
    }
    bool vusb = false;
    nc_charger_state_t st = charger_poll(&vusb);
    resp[0] = vusb ? 1 : 0;
    resp[1] = (uint8_t)gpio_get_level(PIN_CHG_STAT);
    resp[2] = (uint8_t)st;
    *resp_len = 3;
    return NC_ST_OK;
}

/* 0xE7 — battery raw: calibrated mV + raw ADC mean (divider/cal DoD). */
static nc_ctrl_status_t op_batt_raw(size_t len, uint8_t *resp,
                                    size_t *resp_len)
{
    if (len != 0) {
        return NC_ST_BAD_LEN;
    }
    uint16_t mv = 0, raw = 0;
    if (battery_read_mv(&mv, &raw) != ESP_OK) {
        return NC_ST_WRONG_STATE;
    }
    nc_wr_u16(resp, mv);
    nc_wr_u16(resp + 2, raw);
    *resp_len = 4;
    return NC_ST_OK;
}

/* 0xE8 — accel live at max ODR. Enable: start the accel pipeline if
 * idle, then push the part to 400 Hz (driver re-config flushes the
 * FIFO; imu_task keeps draining at the watermark). Disable: restore
 * knob ODR/FS, stop the pipeline if we started it. Response always
 * carries the cumulative FIFO overrun counter. */
static bool s_e8_started;

static nc_ctrl_status_t op_accel_live(const uint8_t *pl, size_t len,
                                      uint8_t *resp, size_t *resp_len)
{
    if (len != 1) {
        return NC_ST_BAD_LEN;
    }
    uint8_t fs = (uint8_t)nc_knob_get(KNOB_ACC_FS);
    if (pl[0]) {
        if (!acq_accel_running()) {
            if (acq_accel_start() != ESP_OK) {
                return NC_ST_WRONG_STATE;
            }
            s_e8_started = true;
        }
        if (lis2dh12_config(NC_ODR_400, fs) != ESP_OK) {
            return NC_ST_WRONG_STATE;
        }
    } else {
        if (s_e8_started) {
            acq_accel_stop();         /* powers the part down */
            s_e8_started = false;
        } else if (acq_accel_running()) {
            lis2dh12_config((nc_acc_odr_t)nc_knob_get(KNOB_ACC_ODR), fs);
        }
    }
    nc_wr_u32(resp, g_diag.fifo_overrun);
    *resp_len = 4;
    return NC_ST_OK;
}

/* 0xE9 — OFF in 10 s for bench current measurement (button wakes). */
static nc_ctrl_status_t op_sleep_now(size_t len)
{
    if (len != 0) {
        return NC_ST_BAD_LEN;
    }
    if (tmr_lazy(&s_sleep_tmr, sleep_now_cb, "to_sleep") != ESP_OK) {
        return NC_ST_BUSY;
    }
    esp_timer_stop(s_sleep_tmr);
    esp_timer_start_once(s_sleep_tmr, 10 * 1000 * 1000);
    ESP_LOGW(TAG, "entering OFF in 10 s (bench current measurement)");
    return NC_ST_OK;
}

/* 0xEA — defensive only: the CONTROL dispatcher serves TEST_REPORT
 * itself via the selftest_blob chunker (we publish sweep reports there
 * with selftest_set_external_blob). Kept for direct callers; same
 * chunk layout [u16 total][u16 offset][u8 n][bytes]. */
static nc_ctrl_status_t op_report(const uint8_t *pl, size_t len,
                                  uint8_t *resp, size_t *resp_len)
{
    if (len != 2) {
        return NC_ST_BAD_LEN;
    }
    if (s_report_len == 0) {
        return NC_ST_WRONG_STATE;
    }
    uint16_t off = nc_rd_u16(pl);
    if (off > s_report_len) {
        return NC_ST_BAD_PARAM;
    }
    size_t n = s_report_len - off;
    const size_t cap = NC_CTRL_RESP_PAYLOAD_MAX - 5;
    if (n > cap) {
        n = cap;
    }
    nc_wr_u16(resp, s_report_len);
    nc_wr_u16(resp + 2, off);
    resp[4] = (uint8_t)n;
    memcpy(resp + 5, s_report + off, n);
    *resp_len = 5 + n;
    return NC_ST_OK;
}

/* 0xEB — continuous triangle sweep {u8 mask b0 IR b1 RED, u8 enable,
 * u8 phase_s (0 -> 5)}. Runs on the bench AFE or over a live stream
 * (the point: watch photocurrent tracking); AGC is frozen on start via
 * the sys-owned path so STATUS shows it and conn teardown unfreezes.
 * enable=0 leaves the freeze latched — the operator (or disconnect)
 * clears it, mirroring NC_OP_AGC_FREEZE semantics. */
static nc_ctrl_status_t op_led_sweep_cont(const uint8_t *pl, size_t len)
{
    if (len != 3) {
        return NC_ST_BAD_LEN;
    }
    const uint8_t mask = pl[0], enable = pl[1];
    uint8_t phase_s = pl[2];

    if (!enable) {
        sweep_stop();              /* idempotent when idle */
        return NC_ST_OK;
    }

    if (mask == 0 || (mask & (uint8_t)~0x03u)) {
        return NC_ST_BAD_PARAM;
    }
    if (phase_s == 0) {
        phase_s = 5;               /* proto.h: 0 -> 5 s default */
    }
    if (tmr_lazy(&s_sweep_tmr, sweep_tick_cb, "to_sweepc") != ESP_OK) {
        return NC_ST_BUSY;
    }
    test_led_ind_release();        /* LED-affecting op: indicator off */
    if (!acq_ppg_running() && ensure_bench_afe(NC_RATE_100) != ESP_OK) {
        return NC_ST_WRONG_STATE;
    }
    sys_agc_freeze_set(true);      /* AGC must not fight the ramp */

    /* (Re)start: a second enable=1 retunes mask/phase in place. */
    s_sweep_on = false;
    esp_timer_stop(s_sweep_tmr);   /* INVALID_STATE when idle — ignored */
    s_sweep_mask = mask;
    s_sweep_phase_us = (uint32_t)phase_s * 1000000u;
    s_sweep_t0 = esp_timer_get_time();
    s_sweep_on = true;
    esp_timer_start_periodic(s_sweep_tmr, 100 * 1000);
    ESP_LOGI(TAG, "continuous sweep: mask=0x%02x phase=%us", mask, phase_s);
    return NC_ST_OK;
}

/* ------------------------------------------------------------------ */
/* sys ctx (conn_sync, disconnect edge): session-scoped TEST machinery */
/* dies with the link. Button echo and the 0xE1 auto-off timer stay    */
/* armed — they are harmless disconnected and their events just drop.  */
/* ------------------------------------------------------------------ */
void test_ops_on_disconnect(void)
{
    sweep_stop();
    s_btn_echo_on = false;   /* session-scoped, like the sweep */
}

/* ------------------------------------------------------------------ */
nc_ctrl_status_t test_ops_dispatch(uint8_t op, const uint8_t *pl,
                                   size_t len, uint8_t *resp,
                                   size_t *resp_len)
{
    switch (op) {
    case NC_OP_TEST_SELFTEST_ONE: return op_selftest_one(pl, len, resp, resp_len);
    case NC_OP_TEST_LED_DRIVE:    return op_led_drive(pl, len, resp, resp_len);
    case NC_OP_TEST_LED_SWEEP:    return op_led_sweep(pl, len, resp, resp_len);
    case NC_OP_TEST_RX_SWEEP:     return op_rx_sweep(pl, len, resp, resp_len);
    case NC_OP_TEST_RATE_COUNT:   return op_rate_count(pl, len, resp, resp_len);
    case NC_OP_TEST_BUTTON_ECHO:  return op_button_echo(pl, len);
    case NC_OP_TEST_CHARGER_LIVE: return op_charger_live(len, resp, resp_len);
    case NC_OP_TEST_BATT_RAW:     return op_batt_raw(len, resp, resp_len);
    case NC_OP_TEST_ACCEL_LIVE:   return op_accel_live(pl, len, resp, resp_len);
    case NC_OP_TEST_SLEEP_NOW:    return op_sleep_now(len);
    case NC_OP_TEST_REPORT:       return op_report(pl, len, resp, resp_len);
    case NC_OP_TEST_LED_SWEEP_CONT: return op_led_sweep_cont(pl, len);
    default:                      return NC_ST_UNKNOWN_OP;
    }
}

#endif /* NARBIS_TEST_MODE */
