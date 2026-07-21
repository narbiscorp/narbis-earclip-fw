/*
 * selftest_hw.c — hardware self-test T01..T08 + the shared bench-capture
 * helpers used by test_ops.c.
 *
 * Runs in sys_task with acquisition stopped (contract in selftest.h):
 * the ADC_RDY GPIO and both sensors are exclusively ours for the
 * duration. AFE-dependent tests bring the AFE up themselves
 * (afe4404_init @100 sps); exit state is documented per test below and
 * restoring the pre-test acquisition state is sys_task's job.
 *
 * ADC_RDY is a ~250 ns pulse — far too narrow to poll, so frame capture
 * uses a temporary rising-edge ISR that timestamps nothing and only
 * gives a semaphore / bumps a counter (ISR discipline: FromISR only,
 * no I2C). The frame data registers are stable until the next
 * conversion window (10 ms at 100 sps), so reading them from task
 * context after the semaphore is race-free.
 *
 * Raw register access exceptions (documented, selftest-only):
 *  - T03 needs TIMEREN=0 for afe4404_reg_read()'s contract, but
 *    afe4404_init() leaves the timer running and the driver has no
 *    stop-only entry point. A private I2C handle writes CONTROL1=0
 *    (mirroring the driver's own stop sequence); afe4404_apply_rate()
 *    afterwards rebuilds CONTROL1 through the driver, so its cached
 *    state never diverges.
 *  - T06 (accel self-test) needs the CTRL_REG4 ST bits, which
 *    lis2dh12.h does not expose. A private handle runs the datasheet
 *    procedure; lis2dh12_init() afterwards restores driver-owned state
 *    (CTRL_REG0 re-applied, power-down).
 */
#include "selftest.h"

#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "narbis/proto.h"
#include "narbis/nc_types.h"
#include "narbis/nc_knobs.h"
#include "narbis/nc_proto_encode.h"

#include "board.h"
#include "i2c_bus.h"
#include "afe4404.h"
#include "afe4404_regs.h"
#include "lis2dh12.h"
#include "battery.h"
#include "charger.h"
#include "acq.h"
#include "sensor_types.h"

static const char *TAG = "selftest";

/* ------------------------------------------------------------------ */
/* Result blob [u8 ver][u64 t_run_us][u8 n=8][8 x 10B records]         */
/* ------------------------------------------------------------------ */
#define BLOB_SIZE (1 + 8 + 1 + NC_TEST_COUNT_ * NC_ST_REC_SIZE)

static uint8_t  s_blob[BLOB_SIZE];
static uint16_t s_blob_len;

static const uint8_t *s_ext_blob;
static uint16_t       s_ext_len;

const uint8_t *selftest_blob(uint16_t *len)
{
    if (s_ext_blob != NULL) {
        *len = s_ext_len;
        return s_ext_blob;
    }
    *len = s_blob_len;
    return s_blob_len ? s_blob : NULL;
}

void selftest_set_external_blob(const uint8_t *blob, uint16_t len)
{
    s_ext_blob = blob;
    s_ext_len = blob ? len : 0;
}

/* ------------------------------------------------------------------ */
/* Temporary ADC_RDY edge ISR (capture + pulse counting)               */
/* ------------------------------------------------------------------ */
static SemaphoreHandle_t s_rdy_sem;
static volatile uint32_t s_pulses;

static void IRAM_ATTR adc_rdy_isr(void *arg)
{
    (void)arg;
    s_pulses++;
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(s_rdy_sem, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static esp_err_t rdy_isr_attach(void)
{
    if (acq_ppg_running()) {
        return ESP_ERR_INVALID_STATE;   /* pin belongs to the PPG pipeline */
    }
    if (s_rdy_sem == NULL) {
        s_rdy_sem = xSemaphoreCreateBinary();
        if (s_rdy_sem == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    xQueueReset((QueueHandle_t)s_rdy_sem);   /* drop stale gives */
    s_pulses = 0;

    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_ADC_RDY,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,   /* AFE drives the line */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;                          /* INVALID_STATE = already up */
    }
    return gpio_isr_handler_add(PIN_ADC_RDY, adc_rdy_isr, NULL);
}

static void rdy_isr_detach(void)
{
    gpio_isr_handler_remove(PIN_ADC_RDY);
    gpio_intr_disable(PIN_ADC_RDY);
}

/* Per-frame stats accumulator for one capture run. */
typedef struct {
    int64_t sum_ir, sum_red, sum_amb;
    int64_t sq_ir, sq_red;
    uint32_t n;
} cap_stats_t;

/* Capture up to n_frames (per-frame timeout 100 ms, generous at any
 * rate >= 50 sps). Returns frames actually captured. ISR must be
 * attached; AFE timer must be running. */
static uint32_t capture_frames(uint32_t n_frames, cap_stats_t *st)
{
    memset(st, 0, sizeof *st);
    while (st->n < n_frames) {
        if (xSemaphoreTake(s_rdy_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
            break;                            /* frames stopped arriving */
        }
        nc_ppg_sample_t s;
        if (afe4404_read_frame(&s, esp_timer_get_time(), true) != ESP_OK) {
            break;
        }
        st->sum_ir  += s.ir;
        st->sum_red += s.red;
        st->sum_amb += s.amb;
        st->sq_ir   += (int64_t)s.ir * s.ir;
        st->sq_red  += (int64_t)s.red * s.red;
        st->n++;
    }
    return st->n;
}

esp_err_t selftest_capture_mean(uint16_t n_frames, int32_t *ir_mean,
                                int32_t *red_mean, int32_t *amb_mean)
{
    if (n_frames == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = rdy_isr_attach();
    if (err != ESP_OK) {
        return err;
    }
    cap_stats_t st;
    uint32_t got = capture_frames(n_frames, &st);
    rdy_isr_detach();
    if (got < n_frames) {
        return ESP_ERR_TIMEOUT;
    }
    if (ir_mean)  *ir_mean  = (int32_t)(st.sum_ir  / (int32_t)got);
    if (red_mean) *red_mean = (int32_t)(st.sum_red / (int32_t)got);
    if (amb_mean) *amb_mean = (int32_t)(st.sum_amb / (int32_t)got);
    return ESP_OK;
}

esp_err_t selftest_count_pulses(uint32_t window_ms, uint32_t *pulses,
                                uint32_t *elapsed_ms)
{
    esp_err_t err = rdy_isr_attach();
    if (err != ESP_OK) {
        return err;
    }
    int64_t t0 = esp_timer_get_time();
    s_pulses = 0;
    vTaskDelay(pdMS_TO_TICKS(window_ms));
    uint32_t n = s_pulses;                    /* 32-bit read is atomic */
    int64_t t1 = esp_timer_get_time();
    rdy_isr_detach();
    if (pulses)     *pulses = n;
    if (elapsed_ms) *elapsed_ms = (uint32_t)((t1 - t0) / 1000);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Small helpers                                                       */
/* ------------------------------------------------------------------ */
static uint32_t isqrt64(uint64_t v)
{
    uint64_t r = 0, b = 1ULL << 62;
    while (b > v) {
        b >>= 2;
    }
    while (b) {
        if (v >= r + b) {
            v -= r + b;
            r = (r >> 1) + b;
        } else {
            r >>= 1;
        }
        b >>= 2;
    }
    return (uint32_t)r;
}

/* RMS about the mean: sqrt(E[x^2] - E[x]^2). Inputs bounded by the
 * 22-bit AFE range so all intermediates fit int64 comfortably. */
static int32_t rms_about_mean(int64_t sum, int64_t sumsq, uint32_t n)
{
    if (n == 0) {
        return 0;
    }
    int64_t var = sumsq / (int64_t)n -
                  (sum / (int64_t)n) * (sum / (int64_t)n);
    if (var < 0) {
        var = 0;
    }
    return (int32_t)isqrt64((uint64_t)var);
}

/* ------------------------------------------------------------------ */
/* Private raw-register handles (see file header for why they exist)   */
/* ------------------------------------------------------------------ */
static i2c_master_dev_handle_t s_afe_raw;
static i2c_master_dev_handle_t s_acc_raw;

#define RAW_TIMEOUT_MS 50

static esp_err_t afe_raw_wr24(uint8_t reg, uint32_t val)
{
    if (s_afe_raw == NULL) {
        esp_err_t err = i2c_bus_add_device(ADDR_AFE4404, I2C_FREQ_HZ,
                                           &s_afe_raw);
        if (err != ESP_OK) {
            return err;
        }
    }
    uint8_t buf[4] = { reg, (uint8_t)(val >> 16), (uint8_t)(val >> 8),
                       (uint8_t)val };
    return i2c_master_transmit(s_afe_raw, buf, sizeof buf, RAW_TIMEOUT_MS);
}

static esp_err_t acc_raw_open(void)
{
    if (s_acc_raw != NULL) {
        return ESP_OK;
    }
    uint8_t addr = ADDR_LIS2DH12;
    if (i2c_master_probe(i2c_bus_get(), addr, RAW_TIMEOUT_MS) != ESP_OK) {
        addr = ADDR_LIS2DH12_ALT;
        if (i2c_master_probe(i2c_bus_get(), addr, RAW_TIMEOUT_MS) != ESP_OK) {
            return ESP_ERR_NOT_FOUND;
        }
    }
    return i2c_bus_add_device(addr, I2C_FREQ_HZ, &s_acc_raw);
}

static esp_err_t acc_wr(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_acc_raw, buf, 2, RAW_TIMEOUT_MS);
}

static esp_err_t acc_rd(uint8_t reg, uint8_t *dst, size_t n)
{
    /* MSB of the sub-address = auto-increment (LIS2DH12 datasheet §6.1.1) */
    uint8_t sub = (n > 1) ? (reg | 0x80) : reg;
    return i2c_master_transmit_receive(s_acc_raw, &sub, 1, dst, n,
                                       RAW_TIMEOUT_MS);
}

/* LIS2DH12 register addresses used by T06 only. */
#define ACC_CTRL_REG1   0x20
#define ACC_CTRL_REG2   0x21
#define ACC_CTRL_REG3   0x22
#define ACC_CTRL_REG4   0x23
#define ACC_STATUS_REG  0x27
#define ACC_OUT_X_L     0x28
#define ACC_ZYXDA       0x08

/* Wait for ZYXDA and read one 3-axis sample in 10-bit counts
 * (normal mode: left-justified 16-bit, >>6). */
static esp_err_t acc_sample_10b(int16_t out[3])
{
    for (int tries = 0; tries < 50; tries++) {       /* <= 500 ms */
        uint8_t sr = 0;
        esp_err_t err = acc_rd(ACC_STATUS_REG, &sr, 1);
        if (err != ESP_OK) {
            return err;
        }
        if (sr & ACC_ZYXDA) {
            uint8_t raw[6];
            err = acc_rd(ACC_OUT_X_L, raw, 6);
            if (err != ESP_OK) {
                return err;
            }
            for (int a = 0; a < 3; a++) {
                int16_t v = (int16_t)((uint16_t)raw[2 * a] |
                                      ((uint16_t)raw[2 * a + 1] << 8));
                out[a] = (int16_t)(v >> 6);
            }
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_ERR_TIMEOUT;
}

/* Average n samples per axis, discarding the first `discard`. */
static esp_err_t acc_avg_10b(int discard, int n, int32_t avg[3])
{
    int32_t acc[3] = { 0, 0, 0 };
    int16_t s[3];
    for (int i = 0; i < discard; i++) {
        esp_err_t err = acc_sample_10b(s);
        if (err != ESP_OK) {
            return err;
        }
    }
    for (int i = 0; i < n; i++) {
        esp_err_t err = acc_sample_10b(s);
        if (err != ESP_OK) {
            return err;
        }
        for (int a = 0; a < 3; a++) {
            acc[a] += s[a];
        }
    }
    for (int a = 0; a < 3; a++) {
        avg[a] = acc[a] / n;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Individual tests. Each fills {status, val, thr}.                    */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t st;      /* nc_test_status_t */
    int32_t val;
    int32_t thr;
} tres_t;

/* T01 — bus scan. val bitmask: b0 AFE@0x58, b1 accel@0x18, b2 accel@0x19.
 * PASS needs the AFE and the accel at its strapped address (alt address
 * answering counts too, but T02's loud log will flag the strap fault). */
static void t01_i2c_scan(tres_t *r)
{
    i2c_master_bus_handle_t bus = i2c_bus_get();
    int32_t found = 0;
    if (bus == NULL) {
        r->st = NC_TR_FAIL;
        r->val = -1;
        r->thr = 0x3;
        return;
    }
    if (i2c_master_probe(bus, ADDR_AFE4404, RAW_TIMEOUT_MS) == ESP_OK) {
        found |= 1;
    }
    if (i2c_master_probe(bus, ADDR_LIS2DH12, RAW_TIMEOUT_MS) == ESP_OK) {
        found |= 2;
    }
    if (i2c_master_probe(bus, ADDR_LIS2DH12_ALT, RAW_TIMEOUT_MS) == ESP_OK) {
        found |= 4;
    }
    r->val = found;
    r->thr = 0x3;                       /* expected: AFE + accel@0x18 */
    r->st = ((found & 1) && (found & 6)) ? NC_TR_PASS : NC_TR_FAIL;
}

/* T02 — WHO_AM_I via the driver's probe (0x18, fallback 0x19 with a
 * loud log, fails unless WHO_AM_I==0x33). Leaves the part powered down
 * — the correct idle state. */
static void t02_accel_whoami(tres_t *r)
{
    esp_err_t err = lis2dh12_init();
    r->thr = 0x33;
    r->val = (err == ESP_OK) ? 0x33 : (int32_t)err;
    r->st = (err == ESP_OK) ? NC_TR_PASS : NC_TR_FAIL;
}

/* T03 — AFE config write/readback. Timer stopped first (raw CONTROL1=0)
 * because afe4404_reg_read() is contract-limited to TIMEREN=0; LEDCNTRL
 * carries a benign 3/2 mA pattern (no light: timer stopped, and the
 * currents are restored to 0 before the timer restarts). Exit: timer
 * re-armed at 100 sps via apply_rate for T04/T05. */
static void t03_afe_reg_rw(tres_t *r)
{
    esp_err_t err = afe_raw_wr24(AFE_REG_CONTROL1, 0);   /* TIMEREN=0 */
    uint32_t rb = 0;
    if (err == ESP_OK) {
        err = afe4404_set_led_ma(3, 2);
    }
    if (err == ESP_OK) {
        err = afe4404_reg_read(AFE_REG_LEDCNTRL, &rb);
    }
    /* Same rounding as the driver: code = (mA*63+25)/50 */
    const uint32_t expect = AFE_LEDCNTRL_VAL((2u * 63 + 25) / 50,
                                             (3u * 63 + 25) / 50);
    afe4404_set_led_ma(0, 0);
    esp_err_t rearm = afe4404_apply_rate(NC_RATE_100);

    if (err != ESP_OK || rearm != ESP_OK) {
        r->st = NC_TR_FAIL;
        r->val = (err != ESP_OK) ? (int32_t)err : (int32_t)rearm;
        r->thr = (int32_t)expect;
        return;
    }
    r->val = (int32_t)rb;
    r->thr = (int32_t)expect;
    r->st = (rb == expect) ? NC_TR_PASS : NC_TR_FAIL;
}

/* T04 — dark test: LEDs 0 mA, 2 s of frames @100 sps. PASS needs
 * (a) frames actually arriving (ADC_RDY net alive), (b) LED-channel RMS
 * about the mean below st_dark_noise_max, (c) ambient mean below
 * st_dark_amb_max. The failing criterion owns the val/thr slot
 * (frames vs 150, then amb vs its knob, then noise vs its knob);
 * a PASS reports the noise figure. */
static void t04_dark(tres_t *r)
{
    afe4404_set_led_ma(0, 0);
    if (rdy_isr_attach() != ESP_OK) {
        r->st = NC_TR_FAIL;
        r->val = -1;
        r->thr = 0;
        return;
    }
    cap_stats_t st;
    uint32_t got = capture_frames(200, &st);          /* 2 s @ 100 sps */
    rdy_isr_detach();

    if (got < 150) {
        r->st = NC_TR_FAIL;
        r->val = (int32_t)got;
        r->thr = 150;
        return;
    }
    int32_t amb_mean = (int32_t)(st.sum_amb / (int32_t)got);
    int32_t rms_ir  = rms_about_mean(st.sum_ir, st.sq_ir, got);
    int32_t rms_red = rms_about_mean(st.sum_red, st.sq_red, got);
    int32_t noise = (rms_ir > rms_red) ? rms_ir : rms_red;

    int32_t amb_max   = nc_knob_get(KNOB_ST_DARK_AMB_MAX);
    int32_t noise_max = nc_knob_get(KNOB_ST_DARK_NOISE_MAX);

    ESP_LOGI(TAG, "T04 dark: n=%" PRIu32 " amb=%" PRId32
             " rms ir/red=%" PRId32 "/%" PRId32, got, amb_mean,
             rms_ir, rms_red);

    if (amb_mean > amb_max) {
        r->st = NC_TR_FAIL;
        r->val = amb_mean;
        r->thr = amb_max;
    } else {
        r->val = noise;
        r->thr = noise_max;
        r->st = (noise <= noise_max) ? NC_TR_PASS : NC_TR_FAIL;
    }
}

/* T05 — crosstalk/light-leak: LEDs at their clamp maxima, clip open on
 * the bench (operator context), 1 s of frames; val = worst LED-minus-
 * ambient DC. Uses AMB1 for both channels (read_frame does not fetch
 * AMB2 — a documented approximation; the phases are 100 us apart).
 * Exit: LEDs back to 0. */
static void t05_xtalk(tres_t *r)
{
    r->thr = nc_knob_get(KNOB_ST_XTALK_MAX);
    if (afe4404_set_led_ma(LED_IR_MAX_MA, LED_RED_MAX_MA) != ESP_OK ||
        rdy_isr_attach() != ESP_OK) {
        afe4404_set_led_ma(0, 0);
        r->st = NC_TR_FAIL;
        r->val = -1;
        return;
    }
    cap_stats_t st;
    capture_frames(5, &st);                    /* settle discard */
    uint32_t got = capture_frames(100, &st);   /* 1 s @ 100 sps  */
    rdy_isr_detach();
    afe4404_set_led_ma(0, 0);

    if (got < 75) {
        r->st = NC_TR_FAIL;
        r->val = (int32_t)got;
        r->thr = 75;
        return;
    }
    int32_t amb = (int32_t)(st.sum_amb / (int32_t)got);
    int32_t xt_ir  = (int32_t)(st.sum_ir / (int32_t)got) - amb;
    int32_t xt_red = (int32_t)(st.sum_red / (int32_t)got) - amb;
    r->val = (xt_ir > xt_red) ? xt_ir : xt_red;
    r->st = (r->val < r->thr) ? NC_TR_PASS : NC_TR_FAIL;
    ESP_LOGI(TAG, "T05 xtalk: ir=%" PRId32 " red=%" PRId32, xt_ir, xt_red);
}

/* T06 — LIS2DH12 electrical self-test, datasheet procedure at the
 * datasheet's reference configuration (FS=±2 g, normal mode, 10-bit,
 * 50 Hz) so the 17..360 LSB acceptance band applies with NO unit
 * conversion. ST0 actuation delta per axis must land inside the band.
 * val = the offending delta on FAIL (weakest axis if too low, largest
 * if too high); on PASS val = weakest axis, thr = lower bound.
 * Exit: lis2dh12_init() restores driver-owned power-down state. */
static void t06_accel_st(tres_t *r)
{
    const int32_t ST_MIN = 17, ST_MAX = 360;   /* LSB @ ±2g 10-bit */
    int32_t base[3], st[3];
    esp_err_t err = acc_raw_open();

    if (err == ESP_OK) err = acc_wr(ACC_CTRL_REG2, 0x00);
    if (err == ESP_OK) err = acc_wr(ACC_CTRL_REG3, 0x00);
    if (err == ESP_OK) err = acc_wr(ACC_CTRL_REG4, 0x80); /* BDU, ±2g   */
    if (err == ESP_OK) err = acc_wr(ACC_CTRL_REG1, 0x47); /* 50 Hz XYZ  */
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(90));                    /* turn-on    */
        err = acc_avg_10b(2, 5, base);
    }
    if (err == ESP_OK) err = acc_wr(ACC_CTRL_REG4, 0x82); /* ST0 on     */
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(90));                    /* ST settle  */
        err = acc_avg_10b(2, 5, st);
    }
    /* Cleanup regardless of outcome; driver re-init restores its model.
     * (Skip raw writes if the raw handle never opened.) */
    if (s_acc_raw != NULL) {
        acc_wr(ACC_CTRL_REG4, 0x80);
        acc_wr(ACC_CTRL_REG1, 0x00);
    }
    lis2dh12_init();

    if (err != ESP_OK) {
        r->st = NC_TR_FAIL;
        r->val = (int32_t)err;
        r->thr = ST_MIN;
        return;
    }
    int32_t lo = INT32_MAX, hi = 0;
    for (int a = 0; a < 3; a++) {
        int32_t d = st[a] - base[a];
        if (d < 0) {
            d = -d;
        }
        if (d < lo) lo = d;
        if (d > hi) hi = d;
    }
    ESP_LOGI(TAG, "T06 accel ST delta min/max = %" PRId32 "/%" PRId32,
             lo, hi);
    if (hi > ST_MAX) {
        r->st = NC_TR_FAIL;
        r->val = hi;
        r->thr = ST_MAX;
    } else {
        r->val = lo;
        r->thr = ST_MIN;
        r->st = (lo >= ST_MIN) ? NC_TR_PASS : NC_TR_FAIL;
    }
}

/* T07 — battery voltage window [st_batt_min_mv, st_batt_max_mv].
 * thr = the violated bound; upper bound shown on PASS. */
static void t07_batt(tres_t *r)
{
    uint16_t mv = 0;
    int32_t lo = nc_knob_get(KNOB_ST_BATT_MIN_MV);
    int32_t hi = nc_knob_get(KNOB_ST_BATT_MAX_MV);
    if (battery_read_mv(&mv, NULL) != ESP_OK) {
        r->st = NC_TR_FAIL;
        r->val = -1;
        r->thr = lo;
        return;
    }
    r->val = mv;
    if (mv < lo) {
        r->st = NC_TR_FAIL;
        r->thr = lo;
    } else {
        r->thr = hi;
        r->st = (mv <= hi) ? NC_TR_PASS : NC_TR_FAIL;
    }
}

/* T08 — charger pin consistency. 10 samples over 100 ms must be stable,
 * and VUSB-low + STAT-high is impossible (an unpowered MCP73831 leaves
 * STAT Hi-Z, and the 150 k lower divider leg then pulls GPIO19 low).
 * val = violation flags (b0 impossible combo, b1 unstable); PASS = 0. */
static void t08_charger(tres_t *r)
{
    int vusb0 = gpio_get_level(PIN_VUSB_SENSE);
    int stat0 = gpio_get_level(PIN_CHG_STAT);
    bool stable = true;
    for (int i = 0; i < 9; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (gpio_get_level(PIN_VUSB_SENSE) != vusb0 ||
            gpio_get_level(PIN_CHG_STAT) != stat0) {
            stable = false;
        }
    }
    int32_t viol = 0;
    if (vusb0 == 0 && stat0 == 1) {
        viol |= 1;
    }
    if (!stable) {
        viol |= 2;
    }
    ESP_LOGI(TAG, "T08 charger: vusb=%d stat=%d viol=0x%" PRIx32,
             vusb0, stat0, viol);
    r->val = viol;
    r->thr = 0;
    r->st = (viol == 0) ? NC_TR_PASS : NC_TR_FAIL;
}

/* ------------------------------------------------------------------ */
/* Runner                                                              */
/* ------------------------------------------------------------------ */
esp_err_t selftest_execute(uint32_t mask)
{
    if (acq_ppg_running() || acq_accel_running()) {
        return ESP_ERR_INVALID_STATE;   /* sys must stop acquisition first */
    }
    if (mask == 0) {
        mask = (1u << NC_TEST_COUNT_) - 1;
    }
    ESP_LOGI(TAG, "run mask 0x%02" PRIx32, mask);

    s_ext_blob = NULL;                  /* T01..T08 blob becomes current */
    s_blob_len = 0;                     /* blob invisible while half-built:
                                           console/BLE readers poll from
                                           other tasks (single-core, so a
                                           len-last publish is race-free) */

    /* AFE bring-up once if any AFE test is selected (T03/T04/T05).
     * Exit state: AFE initialized @100 sps, timer running, LEDs 0 —
     * sys_task restores/powers down per its policy afterwards. */
    esp_err_t afe_err = ESP_ERR_INVALID_STATE;
    bool need_afe = (mask & ((1u << (NC_TEST_AFE_REG_RW - 1)) |
                             (1u << (NC_TEST_AFE_DARK - 1)) |
                             (1u << (NC_TEST_XTALK - 1)))) != 0;
    if (need_afe) {
        afe_err = afe4404_init(NC_RATE_100);
        if (afe_err != ESP_OK) {
            ESP_LOGE(TAG, "AFE bring-up failed: %s",
                     esp_err_to_name(afe_err));
        }
    }

    uint8_t *p = s_blob;
    *p++ = NC_ST_BLOB_VER;
    nc_wr_u64(p, (uint64_t)esp_timer_get_time());
    p += 8;
    *p++ = NC_TEST_COUNT_;

    uint8_t pass = 0, fail = 0;

    for (uint8_t id = 1; id <= NC_TEST_COUNT_; id++) {
        tres_t r = { .st = NC_TR_SKIP, .val = 0, .thr = 0 };

        if (mask & (1u << (id - 1))) {
            bool afe_test = (id == NC_TEST_AFE_REG_RW ||
                             id == NC_TEST_AFE_DARK || id == NC_TEST_XTALK);
            if (afe_test && afe_err != ESP_OK) {
                r.st = NC_TR_FAIL;      /* bring-up failure fails them all */
                r.val = (int32_t)afe_err;
            } else {
                switch (id) {
                case NC_TEST_I2C_SCAN:     t01_i2c_scan(&r);    break;
                case NC_TEST_ACCEL_WHOAMI: t02_accel_whoami(&r); break;
                case NC_TEST_AFE_REG_RW:   t03_afe_reg_rw(&r);  break;
                case NC_TEST_AFE_DARK:     t04_dark(&r);        break;
                case NC_TEST_XTALK:        t05_xtalk(&r);       break;
                case NC_TEST_ACCEL_ST:     t06_accel_st(&r);    break;
                case NC_TEST_BATT:         t07_batt(&r);        break;
                case NC_TEST_CHARGER:      t08_charger(&r);     break;
                default:                                        break;
                }
            }
            if (r.st == NC_TR_PASS) {
                pass++;
            } else if (r.st == NC_TR_FAIL) {
                fail++;
            }
        }
        *p++ = id;
        *p++ = r.st;
        nc_wr_i32(p, r.val);
        p += 4;
        nc_wr_i32(p, r.thr);
        p += 4;
    }
    s_blob_len = (uint16_t)(p - s_blob);
    ESP_LOGI(TAG, "done: %u pass, %u fail (blob %u B)", pass, fail,
             s_blob_len);

    /* SELFTEST_DONE event; sys_task's event drain encodes it onto
     * EVENT_STREAM (do not also emit from sys — this is the source). */
    if (event_q != NULL) {
        nc_event_t ev = {
            .t_us = (uint64_t)esp_timer_get_time(),
            .type = NC_EV_SELFTEST_DONE,
            .len = 2,
            .data = { pass, fail },
        };
        xQueueSend(event_q, &ev, 0);
    }
    return ESP_OK;
}
