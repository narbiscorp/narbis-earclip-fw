/*
 * afe4404.c — AFE4404 driver (SBAS689D + Addendum 1 rev-proof init).
 *
 * All I2C goes through the shared bus (i2c_bus.c) under a driver mutex.
 * Timing tables are generated: see tools/goldens/gen_afe_timing.py and the
 * committed firmware/main/afe4404_timing.inc it emits.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "board.h"
#include "i2c_bus.h"
#include "afe4404.h"
#include "afe4404_regs.h"
#include "afe4404_timing.inc"   /* AFE_RATE_TABLE / afe_regs_* (typedefs above) */

static const char *TAG = "afe4404";

#define AFE_I2C_TIMEOUT_MS 100

static i2c_master_dev_handle_t s_dev;
static SemaphoreHandle_t s_mtx;
/* Spinlock for the RESETZ pulse: a context switch mid-pulse could stretch
 * the 25-50 us low past 200 us, which the AFE takes as hardware power-down
 * entry instead of a reset. */
static portMUX_TYPE s_pulse_mux = portMUX_INITIALIZER_UNLOCKED;

/* Cached actuals (post-clamp) — single-byte reads are atomic, but writers
 * update them under s_mtx together with the hardware register. */
static uint8_t s_ir_ma, s_red_ma;
static uint8_t s_rf = AFE_RF_100K, s_cf = AFE_CF_5PF;
static int8_t  s_offdac[4];          /* [0] LED1, [1] LED2, [2] AMB1, [3] AMB2 */

static void lock(void)   { xSemaphoreTake(s_mtx, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(s_mtx); }

/* vTaskDelay(pdMS_TO_TICKS(ms)) can round to 0 ticks (2 ms at the default
 * 100 Hz tick); +1 tick guarantees the delay is at least ms. */
static void delay_ms_min(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms) + 1);
}

/* Register write: [reg][b23:16][b15:8][b7:0]. */
static esp_err_t wr(uint8_t reg, uint32_t val)
{
    uint8_t buf[4] = {
        reg, (uint8_t)(val >> 16), (uint8_t)(val >> 8), (uint8_t)val
    };
    return i2c_master_transmit(s_dev, buf, sizeof buf, AFE_I2C_TIMEOUT_MS);
}

/* Raw 24-bit read. Sufficient alone for data regs 0x2A-0x2F; config regs
 * additionally need the CONTROL0 REG_READ wrap (afe4404_reg_read). */
static esp_err_t rd24(uint8_t reg, uint32_t *val)
{
    uint8_t rx[3];
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, rx, sizeof rx,
                                                AFE_I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        *val = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
    }
    return err;
}

/* ADC results are 24-bit two's complement (sign in bit 23). */
static int32_t sext24(uint32_t raw)
{
    raw &= 0x00FFFFFFu;
    if (raw & 0x00800000u) {
        raw |= 0xFF000000u;
    }
    return (int32_t)raw;
}

esp_err_t afe4404_init(nc_rate_t rate)
{
    if (rate >= NC_RATE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_mtx == NULL) {
        s_mtx = xSemaphoreCreateMutex();
        if (s_mtx == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    /* A previous deep sleep may have latched RESETZ low (power.c uses
     * gpio_hold_en to keep the AFE in hardware PWDN). Release the latch
     * and preload HIGH before switching direction so the pin never
     * glitches low on reconfig. */
    gpio_hold_dis(PIN_AFE_RESET);
    gpio_set_level(PIN_AFE_RESET, 1);
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_AFE_RESET,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,      /* external R1 100k defines idle */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }

    /* Rev-proof power-up (Table 79 + Addendum 1): on V2.2 the pull-down
     * strap means cold boot sits in hardware PWDN, and PWDN exit needs
     * RESETZ high for tACTIVE (10 ms) before the reset pulse. Harmless
     * extra wait on V2.1 (pull-up: already high through supply ramp). */
    delay_ms_min(AFE_T_SUPPLY_TO_RESET_MS);

    /* Reset pulse must land in the 25-50 us window: <25 us may not reset,
     * >200 us re-enters hardware PWDN. Interrupts off for the duration. */
    portENTER_CRITICAL(&s_pulse_mux);
    gpio_set_level(PIN_AFE_RESET, 0);
    esp_rom_delay_us(AFE_T_RESET_PULSE_US);
    gpio_set_level(PIN_AFE_RESET, 1);
    portEXIT_CRITICAL(&s_pulse_mux);

    /* >1 ms from reset release to first I2C access. */
    delay_ms_min(AFE_T_RESET_TO_I2C_MS);

    err = i2c_bus_init();
    if (err != ESP_OK) {
        return err;
    }
    if (s_dev == NULL) {
        err = i2c_bus_add_device(ADDR_AFE4404, I2C_FREQ_HZ, &s_dev);
        if (err != ESP_OK) {
            s_dev = NULL;
            return err;
        }
    }

    /* Static config. Hardware reset cleared every register, so only the
     * non-defaults need writing; LED currents and offset DACs stay 0 until
     * AGC/manual control raises them (timer will run with dark LEDs). */
    lock();
    err = wr(AFE_REG_TIAGAIN_SEP, 0);                 /* ENSEPGAIN=0: shared gain */
    if (err == ESP_OK) {
        err = wr(AFE_REG_TIA_GAIN, AFE_TIA_GAIN_VAL(AFE_CF_5PF, AFE_RF_100K));
    }
    if (err == ESP_OK) {
        err = wr(AFE_REG_LEDCNTRL, 0);
    }
    if (err == ESP_OK) {
        err = wr(AFE_REG_OFFDAC, 0);
    }
    if (err == ESP_OK) {
        s_rf = AFE_RF_100K;
        s_cf = AFE_CF_5PF;
        s_ir_ma = 0;
        s_red_ma = 0;
        memset(s_offdac, 0, sizeof s_offdac);
    }
    unlock();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "static config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = afe4404_apply_rate(rate);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "up: %u sps, RF=100k CF=5pF, LEDs off",
                 nc_rate_sps(rate));
    }
    return err;
}

esp_err_t afe4404_apply_rate(nc_rate_t rate)
{
    if (rate >= NC_RATE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    const afe_rate_cfg_t *cfg = &AFE_RATE_TABLE[rate];

    lock();
    /* Stop the timing engine and hold its counter in reset so no partial
     * window set ever runs. */
    esp_err_t err = wr(AFE_REG_CONTROL1, 0);
    if (err == ESP_OK) {
        err = wr(AFE_REG_CONTROL0, AFE_C0_TM_COUNT_RST);
    }
    for (uint8_t i = 0; err == ESP_OK && i < cfg->n_regs; i++) {
        err = wr(cfg->regs[i].reg, cfg->regs[i].val);
    }
    if (err == ESP_OK) {
        err = wr(AFE_REG_CONTROL0, 0);                /* release counter reset */
    }
    if (err == ESP_OK) {
        /* CONTROL1 written LAST: averaging + timer enable arm the frame. */
        err = wr(AFE_REG_CONTROL1,
                 AFE_C1_TIMEREN | (cfg->numav & AFE_C1_NUMAV_MASK));
    }
    unlock();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "apply_rate(%u sps): %s", cfg->rate_sps,
                 esp_err_to_name(err));
    }
    return err;
}

esp_err_t afe4404_read_frame(nc_ppg_sample_t *out, uint64_t t_us_isr,
                             bool want_amb)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t ir = 0, red = 0, amb = 0;

    lock();
    esp_err_t err = rd24(AFE_REG_LED1VAL, &ir);       /* 0x2C: IR   */
    if (err == ESP_OK) {
        err = rd24(AFE_REG_LED2VAL, &red);            /* 0x2A: red  */
    }
    if (err == ESP_OK && want_amb) {
        err = rd24(AFE_REG_ALED1VAL, &amb);           /* 0x2D: amb  */
    }
    unlock();
    if (err != ESP_OK) {
        return err;
    }

    out->t_us = t_us_isr;
    out->ir   = sext24(ir);
    out->red  = sext24(red);
    out->amb  = want_amb ? sext24(amb) : 0;
    return ESP_OK;
}

esp_err_t afe4404_set_led_ma(uint8_t ir_ma, uint8_t red_ma)
{
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Safety clamps BEFORE the code conversion (SFH 7016 DC abs-max;
     * enforced here regardless of what AGC or a knob asked for). */
    if (ir_ma > LED_IR_MAX_MA) {
        ir_ma = LED_IR_MAX_MA;
    }
    if (red_ma > LED_RED_MAX_MA) {
        red_ma = LED_RED_MAX_MA;
    }
    /* code = round(mA * 63 / 50); full range 0-50 mA (ILED_2X never set) */
    const uint32_t ir_code  = ((uint32_t)ir_ma * 63u + 25u) / 50u;
    const uint32_t red_code = ((uint32_t)red_ma * 63u + 25u) / 50u;

    lock();
    esp_err_t err = wr(AFE_REG_LEDCNTRL, AFE_LEDCNTRL_VAL(red_code, ir_code));
    if (err == ESP_OK) {
        s_ir_ma = ir_ma;
        s_red_ma = red_ma;
    }
    unlock();
    return err;
}

esp_err_t afe4404_set_tia(uint8_t rf_code, uint8_t cf_code)
{
    if (rf_code > 7 || cf_code > 7) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    lock();
    esp_err_t err = wr(AFE_REG_TIA_GAIN, AFE_TIA_GAIN_VAL(cf_code, rf_code));
    if (err == ESP_OK) {
        s_rf = rf_code;
        s_cf = cf_code;
    }
    unlock();
    return err;
}

esp_err_t afe4404_set_offdac(uint8_t phase, int8_t code)
{
    /* Field placement per SBAS689D 0x3A, indexed by the API phase order
     * 0 LED1(IR), 1 LED2(red), 2 AMB1, 3 AMB2. */
    static const struct { uint8_t pol_bit, mag_shift; } f[4] = {
        { AFE_OFFDAC_LED1_POL, AFE_OFFDAC_LED1_SHIFT },
        { AFE_OFFDAC_LED2_POL, AFE_OFFDAC_LED2_SHIFT },
        { AFE_OFFDAC_AMB1_POL, AFE_OFFDAC_AMB1_SHIFT },
        { AFE_OFFDAC_AMB2_POL, AFE_OFFDAC_AMB2_SHIFT },
    };
    if (phase >= 4 || code < -AFE_OFFDAC_MAG_MAX || code > AFE_OFFDAC_MAG_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    lock();
    const int8_t prev = s_offdac[phase];
    s_offdac[phase] = code;
    /* 0x3A carries all four phases — rebuild the whole register from the
     * cache so one phase's update can't clobber the others. */
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) {
        const int8_t c = s_offdac[i];
        const uint32_t mag = (uint32_t)(c < 0 ? -c : c);
        v |= mag << f[i].mag_shift;
        if (c < 0) {
            v |= 1u << f[i].pol_bit;   /* pol=1 -> negative offset current */
        }
    }
    esp_err_t err = wr(AFE_REG_OFFDAC, v);
    if (err != ESP_OK) {
        s_offdac[phase] = prev;        /* keep cache honest on I2C failure */
    }
    unlock();
    return err;
}

esp_err_t afe4404_powerdown_hw(void)
{
    /* RESETZ held low >200 us = hardware power-down (~8 uA total). All
     * registers are LOST; the only way back is afe4404_init(). power.c is
     * responsible for gpio_hold_en before deep sleep so R1 cannot pull the
     * pin (and the AFE) back up. */
    return gpio_set_level(PIN_AFE_RESET, 0);
}

esp_err_t afe4404_reg_read(uint8_t reg, uint32_t *val)
{
    if (val == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    /* CONFIG-register readback wrap (CONTROL0 REG_READ). Caller contract:
     * TIMEREN must be 0 — see afe4404.h. */
    lock();
    esp_err_t err = wr(AFE_REG_CONTROL0, AFE_C0_REG_READ);
    if (err == ESP_OK) {
        err = rd24(reg, val);
    }
    const esp_err_t err2 = wr(AFE_REG_CONTROL0, 0);
    unlock();
    return (err != ESP_OK) ? err : err2;
}

uint8_t afe4404_cur_ir_ma(void)  { return s_ir_ma; }
uint8_t afe4404_cur_red_ma(void) { return s_red_ma; }
uint8_t afe4404_cur_rf(void)     { return s_rf; }
uint8_t afe4404_cur_cf(void)     { return s_cf; }
