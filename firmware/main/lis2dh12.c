/*
 * lis2dh12.c — LIS2DH12 driver. Register map per ST DS10542, FIFO usage
 * per AN5005. Shared 400 kHz I2C via i2c_bus.h (AFE4404 hard max — the
 * accelerometer tolerates it).
 */
#include "lis2dh12.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "board.h"
#include "i2c_bus.h"

static const char *TAG = "lis2dh12";

/* ---- register map (DS10542 §8) ----------------------------------- */
#define REG_WHO_AM_I        0x0F
#define REG_CTRL_REG0       0x1E
#define REG_CTRL_REG1       0x20
#define REG_CTRL_REG3       0x22
#define REG_CTRL_REG4       0x23
#define REG_CTRL_REG5       0x24
#define REG_OUT_X_L         0x28
#define REG_FIFO_CTRL_REG   0x2E
#define REG_FIFO_SRC_REG    0x2F
#define REG_INT1_CFG        0x30
#define REG_INT1_THS        0x32
#define REG_INT1_DURATION   0x33

#define REG_AUTO_INC        0x80  /* MSB of subaddress enables auto-increment */

#define WHO_AM_I_VAL        0x33

/* CTRL_REG0 = 0b10010000: bit7 disconnects the internal SA0 pull-up,
 * which otherwise leaks through the SA0-to-GND strap continuously (board
 * fact — this write is MANDATORY and must be the first register write).
 * Bits[6:0] must stay at the datasheet's 0010000 pattern; any other
 * value is "may cause permanent damage" territory (DS10542 §8.4). */
#define CTRL_REG0_SA0_PU_DISC 0x90

#define CTRL_REG1_XYZ_EN    0x07  /* Zen|Yen|Xen; LPen=0 (normal mode)   */
#define CTRL_REG3_I1_WTM    0x04  /* FIFO watermark -> INT1              */
#define CTRL_REG3_I1_IA1    0x40  /* IA1 (motion) -> INT1                */
#define CTRL_REG4_BDU       0x80  /* no OUT_L/OUT_H tearing on burst read*/
#define CTRL_REG4_HR        0x08  /* 12-bit high-resolution mode         */
#define CTRL_REG5_FIFO_EN   0x40
#define FIFO_MODE_BYPASS    0x00
#define FIFO_MODE_STREAM    0x80  /* FM[1:0] = 0b10 in bits[7:6]         */
#define FIFO_SRC_OVRN       0x40
#define FIFO_SRC_FSS_MASK   0x1F
#define INT1_CFG_XYZ_HIGH   0x2A  /* ZHIE|YHIE|XHIE (OR combination)     */

#define LIS_TIMEOUT_MS      20

/* Burst-read granularity: 8 samples = 48 data bytes ≈ 1.13 ms of bus
 * occupancy at 400 kHz (subaddress + 48 reads ≈ 50 byte-frames × 9
 * clocks). The AFE4404 must be read once per ADC_RDY frame — 2 ms at
 * 500 sps — so a watermark drain (25+ samples) is split into ≤8-sample
 * transactions to guarantee the bus frees up between PPG frame reads. */
#define FIFO_READ_CHUNK     8

static i2c_master_dev_handle_t s_dev;
static uint32_t s_period_us;  /* sample period at the configured ODR; 0 = unconfigured */
static bool     s_wake_en;    /* motion-wake routing active on INT1     */

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    const uint8_t tx[2] = { reg, val };
    return i2c_master_transmit(s_dev, tx, sizeof tx, LIS_TIMEOUT_MS);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len,
                                       LIS_TIMEOUT_MS);
}

esp_err_t lis2dh12_init(void)
{
    /* INT1 idle level is defined by the external 10 k pull-down R17;
     * internal pulls stay off so they can't fight the divider-of-one.
     * ISR wiring is the acquisition task's job. */
    const gpio_config_t int1_cfg = {
        .pin_bit_mask  = 1ULL << PIN_ACC_INT1,
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&int1_cfg);
    if (err != ESP_OK) {
        return err;
    }

    i2c_master_bus_handle_t bus = i2c_bus_get();
    uint8_t addr = ADDR_LIS2DH12;

    err = i2c_master_probe(bus, addr, LIS_TIMEOUT_MS);
    if (err != ESP_OK) {
        /* SA0 is strapped to GND on this board — 0x18 is the only
         * legitimate address. Answering at 0x19 means the strap (or the
         * CTRL_REG0 pull-up assumption) is wrong: keep going so bring-up
         * can proceed, but shout. */
        ESP_LOGE(TAG, "no ACK at 0x%02X (SA0 straps low on this board); "
                      "probing alt 0x%02X", addr, ADDR_LIS2DH12_ALT);
        addr = ADDR_LIS2DH12_ALT;
        err = i2c_master_probe(bus, addr, LIS_TIMEOUT_MS);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "no ACK at 0x%02X either — accel absent", addr);
            return err;
        }
        ESP_LOGE(TAG, "accel answered at ALT address 0x%02X — SA0 strap "
                      "fault, check R-strap / solder bridge", addr);
    }

    /* Re-init (selftest, post-OTA validation) must not leak the old
     * handle — and the strap-fault fallback may resolve a DIFFERENT
     * address than last time, so remove-then-add rather than reuse. */
    if (s_dev != NULL) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    err = i2c_bus_add_device(addr, I2C_FREQ_HZ, &s_dev);
    if (err != ESP_OK) {
        return err;
    }

    /* MANDATORY first register write — see CTRL_REG0_SA0_PU_DISC. */
    err = reg_write(REG_CTRL_REG0, CTRL_REG0_SA0_PU_DISC);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t who = 0;
    err = reg_read(REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        return err;
    }
    if (who != WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02X (want 0x%02X) at 0x%02X",
                 who, WHO_AM_I_VAL, addr);
        return ESP_ERR_INVALID_RESPONSE;
    }

    s_period_us = 0;
    s_wake_en = false;

    /* Power-down until config() picks ODR/FS. */
    return reg_write(REG_CTRL_REG1, 0x00);
}

esp_err_t lis2dh12_config(nc_acc_odr_t odr_code, uint8_t fs_code)
{
    /* ODR[7:4] encoding, DS10542 Table 28: 10 Hz=0x2, 25=0x3, 50=0x4,
     * 100=0x5, 200=0x6, 400=0x7 — indexed by nc_acc_odr_t. */
    static const uint8_t odr_nibble[NC_ODR_COUNT] = {
        0x2, 0x3, 0x4, 0x5, 0x6, 0x7
    };

    if (odr_code >= NC_ODR_COUNT || fs_code > 3 || s_dev == NULL) {
        return (s_dev == NULL) ? ESP_ERR_INVALID_STATE : ESP_ERR_INVALID_ARG;
    }

    /* BDU/HR/FS before turning the ODR on so the first samples are
     * already in the final format. */
    esp_err_t err = reg_write(REG_CTRL_REG4, CTRL_REG4_BDU |
                              (uint8_t)(fs_code << 4) | CTRL_REG4_HR);
    if (err != ESP_OK) {
        return err;
    }

    err = reg_write(REG_CTRL_REG5, CTRL_REG5_FIFO_EN);
    if (err != ESP_OK) {
        return err;
    }

    /* Bypass -> stream flushes any stale FIFO content (AN5005: FIFO is
     * reset by passing through bypass mode). */
    err = reg_write(REG_FIFO_CTRL_REG, FIFO_MODE_BYPASS);
    if (err != ESP_OK) {
        return err;
    }
    err = reg_write(REG_FIFO_CTRL_REG,
                    FIFO_MODE_STREAM | LIS2DH12_FIFO_WATERMARK);
    if (err != ESP_OK) {
        return err;
    }

    err = reg_write(REG_CTRL_REG3, CTRL_REG3_I1_WTM |
                    (s_wake_en ? CTRL_REG3_I1_IA1 : 0));
    if (err != ESP_OK) {
        return err;
    }

    err = reg_write(REG_CTRL_REG1,
                    (uint8_t)(odr_nibble[odr_code] << 4) | CTRL_REG1_XYZ_EN);
    if (err != ESP_OK) {
        return err;
    }

    s_period_us = 1000000u / nc_acc_odr_hz(odr_code);
    return ESP_OK;
}

esp_err_t lis2dh12_read_fifo(nc_accel_sample_t *buf, size_t max,
                             size_t *n_out, uint64_t t_last_us,
                             bool *overrun)
{
    if (buf == NULL || n_out == NULL || max == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    *n_out = 0;
    if (overrun != NULL) {
        *overrun = false;
    }
    if (s_period_us == 0) {
        return ESP_ERR_INVALID_STATE;  /* config() not run */
    }

    uint8_t src = 0;
    esp_err_t err = reg_read(REG_FIFO_SRC_REG, &src, 1);
    if (err != ESP_OK) {
        return err;
    }
    if ((src & FIFO_SRC_OVRN) && overrun != NULL) {
        /* Stream mode overwrote the oldest sample(s); data below is
         * still valid, just gapped. Caller reports NC_ERR_FIFO_OVERRUN. */
        *overrun = true;
    }

    const size_t avail = src & FIFO_SRC_FSS_MASK;
    size_t count = (avail > max) ? max : avail;
    if (count == 0) {
        return ESP_OK;
    }

    size_t done = 0;
    while (done < count) {
        size_t chunk = count - done;
        if (chunk > FIFO_READ_CHUNK) {
            chunk = FIFO_READ_CHUNK;
        }

        uint8_t raw[FIFO_READ_CHUNK * 6];
        err = reg_read(REG_OUT_X_L | REG_AUTO_INC, raw, chunk * 6);
        if (err != ESP_OK) {
            return err;  /* discard partial drain; *n_out stays 0 */
        }

        for (size_t i = 0; i < chunk; i++) {
            const uint8_t *p = &raw[i * 6];
            /* Left-justified 16-bit, little-endian register pairs; HR
             * mode is 12 significant bits -> arithmetic >>4. */
            buf[done + i].x = (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)) >> 4;
            buf[done + i].y = (int16_t)((uint16_t)p[2] | ((uint16_t)p[3] << 8)) >> 4;
            buf[done + i].z = (int16_t)((uint16_t)p[4] | ((uint16_t)p[5] << 8)) >> 4;
        }
        done += chunk;
    }

    /* Timestamps: t_last_us is the NEWEST sample still in the FIFO at
     * poll time. The FIFO drains oldest-first, so if max truncated the
     * read, the last sample we hold is (avail - count) periods older. */
    const int64_t last_t = (int64_t)t_last_us
                         - (int64_t)(avail - count) * (int64_t)s_period_us;
    for (size_t i = 0; i < count; i++) {
        const int64_t t = last_t
                        - (int64_t)(count - 1 - i) * (int64_t)s_period_us;
        buf[i].t_us = (t > 0) ? (uint64_t)t : 0;
    }

    *n_out = count;
    return ESP_OK;
}

esp_err_t lis2dh12_motion_wake(bool en, uint8_t ths, uint8_t dur)
{
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err;
    if (en) {
        err = reg_write(REG_INT1_THS, ths & 0x7F);
        if (err != ESP_OK) {
            return err;
        }
        err = reg_write(REG_INT1_DURATION, dur & 0x7F);
        if (err != ESP_OK) {
            return err;
        }
        err = reg_write(REG_INT1_CFG, INT1_CFG_XYZ_HIGH);
        if (err != ESP_OK) {
            return err;
        }
    } else {
        err = reg_write(REG_INT1_CFG, 0x00);
        if (err != ESP_OK) {
            return err;
        }
    }
    s_wake_en = en;

    /* Keep the FIFO watermark routed regardless — streaming and motion
     * wake share INT1 (active-high, external 10 k pull-down). */
    return reg_write(REG_CTRL_REG3, CTRL_REG3_I1_WTM |
                     (en ? CTRL_REG3_I1_IA1 : 0));
}

esp_err_t lis2dh12_powerdown(void)
{
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_period_us = 0;
    return reg_write(REG_CTRL_REG1, 0x00);
}
