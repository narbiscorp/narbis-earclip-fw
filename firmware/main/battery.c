/*
 * battery.c — ADC oneshot + curve-fitting calibration (the only scheme
 * the ESP32-C6 supports) for the VBAT_CELL divider on ADC1_CH1.
 *
 * Pin voltage tops out at ~4.3 V / 2 ≈ 2.15 V, comfortably inside the
 * 12 dB-attenuation range, so ADC_ATTEN_DB_12 covers the whole LiPo
 * window with calibrated headroom.
 */
#include "battery.h"

#include <stdbool.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#include "board.h"
#include "narbis/nc_batt_curve.h"

static const char *TAG = "battery";

/* PIN_BATT_ADC (GPIO1) maps to ADC1 channel 1 on the C6. */
#define BATT_ADC_UNIT     ADC_UNIT_1
#define BATT_ADC_CHANNEL  ADC_CHANNEL_1

/* Nominal full-scale for the uncalibrated fallback: ~3300 mV at 12 dB
 * attenuation, 12-bit codes. Only used when eFuse cal data is missing. */
#define BATT_FALLBACK_FS_MV   3300u
#define BATT_RAW_MAX          4095u

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static bool s_cali_ok;

esp_err_t battery_init(void)
{
    const adc_oneshot_unit_init_cfg_t ucfg = {
        .unit_id = BATT_ADC_UNIT,
    };
    esp_err_t err = adc_oneshot_new_unit(&ucfg, &s_adc);
    if (err != ESP_OK) {
        return err;
    }

    const adc_oneshot_chan_cfg_t ccfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc, BATT_ADC_CHANNEL, &ccfg);
    if (err != ESP_OK) {
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
        return err;
    }

    const adc_cali_curve_fitting_config_t cal_cfg = {
        .unit_id  = BATT_ADC_UNIT,
        .chan     = BATT_ADC_CHANNEL,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_curve_fitting(&cal_cfg, &s_cali);
    s_cali_ok = (err == ESP_OK);
    if (!s_cali_ok) {
        /* Missing eFuse cal (engineering sample?) — keep running on the
         * nominal transfer function; the ±50 mV DoD will not hold. */
        ESP_LOGW(TAG, "curve-fitting cal unavailable (%d) — "
                      "uncalibrated nominal conversion in use", err);
    }
    return ESP_OK;
}

esp_err_t battery_read_mv(uint16_t *mv, uint16_t *raw_avg)
{
    if (s_adc == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 64× oversample: the ~500 k source impedance + 100 nF cap make a
     * single conversion noisy/undercharged; the average is also what we
     * calibrate, so raw->mV runs once (max sum 64 × 4095 fits u32). */
    uint32_t sum = 0;
    for (int i = 0; i < BATT_OVERSAMPLE; i++) {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(s_adc, BATT_ADC_CHANNEL, &raw);
        if (err != ESP_OK) {
            return err;
        }
        sum += (uint32_t)raw;
    }
    const uint32_t avg = sum / BATT_OVERSAMPLE;

    int pin_mv;
    if (s_cali_ok) {
        esp_err_t err = adc_cali_raw_to_voltage(s_cali, (int)avg, &pin_mv);
        if (err != ESP_OK) {
            return err;
        }
    } else {
        pin_mv = (int)((avg * BATT_FALLBACK_FS_MV) / BATT_RAW_MAX);
    }

    if (mv != NULL) {
        *mv = (uint16_t)(pin_mv * BATT_DIVIDER_NUM);
    }
    if (raw_avg != NULL) {
        *raw_avg = (uint16_t)avg;
    }
    return ESP_OK;
}

esp_err_t battery_status(uint16_t *mv, uint8_t *pct)
{
    uint16_t cell_mv = 0;
    esp_err_t err = battery_read_mv(&cell_mv, NULL);
    if (err != ESP_OK) {
        return err;
    }
    if (mv != NULL) {
        *mv = cell_mv;
    }
    if (pct != NULL) {
        *pct = nc_batt_pct(cell_mv);
    }
    return ESP_OK;
}
