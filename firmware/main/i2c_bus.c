/*
 * i2c_bus.c — shared I2C master bus (IDF v5.x driver/i2c_master.h).
 */
#include <stdbool.h>

#include "i2c_bus.h"
#include "board.h"
#include "esp_log.h"

static const char *TAG = "i2c_bus";

static i2c_master_bus_handle_t s_bus;

esp_err_t i2c_bus_init(void)
{
    if (s_bus != NULL) {
        return ESP_OK;
    }
    const i2c_master_bus_config_t cfg = {
        .i2c_port = -1,                     /* auto-select (C6: one LP-free port) */
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            /* R8/R9 4.7 k on-board; internal ~45 k pulls would only slow edges */
            .enable_internal_pullup = false,
        },
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus: %s", esp_err_to_name(err));
        s_bus = NULL;
    }
    return err;
}

i2c_master_bus_handle_t i2c_bus_get(void)
{
    return s_bus;
}

esp_err_t i2c_bus_add_device(uint8_t addr7, uint32_t scl_speed_hz,
                             i2c_master_dev_handle_t *out)
{
    if (out == NULL || scl_speed_hz == 0 || scl_speed_hz > I2C_FREQ_HZ) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = i2c_bus_init();
    if (err != ESP_OK) {
        return err;
    }
    const i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr7,
        .scl_speed_hz = scl_speed_hz,
    };
    err = i2c_master_bus_add_device(s_bus, &dev, out);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add device 0x%02X: %s", addr7, esp_err_to_name(err));
    }
    return err;
}
