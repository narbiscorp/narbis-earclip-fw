/*
 * i2c_bus.h — shared I2C master bus (AFE4404 + LIS2DH12).
 *
 * One bus on SDA GPIO22 / SCL GPIO23 (external 4.7 k pull-ups R8/R9),
 * 400 kHz — the AFE4404 hard maximum. The IDF i2c_master driver holds a
 * per-bus lock, so transactions from different device handles are safely
 * interleaved; drivers that need multi-transaction atomicity (e.g. the
 * AFE4404 REG_READ wrap) add their own mutex on top.
 */
#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/* Creates the bus once; subsequent calls are no-ops returning ESP_OK.
 * Call from the boot init sequence (not concurrently from tasks). */
esp_err_t i2c_bus_init(void);

/* NULL until i2c_bus_init() has succeeded. */
i2c_master_bus_handle_t i2c_bus_get(void);

/* Registers a 7-bit device on the shared bus (initializing the bus if
 * needed). scl_speed_hz must not exceed I2C_FREQ_HZ (board.h). */
esp_err_t i2c_bus_add_device(uint8_t addr7, uint32_t scl_speed_hz,
                             i2c_master_dev_handle_t *out);
