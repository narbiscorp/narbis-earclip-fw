/*
 * knobs_nvs.h — NVS persistence for the knob registry.
 * Delta-from-default storage: only knobs whose current value differs from
 * the compiled default get a key, so untouched knobs track new firmware
 * defaults and NVS wear stays minimal. Writes happen only on explicit
 * SAVE (never per-set).
 */
#pragma once
#include "esp_err.h"

esp_err_t knobs_nvs_load(void);   /* call once after nc_knobs_init()     */
esp_err_t knobs_nvs_save(void);   /* diff RAM vs defaults, write/erase   */
esp_err_t knobs_nvs_reset(void);  /* erase the whole namespace           */
