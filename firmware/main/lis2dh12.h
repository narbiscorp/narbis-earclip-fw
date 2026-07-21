/*
 * lis2dh12.h — LIS2DH12 accelerometer driver (shared I2C bus, FIFO
 * streaming, motion wake).
 *
 * Board facts (board.h): ADDR_LIS2DH12 = 0x18 (SA0 strapped to GND);
 * PIN_ACC_INT1 = GPIO17, ACTIVE-HIGH with external 10 k pull-down R17 —
 * NOT an LP pin, so motion wake fires from light sleep only.
 *
 * The driver owns the sensor's registers and configures PIN_ACC_INT1 as
 * a plain input (no internal pull — the external 10 k defines the idle
 * level). GPIO ISR registration on that pin belongs to the acquisition
 * task, not to this driver.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "narbis/nc_types.h"

/* FIFO watermark programmed by lis2dh12_config(). The FIFO is 32 deep
 * and keeps filling past the watermark (stream mode), so callers should
 * size read buffers for 32 samples, not 25. */
#define LIS2DH12_FIFO_WATERMARK  25
#define LIS2DH12_FIFO_DEPTH      32

/*
 * Probe + identify the device and apply the mandatory CTRL_REG0 write
 * (SA0 pull-up disconnect — see .c). Leaves the sensor in power-down
 * (CTRL_REG1 = 0x00) until lis2dh12_config().
 *
 * Probes ADDR_LIS2DH12 first; on NACK falls back to ADDR_LIS2DH12_ALT
 * with a loud error log (the board straps SA0 low — answering at 0x19
 * means a hardware fault). Fails if WHO_AM_I != 0x33.
 */
esp_err_t lis2dh12_init(void);

/*
 * Program ODR + full scale and start acquisition:
 *   CTRL_REG1: ODR per odr_code, LPen=0, XYZ enabled (normal mode)
 *   CTRL_REG4: BDU=1, FS=fs_code (0=±2g 1=±4g 2=±8g 3=±16g), HR=1 (12-bit)
 *   FIFO: stream mode, watermark LIS2DH12_FIFO_WATERMARK, routed to INT1
 * The FIFO is flushed (bypass -> stream) so no stale pre-config samples
 * survive a reconfigure. fs_code matches NC_ACCF_FS_MASK wire values.
 */
esp_err_t lis2dh12_config(nc_acc_odr_t odr_code, uint8_t fs_code);

/*
 * Drain the FIFO into buf (up to max samples).
 *
 * t_last_us is the caller's timestamp for the NEWEST sample in the FIFO
 * (captured at the watermark interrupt / poll instant). Sample times are
 * back-computed at the configured ODR period, evenly spaced, so that the
 * newest FIFO sample lands exactly on t_last_us; if max truncates the
 * drain, the timestamps account for the samples left behind.
 *
 * *overrun is set (if non-NULL) when FIFO_SRC reports OVRN — the oldest
 * data was overwritten; the caller emits NC_ERR_FIFO_OVERRUN / sets
 * NC_ACCF_FIFO_OVERRUN. On an I2C error, *n_out is 0 and the partial
 * drain is discarded.
 */
esp_err_t lis2dh12_read_fifo(nc_accel_sample_t *buf, size_t max,
                             size_t *n_out, uint64_t t_last_us,
                             bool *overrun);

/*
 * Enable/disable the high-g motion interrupt on INT1 (light-sleep wake).
 * ths: INT1_THS, 7-bit; LSB = 16/32/62/186 mg at FS = ±2/4/8/16 g.
 * dur: INT1_DURATION, 7-bit; LSB = 1/ODR.
 * The FIFO watermark routing (I1_WTM) is preserved either way.
 */
esp_err_t lis2dh12_motion_wake(bool en, uint8_t ths, uint8_t dur);

/* CTRL_REG1 = 0x00 — full power-down (~0.5 uA). Re-arm via config(). */
esp_err_t lis2dh12_powerdown(void);
