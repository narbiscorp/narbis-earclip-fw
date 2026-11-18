/*
 * test_led_ind.h — the LED indicator on the clip's own RED emitter. Born
 * as the TEST-build connect indicator (hence the name); production since
 * v0.4.0 (open profile, NARBIS_GATED_PROFILE=0). Red only: the IR LED is
 * invisible and only cost power (user, 2026-11-17).
 *
 * What it shows while it owns the LEDs (nothing sensing):
 *  - on battery: one smooth red breath per ind_breath_ms (1.5 s) at
 *    IND_RED_MA — "on, waiting for a phone / an ear";
 *  - on USB, charging: a smooth fade at ind_chg_ma (5 mA) whose period
 *    shortens as the battery fills, ind_chg_slow_ms (4 s) at 0 % to
 *    ind_chg_fast_ms (1 s) at 100 %. Brightness is faded by the red LED's
 *    pulse WIDTH (afe4404_set_red_drive_q16) at a fixed current: the
 *    current DAC's ~0.8 mA steps would make a 5 mA fade visibly stepped;
 *  - on USB, charge complete: solid red at ind_chg_ma.
 * A steady light at the AGC's level means SENSING (the AGC owns the LEDs).
 *
 * TEST builds also drive the XIAO module LED (GPIO15): 1 Hz blink =
 * powered + advertising, steady = connected. Production keeps it dark.
 *
 * Ownership state machine (all transitions in sys_task context):
 *   OWNING   — indicator drives the LEDs.
 *   RELEASED — entered when PPG acquisition starts (AGC owns the LEDs)
 *              or any LED-affecting op runs (TEST_LED_DRIVE, LED
 *              sweeps 0xE2/0xE3/0xEB, AGC_MANUAL). The indicator then
 *              never touches the LEDs again until acquisition is
 *              stopped AND the app session has ended — then it re-arms.
 *
 * All LED I2C runs in sys_task context: the 30 ms esp_timer callback
 * only posts SYS_TEST_IND, so indicator writes serialize with the test
 * ops and AGC actuation and can never fight them mid-transaction.
 *
 * The gated profile compiles this to no-ops (no indicator at all).
 */
#pragma once
#include "board.h"

#if !NARBIS_GATED_PROFILE

/* Arm the indicator once at boot (main.c, after acq_init + BLE up).
 * Creates the tick timer; the first paint happens on the first tick
 * processed by sys_task. */
void test_led_ind_start(void);

/* sys_task context only: evaluate the state machine and actuate.
 * Called from the SYS_TEST_IND tick and from conn_sync (so
 * connect/disconnect repaint immediately instead of at the next tick). */
void test_led_ind_poll(void);

/* sys_task context only: an LED-affecting op is about to run — hand the
 * LEDs over (OWNING -> RELEASED). Idempotent. */
void test_led_ind_release(void);

#else /* NARBIS_GATED_PROFILE — no indicator, no code */

static inline void test_led_ind_start(void)   {}
static inline void test_led_ind_poll(void)    {}
static inline void test_led_ind_release(void) {}

#endif
