/*
 * test_led_ind.h — TEST-build LED connect indicator (vendor bench UX).
 *
 * While the earclip is powered and NOT BLE-connected both LEDs pulse at
 * 1 Hz (500 ms at 50 % drive, 500 ms off); a connection holds them
 * steady at 50 %. 50 % drive = IR 25 mA / red 20 mA (half of the
 * board.h SFH 7016 clamps). The AFE timing engine keeps the optical
 * duty at ~1 % regardless of drive current, so "50 %" reads
 * dim-but-visible to the operator — expected, not a defect.
 *
 * Ownership state machine (all transitions in sys_task context):
 *   OWNING   — indicator drives the LEDs (pulse / steady).
 *   RELEASED — entered when PPG acquisition starts (AGC owns the LEDs)
 *              or any LED-affecting op runs (TEST_LED_DRIVE, LED
 *              sweeps 0xE2/0xE3/0xEB, AGC_MANUAL). The indicator then
 *              never touches the LEDs again until acquisition is
 *              stopped AND the central has disconnected — then it
 *              re-arms and resumes pulsing.
 *
 * All LED I2C runs in sys_task context: the 500 ms esp_timer callback
 * only posts SYS_TEST_IND, so indicator writes serialize with the test
 * ops and AGC actuation and can never fight them mid-transaction.
 *
 * Production builds compile to no-ops.
 */
#pragma once
#include "board.h"

#if NARBIS_TEST_MODE

/* Arm the indicator once at boot (main.c, after acq_init + BLE up).
 * Creates the 500 ms tick timer; the first paint happens on the first
 * tick processed by sys_task. */
void test_led_ind_start(void);

/* sys_task context only: evaluate the state machine and actuate.
 * Called from the SYS_TEST_IND tick and from conn_sync (so
 * connect/disconnect repaint immediately instead of at the next tick). */
void test_led_ind_poll(void);

/* sys_task context only: an LED-affecting op is about to run — hand the
 * LEDs over (OWNING -> RELEASED). Idempotent. */
void test_led_ind_release(void);

#else /* !NARBIS_TEST_MODE — production: no indicator, no code */

static inline void test_led_ind_start(void)   {}
static inline void test_led_ind_poll(void)    {}
static inline void test_led_ind_release(void) {}

#endif
