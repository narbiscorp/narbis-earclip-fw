/*
 * nc_acq_policy.h — subscription-gated acquisition demand (handoff §5.4).
 *
 * Pure combinational policy: given who is subscribed, what CONTROL
 * overrides are latched, and the power/wear context, decide which
 * acquisition blocks must run. Stateless — the caller (sys task) owns
 * the latched masks and re-evaluates on every input change.
 *
 * Rules (DECIDED):
 *  - PPG demand = sub_ppg || sub_ibi || sub_hrs || CONTROL start
 *    PPG/IBI bits. EVENT and STATUS subscribers alone start NOTHING
 *    (sub_event is carried for completeness/logging only).
 *  - ctrl_stop_mask wins over subscriptions, per source: the PPG stop
 *    bit suppresses PPG-side demand (sub_ppg / start-PPG), the IBI stop
 *    bit suppresses IBI-side demand (sub_ibi / sub_hrs / start-IBI).
 *    A stop bit keeps suppressing until the CALLER clears it — the
 *    caller clears a stop bit on re-subscribe or a new STREAM_START of
 *    that source (this function is stateless and never clears masks).
 *  - accel demand = (sub_accel || start-ACCEL bit, minus the ACCEL stop
 *    bit) || (gate_en && PPG demand): the artifact gate needs motion
 *    data whenever PPG runs, and that internal use is NOT maskable by
 *    a BLE accel stop.
 *  - wear_paused (auto-pause off-ear) suppresses ppg_on but NOT the
 *    gate/motion-wake accel coupling — accel keeps running off-ear so
 *    re-wear/motion can resume acquisition.
 *  - !usb_stream_ok (stream_on_usb=0 while charging) forces everything
 *    off, unconditionally.
 *  - dsp_on == ppg_on (the DSP chain has no other consumer).
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool sub_ppg;            /* central subscribed to PPG_STREAM   */
    bool sub_accel;          /* ...ACCEL_STREAM                    */
    bool sub_ibi;            /* ...IBI_STREAM                      */
    bool sub_event;          /* ...EVENT_STREAM (never starts acq) */
    bool sub_hrs;            /* std Heart Rate Service subscriber  */
    uint8_t ctrl_start_mask; /* NC_STREAM_MASK_* latched by caller */
    uint8_t ctrl_stop_mask;  /* NC_STREAM_MASK_* latched by caller */
    bool gate_en;            /* knob gate_en                       */
    bool wear_paused;        /* wear detector says off-ear         */
    bool usb_stream_ok;      /* !(USB present && stream_on_usb==0) */
} nc_acq_in_t;

typedef struct {
    bool ppg_on;    /* AFE4404 acquiring          */
    bool accel_on;  /* LIS2DH12 sampling          */
    bool dsp_on;    /* filter/IBI chain running   */
} nc_acq_out_t;

void nc_acq_eval(const nc_acq_in_t *in, nc_acq_out_t *out);
