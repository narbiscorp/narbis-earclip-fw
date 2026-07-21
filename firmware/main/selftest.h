/*
 * selftest.h — hardware self-test T01..T08 (proto.h nc_selftest_id_t).
 *
 * Execution contract: selftest_execute() BLOCKS (dark test alone is
 * 2 s) and must be called from sys_task with acquisition stopped —
 * it takes the ADC_RDY GPIO ISR for itself and brings the AFE up/down
 * as it pleases. Restoring the prior acquisition state afterwards is
 * sys_task's job (typically acq_ppg_start()/powerdown per policy).
 */
#pragma once
#include <stdint.h>
#include "esp_err.h"

/*
 * Run tests whose mask bit(id-1) is set (mask 0 = all). Always emits a
 * full 8-record blob ([u8 ver][u64 t_run_us][u8 n=8][8 x {u8 id,
 * u8 status, i32 val, i32 thr}], NC_ST_BLOB_VER) — masked-out tests
 * appear as SKIP records so the host table stays positional.
 * Returns ESP_ERR_INVALID_STATE if acquisition is still running
 * (blob untouched); ESP_OK otherwise — per-test FAILs live in the blob.
 * Also queues NC_EV_SELFTEST_DONE {pass_count, fail_count} on event_q.
 */
esp_err_t selftest_execute(uint32_t mask);

/*
 * Blob served to CONTROL 0x51 / 0xEA chunk readers (sys wires this into
 * nc_ctrl_ctx_t.selftest_blob). Returns the most recently generated
 * report: the T01..T08 blob after selftest_execute(), or an external
 * blob (test_ops sweep report) after selftest_set_external_blob() —
 * last writer wins. NULL + *len=0 if nothing has run since boot.
 */
const uint8_t *selftest_blob(uint16_t *len);

/* ------------------------------------------------------------------ */
/* Firmware-internal surface (test_ops.c) — not a host-facing API.     */
/* ------------------------------------------------------------------ */

/* Repoint selftest_blob() at an externally owned report (test mode
 * LED/RX sweep results). blob must stay valid until the next repoint /
 * selftest_execute(). NULL restores the internal T01..T08 blob. */
void selftest_set_external_blob(const uint8_t *blob, uint16_t len);

/*
 * Capture n_frames AFE frames via a temporary ADC_RDY edge ISR and
 * return per-channel means. Requires acquisition stopped and the AFE
 * initialized with its timer running. ESP_ERR_INVALID_STATE if the
 * PPG pipeline owns the pin; ESP_ERR_TIMEOUT if frames stop arriving
 * (ADC_RDY net dead / timer off). Out pointers may be NULL.
 */
esp_err_t selftest_capture_mean(uint16_t n_frames, int32_t *ir_mean,
                                int32_t *red_mean, int32_t *amb_mean);

/*
 * Count ADC_RDY rising edges over window_ms (temporary ISR counter;
 * same precondition as above). *elapsed_ms gets the measured window
 * (esp_timer, so rate = pulses * 1000 / elapsed_ms). */
esp_err_t selftest_count_pulses(uint32_t window_ms, uint32_t *pulses,
                                uint32_t *elapsed_ms);
