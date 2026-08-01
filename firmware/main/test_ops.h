/*
 * test_ops.h — TEST opcode block 0xE0..0xE9 (PCB design verification).
 *
 * sys_task calls test_ops_dispatch() INLINE from its CONTROL handling
 * (as nc_ctrl_ctx_t.test_op), so every op runs in sys_task context —
 * the same serialization point as all other config I2C. Sweep ops
 * (0xE2/0xE3) and the rate counter (0xE4) BLOCK sys_task for up to
 * ~15 s / ~30 s by design: bench-only, documented, acceptable.
 *
 * When NARBIS_TEST_MODE=0 (production) the implementation compiles out
 * and this function is a stub returning NC_ST_UNAUTHORIZED — matching
 * the dispatcher, which refuses 0xE0..0xEF before ever calling it.
 *
 * Sweep report blob (served through CONTROL 0xEA / 0x51 chunking via
 * selftest_set_external_blob; little-endian):
 *   [u8 ver=2][u8 kind: 1=LED I-V sweep, 2=RX sweep]
 *   [u8 param: kind1 -> led (0 IR, 1 RED); kind2 -> what (0 TIA RF,
 *              1 offset DAC)]
 *   [u8 n_points][n_points x {u8 setting, i32 dc_ir, i32 dc_red,
 *                             i32 dc_amb}]
 * "setting" is mA (kind 1), RF code 0..7, or offset-DAC code as int8
 * cast to u8 (kind 2/what 1).
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "narbis/proto.h"

/*
 * Handle one TEST opcode. pl/len = request payload (no envelope);
 * resp has room for NC_CTRL_RESP_PAYLOAD_MAX bytes, *resp_len starts
 * at 0. Returns the wire status for the response envelope.
 */
nc_ctrl_status_t test_ops_dispatch(uint8_t op, const uint8_t *pl,
                                   size_t len, uint8_t *resp,
                                   size_t *resp_len);

/*
 * sys_task context, from conn_sync on the disconnect edge: stop
 * session-scoped TEST machinery — currently the 0xEB continuous LED
 * sweep (swept LEDs driven to 0; the connect indicator re-arms on its
 * next tick). No-op stub in production builds.
 */
void test_ops_on_disconnect(void);

/*
 * sys_task context (TEST builds only — production never links a
 * caller): bring the bench AFE up at 100 sps if the PPG pipeline does
 * not own it, self-healing after anything powered the AFE down. Shared
 * with test_led_ind.c so bench-state bookkeeping has one owner.
 */
esp_err_t test_ops_ensure_bench_afe(void);

/*
 * sys_task context, from the SYS_BTN_EDGE handler: when the 0xE5
 * button-echo is armed, emit the edge marker (id 1000+level) on
 * EVENT_STREAM. Rides the debounced ISR->FSM path — the original
 * 50 Hz GPIO-poll echo went silent after the first press on real
 * hardware (power-managed idle) while the FSM path kept working.
 * No-op stub in production builds.
 */
void test_ops_btn_edge(bool pressed, uint32_t t_ms);
