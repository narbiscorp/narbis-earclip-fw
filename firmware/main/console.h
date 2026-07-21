/*
 * console.h — esp_console REPL on USB-Serial-JTAG.
 *
 * The console is the SECOND CONTROL transport: every state-changing
 * command builds a proto.h CONTROL request ([op][tid][payload]) and
 * posts it to sys_q as SYS_CTRL_REQ, so all mutations serialize through
 * sys_task exactly like BLE writes. Read-only commands (stats, knob
 * get/list, batt, state, tap) touch only lock-free surfaces.
 */
#pragma once
#include "esp_err.h"

/* Start the REPL (esp_console_new_repl_usb_serial_jtag + start_repl).
 * Call once after acq/diag/knobs are up; commands assume sys_task runs. */
esp_err_t console_init(void);
