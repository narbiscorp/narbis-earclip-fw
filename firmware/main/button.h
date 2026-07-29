/*
 * button.h — GPIO2 side-button front end (handoff §5.3).
 *
 * Ownership split: this module owns the pad, the ISR and the 30 ms
 * debounce; it emits CONFIRMED edges as SYS_BTN_EDGE messages. The
 * gesture FSM (narbis_core nc_button) runs in sys_task, which also owns
 * the FSM's single one-shot timeout timer.
 *
 * Board facts (board.h): SW3 to GND on GPIO2, C22 + TVS D3 on the net,
 * NO external pull-up — the internal pull-up is mandatory. Pressed =
 * pin low.
 */
#pragma once
#include <stdbool.h>
#include "esp_err.h"

/* Configure GPIO2 (input + internal pull-up + any-edge ISR), create the
 * debounce timer, and arm the light-sleep level wake. Call after sys_q
 * exists (confirmed edges are posted there). */
esp_err_t button_init(void);

/* Current DEBOUNCED level (true = held), i.e. the last confirmed edge
 * posted to sys_task — not the raw pad. Any task. Starts false even if
 * the wake press is still in progress at init (that press belongs to
 * the wake gesture; its release is swallowed by design). Feeds
 * nc_status_t.btn_pressed and the TEST hold-to-off guard. */
bool button_is_pressed(void);
