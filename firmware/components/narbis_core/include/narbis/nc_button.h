/*
 * nc_button.h — pure button gesture FSM (handoff §5.3, DECIDED).
 *
 * Timestamp-driven: the FSM owns no timers. Every nc_btn_step() call
 * outputs *arm_timeout_ms — the caller maintains exactly ONE one-shot:
 *   0        -> cancel any pending one-shot
 *   nonzero  -> (re)arm the one-shot to fire at t_ms + value, replacing
 *               any pending one; deliver NC_BTN_EV_TIMEOUT when it fires.
 * The FSM never needs more than one outstanding timeout.
 *
 * Gesture map:
 *   short press, no second DOWN in press_double_ms  -> MARKER (fires at
 *       window expiry: <= press_double_ms + 50 ms latency budget)
 *   short press + second DOWN inside window         -> PAIRING on 2nd UP
 *   hold >= press_long_ms, then release             -> OFF on the UP
 *   hold >= press_reboot_ms without release         -> REBOOT at timeout
 *
 * Conflict resolution (DECIDED here, tested in t_button.c): a second
 * DOWN inside the double window that ends up held >= press_long_ms
 * CANCELS pairing and follows the hold path (OFF on release / REBOOT).
 *
 * Debounce: the caller feeds debounced edges, but the FSM additionally
 * treats any DOWN..UP pulse shorter than NC_BTN_DEBOUNCE_MS as a
 * non-event (TVS/cap ringing on the GPIO2 net can slip past a naive
 * debouncer) — state reverts as if the pulse never happened.
 *
 * All times are uint32 milliseconds; comparisons use wraparound-safe
 * subtraction, so the 49.7-day rollover is harmless.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Board-level constant (mirrors BUTTON_DEBOUNCE_MS in main/board.h;
 * duplicated because narbis_core must not include ESP-IDF headers). */
#define NC_BTN_DEBOUNCE_MS 30

typedef enum {
    NC_BTN_EV_DOWN = 0,
    NC_BTN_EV_UP = 1,
    NC_BTN_EV_TIMEOUT = 2
} nc_btn_ev_t;

typedef enum {
    NC_BTN_ACT_NONE = 0,
    NC_BTN_ACT_MARKER = 1,   /* EVENT_STREAM marker, src=button */
    NC_BTN_ACT_PAIRING = 2,  /* open pairing window              */
    NC_BTN_ACT_OFF = 3,      /* power off (fired on release)     */
    NC_BTN_ACT_REBOOT = 4    /* esp_restart escape hatch         */
} nc_btn_act_t;

/* Internal states — exposed only so tests can assert; callers must not
 * dispatch on them. */
typedef enum {
    NC_BTN_S_IDLE = 0,   /* button up, no gesture in flight            */
    NC_BTN_S_DOWN1,      /* first press held, long threshold pending   */
    NC_BTN_S_WAIT2,      /* released short; double window open         */
    NC_BTN_S_DOWN2,      /* second press held inside the window        */
    NC_BTN_S_HELD        /* past long threshold; OFF armed for release */
} nc_btn_state_t;

typedef struct {
    uint8_t  state;      /* nc_btn_state_t */
    bool     pending;    /* a one-shot is outstanding at .deadline */
    uint32_t t_down;     /* DOWN edge of the press being tracked   */
    uint32_t t_up1;      /* UP that opened the double window       */
    uint32_t deadline;   /* absolute t_ms of the pending one-shot  */
} nc_btn_fsm_t;

void nc_btn_init(nc_btn_fsm_t *f);

/* Feed one event at time t_ms. Returns the action to perform (at most
 * one per call). Timings come from the live knob registry
 * (press_double_ms / press_long_ms / press_reboot_ms). */
nc_btn_act_t nc_btn_step(nc_btn_fsm_t *f, nc_btn_ev_t ev, uint32_t t_ms,
                         uint32_t *arm_timeout_ms);
