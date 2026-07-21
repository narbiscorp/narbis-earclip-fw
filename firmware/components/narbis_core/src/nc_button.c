/*
 * nc_button.c — button gesture FSM. See nc_button.h for the contract.
 *
 * Deadline discipline: the FSM stores the ABSOLUTE deadline of its one
 * pending one-shot. Ignore-paths (duplicate edges, stray UPs) re-emit
 * the remaining time to the SAME absolute deadline, so a caller that
 * blindly re-arms on every call never drifts a threshold.
 */
#include "narbis/nc_button.h"
#include "narbis/nc_knobs.h"

void nc_btn_init(nc_btn_fsm_t *f)
{
    f->state = NC_BTN_S_IDLE;
    f->pending = false;
    f->t_down = f->t_up1 = f->deadline = 0;
}

/* Arm the one-shot to fire at absolute time 'deadline'. Returns the
 * relative value for *arm_timeout_ms, clamped to >= 1: a deadline that
 * is now/past (late event delivery, or pathological knob combinations
 * like press_reboot_ms <= press_long_ms) degrades to a 1 ms one-shot,
 * never to 0 — 0 on the wire means "cancel". */
static uint32_t arm_abs(nc_btn_fsm_t *f, uint32_t t, uint32_t deadline)
{
    f->deadline = deadline;
    f->pending = true;
    uint32_t rem = deadline - t;                 /* wraparound-safe */
    if (rem == 0 || rem > 0x7FFFFFFFu) rem = 1;  /* past/now -> ASAP */
    return rem;
}

static uint32_t arm_rel(nc_btn_fsm_t *f, uint32_t t, uint32_t delta)
{
    return arm_abs(f, t, t + delta);
}

/* Ignored event: keep the pending deadline (re-emit remaining time). */
static uint32_t keep(const nc_btn_fsm_t *f, uint32_t t)
{
    if (!f->pending) return 0;
    uint32_t rem = f->deadline - t;
    if (rem == 0 || rem > 0x7FFFFFFFu) rem = 1;
    return rem;
}

static uint32_t cancel(nc_btn_fsm_t *f)
{
    f->pending = false;
    return 0;
}

nc_btn_act_t nc_btn_step(nc_btn_fsm_t *f, nc_btn_ev_t ev, uint32_t t_ms,
                         uint32_t *arm_timeout_ms)
{
    const uint32_t dbl_ms    = (uint32_t)nc_knob_get(KNOB_PRESS_DOUBLE_MS);
    const uint32_t long_ms   = (uint32_t)nc_knob_get(KNOB_PRESS_LONG_MS);
    const uint32_t reboot_ms = (uint32_t)nc_knob_get(KNOB_PRESS_REBOOT_MS);

    switch ((nc_btn_state_t)f->state) {

    case NC_BTN_S_IDLE:
        if (ev == NC_BTN_EV_DOWN) {
            f->t_down = t_ms;
            f->state = NC_BTN_S_DOWN1;
            *arm_timeout_ms = arm_rel(f, t_ms, long_ms);
            return NC_BTN_ACT_NONE;
        }
        /* stray UP / stale TIMEOUT: nothing in flight */
        *arm_timeout_ms = cancel(f);
        return NC_BTN_ACT_NONE;

    case NC_BTN_S_DOWN1:
        if (ev == NC_BTN_EV_UP) {
            uint32_t held = t_ms - f->t_down;
            if (held < NC_BTN_DEBOUNCE_MS) {
                /* sub-debounce ghost pulse: unwind as a non-event */
                f->state = NC_BTN_S_IDLE;
                *arm_timeout_ms = cancel(f);
                return NC_BTN_ACT_NONE;
            }
            if (held >= long_ms) {
                /* release crossed the long threshold before the
                 * TIMEOUT was delivered (ISR/timer race): hold path */
                f->state = NC_BTN_S_IDLE;
                *arm_timeout_ms = cancel(f);
                return NC_BTN_ACT_OFF;
            }
            /* short press: open the double window; MARKER only fires
             * at expiry so a double press never emits a marker */
            f->t_up1 = t_ms;
            f->state = NC_BTN_S_WAIT2;
            *arm_timeout_ms = arm_rel(f, t_ms, dbl_ms);
            return NC_BTN_ACT_NONE;
        }
        if (ev == NC_BTN_EV_TIMEOUT) {
            /* long threshold while held: OFF now armed on release;
             * REBOOT deadline stays anchored to the DOWN edge */
            f->state = NC_BTN_S_HELD;
            *arm_timeout_ms = arm_abs(f, t_ms, f->t_down + reboot_ms);
            return NC_BTN_ACT_NONE;
        }
        *arm_timeout_ms = keep(f, t_ms); /* duplicate DOWN */
        return NC_BTN_ACT_NONE;

    case NC_BTN_S_WAIT2:
        if (ev == NC_BTN_EV_DOWN) {
            if (t_ms - f->t_up1 >= dbl_ms) {
                /* window expired but the TIMEOUT lost the race with
                 * this edge: emit the overdue MARKER and treat this
                 * DOWN as a fresh first press */
                f->t_down = t_ms;
                f->state = NC_BTN_S_DOWN1;
                *arm_timeout_ms = arm_rel(f, t_ms, long_ms);
                return NC_BTN_ACT_MARKER;
            }
            f->t_down = t_ms;
            f->state = NC_BTN_S_DOWN2;
            *arm_timeout_ms = arm_rel(f, t_ms, long_ms);
            return NC_BTN_ACT_NONE;
        }
        if (ev == NC_BTN_EV_TIMEOUT) {
            /* single short press confirmed — exactly at window expiry,
             * i.e. <= press_double_ms after the UP (450 ms budget) */
            f->state = NC_BTN_S_IDLE;
            *arm_timeout_ms = cancel(f);
            return NC_BTN_ACT_MARKER;
        }
        *arm_timeout_ms = keep(f, t_ms); /* stray UP */
        return NC_BTN_ACT_NONE;

    case NC_BTN_S_DOWN2:
        if (ev == NC_BTN_EV_UP) {
            uint32_t held = t_ms - f->t_down;
            if (held < NC_BTN_DEBOUNCE_MS) {
                /* ghost pulse inside the window: not a second press.
                 * Restore the window with its ORIGINAL absolute end. */
                f->state = NC_BTN_S_WAIT2;
                *arm_timeout_ms = arm_abs(f, t_ms, f->t_up1 + dbl_ms);
                return NC_BTN_ACT_NONE;
            }
            if (held >= long_ms) {
                /* hold path won but TIMEOUT raced the release */
                f->state = NC_BTN_S_IDLE;
                *arm_timeout_ms = cancel(f);
                return NC_BTN_ACT_OFF;
            }
            f->state = NC_BTN_S_IDLE;
            *arm_timeout_ms = cancel(f);
            return NC_BTN_ACT_PAIRING;
        }
        if (ev == NC_BTN_EV_TIMEOUT) {
            /* DECIDED: second press held to the long threshold cancels
             * pairing and follows the hold path (OFF/REBOOT) */
            f->state = NC_BTN_S_HELD;
            *arm_timeout_ms = arm_abs(f, t_ms, f->t_down + reboot_ms);
            return NC_BTN_ACT_NONE;
        }
        *arm_timeout_ms = keep(f, t_ms); /* duplicate DOWN */
        return NC_BTN_ACT_NONE;

    case NC_BTN_S_HELD:
        if (ev == NC_BTN_EV_UP) {
            f->state = NC_BTN_S_IDLE;
            *arm_timeout_ms = cancel(f);
            return NC_BTN_ACT_OFF;
        }
        if (ev == NC_BTN_EV_TIMEOUT) {
            /* press_reboot_ms without release: last-resort escape
             * hatch (reset button is sealed in the enclosure) */
            f->state = NC_BTN_S_IDLE;
            *arm_timeout_ms = cancel(f);
            return NC_BTN_ACT_REBOOT;
        }
        *arm_timeout_ms = keep(f, t_ms); /* duplicate DOWN */
        return NC_BTN_ACT_NONE;
    }

    /* unreachable — state is a private enum */
    *arm_timeout_ms = cancel(f);
    return NC_BTN_ACT_NONE;
}
