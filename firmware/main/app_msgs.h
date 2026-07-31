/*
 * app_msgs.h — sys_task mailbox contract. Every cross-task side effect
 * lands here; sys_task is the single serialization point for AFE config
 * I2C, state transitions, and CONTROL responses.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "narbis/nc_types.h"
#include "narbis/nc_agc.h"

#define SYS_CTRL_REQ_MAX 24  /* longest CONTROL request wire size + slack */

typedef enum {
    SYS_BTN_EDGE,        /* u.btn: debounced GPIO2 edge                  */
    SYS_BTN_TIMEOUT,     /* button FSM one-shot fired                    */
    SYS_AGC_ACTIONS,     /* u.agc: decisions from dsp_task slow tick     */
    SYS_SUB_CHANGE,      /* u.subs: CCCD snapshot from ble_gatt          */
    SYS_CTRL_REQ,        /* u.ctrl: copied CONTROL write (respond via
                            BLE_CH_CTRL_RESP after dispatch in sys ctx)   */
    SYS_TICK_1HZ,        /* battery/charger/STATUS housekeeping          */
    SYS_WEAR_CHANGED,    /* u.flag: from dsp_task 1 Hz wear eval         */
    SYS_GATE_CHANGED,    /* u.gate: state+reason for EVENT emission      */
    SYS_POWER_OFF_REQ,   /* button/CONTROL/low-batt initiated            */
    SYS_CONN_CHANGE,     /* u.flag: BLE connect(1)/disconnect(0)         */
    SYS_ENTER_OTA,       /* OTA service asked for acquisition stop       */
    SYS_TEST_IND,        /* TEST builds: LED connect-indicator 500 ms
                            tick (test_led_ind.c) — LED I2C runs in sys
                            ctx so it serializes with ops/AGC           */
} sys_msg_type_t;

typedef struct {
    sys_msg_type_t type;
    union {
        struct { bool pressed; uint32_t t_ms; } btn;
        struct { nc_agc_act_t acts[4]; uint8_t count; } agc;
        struct { bool ppg, accel, ibi, event, hrs; } subs;
        struct { uint8_t len; uint8_t buf[SYS_CTRL_REQ_MAX]; } ctrl;
        struct { bool on; uint8_t reason; } gate;
        bool flag;
    } u;
} sys_msg_t;

extern QueueHandle_t sys_q;

/* Non-blocking post (timeout 0): hot paths must never block on sys_q.
 * Returns false on overflow (caller increments a diag counter). */
bool sys_post(const sys_msg_t *msg);
bool sys_post_from_isr(const sys_msg_t *msg, BaseType_t *hpw);
