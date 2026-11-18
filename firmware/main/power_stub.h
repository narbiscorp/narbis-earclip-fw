/*
 * power_stub.h — the deep-sleep wake stub (rtc_wake_stub_usb.c) and the
 * RTC-memory state it shares with power.c. See that file's header.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* LP-timer poll period while OFF: worst-case delay from plugging in the
 * charger to the clip booting. Each poll is a sub-millisecond wake. */
#define POWER_STUB_POLL_US   (3ULL * 1000000ULL)

/* Why the stub let the boot continue (power_stub_boot_reason). */
#define POWER_STUB_BOOT_NONE     0   /* button / cold / other            */
#define POWER_STUB_BOOT_USB      1   /* charger plugged in while OFF     */
#define POWER_STUB_BOOT_RECHECK  2   /* battery-forced OFF, 12 h recheck */
#define POWER_STUB_BOOT_TEST     3   /* console 'stubtest' finished      */

extern bool     power_stub_vusb_prev;
extern uint32_t power_stub_recheck_left;
extern uint8_t  power_stub_boot_reason;
extern uint32_t power_stub_test_polls;
extern uint32_t power_stub_polls;

void power_wake_stub(void);
