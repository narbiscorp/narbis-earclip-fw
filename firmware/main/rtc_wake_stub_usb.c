/*
 * rtc_wake_stub_usb.c — deep-sleep wake stub: power on when a charger is
 * plugged in (v0.4.0, user 2026-11-17: the red LED shows charging).
 *
 * VUSB_SENSE (GPIO20) is not an LP pin, so ext1 cannot watch it. Instead
 * OFF also arms the LP timer, and this stub runs on every timer wake —
 * from RTC fast memory, before the ROM loads the bootloader — reads the
 * pad, and goes straight back to sleep unless USB just APPEARED (an edge
 * against the level seen at the previous poll / at OFF entry). An OFF
 * entered while already on the charger therefore stays off until the
 * cable is pulled and plugged again (or the button is pressed).
 *
 * The battery-forced OFF's 12 h recheck (power.c) is counted here in
 * polls, since the LP timer now fires every POWER_STUB_POLL_US.
 *
 * Rules for this file (IDF "Deep-sleep Wake Stubs"; the rtc_wake_stub*
 * file name puts ALL of its text/data/bss in RTC memory):
 *  - call only ROM functions and RTC_IRAM_ATTR functions (esp_wake_stub_*,
 *    esp_default_wake_deep_sleep);
 *  - touch only data in RTC memory (this file's globals) and registers.
 *
 * Shared state is plain globals of this file (RTC bss/data); power.c
 * reads and writes them through power_stub.h before sleeping and after
 * waking.
 */
#include <stdbool.h>
#include <stdint.h>

#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_wake_stub.h"
#include "esp_private/esp_pmu.h"   /* RTC_TIMER_TRIG_EN / RTC_EXT1_TRIG_EN */
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/soc.h"

#include "board.h"
#include "power_stub.h"

/* RTC memory (this file's sections). Initialized at cold boot only. */
bool     power_stub_vusb_prev;      /* VUSB level at the last look       */
uint32_t power_stub_recheck_left;   /* polls until the 12 h recheck; 0 = none */
uint8_t  power_stub_boot_reason;    /* POWER_STUB_BOOT_*, for power.c    */
uint32_t power_stub_test_polls;     /* console 'stubtest': forced re-sleeps */
uint32_t power_stub_polls;          /* polls since OFF (diagnostics)     */

static inline bool stub_vusb_present(void)
{
    /* Pad 20 after the wake reset: make sure the input path is on and no
     * internal pull fights the 100 k / 150 k divider that defines the
     * level. charger_init() re-configures the pad on a full boot. */
    uint32_t r = REG_READ(IO_MUX_GPIO20_REG);
    r |= FUN_IE;
    r &= ~(FUN_PU | FUN_PD);
    REG_WRITE(IO_MUX_GPIO20_REG, r);
    for (volatile int i = 0; i < 64; i++) {
        /* a few us for the pad input to settle */
    }
    return ((REG_READ(GPIO_IN_REG) >> PIN_VUSB_SENSE) & 1u) != 0;
}

static void boot_now(uint8_t reason)
{
    power_stub_boot_reason = reason;
    esp_default_wake_deep_sleep();
    /* returning from the stub continues into the normal boot */
}

void power_wake_stub(void)
{
    const uint32_t cause = esp_wake_stub_get_wakeup_cause();

    if (!(cause & RTC_TIMER_TRIG_EN) || (cause & RTC_EXT1_TRIG_EN)) {
        /* The button (or anything unexpected): a normal boot, with the
         * pad holds untouched so power_early_wake_capture still works. */
        boot_now(POWER_STUB_BOOT_NONE);
        return;
    }

    power_stub_polls++;
    const bool vusb = stub_vusb_present();
    const bool plugged = vusb && !power_stub_vusb_prev;
    power_stub_vusb_prev = vusb;

    if (power_stub_test_polls != 0) {
        /* Bench self-test of the re-sleep path (console 'stubtest'): a
         * fixed number of no-edge polls, then boot so the console can
         * report that the stub survived them. */
        if (--power_stub_test_polls == 0) {
            boot_now(POWER_STUB_BOOT_TEST);
            return;
        }
    } else if (plugged) {
        boot_now(POWER_STUB_BOOT_USB);
        return;
    } else if (power_stub_recheck_left != 0 && --power_stub_recheck_left == 0) {
        boot_now(POWER_STUB_BOOT_RECHECK);   /* battery-forced 12 h recheck */
        return;
    }

    esp_wake_stub_set_wakeup_time(POWER_STUB_POLL_US);
    esp_wake_stub_sleep(&power_wake_stub);
}
