/*
 * charger.c — MCP73831 STAT + VUSB presence decoding. Pure polled GPIO;
 * the 1 Hz cadence and notification pushes live in the power/sys task.
 */
#include "charger.h"

#include "driver/gpio.h"

#include "board.h"

esp_err_t charger_init(void)
{
    /* External 100 k / 150 k dividers define both levels — internal
     * pulls would re-bias them, so they stay off. */
    const gpio_config_t cfg = {
        .pin_bit_mask  = (1ULL << PIN_CHG_STAT) | (1ULL << PIN_VUSB_SENSE),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    return gpio_config(&cfg);
}

/* ------------------------------------------------------------------
 * MCP73831 STAT interpretation — VERIFY ON BENCH BEFORE TRUSTING.
 *
 * Datasheet (DS20001984, Table 5-1): STAT drives LOW during a charge
 * cycle, HIGH on charge complete, and goes Hi-Z in shutdown / standby.
 * On this board STAT reaches PIN_CHG_STAT through the 100 k / 150 k
 * divider (5 V -> 3 V domain); with STAT Hi-Z the 150 k bottom leg
 * drags the pin LOW, i.e. an undriven STAT reads as "charging".
 * charger_poll() only consults STAT while VUSB is present (the part is
 * powered then, so STAT should never be Hi-Z), but tri-state pins
 * behind resistive dividers are exactly where bench reality diverges
 * from datasheets.
 *
 * If bench measurement shows the opposite sense, flip ONLY the return
 * expression below — every STAT interpretation in the firmware funnels
 * through this function.
 * ------------------------------------------------------------------ */
static nc_charger_state_t charger_decode_stat(int stat_level)
{
    return stat_level ? NC_CHG_COMPLETE : NC_CHG_CHARGING;
}

nc_charger_state_t charger_poll(bool *vusb_out)
{
    const bool vusb = gpio_get_level(PIN_VUSB_SENSE) != 0;
    if (vusb_out != NULL) {
        *vusb_out = vusb;
    }
    if (!vusb) {
        /* Charger unpowered: STAT is Hi-Z and its divider reads low —
         * meaningless, so it is ignored by contract. */
        return NC_CHG_ON_BATTERY;
    }
    return charger_decode_stat(gpio_get_level(PIN_CHG_STAT));
}
