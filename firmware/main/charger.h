/*
 * charger.h — MCP73831 charge-state + VUSB presence monitoring.
 *
 * Board facts (board.h): PIN_CHG_STAT = GPIO19 (MCP73831 STAT through a
 * 100 k / 150 k divider, 5 V -> 3 V domain), PIN_VUSB_SENSE = GPIO20
 * (VBUS through an identical divider — digital presence only, the pin
 * is not ADC-capable). Both are plain 1 Hz-polled GPIO reads; no
 * interrupts, no internal pulls (the dividers define the levels).
 *
 * Load-switch consequence (LM66100, CE# from VUSB): USB present means
 * the cell is disconnected from the load and charging at 50 mA
 * (CHG_CURRENT_MA); the system runs from USB.
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#include "narbis/nc_types.h"

/* Configure PIN_CHG_STAT + PIN_VUSB_SENSE as pull-less inputs. */
esp_err_t charger_init(void);

/*
 * Sample both pins and decode:
 *   VUSB low                -> NC_CHG_ON_BATTERY (STAT ignored: the
 *                              MCP73831 is unpowered, STAT is Hi-Z)
 *   VUSB high && STAT low   -> NC_CHG_CHARGING
 *   VUSB high && STAT high  -> NC_CHG_COMPLETE
 * *vusb_out (if non-NULL) gets the raw VUSB presence for NC_STF_USB /
 * NC_PPGF_USB_PRESENT.
 */
nc_charger_state_t charger_poll(bool *vusb_out);
