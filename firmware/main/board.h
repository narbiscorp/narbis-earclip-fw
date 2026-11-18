/*
 * board.h — Narbis Edge Earclip V2.1 hardware constants.
 *
 * Source of truth: firmware_handoff.md §1 + Addendum 1 (netlist-verified
 * against ProPrj_STS-USA60630 ..._2026-07-20.epro2). Hardware is FROZEN.
 * No logic in this file.
 */
#pragma once

/* ------------------------------------------------------------------ */
/* Build-mode switch                                                   */
/* ------------------------------------------------------------------ */
/* 1 = PCB design-verification (vendor functest) build: advertises
 *     "Narbis Edge Earclip TEST", unlocks the TEST opcode block
 *     (0xE0-0xEF) on CONTROL, never drops to slow advertising, and the
 *     XIAO module LED (GPIO15) shows power/link state.
 * 0 = production: advertises NARBIS_PRODUCT_NAME, TEST ops answer
 *     UNAUTHORIZED, module LED dark. Everything else (NARBIS_GATED_PROFILE
 *     below) is the same code in both. */
#define NARBIS_TEST_MODE 0

/* Production advertising name (v0.4.0, user 2026-11-17). The TEST build
 * keeps "Narbis Edge Earclip TEST": the vendor tester and the web OTA
 * updater route TEST boards by that name. */
#define NARBIS_PRODUCT_NAME "Narbis Edge Clip"

/* 1 = the ORIGINAL handoff production policy, never run on hardware:
 *     bonds only inside a button double-press pairing window, encrypted
 *     custom + OTA characteristics, marker / pairing / 1.5 s off / 8 s
 *     reboot gestures, auto light sleep, wear-gated streaming, idle and
 *     off-ear auto-off, no LED indicator.
 * 0 = the open profile proven on the functest builds through v0.3.0-63
 *     and shipped as production from v0.4.0 (user, 2026-11-17: "keep
 *     with what we have now and move that to production"): open
 *     pairing like a Polar strap, no encryption, any tap on / 0.3 s
 *     hold off, ear-posture standby, red-LED breathing and charge
 *     indicator, no light sleep. OFF is the v0.3.0-63 deep sleep
 *     exactly: button wake only. (A 3 s LP-timer wake to catch a
 *     charger being plugged in roughly doubled the OFF current, ~40 to
 *     ~100 uA on the PPK2, 2026-11-18, and was removed.) So the charge
 *     indicator shows on a clip that is on; an OFF clip on the charger
 *     stays dark until the button is tapped. */
#define NARBIS_GATED_PROFILE 0

/* 1 = BENCH build for BARE XIAO modules (pre-soldering): sensor init
 *     failures are tolerated (logged, marked absent — selftest fails
 *     honestly), BLE + console + BOTH OTA paths stay up so the real
 *     firmware can be loaded over the air after assembly, and the
 *     module's onboard LED (GPIO15) pulses at 2 Hz / steady when
 *     connected. Only meaningful together with NARBIS_TEST_MODE=1.
 *     Production and vendor functest builds keep this 0: an assembled
 *     board with a dead sensor must fail loudly, never limp. */
#define NARBIS_BENCH_BUILD 0

/* ------------------------------------------------------------------ */
/* Pin map (XIAO ESP32-C6 castellated pads -> C6 GPIOs)                */
/* ------------------------------------------------------------------ */
#define PIN_BATT_ADC        1   /* BS1: VBAT_CELL via 1M:1M divider + 100nF   */
#define PIN_BUTTON          2   /* SW3 to GND; NO external pull-up: internal  */
                                /* LP pull-up must be enabled AND retained    */
                                /* through deep sleep or ext1 wake never fires*/
#define PIN_ADC_RDY         16  /* AFE4404 data-ready, ~250 ns pulse, rising  */
#define PIN_ACC_INT1        17  /* LIS2DH12 INT1, ext 10k pull-DOWN, active-  */
                                /* high; NOT an LP pin (light-sleep wake only)*/
#define PIN_AFE_CLK         18  /* AFE CLK via 500R; ext 500k pull-down.      */
                                /* Unused v1: input, NO internal pull.        */
#define PIN_CHG_STAT        19  /* MCP73831 STAT via 100k/150k divider        */
#define PIN_VUSB_SENSE      20  /* VBUS via 100k/150k divider, digital only   */
#define PIN_AFE_RESET       21  /* AFE4404 RESETZ; ext pull-up R1=100k (V2.1).*/
                                /* NOT an LP pin: deep sleep needs            */
                                /* gpio_hold_en + gpio_deep_sleep_hold_en.    */
                                /* V2.2 flips R1 to pull-down (rev-proof init)*/
#define PIN_I2C_SDA         22  /* 4.7k pull-up */
#define PIN_I2C_SCL         23  /* 4.7k pull-up */
/* GPIO0 is a no-connect on this board. Never drive or read it. */
/* XIAO-internal RF switch (FM8625H; Seeed schematic v1.0: "RF Switch
 * Power 0: ON 1: OFF", "Port Selection 0: RF1 1: RF2"). GPIO3 low powers
 * it, GPIO14 low selects the onboard ceramic antenna (the U.FL port is
 * unused here). Not on the castellated pads — module-internal pins. */
#define PIN_RF_SW_PWR       3
#define PIN_RF_SW_SEL       14

/* ------------------------------------------------------------------ */
/* I2C                                                                 */
/* ------------------------------------------------------------------ */
#define I2C_FREQ_HZ         400000  /* AFE4404 hard maximum — do not exceed */
#define ADDR_AFE4404        0x58
#define ADDR_LIS2DH12       0x18    /* SA0 strapped to GND (settled). If NACK,
                                       probe 0x19 and log loudly. */
#define ADDR_LIS2DH12_ALT   0x19

/* ------------------------------------------------------------------ */
/* Battery / charger                                                   */
/* ------------------------------------------------------------------ */
#define BATT_DIVIDER_NUM    2       /* V_batt = 2 x V_adc (1M:1M)  */
#define BATT_OVERSAMPLE     64      /* ~500k source impedance      */
#define BATT_CAPACITY_MAH   62      /* LP401515                    */
#define CHG_CURRENT_MA      50      /* MCP73831, PROG=20k          */

/* ------------------------------------------------------------------ */
/* LED safety clamps (SFH 7016 DC abs-max; enforced in driver AND AGC) */
/* ------------------------------------------------------------------ */
#define LED_IR_MAX_MA       50      /* AFE DAC full range          */
#define LED_RED_MAX_MA      40      /* red die DC abs-max          */
/* ILED_2X (0x23 bit17) must NEVER be set: 0-100 mA range exceeds the
 * red die abs-max and the 3.3 V TX_SUP headroom. */

/* ------------------------------------------------------------------ */
/* Timing constants (Addendum 1 + SBAS689D Table 79)                   */
/* ------------------------------------------------------------------ */
#define AFE_T_SUPPLY_TO_RESET_MS   10   /* supplies stable -> reset pulse */
#define AFE_T_ACTIVE_MS            10   /* recovery from PWDN (tACTIVE)   */
#define AFE_T_RESET_PULSE_US       30   /* reset pulse: 25-50 us window   */
#define AFE_T_PWDN_MIN_US          250  /* RESETZ low > 200 us = PWDN     */
#define AFE_T_RESET_TO_I2C_MS      2    /* > 1 ms before first I2C        */

#define BUTTON_DEBOUNCE_MS         30

/* Power targets (README measured-actuals table tracks reality) */
#define TARGET_OFF_UA              80   /* Addendum 1 (33 uA is R1 hold-low) */
