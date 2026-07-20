/*
 * afe4404_regs.h — AFE4404 (TI SBAS689D) register map, field encodings and
 * the timing-table types filled by the generated afe4404_timing.inc.
 *
 * All registers are 24-bit. I2C write = [reg][b23:16][b15:8][b7:0].
 * Reading a CONFIG register requires CONTROL0 REG_READ=1 around the read;
 * data registers 0x2A-0x2F need no REG_READ.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* Register addresses                                                  */
/* ------------------------------------------------------------------ */
#define AFE_REG_CONTROL0       0x00  /* REG_READ b0, TM_COUNT_RST b1, SW_RESET b3 */
#define AFE_REG_LED2STC        0x01  /* RED sample window                */
#define AFE_REG_LED2ENDC       0x02
#define AFE_REG_LED1LEDSTC     0x03  /* IR LED drive window              */
#define AFE_REG_LED1LEDENDC    0x04
#define AFE_REG_ALED2STC       0x05  /* ambient-2 sample window          */
#define AFE_REG_ALED2ENDC      0x06
#define AFE_REG_LED1STC        0x07  /* IR sample window                 */
#define AFE_REG_LED1ENDC       0x08
#define AFE_REG_LED2LEDSTC     0x09  /* RED LED drive window             */
#define AFE_REG_LED2LEDENDC    0x0A
#define AFE_REG_ALED1STC       0x0B  /* ambient-1 sample window          */
#define AFE_REG_ALED1ENDC      0x0C
#define AFE_REG_LED2CONVST     0x0D
#define AFE_REG_LED2CONVEND    0x0E
#define AFE_REG_ALED2CONVST    0x0F
#define AFE_REG_ALED2CONVEND   0x10
#define AFE_REG_LED1CONVST     0x11
#define AFE_REG_LED1CONVEND    0x12
#define AFE_REG_ALED1CONVST    0x13
#define AFE_REG_ALED1CONVEND   0x14  /* ADC_RDY is generated at this count */
#define AFE_REG_ADCRSTSTCT0    0x15
#define AFE_REG_ADCRSTENDCT0   0x16
#define AFE_REG_ADCRSTSTCT1    0x17
#define AFE_REG_ADCRSTENDCT1   0x18
#define AFE_REG_ADCRSTSTCT2    0x19
#define AFE_REG_ADCRSTENDCT2   0x1A
#define AFE_REG_ADCRSTSTCT3    0x1B
#define AFE_REG_ADCRSTENDCT3   0x1C
#define AFE_REG_PRPCT          0x1D  /* [15:0] = fTE/PRF - 1             */
#define AFE_REG_CONTROL1       0x1E  /* TIMEREN b8 | NUMAV[3:0]          */
#define AFE_REG_TIAGAIN_SEP    0x20  /* ENSEPGAIN b15 (0 = shared gain)  */
#define AFE_REG_TIA_GAIN       0x21  /* CF[5:3] | RF[2:0]                */
#define AFE_REG_LEDCNTRL       0x22  /* ILED2[11:6] red | ILED1[5:0] IR  */
#define AFE_REG_CONTROL2       0x23
#define AFE_REG_LED2VAL        0x2A  /* RED   (24-bit two's complement)  */
#define AFE_REG_ALED2VAL       0x2B  /* ambient-2                        */
#define AFE_REG_LED1VAL        0x2C  /* IR                               */
#define AFE_REG_ALED1VAL       0x2D  /* ambient-1                        */
#define AFE_REG_LED2_ALED2VAL  0x2E  /* on-chip differences (unused)     */
#define AFE_REG_LED1_ALED1VAL  0x2F
#define AFE_REG_PDNCYCLESTC    0x32  /* dynamic power-down window        */
#define AFE_REG_PDNCYCLEENDC   0x33
#define AFE_REG_LED3LEDSTC     0x36  /* TX3 unused on this board: 0      */
#define AFE_REG_LED3LEDENDC    0x37
#define AFE_REG_CLKDIV_PRF     0x39  /* [2:0] non-monotonic, see below   */
#define AFE_REG_OFFDAC         0x3A

/* ------------------------------------------------------------------ */
/* Field encodings                                                     */
/* ------------------------------------------------------------------ */
#define AFE_C0_REG_READ        (1u << 0)
#define AFE_C0_TM_COUNT_RST    (1u << 1)  /* holds timing counter in reset */
#define AFE_C0_SW_RESET        (1u << 3)  /* self-clearing                 */

#define AFE_C1_TIMEREN         (1u << 8)
#define AFE_C1_NUMAV_MASK      0x0Fu

#define AFE_C2_PDNAFE          (1u << 0)
#define AFE_C2_PDNRX           (1u << 1)
#define AFE_C2_DYNAMIC4        (1u << 3)
#define AFE_C2_DYNAMIC3        (1u << 4)
#define AFE_C2_OSC_ENABLE      (1u << 9)  /* internal 4 MHz oscillator     */
#define AFE_C2_DYNAMIC2        (1u << 14)
#define AFE_C2_ILED_2X         (1u << 17) /* NEVER set: 0-100 mA range exceeds
                                             SFH 7016 red die abs-max and the
                                             3.3 V TX_SUP headroom (board.h) */
#define AFE_C2_DYNAMIC1        (1u << 20)

#define AFE_TIAGAIN_ENSEPGAIN  (1u << 15) /* kept 0: single shared TIA gain */

/* TIA_GAIN (0x21): RF code 0..7 = 500k,250k,100k,50k,25k,10k,1M,2M */
#define AFE_RF_500K  0u
#define AFE_RF_250K  1u
#define AFE_RF_100K  2u   /* power-on choice */
#define AFE_RF_50K   3u
#define AFE_RF_25K   4u
#define AFE_RF_10K   5u
#define AFE_RF_1M    6u
#define AFE_RF_2M    7u
#define AFE_CF_5PF   0u   /* power-on choice; CF codes sit in bits [5:3] */
#define AFE_TIA_GAIN_VAL(cf, rf)  ((uint32_t)(((cf) & 7u) << 3) | ((rf) & 7u))

/* LEDCNTRL (0x22): code = round(mA * 63 / 50); driver clamps mA first */
#define AFE_LEDCNTRL_VAL(red_code, ir_code) \
    ((uint32_t)(((red_code) & 0x3Fu) << 6) | ((ir_code) & 0x3Fu))

/* OFFDAC (0x3A) field positions: {pol bit, magnitude shift} per phase.
 * Magnitude 0-15 ~= 0-7 uA, pol=1 negative. */
#define AFE_OFFDAC_LED2_POL    19
#define AFE_OFFDAC_LED2_SHIFT  15
#define AFE_OFFDAC_AMB1_POL    14
#define AFE_OFFDAC_AMB1_SHIFT  10
#define AFE_OFFDAC_LED1_POL    9
#define AFE_OFFDAC_LED1_SHIFT  5
#define AFE_OFFDAC_AMB2_POL    4
#define AFE_OFFDAC_AMB2_SHIFT  0
#define AFE_OFFDAC_MAG_MAX     15

/* CLKDIV_PRF (0x39[2:0]) — non-monotonic; 1,2,3 are FORBIDDEN codes */
#define AFE_CLKDIV_1   0u
#define AFE_CLKDIV_2   4u
#define AFE_CLKDIV_4   5u
#define AFE_CLKDIV_8   6u
#define AFE_CLKDIV_16  7u

/* ------------------------------------------------------------------ */
/* Timing-table types (data in generated afe4404_timing.inc)           */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t  reg;
    uint32_t val;
} afe_regval_t;

typedef struct {
    uint16_t rate_sps;
    uint8_t  clkdiv_field;       /* value written to 0x39                */
    uint16_t prpct;              /* value written to 0x1D                */
    uint8_t  numav;              /* CONTROL1 NUMAV[3:0]; averages = +1   */
    uint32_t period_us_nominal;  /* (PRPCT+1) * tick                     */
    bool     dyn_pd;             /* CONTROL2 DYNAMIC1..4 + PDNCYCLE used */
    const afe_regval_t *regs;
    uint8_t  n_regs;
} afe_rate_cfg_t;
