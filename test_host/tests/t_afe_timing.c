/*
 * t_afe_timing.c — timing-invariant verification of the committed AFE4404
 * rate tables (firmware/main/afe4404_timing.inc).
 *
 * Defines the table typedefs locally (mirrors firmware/main/afe4404_regs.h;
 * that header is ESP-side and deliberately not included) and re-derives the
 * SBAS689D constraints from first principles, so any regenerated layout
 * that still honors the datasheet passes and any violation fails:
 *   PRPCT/CLKDIV per rate, END>=ST, counts within [0,PRPCT], 100 us sample
 *   windows in ticks, convert >= (NUMAV+2)*200*tADC + 15 us, all four
 *   ADCRSTx present, PDNCYCLE margins >= 200 us at 50/100 only, address
 *   whitelist, and frame period == (PRPCT+1) ticks == 1/rate.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "tst.h"

typedef struct {
    uint8_t  reg;
    uint32_t val;
} afe_regval_t;

typedef struct {
    uint16_t rate_sps;
    uint8_t  clkdiv_field;
    uint16_t prpct;
    uint8_t  numav;
    uint32_t period_us_nominal;
    bool     dyn_pd;
    const afe_regval_t *regs;
    uint8_t  n_regs;
} afe_rate_cfg_t;

#include "../../firmware/main/afe4404_timing.inc"

TST_SUITE("afe_timing");

/* Register addresses (SBAS689D) */
#define R_LED2STC       0x01
#define R_LED2ENDC      0x02
#define R_LED1LEDSTC    0x03
#define R_LED1LEDENDC   0x04
#define R_ALED2STC      0x05
#define R_ALED2ENDC     0x06
#define R_LED1STC       0x07
#define R_LED1ENDC      0x08
#define R_LED2LEDSTC    0x09
#define R_LED2LEDENDC   0x0A
#define R_ALED1STC      0x0B
#define R_ALED1ENDC     0x0C
#define R_PRPCT         0x1D
#define R_CONTROL2      0x23
#define R_PDNCYCLESTC   0x32
#define R_PDNCYCLEENDC  0x33
#define R_LED3LEDSTC    0x36
#define R_LED3LEDENDC   0x37
#define R_CLKDIV_PRF    0x39

#define C2_PDNAFE       (1u << 0)
#define C2_PDNRX        (1u << 1)
#define C2_OSC_ENABLE   (1u << 9)
#define C2_ILED_2X      (1u << 17)
#define C2_DYNAMIC_ALL  ((1u << 20) | (1u << 14) | (1u << 4) | (1u << 3))

#define T_ADC_NS        250u
#define SAMPLE_NS       100000u   /* every sample window is 100 us */
#define PDN_MARGIN_NS   200000u

/* Expected per-rate contract, index == nc_rate_t order. */
static const struct {
    uint16_t sps;
    uint8_t  field;      /* CLKDIV_PRF; non-monotonic encoding */
    uint16_t prpct;
    uint8_t  numav;
    bool     dyn;
} EXP[5] = {
    {  50, 4, 39999, 15, true  },   /* 50 sps MUST divide by 2 (field 4) */
    { 100, 0, 39999, 15, true  },
    { 200, 0, 19999, 15, false },
    { 250, 0, 15999, 15, false },
    { 500, 0,  7999,  5, false },
};

/* Sample/convert/reset window register pairs in phase order:
 * RED(LED2), ambient-2, IR(LED1), ambient-1. */
static const uint8_t SMP[4][2] = {
    { 0x01, 0x02 }, { 0x05, 0x06 }, { 0x07, 0x08 }, { 0x0B, 0x0C }
};
static const uint8_t CONV[4][2] = {
    { 0x0D, 0x0E }, { 0x0F, 0x10 }, { 0x11, 0x12 }, { 0x13, 0x14 }
};
static const uint8_t RST[4][2] = {
    { 0x15, 0x16 }, { 0x17, 0x18 }, { 0x19, 0x1A }, { 0x1B, 0x1C }
};

static int find_reg(const afe_rate_cfg_t *c, uint8_t reg, uint32_t *val)
{
    int n = 0;
    for (int i = 0; i < c->n_regs; i++) {
        if (c->regs[i].reg == reg) {
            if (val) *val = c->regs[i].val;
            n++;
        }
    }
    return n;
}

/* Value of a register that must appear exactly once. */
static uint32_t rv(const afe_rate_cfg_t *c, uint8_t reg)
{
    uint32_t v = 0;
    CHECK_EQ(find_reg(c, reg, &v), 1);
    return v;
}

static uint32_t tick_ns_of_field(uint8_t field)
{
    switch (field) {
    case 0: return 250;    /* /1  */
    case 4: return 500;    /* /2  */
    case 5: return 1000;   /* /4  */
    case 6: return 2000;   /* /8  */
    case 7: return 4000;   /* /16 */
    default: return 0;     /* 1,2,3 forbidden; anything else invalid */
    }
}

static int addr_documented(uint8_t reg)
{
    return (reg >= 0x01 && reg <= 0x1D) ||    /* timing engine + PRPCT   */
           reg == 0x31 ||                     /* CONTROL3: INPUT_SHORT   */
           reg == R_CONTROL2 ||
           reg == R_PDNCYCLESTC || reg == R_PDNCYCLEENDC ||
           reg == R_LED3LEDSTC || reg == R_LED3LEDENDC ||
           reg == R_CLKDIV_PRF;
}

static void t_table_shape(void)
{
    CHECK_EQ(sizeof AFE_RATE_TABLE / sizeof AFE_RATE_TABLE[0], 5);
    for (int r = 0; r < 5; r++) {
        const afe_rate_cfg_t *c = &AFE_RATE_TABLE[r];
        CHECK_EQ(c->rate_sps, EXP[r].sps);
        CHECK(c->regs != NULL);
        CHECK(c->n_regs > 0);
        /* every address documented, no duplicates */
        for (int i = 0; i < c->n_regs; i++) {
            CHECK(addr_documented(c->regs[i].reg));
            for (int j = i + 1; j < c->n_regs; j++) {
                CHECK(c->regs[i].reg != c->regs[j].reg);
            }
        }
    }
}

static void t_clkdiv_prpct_period(void)
{
    for (int r = 0; r < 5; r++) {
        const afe_rate_cfg_t *c = &AFE_RATE_TABLE[r];
        const uint32_t field = rv(c, R_CLKDIV_PRF);

        CHECK(field != 1 && field != 2 && field != 3);   /* forbidden codes */
        CHECK_EQ(field, EXP[r].field);
        CHECK_EQ(c->clkdiv_field, field);

        const uint64_t tick_ns = tick_ns_of_field((uint8_t)field);
        CHECK(tick_ns != 0);

        CHECK_EQ(rv(c, R_PRPCT), EXP[r].prpct);
        CHECK_EQ(c->prpct, EXP[r].prpct);

        /* Frame period: (PRPCT+1) ticks must equal exactly 1/rate and the
         * table's nominal microsecond period. */
        const uint64_t period_ns = ((uint64_t)c->prpct + 1u) * tick_ns;
        CHECK_EQ(period_ns * c->rate_sps, 1000000000ull);
        CHECK_EQ(period_ns, (uint64_t)c->period_us_nominal * 1000u);
        CHECK_EQ((uint64_t)c->period_us_nominal * c->rate_sps, 1000000ull);
    }
}

static void t_windows_ranges(void)
{
    /* Every ST/END pair in the table: END >= ST and both within [0,PRPCT].
     * Counts are unsigned, so >= 0 is inherent. */
    static const uint8_t PAIRS[][2] = {
        { 0x01, 0x02 }, { 0x03, 0x04 }, { 0x05, 0x06 }, { 0x07, 0x08 },
        { 0x09, 0x0A }, { 0x0B, 0x0C }, { 0x0D, 0x0E }, { 0x0F, 0x10 },
        { 0x11, 0x12 }, { 0x13, 0x14 }, { 0x15, 0x16 }, { 0x17, 0x18 },
        { 0x19, 0x1A }, { 0x1B, 0x1C }, { 0x32, 0x33 }, { 0x36, 0x37 },
    };
    for (int r = 0; r < 5; r++) {
        const afe_rate_cfg_t *c = &AFE_RATE_TABLE[r];
        for (size_t p = 0; p < sizeof PAIRS / sizeof PAIRS[0]; p++) {
            const uint32_t st = rv(c, PAIRS[p][0]);
            const uint32_t end = rv(c, PAIRS[p][1]);
            CHECK(end >= st);
            CHECK(end <= c->prpct);
        }
        /* LED3 unused: hard zero */
        CHECK_EQ(rv(c, R_LED3LEDSTC), 0);
        CHECK_EQ(rv(c, R_LED3LEDENDC), 0);
    }
}

static void t_sample_phases(void)
{
    /* Sample window per rate: 100 us, except 500 sps where 70 us keeps
     * cumulative LED duty under the 10% abs-max with the 25 us t1 lead
     * (2026-08-01 LED audit — SBAS689D p.6 / p.24 Table 7). */
    static const uint64_t SMP_NS[5] = {
        100000, 100000, 100000, 100000, 70000,
    };
    for (int r = 0; r < 5; r++) {
        const afe_rate_cfg_t *c = &AFE_RATE_TABLE[r];
        const uint64_t tick_ns = tick_ns_of_field(c->clkdiv_field);
        uint32_t st[4], end[4];

        for (int k = 0; k < 4; k++) {
            st[k] = rv(c, SMP[k][0]);
            end[k] = rv(c, SMP[k][1]);
            /* window duration = END - ST + 1 ticks == the rate's budget */
            CHECK_EQ((uint64_t)(end[k] - st[k] + 1u) * tick_ns, SMP_NS[r]);
        }
        /* phase order RED, amb-2, IR, amb-1: strictly sequential */
        for (int k = 0; k < 3; k++) {
            CHECK(st[k + 1] > end[k]);
        }

        /* LED drive brackets its own sample window and must not overlap any
         * other phase's sample window (would corrupt the ambients). */
        const uint32_t red_dst = rv(c, R_LED2LEDSTC);
        const uint32_t red_den = rv(c, R_LED2LEDENDC);
        CHECK(red_dst <= st[0] && red_den >= end[0]);
        CHECK(red_den < st[1]);                        /* off before amb-2  */

        const uint32_t ir_dst = rv(c, R_LED1LEDSTC);
        const uint32_t ir_den = rv(c, R_LED1LEDENDC);
        CHECK(ir_dst <= st[2] && ir_den >= end[2]);
        CHECK(ir_dst > end[1]);                        /* on after amb-2    */
        CHECK(ir_den < st[3]);                         /* off before amb-1  */

        /* t1 (LED start -> sample start) >= max[25 us, 0.2 x pulse]
         * (SBAS689D p.24 Table 7) — the original tables had 5 us. */
        const uint64_t red_pulse = (uint64_t)(red_den - red_dst + 1u) * tick_ns;
        const uint64_t ir_pulse  = (uint64_t)(ir_den - ir_dst + 1u) * tick_ns;
        const uint64_t red_t1 = (uint64_t)(st[0] - red_dst) * tick_ns;
        const uint64_t ir_t1  = (uint64_t)(st[2] - ir_dst) * tick_ns;
        CHECK(red_t1 >= 25000 && red_t1 * 5 >= red_pulse);
        CHECK(ir_t1 >= 25000 && ir_t1 * 5 >= ir_pulse);

        /* Cumulative LED duty <= 10% abs-max at ILED_2X=0 (p.6). */
        const uint64_t frame_ns = ((uint64_t)c->prpct + 1u) * tick_ns;
        CHECK((red_pulse + ir_pulse) * 10 <= frame_ns);

        /* CONTROL3 INPUT_SHORT accompanies DYNAMIC3 (p.29 mode 4). */
        const uint32_t dyn3 = rv(c, R_CONTROL2) & (1u << 4);
        CHECK_EQ(rv(c, 0x31), dyn3 ? (1u << 5) : 0u);
    }
}

static void t_conversions(void)
{
    for (int r = 0; r < 5; r++) {
        const afe_rate_cfg_t *c = &AFE_RATE_TABLE[r];
        const uint64_t tick_ns = tick_ns_of_field(c->clkdiv_field);

        CHECK_EQ(c->numav, EXP[r].numav);
        CHECK(c->numav <= 15);                         /* 4-bit field       */
        const uint64_t conv_min_ns =
            ((uint64_t)c->numav + 2u) * 200u * T_ADC_NS + 15000u;

        const uint32_t aled1endc = rv(c, R_ALED1ENDC);
        uint32_t prev_end = 0;

        for (int k = 0; k < 4; k++) {
            const uint32_t rst_st = rv(c, RST[k][0]);
            const uint32_t rst_end = rv(c, RST[k][1]);
            const uint32_t cv_st = rv(c, CONV[k][0]);
            const uint32_t cv_end = rv(c, CONV[k][1]);

            CHECK(rst_end != 0);                       /* all four present;
                                                          ADC_RDY needs #4  */
            CHECK(rst_st > aled1endc);                 /* after sampling    */
            CHECK(rst_end >= rst_st);
            CHECK(cv_st > rst_end);                    /* reset precedes conv */
            /* convert window >= (NUMAV+2)*200*tADC + 15 us */
            CHECK((uint64_t)(cv_end - cv_st + 1u) * tick_ns >= conv_min_ns);

            if (k > 0) {
                CHECK(rst_st > prev_end);              /* single ADC: no overlap */
            }
            prev_end = cv_end;
        }
    }
}

static void t_control2(void)
{
    for (int r = 0; r < 5; r++) {
        const afe_rate_cfg_t *c = &AFE_RATE_TABLE[r];
        const uint32_t v = rv(c, R_CONTROL2);

        CHECK(v & C2_OSC_ENABLE);                      /* internal 4 MHz    */
        CHECK(!(v & C2_ILED_2X));                      /* NEVER 2x LED range */
        CHECK(!(v & (C2_PDNAFE | C2_PDNRX)));          /* running config    */
        CHECK_EQ(c->dyn_pd, EXP[r].dyn);
        if (c->dyn_pd) {
            CHECK_EQ(v & C2_DYNAMIC_ALL, C2_DYNAMIC_ALL);
        } else {
            CHECK_EQ(v & C2_DYNAMIC_ALL, 0);
        }
    }
}

static void t_pdn_cycle(void)
{
    for (int r = 0; r < 5; r++) {
        const afe_rate_cfg_t *c = &AFE_RATE_TABLE[r];
        const uint64_t tick_ns = tick_ns_of_field(c->clkdiv_field);
        const uint32_t st = rv(c, R_PDNCYCLESTC);
        const uint32_t end = rv(c, R_PDNCYCLEENDC);

        if (c->dyn_pd) {
            const uint32_t aled1convend = rv(c, CONV[3][1]);
            CHECK(end > st);
            /* >= 200 us after the last conversion, >= 200 us before frame end */
            CHECK(st > aled1convend);
            CHECK((uint64_t)(st - aled1convend) * tick_ns >= PDN_MARGIN_NS);
            CHECK((uint64_t)(c->prpct - end) * tick_ns >= PDN_MARGIN_NS);
        } else {
            CHECK_EQ(st, 0);
            CHECK_EQ(end, 0);
        }
    }
}

static void t_frame_fit(void)
{
    /* Nothing may run past the frame: every count register <= PRPCT, so the
     * measured period is exactly PRPCT+1 ticks (verified against 1/rate in
     * t_clkdiv_prpct_period). 0x39 and 0x23 are the only non-count values. */
    for (int r = 0; r < 5; r++) {
        const afe_rate_cfg_t *c = &AFE_RATE_TABLE[r];
        for (int i = 0; i < c->n_regs; i++) {
            const uint8_t reg = c->regs[i].reg;
            if (reg == R_CLKDIV_PRF || reg == R_CONTROL2) {
                continue;
            }
            CHECK(c->regs[i].val <= c->prpct);
        }
    }
}

int main(void)
{
    TST_RUN(t_table_shape);
    TST_RUN(t_clkdiv_prpct_period);
    TST_RUN(t_windows_ranges);
    TST_RUN(t_sample_phases);
    TST_RUN(t_conversions);
    TST_RUN(t_control2);
    TST_RUN(t_pdn_cycle);
    TST_RUN(t_frame_fit);
    TST_REPORT();
}
