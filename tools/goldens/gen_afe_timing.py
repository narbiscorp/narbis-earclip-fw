#!/usr/bin/env python3
"""gen_afe_timing.py - AFE4404 (TI SBAS689D) timing-engine table generator.

Emits firmware/main/afe4404_timing.inc: one auditable {reg,val} table per
nc_rate_t index (0..4 = 50/100/200/250/500 sps) plus AFE_RATE_TABLE[5].
Every emitted register line carries its derivation (ticks and microseconds)
so the tables can be reviewed against the datasheet without re-running this
script.

Datasheet constraints enforced (SBAS689D):
  - CLKDIV_PRF (0x39 bits[2:0]) NON-MONOTONIC encoding:
      0 -> /1, 4 -> /2, 5 -> /4, 6 -> /8, 7 -> /16; codes 1,2,3 FORBIDDEN.
    tick = DIV / 4 MHz (internal oscillator).
  - PRPCT (0x1D bits[15:0]) = fTE/PRF - 1, must fit 16 bits.
  - Window duration = (END - ST + 1) ticks; all counts in [0, PRPCT].
  - Convert window >= (NUMAV+2)*200*tADC + 15 us, tADC = 250 ns.
  - All four ADCRSTx windows defined (ADC_RDY is generated from the 4th).
  - Dynamic power-down (50/100 sps only): PDNCYCLE starts >= 200 us after
    ALED1CONVEND and ends >= 200 us before frame end; CONTROL2 (0x23)
    DYNAMIC1(b20)/DYNAMIC2(b14)/DYNAMIC3(b4)/DYNAMIC4(b3) set.
  - CONTROL2 base: OSC_ENABLE(b9)=1; ILED_2X(b17)=0 ALWAYS (SFH 7016 red die
    DC abs-max + 3.3 V TX_SUP headroom); PDNAFE(b0)/PDNRX(b1)=0 in run.
  - LED3 unused: 0x36/0x37 written 0.

Usage:
  python gen_afe_timing.py            regenerate firmware/main/afe4404_timing.inc
  python gen_afe_timing.py --check    re-derive + compare against committed file;
                                      nonzero exit on any assertion or mismatch
"""

import argparse
import sys
from pathlib import Path

# ---------------------------------------------------------------- constants
F_OSC_HZ      = 4_000_000   # internal oscillator (CONTROL2 OSC_ENABLE)
TICK0_NS      = 250         # 1 / 4 MHz
T_ADC_NS      = 250         # tADC per SBAS689D
CONV_FIXED_NS = 15_000      # fixed 15 us term of the convert-window minimum
SAMPLE_NS     = 100_000     # every sample window is exactly 100 us

# Layout choices (documented in the emitted header):
LED_LEAD_TICKS    = 20      # LED on -> sample start (LED + TIA settling)
PHASE_GAP_TICKS   = 4       # gap between sample phases
CONV_LEADIN_TICKS = 2       # ALED1ENDC -> first ADC reset
ADCRST_TICKS      = 6       # ADC reset window width
PDN_MARGIN_NS     = 200_000 # PDNCYCLE margin each side (datasheet minimum)
PDN_PAD_TICKS     = 4       # extra ticks on top of the 200 us margins

CLKDIV_FIELD = {1: 0, 2: 4, 4: 5, 8: 6, 16: 7}   # divide -> register code
FORBIDDEN_FIELDS = {1, 2, 3}

# Index order == nc_rate_t (nc_types.h): 50, 100, 200, 250, 500.
# NUMAV: averages = NUMAV+1; 16 averages at <=250 sps for SNR, 6 at 500 sps
# (the (NUMAV+2)*200*tADC+15us convert minimum no longer fits 4x otherwise).
# 50 sps MUST use /2: at /1 PRPCT would be 79999 (16-bit overflow) and the
# timing engine has a 61 Hz floor at /1.
RATES = [
    dict(sps=50,  div=2, numav=15, dyn_pd=True),
    dict(sps=100, div=1, numav=15, dyn_pd=True),
    dict(sps=200, div=1, numav=15, dyn_pd=False),
    dict(sps=250, div=1, numav=15, dyn_pd=False),
    dict(sps=500, div=1, numav=5,  dyn_pd=False),
]
NC_RATE_NAMES = ["NC_RATE_50", "NC_RATE_100", "NC_RATE_200",
                 "NC_RATE_250", "NC_RATE_500"]

DEFAULT_OUT = Path(__file__).resolve().parents[2] / "firmware" / "main" / "afe4404_timing.inc"

# CONTROL2 (0x23) bits
C2_OSC_ENABLE = 1 << 9
C2_DYNAMIC    = (1 << 20) | (1 << 14) | (1 << 4) | (1 << 3)  # DYNAMIC1..4


class GenError(Exception):
    pass


def req(cond, msg):
    if not cond:
        raise GenError(msg)


def us(ticks, tick_ns):
    return f"{ticks * tick_ns / 1000:.2f}"


# ---------------------------------------------------------------- derivation
def build_rate(spec):
    sps, div, numav, dyn = spec["sps"], spec["div"], spec["numav"], spec["dyn_pd"]

    req(div in CLKDIV_FIELD, f"{sps} sps: no CLKDIV code for /{div}")
    field = CLKDIV_FIELD[div]
    req(field not in FORBIDDEN_FIELDS, f"{sps} sps: forbidden CLKDIV code {field}")
    tick_ns = TICK0_NS * div
    fte = F_OSC_HZ // div
    req(F_OSC_HZ % div == 0, f"{sps} sps: non-integer timing clock")
    req(fte % sps == 0, f"{sps} sps: fTE/PRF not integer")
    prpct = fte // sps - 1
    req(1 <= prpct <= 0xFFFF, f"{sps} sps: PRPCT {prpct} outside 16-bit range")
    if sps == 50:
        req(div == 2, "50 sps must use CLKDIV /2 (PRPCT overflow + 61 Hz floor at /1)")
    period_ns = (prpct + 1) * tick_ns
    req(period_ns * sps == 1_000_000_000, f"{sps} sps: period not exact")
    period_us = period_ns // 1000
    req(period_us * 1000 == period_ns, f"{sps} sps: period not whole microseconds")

    req(0 <= numav <= 15, f"{sps} sps: NUMAV {numav} outside 4-bit field")
    req(numav == (5 if sps == 500 else 15), f"{sps} sps: unexpected NUMAV {numav}")

    # Sample phases, order per datasheet default: RED(LED2), ambient-2,
    # IR(LED1), ambient-1. Uniform slot pitch; LED drive leads its sample
    # window by LED_LEAD_TICKS and both end together.
    s_ticks = SAMPLE_NS // tick_ns
    req(s_ticks * tick_ns == SAMPLE_NS, f"{sps} sps: 100 us not integer ticks")
    pitch = LED_LEAD_TICKS + s_ticks + PHASE_GAP_TICKS
    base = [k * pitch for k in range(4)]
    smp = [(b + LED_LEAD_TICKS, b + LED_LEAD_TICKS + s_ticks - 1) for b in base]
    led2_drv = (base[0], smp[0][1])   # RED drive brackets slot 0 sample
    led1_drv = (base[2], smp[2][1])   # IR drive brackets slot 2 sample
    # LED drive must never overlap another phase's sample window (would
    # corrupt the ambient measurements).
    req(led2_drv[1] < smp[1][0], f"{sps} sps: RED drive overlaps ambient-2 sample")
    req(led1_drv[0] > smp[1][1] and led1_drv[1] < smp[3][0],
        f"{sps} sps: IR drive overlaps an ambient sample")

    # Conversions: back-to-back after sampling, each preceded by its 6-tick
    # ADCRSTx. Convert window minimum: (NUMAV+2)*200*tADC + 15 us.
    conv_min_ns = (numav + 2) * 200 * T_ADC_NS + CONV_FIXED_NS
    c_ticks = -(-conv_min_ns // tick_ns)          # ceil division
    req(c_ticks * tick_ns >= conv_min_ns, f"{sps} sps: convert window arithmetic")

    t = smp[3][1] + CONV_LEADIN_TICKS
    rst, conv = [], []
    for _ in range(4):
        rst.append((t, t + ADCRST_TICKS - 1))
        conv.append((t + ADCRST_TICKS, t + ADCRST_TICKS + c_ticks - 1))
        t = conv[-1][1] + 1
    for k in range(4):
        req(rst[k][1] > 0, f"{sps} sps: ADCRST{k} END is zero")
        req(conv[k][0] == rst[k][1] + 1, f"{sps} sps: conv {k} not after its reset")
        req((conv[k][1] - conv[k][0] + 1) * tick_ns >= conv_min_ns,
            f"{sps} sps: convert window {k} too short")
        if k:
            req(rst[k][0] > conv[k - 1][1], f"{sps} sps: conversions overlap at {k}")

    # Dynamic power-down window (50/100 sps only).
    if dyn:
        margin = PDN_MARGIN_NS // tick_ns + PDN_PAD_TICKS
        pdn = (conv[3][1] + margin, prpct - margin)
        req(pdn[1] > pdn[0], f"{sps} sps: PDNCYCLE window empty")
        req((pdn[0] - conv[3][1]) * tick_ns >= PDN_MARGIN_NS,
            f"{sps} sps: PDNCYCLE start margin < 200 us")
        req((prpct - pdn[1]) * tick_ns >= PDN_MARGIN_NS,
            f"{sps} sps: PDNCYCLE end margin < 200 us")
    else:
        pdn = (0, 0)

    last_used = pdn[1] if dyn else conv[3][1]
    req(last_used <= prpct, f"{sps} sps: frame overflows PRPCT ({last_used} > {prpct})")

    control2 = C2_OSC_ENABLE | (C2_DYNAMIC if dyn else 0)

    # ------------------------------------------------------------ reg list
    regs = []

    def w(addr, val, comment):
        req(0 <= val <= 0xFFFFFF, f"{sps} sps: reg 0x{addr:02X} value overflow")
        regs.append((addr, val, comment))

    def win(st_addr, end_addr, st_name, end_name, wnd, what, extra=""):
        st, end = wnd
        dur = end - st + 1
        w(st_addr, st,
          f"{st_name:<12} = {st:5d} ({us(st, tick_ns):>9} us)  {what} start")
        w(end_addr, end,
          f"{end_name:<12} = {end:5d} ({us(end, tick_ns):>9} us)  {what} end: "
          f"{dur} ticks = {us(dur, tick_ns)} us{extra}")

    w(0x39, field,
      f"CLKDIV_PRF   = {field}: divide-by-{div}, tick = {tick_ns} ns "
      f"(codes 1,2,3 forbidden)")
    win(0x01, 0x02, "LED2STC", "LED2ENDC", smp[0], "RED sample")
    win(0x03, 0x04, "LED1LEDSTC", "LED1LEDENDC", led1_drv, "IR LED drive",
        extra=f" (leads IR sample by {LED_LEAD_TICKS} ticks)")
    win(0x05, 0x06, "ALED2STC", "ALED2ENDC", smp[1], "ambient-2 sample")
    win(0x07, 0x08, "LED1STC", "LED1ENDC", smp[2], "IR sample")
    win(0x09, 0x0A, "LED2LEDSTC", "LED2LEDENDC", led2_drv, "RED LED drive",
        extra=f" (leads RED sample by {LED_LEAD_TICKS} ticks)")
    win(0x0B, 0x0C, "ALED1STC", "ALED1ENDC", smp[3], "ambient-1 sample")
    conv_extra = (f" >= (NUMAV+2)*200*tADC+15us = ({numav}+2)*200*{T_ADC_NS}ns+15us"
                  f" = {conv_min_ns / 1000:.2f} us")
    win(0x0D, 0x0E, "LED2CONVST", "LED2CONVEND", conv[0], "RED convert", extra=conv_extra)
    win(0x0F, 0x10, "ALED2CONVST", "ALED2CONVEND", conv[1], "ambient-2 convert", extra=conv_extra)
    win(0x11, 0x12, "LED1CONVST", "LED1CONVEND", conv[2], "IR convert", extra=conv_extra)
    win(0x13, 0x14, "ALED1CONVST", "ALED1CONVEND", conv[3], "ambient-1 convert",
        extra=conv_extra + "; ADC_RDY pulses here")
    win(0x15, 0x16, "ADCRSTSTCT0", "ADCRSTENDCT0", rst[0], "ADC reset 0 (pre RED conv)")
    win(0x17, 0x18, "ADCRSTSTCT1", "ADCRSTENDCT1", rst[1], "ADC reset 1 (pre amb-2 conv)")
    win(0x19, 0x1A, "ADCRSTSTCT2", "ADCRSTENDCT2", rst[2], "ADC reset 2 (pre IR conv)")
    win(0x1B, 0x1C, "ADCRSTSTCT3", "ADCRSTENDCT3", rst[3], "ADC reset 3 (pre amb-1 conv)")
    w(0x1D, prpct,
      f"PRPCT        = {prpct}: frame = {prpct + 1} ticks = {period_us} us -> {sps} sps")
    if dyn:
        win(0x32, 0x33, "PDNCYCLESTC", "PDNCYCLEENDC", pdn, "dynamic PWDN",
            extra=(f"; margins {us(pdn[0] - conv[3][1], tick_ns)} us after ALED1CONVEND,"
                   f" {us(prpct - pdn[1], tick_ns)} us before frame end (>= 200 us each)"))
    else:
        w(0x32, 0, "PDNCYCLESTC  =     0: dynamic PWDN unused at this rate")
        w(0x33, 0, "PDNCYCLEENDC =     0: dynamic PWDN unused at this rate")
    w(0x36, 0, "LED3LEDSTC   =     0: LED3/TX3 unused on this board")
    w(0x37, 0, "LED3LEDENDC  =     0: LED3/TX3 unused on this board")
    w(0x23, control2,
      "CONTROL2: OSC_ENABLE(b9)"
      + (" | DYNAMIC1(b20)|DYNAMIC2(b14)|DYNAMIC3(b4)|DYNAMIC4(b3)" if dyn else "")
      + "; ILED_2X(b17)=0 ALWAYS, PDNAFE(b0)=PDNRX(b1)=0")

    # Final cross-checks over the emitted list.
    seen = set()
    for addr, val, _ in regs:
        req(addr not in seen, f"{sps} sps: duplicate reg 0x{addr:02X}")
        seen.add(addr)
        if addr not in (0x39, 0x23):
            req(val <= prpct, f"{sps} sps: reg 0x{addr:02X} count {val} > PRPCT")

    return dict(sps=sps, div=div, field=field, tick_ns=tick_ns, prpct=prpct,
                numav=numav, dyn=dyn, period_us=period_us, regs=regs,
                conv_min_ns=conv_min_ns, c_ticks=c_ticks, s_ticks=s_ticks)


# ------------------------------------------------------------------ emission
def generate():
    built = [build_rate(r) for r in RATES]
    req([b["sps"] for b in built] == [50, 100, 200, 250, 500],
        "rate order must match nc_rate_t")

    L = []
    L.append("/*")
    L.append(" * afe4404_timing.inc - GENERATED by tools/goldens/gen_afe_timing.py."
             "  DO NOT EDIT.")
    L.append(" *")
    L.append(" * AFE4404 (TI SBAS689D) timing-engine register tables, one per")
    L.append(" * nc_rate_t index (0..4 = 50/100/200/250/500 sps). Include AFTER the")
    L.append(" * afe_regval_t / afe_rate_cfg_t typedefs (firmware/main/afe4404_regs.h).")
    L.append(" * Data + comments only: no typedefs, no preprocessor.")
    L.append(" *")
    L.append(" * Construction (window duration = END - ST + 1 ticks; tick = DIV/4MHz):")
    L.append(" *  - 4 sample phases in datasheet default order RED(LED2), ambient-2,")
    L.append(f" *    IR(LED1), ambient-1; each sample window exactly 100 us; LED drive")
    L.append(f" *    leads its sample window by {LED_LEAD_TICKS} ticks (LED + TIA settling);")
    L.append(f" *    {PHASE_GAP_TICKS}-tick gaps between phases.")
    L.append(" *  - 4 conversions back-to-back after sampling, each preceded by its")
    L.append(f" *    {ADCRST_TICKS}-tick ADCRSTx window; convert >= (NUMAV+2)*200*tADC + 15 us")
    L.append(" *    (tADC = 250 ns); ADC_RDY is generated from ALED1CONVEND (4th).")
    L.append(" *  - Dynamic power-down at 50/100 sps only: PDNCYCLE >= 200 us after")
    L.append(" *    ALED1CONVEND and >= 200 us before frame end; CONTROL2 DYNAMIC1..4.")
    L.append(" *  - 50 sps uses CLKDIV /2 (16-bit PRPCT overflow and 61 Hz engine")
    L.append(" *    floor at /1).")
    L.append(" *")
    L.append(" * Regenerate: python tools/goldens/gen_afe_timing.py")
    L.append(" * Verify:     python tools/goldens/gen_afe_timing.py --check")
    L.append(" */")
    L.append("")

    for b in built:
        avg = b["numav"] + 1
        L.append("/* -------------------------------------------------------------------- *")
        L.append(f" * {b['sps']} sps: CLKDIV /{b['div']} (field {b['field']}), "
                 f"tick {b['tick_ns']} ns, PRPCT {b['prpct']} "
                 f"(frame {b['prpct'] + 1} ticks = {b['period_us']} us)")
        L.append(f" * NUMAV {b['numav']} ({avg} averages) -> convert >= "
                 f"{b['conv_min_ns'] / 1000:.2f} us = {b['c_ticks']} ticks; "
                 f"sample 100 us = {b['s_ticks']} ticks")
        L.append(f" * dynamic power-down: {'ON' if b['dyn'] else 'off'}")
        L.append(" * -------------------------------------------------------------------- */")
        L.append(f"static const afe_regval_t afe_regs_{b['sps']}[] = {{")
        for addr, val, comment in b["regs"]:
            L.append(f"    {{ 0x{addr:02X}, 0x{val:06X} }}, /* {comment} */")
        L.append("};")
        L.append("")

    L.append("/* Indexed by nc_rate_t. NUMAV is applied by the driver via CONTROL1")
    L.append(" * (0x1E) = TIMEREN(b8) | NUMAV[3:0], written LAST after the table. */")
    L.append("static const afe_rate_cfg_t AFE_RATE_TABLE[5] = {")
    for i, b in enumerate(built):
        L.append(f"    /* [{i}] {NC_RATE_NAMES[i]:<11} */ "
                 f"{{ {b['sps']:3d}, {b['field']}, {b['prpct']:5d}, {b['numav']:2d}, "
                 f"{b['period_us']:5d}, {'true, ' if b['dyn'] else 'false,'} "
                 f"afe_regs_{b['sps']}, {len(b['regs'])} }},")
    L.append("};")
    L.append("")
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description="AFE4404 timing table generator")
    ap.add_argument("--check", action="store_true",
                    help="re-derive and compare against the committed .inc")
    ap.add_argument("-o", "--out", default=None, help="output path override")
    args = ap.parse_args()

    try:
        text = generate()
    except GenError as e:
        print(f"gen_afe_timing: DERIVATION FAIL: {e}", file=sys.stderr)
        return 1

    out = Path(args.out) if args.out else DEFAULT_OUT
    if args.check:
        if not out.exists():
            print(f"gen_afe_timing: CHECK FAIL: {out} missing", file=sys.stderr)
            return 1
        if out.read_text() != text:
            print(f"gen_afe_timing: CHECK FAIL: {out} does not match derivation "
                  f"(regenerate with: python {Path(__file__).name})", file=sys.stderr)
            return 1
        print("gen_afe_timing: check OK (derivation + committed file agree)")
        return 0

    out.write_text(text, newline="\n")
    print(f"gen_afe_timing: wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
