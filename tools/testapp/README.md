# Narbis Edge Earclip — Web Bluetooth PCB-verification app

Single static page, vanilla JS, **no build step, no external resources**.
Walks an operator through the full V2.1 board verification (every net,
every component) over BLE using the TEST opcode block (`0xE0–0xEA`,
firmware built with `NARBIS_TEST_MODE 1` in `firmware/main/board.h`).

## Running it

| Scenario | How |
|---|---|
| No hardware (demo / self-test) | open `index.html?mock=1` — works from `file://` |
| Bench, this PC | `python -m http.server 8000` in this directory → `http://localhost:8000` |
| Bench, Android phone/tablet | serve over **https** (any static host, or `ngrok`/`caddy`); Web Bluetooth requires a secure context |

Browsers: **Chrome or Edge**, desktop (Windows/macOS/Linux) and Android.
**iOS/iPadOS is unsupported** — Safari has no Web Bluetooth and Apple ships
no alternative; use the python client (`tools/narbis_client`, bleak) for
the same operations there.

Files: `index.html` + `style.css` (UI), `proto_consts.js` (**generated** by
`tools/goldens/gen_proto_consts.py` from `proto.h`/`nc_types.h`/`knob_list.h`
— never edit by hand, regenerate after any header change), `proto.js`
(DataView codecs, no magic numbers), `app.js` (connect flow + guided
sequence + report card), `mock.js` (scripted fake device).

## Mock mode

`?mock=1` swaps `navigator.bluetooth` for a scripted in-page device that
walks the entire sequence with realistic values, **including one
deliberate failure** (T05 crosstalk: 81 234 counts > 50 000 threshold) so
FAIL rendering and the report card are exercised. All bench waits are
compressed by `?timescale=` (default 0.15, so the 60 s PPG soak takes 9 s).
This doubles as the app's no-hardware self-test; `node selfcheck.mjs`
drives the same codecs + mock device headlessly (no DOM) in CI.

## The guided sequence

Steps 1–3 run automatically; the rest prompt the operator as needed.

| # | Step | Op | Pass criterion |
|---|---|---|---|
| 1 | T01 I2C bus scan | 0xE0 id 1 | firmware PASS record |
| 2 | T02 WHO_AM_I ×2 | 0xE0 id 2 | firmware PASS record |
| 3 | T03 AFE register R/W | 0xE0 id 3 | firmware PASS record |
| 4 | T04 dark noise/ambient (clip closed, dark) | 0xE0 id 4 | value ≤ threshold |
| 5 | T05 crosstalk (clip OPEN, no tissue) | 0xE0 id 5 | value ≤ threshold |
| 6 | Red LED visual (20 mA, 3 s) | 0xE1 | operator confirms glow |
| 7 | LED I-V sweeps, IR + red | 0xE2 ×2 → 0xEA | monotonic rise, not flat (flat = open emitter/FFC) |
| 8 | RX sweep (TIA gain + offset DAC) | 0xE3 ×2 → 0xEA | dc rises with gain code |
| 9 | ADC_RDY rate count, 10 s @100 sps | 0xE4 | within ±0.5 % of nominal |
| 10 | Accel orientation, 6 faces | 0xE8 + ACCEL stream | all six ±1 g faces captured, 0 FIFO overruns |
| 11 | Button echo, 5 presses | 0xE5 + MARKER events | ≥10 edges, no <5 ms ghosts (TVS/C22) |
| 12 | Charger walk (plug/unplug USB) | 0xE6 snapshots @2 Hz | ON_BATTERY→CHARGING→ON_BATTERY; **records raw STAT bit per state → resolves the STAT-polarity VERIFY-ON-BENCH item** |
| 13 | Battery raw vs bench meter | 0xE7 | Δ ≤ 50 mV |
| 14 | Live PPG smoke, 60 s @100 sps | production stream | seq-gap counter stays 0 |
| 15 | Sleep current + button wake | 0xE9 | ≤ 80 µA, wakes and re-advertises |

The report card (footer) records per-step pass/fail/skip, measured value,
expectation and operator note; **Download JSON** produces the
manufacturing record, **Print** gives a paper copy (print CSS shows the
report only).

## Interface assumptions pending firmware (test-mode agent)

`proto.h` pins the envelopes but not every TEST payload; the mock (and
this app) implement the following conventions, which the firmware TEST
handlers must match — flag any deviation back to this app:

1. **0xE0 response payload** = one 10-byte self-test record
   `{u8 id, u8 status, i32 value, i32 threshold}` (`NC_ST_REC_SIZE`).
2. **0xE4** acks immediately (`ST_OK`, empty payload), then sends a
   *second indication with the same op and tid* carrying
   `{u32 pulses, u32 elapsed_ms}` when the count window closes.
3. **0xE6 snapshots** ride EVENT_STREAM as records of type `0xE6` (the
   opcode value), payload `{u64 t_us, u8 vusb, u8 stat_raw, u8 chg_state}`
   (len 11). Unknown-type records are skippable by contract, so
   production clients ignore them.
4. **0xEA report blob** (chunked `{u16 total, u16 off, u8 n, bytes}`) is
   UTF-8 JSON with a `tests` object containing at least
   `led_sweep_ir`/`led_sweep_red` (`{ma:[], dc:[]}`) and `rx_sweep_gain`
   /`rx_sweep_dac` (`{code:[], dc:[]}`).
5. Responses to TEST opcodes echo the opcode **unchanged** (bit7 is
   already set); correlation is by tid alone — per the proto.h comment.

## Self-test (CI)

```sh
python tools/goldens/gen_proto_consts.py --check   # headers ↔ JS ↔ python agree
python -m pytest tools/tests -q                    # includes test_proto_consts.py
node tools/testapp/selfcheck.mjs                   # codecs + full mock walk, headless
```
