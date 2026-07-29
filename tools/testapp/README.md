# Narbis Edge Earclip — Web Bluetooth PCB-verification app

Single static page, vanilla JS, **no build step, no external resources**.
Walks an operator through the full V2.1 board verification (every net,
every component) over BLE using the TEST opcode block (`0xE0–0xEB`,
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
| 4 | T04 dark noise/ambient (clip closed, dark) | 0xE0 id 4 | values recorded; over-limit = ⓘ INFO (frames missing = FAIL) |
| 5 | T05 crosstalk (clip OPEN, no tissue) | 0xE0 id 5 | values recorded; over-limit = ⓘ INFO (frames missing = FAIL) |
| 6 | Red LED visual (dark 1 s → red 20 mA 3 s → dark) | 0xE1 ×2 | operator confirms the off/red/off pattern |
| 7 | LED I-V sweeps, IR + red | 0xE2 → 0xEA, ×2 | monotonic rise, not flat (flat = open emitter/FFC) |
| 8 | RX sweep (TIA gain + offset DAC) | 0xE3 → 0xEA, ×2 | dc tracks RF, **ohm-ordered** (codes are not monotonic) |
| 9 | ADC_RDY rate count, 10 s @100 sps | 0xE4 (synchronous) | within ±0.5 % of nominal |
| 10 | Accel orientation, 6 faces | 0xE8 + ACCEL stream | all six ±1 g faces captured, 0 FIFO overruns |
| 11 | Button echo, 5 presses | 0xE5 + MARKER events | ≥10 edges, no <5 ms ghosts (TVS/C22) |
| 12 | Charger walk (plug/unplug USB) | 0xE6 polled @2 Hz | ON_BATTERY→CHARGING→ON_BATTERY; **records raw STAT bit per state → resolves the STAT-polarity VERIFY-ON-BENCH item** |
| 13 | Battery raw vs bench meter | 0xE7 | Δ ≤ 50 mV |
| 14 | Live PPG smoke, 60 s @100 sps | production stream | seq-gap counter stays 0 |
| 15 | Sleep current + button wake | 0xE9 | ≤ 80 µA, wakes and re-advertises |

The sequence requires a **serial number** (field next to Start; mirrored
into the dashboard recording box). The report card (footer) records
per-step pass/fail/info/skip, measured value, expectation and operator
note; **Download JSON** produces the manufacturing record — including
the serial, a pass/fail/info/skip summary and the **full session log**
(BLE traffic, errors, disconnects), so a failed unit is diagnosable from
the report alone. **Print** gives a paper copy (print CSS shows the
report only). The dashboard recording zip also packs `log.txt`.

## TEST-payload contract (firmware truth — `test_ops.c` is authoritative)

`proto.h` pins the envelopes; the payload conventions below are what the
firmware actually implements. The mock and this app mirror them exactly
(`node selfcheck.mjs` locks them). History note: an earlier revision of
this section listed *assumptions* (JSON report blob, 0xE6 event stream,
async 0xE4) that the firmware deliberately did **not** adopt — the app
has been reconciled to the firmware, not the other way around.

1. **0xE0 response payload** = one 10-byte self-test record
   `{u8 id, u8 status, i32 value, i32 threshold}` (`NC_ST_REC_SIZE`).
   Running any 0xE0 clears a previously published sweep blob.
2. **0xE4 is synchronous**: sys_task blocks for the count window (bench
   behavior by design) and the single response carries
   `{u32 pulses, u32 elapsed_ms}`. Give it `window + ~8 s` of timeout;
   the 1 Hz STATUS heartbeat pauses while it runs.
3. **0xE6 is a stateless polled snapshot**: each request answers
   `{u8 vusb, u8 stat_raw, u8 decoded_state}`; the legacy enable byte is
   accepted and ignored. Nothing rides EVENT_STREAM. The app polls at
   2 Hz during the charger walk.
4. **0xEA report blob** (chunked `{u16 total, u16 off, u8 n, bytes}`) is
   **binary, blob ver 2**: `[u8 2][u8 kind: 1 LED, 2 RX][u8 param]
   [u8 n][n × {u8 setting, i32 ir, i32 red, i32 amb}]` little-endian,
   and holds **only the most recent sweep** — fetch after *each*
   0xE2/0xE3. For the offset-DAC sweep, `setting` is an int8 cast to u8.
   With no sweep published it serves the T01..T08 selftest blob (ver 1).
5. Responses to TEST opcodes echo the opcode **unchanged** (bit7 is
   already set); correlation is by tid alone — per the proto.h comment.
6. **TIA RF codes are not ohm-ordered** (0..7 = 500k, 250k, 100k, 50k,
   25k, 10k, 1M, 2M): any "gain rises" analysis must re-order by ohms
   (`NarbisAnalysis.gainSweepVerdict`).
7. **Optical limits are informational** at this stage (per the vendor
   instructions): T04/T05 over-threshold results render as ⓘ INFO and do
   not gate shipment; only mechanical failures (frames missing, AFE
   bring-up error) hard-fail those steps.

## Self-test (CI)

```sh
python tools/goldens/gen_proto_consts.py --check   # headers ↔ JS ↔ python agree
python -m pytest tools/tests -q                    # includes test_proto_consts.py
node tools/testapp/selfcheck.mjs                   # codecs + full mock walk, headless
```
