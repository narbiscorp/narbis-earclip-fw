# Narbis Edge Earclip — Firmware

Production firmware for the Narbis Edge Earclip: a clip-on earlobe transmissive
PPG sensor (XIAO ESP32-C6 + TI AFE4404 + VEMD8081 photodiode + SFH 7016 red/IR
LED + LIS2DH12 accelerometer, V2.1 boards). Streams raw PPG + accel over BLE,
computes beats (IBI) on-device, with per-LED AGC, artifact gating, wear
detection, OTA, and a knob registry that exposes every tunable over BLE and USB.

Requirements: `firmware_handoff.md` + `firmware_handoff_addendum_1.md`
(addendum wins on conflict) in the parent folder. Hardware is **frozen**;
firmware adapts to it, never the reverse.

## Repo layout

```
firmware/                      ESP-IDF v5.5.1 project (esp32c6, NimBLE, FreeRTOS)
  components/narbis_core/      pure C11, no ESP-IDF — dual-compiled for host tests
    include/narbis/proto.h       THE wire contract (all BLE bytes)
    include/narbis/knob_list.h   THE tunables (62-knob X-macro registry)
  main/                        drivers, tasks, BLE, power, console (ESP-specific)
    board.h                      pin map + NARBIS_TEST_MODE switch
test_host/                     host unit tests (mingw gcc), run_tests.sh
tools/
  narbis_client/               python client (bleak): proto.py is the ONE parser
  scripts/                     narbis-record / -plot / -knobs / -ota / -selftest
  goldens/                     generators: AFE timing tables, DSP coeffs, vectors
  acceptance/                  bench-day DoD scripts
  testapp/                     Web Bluetooth PCB-verification app (see Test mode)
```

> **First flash / OTA / recovery:** see [PROGRAMMING.md](PROGRAMMING.md).

## Building

```powershell
. C:\Espressif\frameworks\esp-idf-v5.5.1\export.ps1
cd firmware
idf.py set-target esp32c6   # first time only
idf.py build                # idf.py -p COMx flash monitor once hardware exists
```

Host tests (no hardware needed — mingw gcc):

```bash
bash test_host/run_tests.sh        # 13+ suites: codecs, DSP (bit-exact), IBI, AGC…
python -m pytest tools/tests -q    # python codec mirror, cross-language fixtures
```

The DSP golden vectors and AFE timing tables are **generated and committed**
(`tools/goldens/gen_*.py`); regenerating must produce identical files (the
timing test re-derives every register and fails on drift).

## Architecture (one paragraph)

`afe_task` (prio 23) is the sole producer of `ppg_queue`: the ADC_RDY ISR
timestamps each frame (`esp_timer`, in the ISR — constant latency), the task
reads the AFE data registers and pushes raw untouched samples. `dsp_task` (19)
consumes them: ambient subtract → DC tracker → Q30 band-pass → IBI detector →
artifact gate → wear, plus slow ticks that evaluate AGC. All *decisions* are
pure narbis_core code; all *actuation* (AFE config I2C, state transitions,
CONTROL responses, events, STATUS) serializes through `sys_task` (12).
`ble_tx_task` (15) drains per-channel staging rings with drop-oldest + counters
so producers never block. Acquisition is subscription-gated: sensors run only
while a central is connected AND subscribed (PPG/IBI/HRS start the PPG engine;
ACCEL or gating starts the accelerometer); the battery service is the only
always-on data.

## BLE protocol

Everything is defined in `firmware/components/narbis_core/include/narbis/proto.h`
and mirrored byte-exactly by `tools/narbis_client/proto.py` and
`tools/testapp/proto.js` (cross-verified fixtures). Services: Battery 0x180F,
Device Info 0x180A, Heart Rate 0x180D (RR intervals from the IBI engine),
Narbis Sensor Service (PPG / ACCEL / IBI / EVENT streams, STATUS, CONTROL
write+indicate, PROTOCOL_VERSION), OTA service. Name "Narbis Edge Earclip";
128-bit service UUID in the ADV payload, full name in the scan response;
2M PHY + DLE + MTU 247 requested after connect. Bonding is LE Secure
Connections; pairing only during the button double-press window
(`open_pairing` knob for dev builds).

Every AGC step, gate span, wear change, marker press, and rate change is
emitted **in-band** on EVENT_STREAM so downstream can reconcile amplitude
discontinuities. Raw samples are never dropped by gating — it only annotates
and suppresses IBI.

## Knobs

Every tunable lives in `knob_list.h` (id, type, range, default, unit, flags)
and is get/set/save/discoverable over CONTROL and the USB console — the app
builds its tuning UI from the discovery op, so new knobs need no app update.
NVS stores only deltas from defaults, written on explicit SAVE.

## Test mode (PCB design verification)

`#define NARBIS_TEST_MODE 1` in `firmware/main/board.h` (one line; default 0 —
production compiles all of it out). Adds " TEST" to the BLE name, forces open
pairing, disables auto-sleep, and unlocks TEST opcodes 0xE0–0xEA (single
self-tests, LED direct drive + I-V sweeps, RX sweeps, ADC_RDY rate counter,
button echo, charger walk, battery raw, accel live, sleep-now, report fetch).

`tools/testapp/index.html` is the matching operator UI: a static Web Bluetooth
page (Chrome/Edge desktop + Android; not iOS) that walks the full guided
verification sequence with live plots and produces a downloadable pass/fail
report card per board. `index.html?mock=1` runs the entire flow against a
scripted fake device — the app's own no-hardware self-test.

## Python tools

```bash
pip install -e tools[dev,viz,data]
narbis-record --out session1/          # record all streams to CSV/parquet
narbis-plot                            # live waveform + IBI + gate spans
narbis-knobs list|get|set|save|diff
narbis-ota firmware/build/narbis_earclip.bin [--loop 20]
narbis-selftest
```

---

# Hardware bring-up checklist (bench day — boards arrive from fab)

Work top to bottom; each line has a box and a place for the measured number.
Firmware side is code-complete; items marked **KNOB** end with writing the
bench-derived value into `knob_list.h` defaults (then SAVE + commit).

## 0. Smoke + flash
- [ ] Visual + shorts check; cell disconnected: bench-supply 3.7 V on VBAT_CELL, current < 5 mA idle
- [ ] `idf.py -p COMx flash monitor` over the XIAO USB-C — boot banner, no panic loop
- [ ] Console up (`ver`, `stats`): reset reason sane

## 1. M0 — devices alive
- [ ] `selftest` T01: I2C scan finds 0x58 + 0x18 (if 0x19 appears, SA0 strap issue — log says so loudly)
- [ ] T02 WHO_AM_I 0x33; T03 AFE reg readback
- [ ] ADC_RDY pulses at 100 sps (test op 0xE4 or scope on GPIO16): ______ Hz (nominal 100, ±1% osc)

## 2. M1 — raw streaming
- [ ] Web test app or `narbis-record`: PPG stream at 100 sps, finger on sensor → pulsatile IR/red
- [ ] Rate accuracy at all five rates (`tools/acceptance/at_rate_accuracy.py`): 50 ____ 100 ____ 200 ____ 250 ____ 500 ____ (±0.5%)
- [ ] 30 min gap-free stream (`at_stream_integrity.py`): seq gaps = 0
- [ ] Accel orientation: 1 g on the correct axis in all six faces (test app step 10)

## 3. M2 — power + charger  ⚠ several VERIFY-ON-BENCH items resolve here
- [ ] **STAT polarity** (test app step 12 / `charger.c` comment): plug USB while watching raw STAT — confirm truth table; if inverted, flip `charger_decode_stat()` (one line)
- [ ] Charge current ≈ 50 mA into cell; battery mV vs bench meter across 3.4–4.2 V: max error ______ mV (≤50)
- [ ] Deep sleep current at cell (test op 0xE9): ______ µA (target ≤ 80; line items below)
- [ ] Button wake after ≥1 h sleep (GPIO2 LP pull-up retention proof) — works? ______
- [ ] 1000 sleep/wake cycles script: hangs = 0; ghost wakes with TVS tap test = 0
- [ ] Light-sleep IDLE advertising average current: ______ mA (target ≤1.5). If NimBLE+light-sleep is unstable, set `POWER_ALLOW_LIGHT_SLEEP 0` (power.c) and re-measure.
- [ ] Streaming 100 sps system current: ______ mA (≤9) · 500 sps: ______ mA (≤12)

| OFF current line item | budget | measured |
|---|---|---|
| R1 100 k hold-low (V2.1 only) | 33 µA | |
| AFE hardware PWDN | ~8 µA | |
| C6 deep sleep | ~7 µA | |
| XIAO LDO quiescent | ? (dominant unknown) | |
| LM66100 | ~10 µA | |
| Battery divider | ~2 µA | |
| **Total** | **≤80 µA** | |

## 4. M3 — AGC on a real earlobe
- [ ] Clip on ear: both channels converge into 50±15% FS band < 5 s; record converged operating point: IR ____ mA, red ____ mA, RF code ____ → consider better `knob_list.h` starting defaults **KNOB**
- [ ] 10 min wear: no AGC hunting (EVENT_STREAM shows no step ping-pong)
- [ ] Every amplitude step paired with an EVENT record (recorder cross-check)

## 5. M4 — signal intelligence  (all thresholds are KNOBs to calibrate)
- [ ] `sat_pct` sanity: observe real near-rail codes, adjust if the ADC range differs from 2^21 assumption **KNOB**
- [ ] `gate_acc_thr`: record quiet vs walking vs shake sessions; set threshold between quiet P99 and motion P1 **KNOB**
- [ ] Shake test: gate spans present, zero IBIs inside them; clean wear gate duty < 2%
- [ ] `wear_on_thr`/`wear_off_thr`/`wear_dark_thr` from: open-clip DC ______, off-body-dark DC ______, on-ear DC range ______–______ **KNOB**
- [ ] IBI vs reference (pulse ox / ECG + `at_ibi_bench.py`): detection ______% (≥98), duplicates 0, clean-segment error ≤ ±1 sample @100 sps
- [ ] Optical self-test floors from 5+ boards: dark noise RMS ______, dark ambient ______, open-clip crosstalk ______ → `st_dark_noise_max`, `st_dark_amb_max`, `st_xtalk_max` **KNOB** (crosstalk validates the light-shield work in manufacturing)

## 6. M5/M6 — productization + soak
- [ ] OTA over BLE ×20 (`narbis-ota --loop 20`): success 20/20; rollback proven once with a self-check-failing build
- [ ] Bonding: pairing refused outside window; bonded reconnect after supervision timeout resumes cleanly
- [ ] Mains flicker: LED-room and fluorescent lighting at each rate — no beat-through in spectra (adjust NUMAV table/notch knob if seen)
- [ ] Streaming-while-charging: noise characterization with USB bit flagged; decide if `stream_on_usb` default stands
- [ ] 24 h soak: no watchdogs, heap stable (`stats` before/after), drift ppm reported vs host ______
- [ ] Time sync: 5× exchange RTT ______ ms; drift ppm ______ (record in session.json)
- [ ] Update this README's measured-actuals; retune knob defaults; tag v1.0.0

## Known deferred items
- SpO₂ R-value, green channel, external AFE clock (GPIO18), LP-core button monitor: designed-for, not built (handoff §5.12.10).
- V2.2 board rev flips R1 to pull-down: init is already rev-proof; delete the 33 µA line item when it lands.
