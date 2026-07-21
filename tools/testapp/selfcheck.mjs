/*
 * selfcheck.mjs — headless (node) end-to-end check of the test app's pure
 * parts: proto.js codecs, the NarbisMock scripted device, and the
 * NarbisAnalysis verdict helpers from app.js. No DOM, no hardware.
 *
 * Drives the SAME control flow app.js uses for every guided step:
 * connect -> subscribe CONTROL/STATUS/EVENT -> all TEST opcodes ->
 * chunked report fetch -> stream soaks -> sleep/disconnect, and asserts
 * the scripted values (including the deliberate T05 crosstalk FAIL).
 *
 * Run:  node tools/testapp/selfcheck.mjs
 */
import { createRequire } from "node:module";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";

const here = dirname(fileURLToPath(import.meta.url));
const require = createRequire(import.meta.url);
const NP = require(join(here, "proto.js"));
const NarbisMock = require(join(here, "mock.js"));
const { NarbisAnalysis: A } = require(join(here, "app.js"));
const P = NP.P;

let checks = 0, failures = 0;
function ok(cond, what) {
  checks++;
  if (!cond) { failures++; console.error(`  FAIL  ${what}`); }
}
function eq(a, b, what) {
  ok(JSON.stringify(a) === JSON.stringify(b), `${what}: ${JSON.stringify(a)} != ${JSON.stringify(b)}`);
}
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/* ================================================================ */
console.log("[1] codec self-checks (no device)");
/* ================================================================ */
{
  /* CONTROL envelope, normal + TEST echo-op convention */
  const req = NP.build.knobSet(7, 0x0405, -2);
  eq([...req], [0x11, 7, 0x05, 0x04, 0xFE, 0xFF, 0xFF, 0xFF], "knobSet wire");
  const r1 = NP.parseControlResponse([0x11 | 0x80, 7, 0, 1, 2]);
  ok(r1.op === P.OP_KNOB_SET && r1.tid === 7 && r1.status === 0, "normal resp op");
  const r2 = NP.parseControlResponse([P.OP_TEST_BATT_RAW, 3, 0, 0x40, 0x0F, 0x00, 0x08]);
  ok(r2.op === P.OP_TEST_BATT_RAW, "TEST resp echoes op unchanged");
  const br = NP.parseBattRawResp(r2.payload);
  ok(br.mvCal === 0x0F40 && br.adcRawAvg === 0x0800, "batt raw payload");
  let threw = false;
  try { NP.buildControlRequest(0x90, 1); } catch (_) { threw = true; }
  ok(threw, "0x90 request rejected (aliases a response)");
  ok(NP.buildControlRequest(P.OP_TEST_REPORT, 1).length === 2, "TEST op with bit7 accepted");

  /* PPG golden bytes — same fixture as test_proto_roundtrip.py */
  const ppgWire = Buffer.from(
    "01000000" + "40420f0000000000" + "04" + "02" + "10" +
    "010000000300000005000000" + "020000000400000006000000", "hex");
  const ppg = NP.parsePpg(ppgWire);
  ok(ppg.seq === 1 && ppg.t0Us === 1000000 && ppg.rateCode === P.RATE_500 &&
     ppg.flags === P.PPGF_AMB, "ppg golden header");
  eq(ppg.ir, [1, 2], "ppg golden ir");
  eq(ppg.red, [3, 4], "ppg golden red");
  eq(ppg.amb, [5, 6], "ppg golden amb");

  /* STATUS golden bytes — same fixture as the pytest suite */
  const stWire = Buffer.from(
    "024d930f4c010c08040" + "27b002a00000007006" + "7ff100e00002c034a0000000000",
    "hex");
  const st = NP.parseStatus(stWire);
  ok(st.sysState === 2 && st.flags === 0x4D && st.battMv === 3987 &&
     st.battPct === 76 && st.hrBpm === 74 && st.clockDriftPpmX10 === -153 &&
     st.uptimeS === 3600 && st.ibiLastMs === 812, "status golden fields");
  const stExtra = NP.parseStatus(Buffer.concat([stWire, Buffer.from([0xAA])]));
  ok(stExtra.extra.length === 1 && stExtra.extra[0] === 0xAA, "status append-only tail");

  /* EVENT batch with an unknown record between known ones */
  const mk = (type, len, fill) => {
    const b = Buffer.alloc(2 + len); b[0] = type; b[1] = len;
    b.writeBigUInt64LE(5000n, 2); if (fill) fill(b); return b;
  };
  const evWire = Buffer.concat([
    Buffer.from([9, 0, 0, 0, 3]),
    mk(P.EV_WEAR, P.EVLEN_WEAR, (b) => { b[10] = 1; }),
    mk(0x7F, 9, (b) => { b[10] = 0xAA; }),
    mk(P.EV_MARKER, P.EVLEN_MARKER, (b) => { b[10] = 0; b.writeUInt16LE(0x0102, 11); }),
  ]);
  const evb = NP.parseEventBatch(evWire);
  ok(evb.seq === 9 && evb.events.length === 3, "event batch shape");
  ok(evb.events[0].known && evb.events[0].worn === 1, "wear event decoded");
  ok(!evb.events[1].known && evb.events[1].tUs === 5000, "unknown event skippable");
  ok(evb.events[2].known && evb.events[2].markerId === 0x0102, "record after unknown ok");

  /* IBI + selftest blob + chunk envelope */
  const ibiWire = Buffer.alloc(P.IBI_HDR_SIZE + P.IBI_REC_SIZE);
  ibiWire.writeUInt32LE(4, 0); ibiWire[4] = 1;
  ibiWire.writeBigUInt64LE(4000000n, 5);
  ibiWire.writeUInt16LE(807, 13); ibiWire[15] = 88; ibiWire[16] = 0x05;
  const ibi = NP.parseIbi(ibiWire);
  ok(ibi.records[0].ibiMs === 807 && ibi.records[0].confidence === 88 &&
     ibi.records[0].flags === 5, "ibi record");
  threw = false;
  try { NP.parseIbi(ibiWire.subarray(0, ibiWire.length - 1)); } catch (_) { threw = true; }
  ok(threw, "truncated ibi rejected");
}

/* ================================================================ */
console.log("[2] mock device walk (the full guided sequence, headless)");
/* ================================================================ */

const bt = NarbisMock.create({ timescale: 0.02, seed: 42 });
const device = await bt.requestDevice({
  filters: [{ namePrefix: "Narbis Edge Earclip" }],
  optionalServices: [P.UUID.SENSOR_SVC, P.UUID.OTA_SVC, P.UUID.BATTERY_SVC,
                     P.UUID.DEVICE_INFO_SVC, P.UUID.HEART_RATE_SVC],
});
ok(device.name === "Narbis Edge Earclip TEST", "advertised TEST name");
const server = await device.gatt.connect();
const svc = await server.getPrimaryService(P.UUID.SENSOR_SVC);
const chars = {
  control: await svc.getCharacteristic(P.UUID.CONTROL),
  status: await svc.getCharacteristic(P.UUID.STATUS),
  event: await svc.getCharacteristic(P.UUID.EVENT),
  ppg: await svc.getCharacteristic(P.UUID.PPG),
  accel: await svc.getCharacteristic(P.UUID.ACCEL),
};

/* DIS + PROTO_VER like the app's connect flow */
const dis = await server.getPrimaryService(P.UUID.DEVICE_INFO_SVC);
const fw = new TextDecoder().decode(
  await (await dis.getCharacteristic(P.UUID.FIRMWARE_REVISION)).readValue());
ok(fw.length > 0, "DIS firmware revision reads");
const pv = await (await svc.getCharacteristic(P.UUID.PROTO_VER)).readValue();
ok(pv.getUint16(0, true) === P.PROTO_VER, "PROTO_VER characteristic");

/* control plumbing: tid-correlated request/response like app.js ctrl() */
let tidNext = 1;
const pending = new Map();
chars.control.addEventListener("characteristicvaluechanged", (ev) => {
  const resp = NP.parseControlResponse(ev.target.value);
  const p = pending.get(resp.tid);
  if (!p) return;
  if (!p.until(resp)) return;
  pending.delete(resp.tid);
  p.resolve(resp);
});
await chars.control.startNotifications();

function ctrl(name, args = [], opts = {}) {
  const tid = tidNext; tidNext = (tidNext & 0xFF) + 1;
  const bytes = NP.build[name](tid, ...args);
  return new Promise((resolve, reject) => {
    const t = setTimeout(() => {
      pending.delete(tid);
      reject(new Error(`${name}: timeout`));
    }, opts.timeoutMs || 4000);
    pending.set(tid, {
      until: opts.until || (() => true),
      resolve: (r) => { clearTimeout(t); resolve(r); },
    });
    chars.control.writeValue(bytes).catch(reject);
  });
}

async function fetchBlob(name) {
  let off = 0, total = 0;
  const parts = [];
  for (;;) {
    const c = NP.parseChunk((await ctrl(name, [off])).payload);
    total = c.total;
    if (!c.data.length) break;
    parts.push(c.data); off += c.data.length;
    if (off >= total) break;
  }
  ok(off === total, `${name}: chunked fetch complete (${off}/${total})`);
  return Buffer.concat(parts.map((p) => Buffer.from(p)));
}

/* -- events collected globally (button, charger, selftest-done) -- */
const events = [];
chars.event.addEventListener("characteristicvaluechanged", (ev) => {
  events.push(...NP.parseEventBatch(ev.target.value).events);
});
await chars.event.startNotifications();

/* steps 1-5: single self-tests; T05 is the scripted FAIL */
for (const [id, wantPass] of [
  [P.TEST_I2C_SCAN, true], [P.TEST_ACCEL_WHOAMI, true],
  [P.TEST_AFE_REG_RW, true], [P.TEST_AFE_DARK, true], [P.TEST_XTALK, false],
]) {
  const resp = await ctrl("testSelftestOne", [id]);
  ok(resp.status === P.ST_OK, `T0${id} responds OK`);
  const rec = NP.parseSelftestRecord(resp.payload);
  ok(rec.id === id, `T0${id} echoes id`);
  eq(rec.status === P.TR_PASS, wantPass, `T0${id} pass/fail as scripted`);
  if (id === P.TEST_XTALK) {
    ok(rec.value === 81234 && rec.threshold === 50000,
       "T05 deliberate FAIL values (81234 > 50000)");
  }
}

/* step 6: LED drive + clamp check */
ok((await ctrl("testLedDrive", [1, 20, 3000])).status === P.ST_OK, "LED drive ok");
ok((await ctrl("testLedDrive", [1, 45, 100])).status === P.ST_OUT_OF_RANGE,
   "red 45 mA rejected (abs-max 40)");

/* step 7: LED sweeps -> report -> monotonic verdicts */
await ctrl("testLedSweep", [0, 2], { timeoutMs: 8000 });
await ctrl("testLedSweep", [1, 2], { timeoutMs: 8000 });
/* step 8: RX sweeps */
await ctrl("testRxSweep", [0]);
await ctrl("testRxSweep", [1]);
const report = JSON.parse(new TextDecoder().decode(await fetchBlob("testReport")));
ok(report.tests.led_sweep_ir && report.tests.led_sweep_red, "report has LED sweeps");
const vIr = A.sweepVerdict(report.tests.led_sweep_ir.dc);
const vRed = A.sweepVerdict(report.tests.led_sweep_red.dc);
ok(vIr.pass && vRed.pass, `LED sweeps monotonic (IR span ${vIr.span})`);
const vGain = A.sweepVerdict(report.tests.rx_sweep_gain.dc, { minRisingFrac: 0.85 });
ok(vGain.pass, "RX gain sweep monotonic");
ok(!A.sweepVerdict([100, 130, 90, 120, 95, 110]).pass,
   "flat sweep correctly fails (open emitter detection)");
ok(report.selftest.some((r) => r.id === P.TEST_XTALK && r.status === P.TR_FAIL),
   "report carries the T05 FAIL");

/* step 9: rate count — ack, then delayed result on the same tid */
{
  const resp = await ctrl("testRateCount", [10],
    { until: (r) => r.payload.length === 8, timeoutMs: 8000 });
  const { pulses, elapsedMs } = NP.parseRateCountResp(resp.payload);
  const v = A.rateVerdict(pulses, elapsedMs, 100);
  ok(v.pass, `rate count within 0.5% (got ${v.hz.toFixed(3)} Hz)`);
  ok(!A.rateVerdict(980, 10000, 100).pass, "2% rate error correctly fails");
}

/* step 10: accel orientation — all six faces captured from the stream */
{
  const captured = new Set();
  chars.accel.addEventListener("characteristicvaluechanged", (ev) => {
    const b = NP.parseAccel(ev.target.value);
    const cpg = NP.fsCountsPerG(b.fsCode);
    let sx = 0, sy = 0, sz = 0;
    for (const [x, y, z] of b.samples) { sx += x; sy += y; sz += z; }
    const n = b.samples.length;
    const f = A.faceOf(sx / n / cpg, sy / n / cpg, sz / n / cpg);
    if (f) captured.add(f);
  });
  await chars.accel.startNotifications();
  await ctrl("testAccelLive", [1]);
  await sleep(13000 * bt.timescale + 200); /* 6 faces x 2 s, scaled */
  await ctrl("testAccelLive", [0]);
  await chars.accel.stopNotifications();
  eq(captured.size, 6, `all 6 faces captured (${[...captured].join(",")})`);
}

/* step 11: button echo — 5 presses = 10 marker edges from src=button */
{
  const before = events.length;
  await ctrl("testButtonEcho", [1]);
  await sleep(4400 * bt.timescale + 200);
  await ctrl("testButtonEcho", [0]);
  const edges = events.slice(before).filter(
    (e) => e.type === P.EV_MARKER && e.known && e.source === P.MARKER_SRC_BUTTON);
  eq(edges.length, 10, "10 button edges (5 presses)");
  ok(edges.every((e, i) => i === 0 || e.tUs > edges[i - 1].tUs),
     "button edge timestamps strictly increase");
}

/* step 12: charger walk — snapshots decode, transitions + STAT polarity */
{
  const before = events.length;
  await ctrl("testChargerLive", [1]);
  await sleep(9500 * bt.timescale + 200);
  await ctrl("testChargerLive", [0]);
  const snaps = events.slice(before)
    .filter((e) => e.type === P.OP_TEST_CHARGER_LIVE && e.len === 11)
    .map((e) => ({ vusb: e.raw[8], stat: e.raw[9], state: e.raw[10] }));
  ok(snaps.length >= 15, `charger snapshots streamed (${snaps.length})`);
  const seq = snaps.map((s) => s.state).filter((s, i, a) => !i || s !== a[i - 1]);
  eq(seq, [P.CHG_ON_BATTERY, P.CHG_CHARGING, P.CHG_COMPLETE, P.CHG_ON_BATTERY],
     "charger state walk");
  const chg = snaps.find((s) => s.state === P.CHG_CHARGING);
  const bat = snaps.find((s) => s.state === P.CHG_ON_BATTERY);
  ok(chg.stat !== bat.stat, "raw STAT bit differs charging vs battery (polarity observable)");
  ok(chg.vusb === 1 && bat.vusb === 0, "VUSB tracks plug state");
}

/* step 13: battery raw */
{
  const { mvCal, adcRawAvg } = NP.parseBattRawResp(
    (await ctrl("testBattRaw")).payload);
  ok(mvCal === 3921 && adcRawAvg === 1943, "batt raw values");
  ok(A.battVerdict(mvCal, 3940).pass && !A.battVerdict(mvCal, 3990).pass,
     "±50 mV verdict");
}

/* step 14: PPG smoke — production stream, zero seq gaps */
{
  let gaps = 0, lastSeq = null, samples = 0;
  chars.ppg.addEventListener("characteristicvaluechanged", (ev) => {
    const b = NP.parsePpg(ev.target.value);
    if (lastSeq !== null && b.seq !== lastSeq + 1) gaps++;
    lastSeq = b.seq;
    samples += b.ir.length;
  });
  await chars.ppg.startNotifications();
  await ctrl("setRate", [P.RATE_100]);
  await ctrl("streamStart", [P.STREAM_MASK_PPG]);
  await sleep(400); /* scaled: several seconds of stream */
  await ctrl("streamStop", [P.STREAM_MASK_PPG]);
  await chars.ppg.stopNotifications();
  ok(samples > 100, `PPG samples flowed (${samples})`);
  eq(gaps, 0, "zero PPG seq gaps");
}

/* knob discovery: full chunked walk matches the generated table */
{
  let idx = 0, total = Infinity;
  const got = [];
  while (idx < total) {
    const chunk = NP.parseKnobDiscoverChunk(
      (await ctrl("knobDiscover", [idx])).payload);
    total = chunk.total;
    ok(chunk.firstIdx === idx, "knob chunk first_idx");
    if (!chunk.records.length) break;
    got.push(...chunk.records);
    idx += chunk.records.length;
  }
  eq(got.length, P.KNOBS.length, `knob discovery walks all ${P.KNOBS.length}`);
  ok(got.every((r, i) => r.id === P.KNOBS[i].id && r.name === P.KNOBS[i].name &&
                         r.unit === P.KNOBS[i].unit),
     "knob records match generated table (id/name/unit)");
}

/* step 15: sleep — ack then the link drops */
{
  let dropped = false;
  device.addEventListener("gattserverdisconnected", () => { dropped = true; });
  ok((await ctrl("testSleepNow")).status === P.ST_OK, "sleep now acks");
  await sleep(10000 * bt.timescale + 300);
  ok(dropped, "device disconnected after sleep delay");
  ok(!server.connected, "gatt reports disconnected");
}

/* ================================================================ */
console.log(`\nselfcheck: ${checks} checks, ${failures} failures`);
if (failures) process.exit(1);
console.log("OK");
