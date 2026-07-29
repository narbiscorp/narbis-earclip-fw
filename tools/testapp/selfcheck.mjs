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
/* [3] firmware update (BLE OTA): codecs + full mock flash walk       */
console.log("[3] firmware update (BLE OTA)");
{
  /* codec sanity */
  ok(NP.ota.crc32(0, new TextEncoder().encode("123456789")) === 0xCBF43926,
     "crc32 check vector");
  const beg = NP.ota.buildBegin(7, 0x12345, 0xDEADBEEF, "v1.2.3");
  ok(beg[0] === P.OTA_BEGIN && beg[1] === 7 && beg[10] === 6, "BEGIN framing");
  const df = NP.ota.buildData(0x1000, Uint8Array.of(1, 2, 3));
  ok(df.length === 7 && NP.dv(df).getUint32(0, true) === 0x1000, "DATA framing");

  /* full flash against a fresh mock device (the sleep test above
   * ended the previous session) */
  const bt2 = NarbisMock.create({ timescale: 0.02, seed: 43 });
  const dev2 = await bt2.requestDevice({ filters: [] });
  const srv2 = await dev2.gatt.connect();
  const osvc = await srv2.getPrimaryService(P.UUID.OTA_SVC);
  const octl = await osvc.getCharacteristic(P.UUID.OTA_CTRL);
  const odat = await osvc.getCharacteristic(P.UUID.OTA_DATA);
  await octl.startNotifications();

  const disSvc = await srv2.getPrimaryService(P.UUID.DEVICE_INFO_SVC);
  const fwChar = await disSvc.getCharacteristic(P.UUID.FIRMWARE_REVISION);
  const fwBefore = new TextDecoder().decode(await fwChar.readValue());

  const oPend = new Map();
  let oTid = 1;
  octl.addEventListener("characteristicvaluechanged", (ev) => {
    const r = NP.parseControlResponse(ev.target.value);
    const p = oPend.get(r.tid);
    if (p) { oPend.delete(r.tid); p(r); }
  });
  const otaReq = (bytes) => new Promise((res) => {
    bytes[1] = oTid = (oTid & 0xFF) + 1;
    oPend.set(bytes[1], res);
    octl.writeValue(bytes);
  });

  /* 5 KB pseudo-image (0xE9 magic like a real ESP app image) */
  const img = new Uint8Array(5120);
  img[0] = 0xE9;
  for (let i = 1; i < img.length; i++) img[i] = (i * 31 + 7) & 0xFF;
  const crc = NP.ota.crc32(0, img);

  let r = await otaReq(NP.ota.buildBegin(0, img.length, crc, "selfcheck"));
  ok(r.status === P.ST_OK, "OTA BEGIN ok");
  ok(NP.ota.parseBeginResp(r.payload).resumeOffset === 0, "fresh start at 0");

  for (let off = 0; off < img.length; off += 240) {
    await odat.writeValueWithoutResponse(
        NP.ota.buildData(off, img.subarray(off, Math.min(off + 240, img.length))));
  }
  r = await otaReq(NP.ota.buildStatus(0));
  const st = NP.ota.parseStatusResp(r.payload);
  ok(st.bytesRx === img.length && st.lastErr === P.OTAERR_NONE,
     `all bytes received (${st.bytesRx})`);

  /* out-of-order write latches EXPECTED_OFFSET and is ignored */
  await odat.writeValueWithoutResponse(NP.ota.buildData(99999, Uint8Array.of(1)));
  const st2 = NP.ota.parseStatusResp((await otaReq(NP.ota.buildStatus(0))).payload);
  ok(st2.lastErr === P.OTAERR_EXPECTED_OFFSET && st2.bytesRx === img.length,
     "offset gap latched, count unchanged");

  let dropped2 = false;
  dev2.addEventListener("gattserverdisconnected", () => { dropped2 = true; });
  r = await otaReq(NP.ota.buildFinish(0));
  ok(r.status === P.ST_OK, "FINISH ok (crc verified)");
  await sleep(400);
  ok(dropped2, "device rebooted after FINISH");

  await dev2.gatt.connect();
  const fwAfter = new TextDecoder().decode(await fwChar.readValue());
  ok(fwAfter !== fwBefore && /\+ota1$/.test(fwAfter),
     `firmware revision bumped (${fwBefore} -> ${fwAfter})`);

  /* bad CRC path: device must refuse at FINISH */
  const img2 = img.slice(); img2[100] ^= 0xFF;
  r = await otaReq(NP.ota.buildBegin(0, img2.length, crc /* stale crc */, "bad"));
  ok(r.status === P.ST_OK, "BEGIN (bad-crc run) ok");
  for (let off = 0; off < img2.length; off += 240) {
    await odat.writeValueWithoutResponse(
        NP.ota.buildData(off, img2.subarray(off, Math.min(off + 240, img2.length))));
  }
  r = await otaReq(NP.ota.buildFinish(0));
  ok(r.status === P.ST_CRC_ERR, "FINISH rejects corrupted image (CRC)");
  dev2.gatt.disconnect();
}

/* ================================================================ */
/* [4] dashboard: pure helpers (rings/zoom/zip/csv), proto 1.1 bits,  */
/*     and a short simulate-mode soak                                 */
console.log("[4] dashboard (rings, zoom, zip, csv, 0xEB, sim soak)");
const { NarbisDashCore: DC } = require(join(here, "dashboard.js"));
{
  /* rolling ring: 30 s window + hard cap FIFO */
  const r = DC.makeRing(30);
  for (let i = 0; i <= 100; i++) r.push(i, i * 2);
  /* trim is amortized: at most 64 aged-out points linger (draw clips) */
  ok(r.t[0] >= 100 - 30 - 64 && r.t.length <= 30 + 65 && r.lastT() === 100,
     `ring trims to window (kept ${r.t.length})`);
  ok(r.last() === 200, "ring last()");
  const mm = DC.ringMinMax(r, 90, 100);
  ok(mm.lo === 180 && mm.hi === 200, "ringMinMax over visible span");
  const r2 = DC.makeRing(Infinity, 100);
  for (let i = 0; i < 250; i++) r2.push(i, i);
  ok(r2.t.length === 100 && r2.t[0] === 150, "session ring hard-cap FIFO");

  /* autoscale + zoom-around-center math */
  const ar = DC.autoRange(0, 100);
  ok(Math.abs(ar.center - 50) < 1e-9 && Math.abs(ar.span - 112) < 1e-9,
     "autoRange pads the data span");
  let z = DC.zoomStep(null, 0, 100, 1);
  ok(Math.abs(z.span - 112 / 1.6) < 1e-6 && z.center === 50,
     "zoom-in shrinks span around center");
  z = DC.zoomStep(z, 0, 100, -1);
  ok(Math.abs(z.span - 112) < 1e-6, "zoom-out restores span");
  const auto = DC.applyZoom(0, 100, null);
  ok(auto[0] < 0 && auto[1] > 100, "applyZoom auto covers data");
  eq(DC.applyZoom(0, 100, { center: 10, span: 4 }), [8, 12],
     "applyZoom manual override");

  /* seq-gap counter: counts skips, tolerates event-batch repeats */
  const gc = DC.gapCounter();
  for (const s of [0, 1, 2, 4, 4, 5, 9]) gc.feed(s);
  eq(gc.gaps, 2, "gap counter (skip at 4 and 9, repeat tolerated)");

  /* csv escaping */
  eq(DC.csv(["a", "b"], [[1, "x"], [2, 'he,"llo']]),
     'a,b\n1,x\n2,"he,""llo"\n', "csv quoting/escaping");

  /* event formatter (ticker strings) */
  ok(DC.fmtEvent({ type: P.EV_MARKER, known: true, tUs: 2e6,
                   source: P.MARKER_SRC_BUTTON, markerId: 1000 }, P)
       .includes("button PRESS"), "fmtEvent button press (id 1000)");
  ok(DC.fmtEvent({ type: P.EV_WEAR, known: true, tUs: 1e6, worn: 0 }, P)
       .includes("wear OFF"), "fmtEvent wear");

  /* STORE zip: verify signatures, offsets and CRCs on our own output */
  const files = [
    { name: "ppg.csv", data: "t,ir\n1,2\n" },
    { name: "log.txt", data: new TextEncoder().encode("hello narbis\n") },
  ];
  const zip = DC.zipStore(files, NP.ota.crc32);
  const zd = new DataView(zip.buffer, zip.byteOffset, zip.byteLength);
  const eo = zip.length - 22;
  ok(zd.getUint32(eo, true) === 0x06054B50, "zip EOCD signature");
  eq(zd.getUint16(eo + 10, true), 2, "zip EOCD entry count");
  const cdLen = zd.getUint32(eo + 12, true);
  const cdOff = zd.getUint32(eo + 16, true);
  ok(cdOff + cdLen + 22 === zip.length, "zip central dir spans to EOCD");
  let p = cdOff;
  files.forEach((f, i) => {
    ok(zd.getUint32(p, true) === 0x02014B50, `zip central hdr ${i} sig`);
    const crc = zd.getUint32(p + 16, true);
    const csize = zd.getUint32(p + 20, true);
    const nlen = zd.getUint16(p + 28, true);
    const lof = zd.getUint32(p + 42, true);
    ok(zd.getUint32(lof, true) === 0x04034B50, `zip local hdr ${i} at offset`);
    eq(zd.getUint16(lof + 8, true), 0, `zip entry ${i} method STORE`);
    const lnlen = zd.getUint16(lof + 26, true);
    eq(lnlen, nlen, `zip entry ${i} name length agreement`);
    const data = zip.subarray(lof + 30 + lnlen, lof + 30 + lnlen + csize);
    ok(NP.ota.crc32(0, data) === crc, `zip entry ${i} CRC over stored bytes`);
    eq(new TextDecoder().decode(zip.subarray(p + 46, p + 46 + nlen)),
       f.name, `zip entry ${i} name`);
    p += 46 + nlen;
  });

  /* proto 1.1: STATUS btnPressed (byte 27) + 0xEB builder */
  const stB = Buffer.alloc(P.STATUS_SIZE);
  stB[27] = 1;
  ok(NP.parseStatus(stB).btnPressed === 1, "parseStatus btnPressed byte 27");
  ok(NP.parseStatus(Buffer.alloc(P.STATUS_SIZE)).btnPressed === 0,
     "legacy frame (reserved byte 0) reads btnPressed 0");
  eq([...NP.build.testLedSweepCont(9, 3, 1, 7)], [0xEB, 9, 3, 1, 7],
     "testLedSweepCont wire bytes");
}

/* ---- simulate-mode soak: 2 s wall, timescale 0.02 (~100 s device) ---- */
{
  const bt3 = NarbisMock.create({ timescale: 0.02, seed: 7, sim: true });
  const dev3 = await bt3.requestDevice({ filters: [] });
  const srv3 = await dev3.gatt.connect();
  const svc3 = await srv3.getPrimaryService(P.UUID.SENSOR_SVC);
  const ch = {};
  for (const [k, u] of [["control", P.UUID.CONTROL], ["status", P.UUID.STATUS],
                        ["event", P.UUID.EVENT], ["ppg", P.UUID.PPG],
                        ["accel", P.UUID.ACCEL], ["ibi", P.UUID.IBI]]) {
    ch[k] = await svc3.getCharacteristic(u);
  }
  const pend3 = new Map();
  let tid3 = 1;
  ch.control.addEventListener("characteristicvaluechanged", (ev) => {
    const resp = NP.parseControlResponse(ev.target.value);
    const f = pend3.get(resp.tid);
    if (f) { pend3.delete(resp.tid); f(resp); }
  });
  const c3 = (name, args = []) => new Promise((resolve, reject) => {
    const tid = tid3; tid3 = (tid3 & 0xFF) + 1;
    pend3.set(tid, resolve);
    setTimeout(() => {
      if (pend3.delete(tid)) reject(new Error(`${name}: timeout`));
    }, 4000);
    ch.control.writeValue(NP.build[name](tid, ...args)).catch(reject);
  });

  let nPpg = 0, nAcc = 0, nIbi = 0, nSt = 0, ambSeen = false;
  const evTypes = new Set(), maSeen = new Set(), btnVals = new Set();
  const ibiMsSeen = [];
  let battFirst = null, battLast = null;
  ch.ppg.addEventListener("characteristicvaluechanged", (ev) => {
    const b = NP.parsePpg(ev.target.value);
    nPpg += b.ir.length;
    if (b.amb) ambSeen = true;
  });
  ch.accel.addEventListener("characteristicvaluechanged", (ev) => {
    nAcc += NP.parseAccel(ev.target.value).samples.length;
  });
  ch.ibi.addEventListener("characteristicvaluechanged", (ev) => {
    for (const rec of NP.parseIbi(ev.target.value).records) {
      nIbi++; ibiMsSeen.push(rec.ibiMs);
    }
  });
  ch.status.addEventListener("characteristicvaluechanged", (ev) => {
    const st = NP.parseStatus(ev.target.value);
    nSt++;
    maSeen.add(st.ledIrMa);
    if (battFirst === null) battFirst = st.battMv;
    battLast = st.battMv;
  });
  ch.event.addEventListener("characteristicvaluechanged", (ev) => {
    for (const rec of NP.parseEventBatch(ev.target.value).events) {
      evTypes.add(rec.type);
    }
  });
  for (const k of ["control", "status", "event", "ppg", "accel", "ibi"]) {
    await ch[k].startNotifications();
  }
  /* tight STATUS readValue poll so the ~0.6 s (device) button hold is
   * caught regardless of the 1 Hz notify phase */
  const btnPoll = setInterval(async () => {
    try {
      btnVals.add(NP.parseStatus(await ch.status.readValue()).btnPressed);
    } catch (_) { /* disconnecting */ }
  }, 3);

  ok((await c3("streamStart",
      [P.STREAM_MASK_PPG | P.STREAM_MASK_ACCEL |
       P.STREAM_MASK_IBI | P.STREAM_MASK_EVENT])).status === P.ST_OK,
     "sim: stream start (all masks)");
  ok((await c3("testLedSweepCont", [1, 1, 1])).status === P.ST_OK,
     "sim: 0xEB sweep start (IR, 1 s phase)");
  await sleep(1400);
  ok((await c3("testLedSweepCont", [0, 0, 0])).status === P.ST_OK,
     "sim: 0xEB sweep stop");
  /* unfreeze so the sim AGC walks the IR current again */
  ok((await c3("agcFreeze", [0])).status === P.ST_OK, "sim: AGC unfreeze");
  await sleep(600);
  clearInterval(btnPoll);

  ok(nPpg > 2000, `sim soak: PPG samples flowed (${nPpg})`);
  ok(ambSeen, "sim soak: ambient present in PPG batches");
  ok(nAcc > 500, `sim soak: accel samples flowed (${nAcc})`);
  ok(nIbi > 30, `sim soak: IBI records flowed (${nIbi})`);
  ok(ibiMsSeen.every((v) => v > 500 && v < 1400),
     "sim soak: IBIs plausible for 60-75 bpm");
  ok(nSt > 40, `sim soak: STATUS at 1 Hz device time (${nSt})`);
  const maArr = [...maSeen];
  ok(maArr.length >= 4 && Math.max(...maArr) >= 45 && Math.min(...maArr) <= 5,
     `sim soak: sweep swept reported IR mA (${maArr.length} values, ` +
     `${Math.min(...maArr)}..${Math.max(...maArr)})`);
  ok(evTypes.has(P.EV_GATE), "sim soak: gate events during motion bursts");
  ok(evTypes.has(P.EV_MARKER), "sim soak: button press markers emitted");
  ok(evTypes.has(P.EV_AGC_STEP), "sim soak: AGC stepped after unfreeze");
  ok(btnVals.has(0) && btnVals.has(1),
     "sim soak: STATUS btnPressed toggled during auto-press");
  ok(battFirst !== null && battFirst - battLast >= 40,
     `sim soak: battery drained (${battFirst} -> ${battLast} mV)`);
  srv3.disconnect();
}

console.log(`\nselfcheck: ${checks} checks, ${failures} failures`);
if (failures) process.exit(1);
console.log("OK");
