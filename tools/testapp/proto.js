/*
 * proto.js — little-endian DataView codecs for the Narbis BLE wire contract.
 *
 * Mirrors firmware proto.h byte-for-byte. Every constant comes from the
 * generated proto_consts.js (const PROTO global) — NO hand-typed magic
 * numbers in this file. Loadable two ways:
 *   - browser: classic <script> after proto_consts.js (global PROTO);
 *   - node:    require("./proto.js") (pulls proto_consts.js itself),
 *     which is how selfcheck.mjs exercises these codecs without hardware.
 *
 * u64 fields ride as BigInt through DataView and are returned as Number:
 * device microsecond timestamps stay far below 2^53 (285 years of uptime).
 */
"use strict";

const NarbisProto = (() => {
  const P = (typeof PROTO !== "undefined")
    ? PROTO
    : require("./proto_consts.js").PROTO;

  /* ---------------- helpers ---------------- */

  /** Accept DataView | ArrayBuffer | TypedArray | number[] -> DataView. */
  function dv(buf) {
    if (buf instanceof DataView) return buf;
    if (buf instanceof ArrayBuffer) return new DataView(buf);
    if (ArrayBuffer.isView(buf)) {
      return new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
    }
    return new DataView(Uint8Array.from(buf).buffer);
  }

  function bytesOf(buf) {
    const d = dv(buf);
    return new Uint8Array(d.buffer, d.byteOffset, d.byteLength);
  }

  function u64(view, off) {
    return Number(view.getBigUint64(off, true));
  }

  function fail(msg) { throw new Error("proto: " + msg); }

  /* ---------------- CONTROL envelope ---------------- */

  /* Request [u8 op][u8 tid][payload]. */
  function buildControlRequest(op, tid, payload) {
    if (!(op >= 0 && op <= 0xFF)) fail(`bad opcode ${op}`);
    /* Sub-0xE0 opcodes with bit7 set would alias a response; the TEST
     * block (0xE0+) legitimately carries bit7 (proto.h echo-op rule). */
    if ((op & P.OP_RESP_FLAG) && op < P.OP_TEST_SELFTEST_ONE) {
      fail(`request opcode 0x${op.toString(16)} aliases a response`);
    }
    if (!(tid >= 0 && tid <= 0xFF)) fail(`bad tid ${tid}`);
    const pl = payload ? bytesOf(payload) : new Uint8Array(0);
    const out = new Uint8Array(2 + pl.length);
    out[0] = op; out[1] = tid; out.set(pl, 2);
    return out;
  }

  /* Response indication [u8 op|0x80][u8 tid][u8 status][payload].
   * TEST opcodes (>= 0xE0) already carry bit7: the firmware echoes the
   * opcode UNCHANGED; correlation for those is by tid alone. */
  function parseControlResponse(buf) {
    const b = bytesOf(buf);
    if (b.length < 3) fail(`CONTROL response too short: ${b.length}`);
    const opRaw = b[0];
    let op;
    if (opRaw >= P.OP_TEST_SELFTEST_ONE) op = opRaw;
    else if (opRaw & P.OP_RESP_FLAG) op = opRaw & ~P.OP_RESP_FLAG;
    else fail(`not a CONTROL response: op byte 0x${opRaw.toString(16)}`);
    return { op, tid: b[1], status: b[2], payload: b.slice(3) };
  }

  /* ---- payload builders (little-endian packers) ---- */

  function pack(fields) {
    /* fields: array of [type, value]; type in u8|u16|u32|i32|u64 */
    let size = 0;
    for (const [t] of fields) size += { u8: 1, u16: 2, u32: 4, i32: 4, u64: 8 }[t];
    const out = new DataView(new ArrayBuffer(size));
    let off = 0;
    for (const [t, v] of fields) {
      switch (t) {
        case "u8": out.setUint8(off, v); off += 1; break;
        case "u16": out.setUint16(off, v, true); off += 2; break;
        case "u32": out.setUint32(off, v >>> 0, true); off += 4; break;
        case "i32": out.setInt32(off, v, true); off += 4; break;
        case "u64": out.setBigUint64(off, BigInt(v), true); off += 8; break;
      }
    }
    return new Uint8Array(out.buffer);
  }

  const build = {
    streamStart: (tid, mask) =>
      buildControlRequest(P.OP_STREAM_START, tid, pack([["u8", mask]])),
    streamStop: (tid, mask) =>
      buildControlRequest(P.OP_STREAM_STOP, tid, pack([["u8", mask]])),
    setRate: (tid, rateCode) =>
      buildControlRequest(P.OP_SET_RATE, tid, pack([["u8", rateCode]])),
    knobGet: (tid, id) =>
      buildControlRequest(P.OP_KNOB_GET, tid, pack([["u16", id]])),
    knobSet: (tid, id, value) =>
      buildControlRequest(P.OP_KNOB_SET, tid, pack([["u16", id], ["i32", value]])),
    knobSave: (tid) => buildControlRequest(P.OP_KNOB_SAVE, tid),
    enterOta: (tid) => buildControlRequest(P.OP_ENTER_OTA, tid),
    knobReset: (tid, scope) =>
      buildControlRequest(P.OP_KNOB_RESET, tid, pack([["u8", scope]])),
    knobDiscover: (tid, startIndex) =>
      buildControlRequest(P.OP_KNOB_DISCOVER, tid, pack([["u16", startIndex]])),
    timeSync: (tid, hostEpochUs) =>
      buildControlRequest(P.OP_TIME_SYNC, tid, pack([["u64", hostEpochUs]])),
    getTime: (tid) => buildControlRequest(P.OP_GET_TIME, tid),
    marker: (tid, markerId) =>
      buildControlRequest(P.OP_MARKER, tid, pack([["u16", markerId || 0]])),
    agcFreeze: (tid, freeze) =>
      buildControlRequest(P.OP_AGC_FREEZE, tid, pack([["u8", freeze ? 1 : 0]])),
    agcManual: (tid, irMa, redMa, rfCode, applyMask) =>
      buildControlRequest(P.OP_AGC_MANUAL, tid,
        pack([["u8", irMa], ["u8", redMa], ["u8", rfCode], ["u8", applyMask]])),
    selftestRun: (tid, mask) =>
      buildControlRequest(P.OP_SELFTEST_RUN, tid, pack([["u32", mask || 0]])),
    selftestResult: (tid, offset) =>
      buildControlRequest(P.OP_SELFTEST_RESULT, tid, pack([["u16", offset]])),
    /* TEST block (NARBIS_TEST_MODE builds only) */
    testSelftestOne: (tid, testId) =>
      buildControlRequest(P.OP_TEST_SELFTEST_ONE, tid, pack([["u8", testId]])),
    testLedDrive: (tid, led, ma, durationMs) =>
      buildControlRequest(P.OP_TEST_LED_DRIVE, tid,
        pack([["u8", led], ["u8", ma], ["u16", durationMs]])),
    testLedSweep: (tid, led, maStep) =>
      buildControlRequest(P.OP_TEST_LED_SWEEP, tid,
        pack([["u8", led], ["u8", maStep]])),
    /* 0xEB continuous LED triangle sweep (proto 1.1): mask b0 IR b1 RED,
     * enable, phase_s (0 -> firmware default 5). */
    testLedSweepCont: (tid, mask, enable, phaseS) =>
      buildControlRequest(P.OP_TEST_LED_SWEEP_CONT, tid,
        pack([["u8", mask], ["u8", enable ? 1 : 0], ["u8", phaseS || 0]])),
    testRxSweep: (tid, what) =>
      buildControlRequest(P.OP_TEST_RX_SWEEP, tid, pack([["u8", what]])),
    testRateCount: (tid, seconds) =>
      buildControlRequest(P.OP_TEST_RATE_COUNT, tid, pack([["u8", seconds]])),
    testButtonEcho: (tid, enable) =>
      buildControlRequest(P.OP_TEST_BUTTON_ECHO, tid, pack([["u8", enable ? 1 : 0]])),
    testChargerLive: (tid, enable) =>
      buildControlRequest(P.OP_TEST_CHARGER_LIVE, tid, pack([["u8", enable ? 1 : 0]])),
    testBattRaw: (tid) => buildControlRequest(P.OP_TEST_BATT_RAW, tid),
    testAccelLive: (tid, enable) =>
      buildControlRequest(P.OP_TEST_ACCEL_LIVE, tid, pack([["u8", enable ? 1 : 0]])),
    testSleepNow: (tid) => buildControlRequest(P.OP_TEST_SLEEP_NOW, tid),
    testReport: (tid, offset) =>
      buildControlRequest(P.OP_TEST_REPORT, tid, pack([["u16", offset]])),
  };

  /* ---- response payload parsers ---- */

  function parseKnobGetResp(payload) {
    const d = dv(payload);
    if (d.byteLength !== 6) fail(`knob get resp len ${d.byteLength}`);
    return { id: d.getUint16(0, true), value: d.getInt32(2, true) };
  }

  function parseTimeSyncResp(payload) {
    const d = dv(payload);
    if (d.byteLength !== 16) fail(`time sync resp len ${d.byteLength}`);
    return { echoUs: u64(d, 0), devTUs: u64(d, 8) };
  }

  function parseGetTimeResp(payload) {
    const d = dv(payload);
    if (d.byteLength !== 8) fail(`get time resp len ${d.byteLength}`);
    return u64(d, 0);
  }

  function parseBattRawResp(payload) {
    const d = dv(payload);
    if (d.byteLength !== 4) fail(`batt raw resp len ${d.byteLength}`);
    return { mvCal: d.getUint16(0, true), adcRawAvg: d.getUint16(2, true) };
  }

  function parseRateCountResp(payload) {
    const d = dv(payload);
    if (d.byteLength !== 8) fail(`rate count resp len ${d.byteLength}`);
    return { pulses: d.getUint32(0, true), elapsedMs: d.getUint32(4, true) };
  }

  /* TEST_SELFTEST_ONE response payload: one selftest record
   * {u8 id, u8 status, i32 val, i32 thr} (NC_ST_REC_SIZE). */
  function parseSelftestRecord(payload) {
    const d = dv(payload);
    if (d.byteLength !== P.ST_REC_SIZE) fail(`selftest rec len ${d.byteLength}`);
    return {
      id: d.getUint8(0), status: d.getUint8(1),
      value: d.getInt32(2, true), threshold: d.getInt32(6, true),
    };
  }

  /* ---------------- chunk envelope {u16 total,u16 off,u8 n,bytes} ------- */

  function parseChunk(payload) {
    const d = dv(payload);
    if (d.byteLength < 5) fail(`chunk too short: ${d.byteLength}`);
    const total = d.getUint16(0, true);
    const offset = d.getUint16(2, true);
    const n = d.getUint8(4);
    const data = bytesOf(payload).slice(5);
    if (data.length !== n) fail(`chunk n=${n} != data len ${data.length}`);
    return { total, offset, data };
  }

  /* ---------------- STATUS (append-only, len >= 32) ---------------- */

  function parseStatus(buf) {
    const d = dv(buf);
    if (d.byteLength < P.STATUS_SIZE) fail(`STATUS too short: ${d.byteLength}`);
    return {
      sysState: d.getUint8(0),
      flags: d.getUint8(1),
      battMv: d.getUint16(2, true),
      battPct: d.getUint8(4),
      ppgRateCode: d.getUint8(5),
      ledIrMa: d.getUint8(6),
      ledRedMa: d.getUint8(7),
      tiaGainCode: d.getUint8(8),
      tiaCfCode: d.getUint8(9),
      gateDutyX100: d.getUint16(10, true),
      notifDropCount: d.getUint32(12, true),
      i2cErrCount: d.getUint16(16, true),
      clockDriftPpmX10: d.getInt16(18, true),
      uptimeS: d.getUint32(20, true),
      ibiLastMs: d.getUint16(24, true),
      hrBpm: d.getUint8(26),
      /* proto 1.1: byte 27 = live button level (1 = held). Older firmware
       * ships 32-byte frames with this byte in reserved[] = 0, so reading
       * it unconditionally is safe for every frame >= STATUS_SIZE. */
      btnPressed: d.byteLength > 27 ? d.getUint8(27) : 0,
      extra: bytesOf(buf).slice(P.STATUS_SIZE),
    };
  }

  /* ---------------- PPG batch ---------------- */

  function parsePpg(buf) {
    const d = dv(buf);
    if (d.byteLength < P.PPG_HDR_SIZE) fail(`PPG too short: ${d.byteLength}`);
    const seq = d.getUint32(0, true);
    const t0Us = u64(d, 4);
    const rateCode = d.getUint8(12);
    const n = d.getUint8(13);
    const flags = d.getUint8(14);
    const hasAmb = !!(flags & P.PPGF_AMB);
    const stride = hasAmb ? 12 : 8;
    if (d.byteLength !== P.PPG_HDR_SIZE + n * stride) {
      fail(`PPG len ${d.byteLength} != ${P.PPG_HDR_SIZE + n * stride}`);
    }
    const ir = new Array(n), red = new Array(n);
    const amb = hasAmb ? new Array(n) : null;
    for (let i = 0, off = P.PPG_HDR_SIZE; i < n; i++, off += stride) {
      ir[i] = d.getInt32(off, true);
      red[i] = d.getInt32(off + 4, true);
      if (hasAmb) amb[i] = d.getInt32(off + 8, true);
    }
    return { seq, t0Us, rateCode, flags, ir, red, amb };
  }

  /* ---------------- ACCEL batch ---------------- */

  function parseAccel(buf) {
    const d = dv(buf);
    if (d.byteLength < P.ACCEL_HDR_SIZE) fail(`ACCEL too short: ${d.byteLength}`);
    const seq = d.getUint32(0, true);
    const t0Us = u64(d, 4);
    const odrCode = d.getUint8(12);
    const n = d.getUint8(13);
    const flags = d.getUint8(14);
    if (d.byteLength !== P.ACCEL_HDR_SIZE + n * 6) {
      fail(`ACCEL len ${d.byteLength} != ${P.ACCEL_HDR_SIZE + n * 6}`);
    }
    const samples = new Array(n);
    for (let i = 0, off = P.ACCEL_HDR_SIZE; i < n; i++, off += 6) {
      samples[i] = [d.getInt16(off, true), d.getInt16(off + 2, true),
                    d.getInt16(off + 4, true)];
    }
    return { seq, t0Us, odrCode, flags, samples,
             fsCode: flags & P.ACCF_FS_MASK,
             fifoOverrun: !!(flags & P.ACCF_FIFO_OVERRUN) };
  }

  /* ---------------- IBI batch ---------------- */

  function parseIbi(buf) {
    const d = dv(buf);
    if (d.byteLength < P.IBI_HDR_SIZE) fail(`IBI too short: ${d.byteLength}`);
    const seq = d.getUint32(0, true);
    const n = d.getUint8(4);
    if (d.byteLength !== P.IBI_HDR_SIZE + n * P.IBI_REC_SIZE) {
      fail(`IBI len ${d.byteLength} != ${P.IBI_HDR_SIZE + n * P.IBI_REC_SIZE}`);
    }
    const records = new Array(n);
    for (let i = 0, off = P.IBI_HDR_SIZE; i < n; i++, off += P.IBI_REC_SIZE) {
      records[i] = {
        tBeatUs: u64(d, off),
        ibiMs: d.getUint16(off + 8, true),
        confidence: d.getUint8(off + 10),
        flags: d.getUint8(off + 11),
      };
    }
    return { seq, records };
  }

  /* ---------------- EVENT batch ---------------- */
  /* Records are {u8 type, u8 len, payload}; payload starts with u64 t_us.
   * Unknown types (or known types with a foreign len) parse as
   * {type, len, tUs?, raw} and are skippable — never a desync. */

  function decodeEvent(type, payload) {
    const d = dv(payload);
    const len = d.byteLength;
    const tUs = len >= 8 ? u64(d, 0) : null;
    const ev = { type, len, tUs, raw: bytesOf(payload) };
    switch (type) {
      case P.EV_AGC_STEP:
        if (len !== P.EVLEN_AGC_STEP) break;
        ev.led = d.getUint8(8); ev.oldMa = d.getUint8(9); ev.newMa = d.getUint8(10);
        ev.oldRf = d.getUint8(11); ev.newRf = d.getUint8(12);
        ev.known = true; break;
      case P.EV_GATE:
        if (len !== P.EVLEN_GATE) break;
        ev.state = d.getUint8(8); ev.reasonMask = d.getUint8(9);
        ev.known = true; break;
      case P.EV_WEAR:
        if (len !== P.EVLEN_WEAR) break;
        ev.worn = d.getUint8(8);
        ev.known = true; break;
      case P.EV_MARKER:
        if (len !== P.EVLEN_MARKER) break;
        ev.source = d.getUint8(8); ev.markerId = d.getUint16(9, true);
        ev.known = true; break;
      case P.EV_ERROR:
        if (len !== P.EVLEN_ERROR) break;
        ev.code = d.getUint16(8, true); ev.arg = d.getUint32(10, true);
        ev.known = true; break;
      case P.EV_RATE_CHANGE:
        if (len !== P.EVLEN_RATE_CHANGE) break;
        ev.oldCode = d.getUint8(8); ev.newCode = d.getUint8(9);
        ev.known = true; break;
      case P.EV_AGC_OFFDAC:
        if (len !== P.EVLEN_AGC_OFFDAC) break;
        ev.phase = d.getUint8(8); ev.oldCode = d.getUint8(9); ev.newCode = d.getUint8(10);
        ev.known = true; break;
      case P.EV_SELFTEST_DONE:
        if (len !== P.EVLEN_SELFTEST_DONE) break;
        ev.passCount = d.getUint8(8); ev.failCount = d.getUint8(9);
        ev.known = true; break;
      default:
        break;
    }
    if (!ev.known) ev.known = false;
    return ev;
  }

  function parseEventBatch(buf) {
    const b = bytesOf(buf);
    if (b.length < P.EVENT_HDR_SIZE) fail(`EVENT too short: ${b.length}`);
    const d = dv(buf);
    const seq = d.getUint32(0, true);
    const n = d.getUint8(4);
    const events = [];
    let off = P.EVENT_HDR_SIZE;
    for (let i = 0; i < n; i++) {
      if (off + 2 > b.length) fail("truncated event record header");
      const type = b[off], len = b[off + 1];
      off += 2;
      if (off + len > b.length) fail("truncated event record payload");
      events.push(decodeEvent(type, b.subarray(off, off + len)));
      off += len;
    }
    if (off !== b.length) fail(`trailing bytes after ${n} event records`);
    return { seq, events };
  }

  /* ---------------- self-test blob ---------------- */
  /* [u8 blob_ver][u64 t_run_us][u8 n][n x {u8 id,u8 status,i32 val,i32 thr}] */

  function parseSelftestBlob(buf) {
    const d = dv(buf);
    if (d.byteLength < 10) fail(`selftest blob too short: ${d.byteLength}`);
    const blobVer = d.getUint8(0);
    const tRunUs = u64(d, 1);
    const n = d.getUint8(9);
    if (d.byteLength !== 10 + n * P.ST_REC_SIZE) {
      fail(`selftest blob len ${d.byteLength} != ${10 + n * P.ST_REC_SIZE}`);
    }
    const records = new Array(n);
    for (let i = 0, off = 10; i < n; i++, off += P.ST_REC_SIZE) {
      records[i] = {
        id: d.getUint8(off), status: d.getUint8(off + 1),
        value: d.getInt32(off + 2, true), threshold: d.getInt32(off + 6, true),
      };
    }
    return { blobVer, tRunUs, records };
  }

  /* ---------------- knob discovery chunk ---------------- */
  /* [u16 total][u16 first_idx][u8 n][n x record]; record = {u16 id, u8 type,
   * u8 flags, i32 min, i32 max, i32 def, i32 current, u8 name_len, name...,
   * u8 unit_len, unit...} */

  function parseKnobDiscoverChunk(payload) {
    const b = bytesOf(payload);
    const d = dv(payload);
    if (b.length < P.KNOB_DISC_HDR_SIZE) fail("knob discover chunk too short");
    const total = d.getUint16(0, true);
    const firstIdx = d.getUint16(2, true);
    const n = d.getUint8(4);
    const records = [];
    let off = P.KNOB_DISC_HDR_SIZE;
    const ascii = (start, len) => {
      let s = "";
      for (let i = 0; i < len; i++) s += String.fromCharCode(b[start + i]);
      return s;
    };
    for (let i = 0; i < n; i++) {
      if (off + P.KNOB_REC_FIXED > b.length) fail("truncated knob record");
      const rec = {
        id: d.getUint16(off, true),
        type: d.getUint8(off + 2),
        flags: d.getUint8(off + 3),
        min: d.getInt32(off + 4, true),
        max: d.getInt32(off + 8, true),
        def: d.getInt32(off + 12, true),
        current: d.getInt32(off + 16, true),
      };
      const nameLen = d.getUint8(off + 20);
      off += P.KNOB_REC_FIXED; /* fixed part ends after name_len */
      if (off + nameLen + 1 > b.length) fail("truncated knob name");
      rec.name = ascii(off, nameLen);
      off += nameLen;
      const unitLen = b[off];
      off += 1;
      if (off + unitLen > b.length) fail("truncated knob unit");
      rec.unit = ascii(off, unitLen);
      off += unitLen;
      records.push(rec);
    }
    if (off !== b.length) fail("trailing bytes after knob records");
    return { total, firstIdx, records };
  }

  /* ---------------- misc value formatters ---------------- */

  function opName(op) { return P.OP_NAME[op] || `0x${op.toString(16)}`; }
  function statusName(st) { return P.CTRL_STATUS_NAME[st] || `status ${st}`; }
  function fsCountsPerG(fsCode) {
    /* LIS2DH12 i16 left-justified: ±2g -> 16384 counts/g, halves per range */
    return 16384 >> (fsCode & P.ACCF_FS_MASK);
  }

  /* ---------------- OTA (firmware flashing over BLE) ------------------- */
  /* OTA_CTRL uses the CONTROL envelope ([op][tid] -> [op|0x80][tid][st]);
   * OTA_DATA is raw write-no-response frames [u32 offset][chunk]. Matches
   * ota.c / proto.h NC_OTA_* and tools/narbis_client/ota.py. */

  /* CRC-32 (zlib polynomial), incremental: crc32(prev, bytes). Must match
   * the device's nc_crc32 and python zlib.crc32. */
  const CRC_TAB = (() => {
    const t = new Uint32Array(256);
    for (let n = 0; n < 256; n++) {
      let c = n;
      for (let k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
      t[n] = c >>> 0;
    }
    return t;
  })();
  function crc32(crc, bytes) {
    let c = (crc ^ 0xFFFFFFFF) >>> 0;
    for (let i = 0; i < bytes.length; i++) {
      c = (CRC_TAB[(c ^ bytes[i]) & 0xFF] ^ (c >>> 8)) >>> 0;
    }
    return (c ^ 0xFFFFFFFF) >>> 0;
  }

  const ota = {
    crc32,
    buildBegin(tid, size, crc, versionStr) {
      const v = new TextEncoder().encode(versionStr || "");
      const out = new Uint8Array(2 + 9 + v.length);
      const d = dv(out);
      out[0] = P.OTA_BEGIN; out[1] = tid;
      d.setUint32(2, size >>> 0, true);
      d.setUint32(6, crc >>> 0, true);
      out[10] = v.length;
      out.set(v, 11);
      return out;
    },
    buildStatus(tid) { return Uint8Array.of(P.OTA_STATUS, tid); },
    buildFinish(tid) { return Uint8Array.of(P.OTA_FINISH, tid); },
    buildAbort(tid) { return Uint8Array.of(P.OTA_ABORT, tid); },
    buildData(offset, chunk) {
      const out = new Uint8Array(4 + chunk.length);
      dv(out).setUint32(0, offset >>> 0, true);
      out.set(chunk, 4);
      return out;
    },
    parseBeginResp(payload) {
      return { resumeOffset: dv(payload).getUint32(0, true) };
    },
    parseStatusResp(payload) {
      const d = dv(payload);
      return { state: d.getUint8(0), bytesRx: d.getUint32(1, true),
               lastErr: d.getUint16(5, true) };
    },
    stateName(s) {
      return ["IDLE", "RECEIVING", "VALIDATING", "READY", "FAILED"][s] || `state${s}`;
    },
    errName(e) {
      return ["NONE", "EXPECTED_OFFSET", "SIZE", "CRC", "IMAGE", "FLASH",
              "STATE"][e] || `err${e}`;
    },
  };

  return {
    P,
    dv, bytesOf,
    buildControlRequest, parseControlResponse,
    build, pack,
    parseKnobGetResp, parseTimeSyncResp, parseGetTimeResp,
    parseBattRawResp, parseRateCountResp, parseSelftestRecord,
    parseChunk, parseStatus, parsePpg, parseAccel, parseIbi,
    parseEventBatch, decodeEvent, parseSelftestBlob, parseKnobDiscoverChunk,
    opName, statusName, fsCountsPerG,
    ota,
  };
})();

if (typeof module !== "undefined" && module.exports) {
  module.exports = NarbisProto;
}
