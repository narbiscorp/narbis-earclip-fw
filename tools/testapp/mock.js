/*
 * mock.js — in-page fake Web Bluetooth device for the Narbis test app.
 *
 * Presents the exact interface subset app.js consumes from real Web
 * Bluetooth (requestDevice -> device.gatt.connect -> getPrimaryService ->
 * getCharacteristic -> read/write/notifications) and scripts a full
 * V2.1 board walk-through with realistic values, including ONE deliberate
 * failure (crosstalk, T05: 81234 counts > 50000 threshold) so the FAIL
 * rendering and report card get exercised.
 *
 * Activated by ?mock=1 (app.js picks NarbisMock.create() over
 * navigator.bluetooth). Also loads under node (selfcheck.mjs) — no DOM
 * APIs used, events are a hand-rolled emitter, timers are setTimeout.
 *
 * opts.timescale compresses every scripted delay (bench steps that take
 * 10-60 s on hardware run in under a second in CI).
 */
"use strict";

const NarbisMock = (() => {
  const NP = (typeof NarbisProto !== "undefined")
    ? NarbisProto
    : require("./proto.js");
  const P = NP.P;

  const MOCK_NAME = "Narbis Edge Earclip TEST";

  /* Deterministic PRNG so selfcheck assertions are stable run-to-run. */
  function mulberry32(seed) {
    let a = seed >>> 0;
    return () => {
      a |= 0; a = (a + 0x6D2B79F5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  /* ---------------- minimal event plumbing (browser + node) ------------ */

  class Emitter {
    constructor() { this._listeners = Object.create(null); }
    addEventListener(type, fn) {
      (this._listeners[type] = this._listeners[type] || []).push(fn);
    }
    removeEventListener(type, fn) {
      const l = this._listeners[type];
      if (l) this._listeners[type] = l.filter((f) => f !== fn);
    }
    _emit(type, extra) {
      const ev = Object.assign({ type, target: this }, extra);
      for (const fn of (this._listeners[type] || []).slice()) fn(ev);
    }
  }

  /* ---------------- GATT façade ---------------- */

  class MockCharacteristic extends Emitter {
    constructor(dev, uuid) {
      super();
      this.uuid = uuid;
      this.value = null;
      this._dev = dev;
      this._notifying = false;
    }
    async readValue() {
      const bytes = this._dev._read(this.uuid);
      this.value = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
      return this.value;
    }
    async writeValue(data) { this._dev._write(this.uuid, NP.bytesOf(data)); }
    async writeValueWithResponse(data) { return this.writeValue(data); }
    async writeValueWithoutResponse(data) { return this.writeValue(data); }
    async startNotifications() { this._notifying = true; return this; }
    async stopNotifications() { this._notifying = false; return this; }
    _push(bytes) {
      if (!this._notifying || !this._dev._connected) return;
      this.value = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
      this._emit("characteristicvaluechanged");
    }
  }

  class MockService {
    constructor(dev, uuid) { this._dev = dev; this.uuid = uuid; }
    async getCharacteristic(uuid) { return this._dev._char(uuid); }
  }

  class MockGattServer {
    constructor(dev) { this.device = dev; }
    get connected() { return this.device._connected; }
    async connect() {
      this.device._connected = true;
      this.device._onConnect();
      return this;
    }
    disconnect() { this.device._disconnect(false); }
    async getPrimaryService(uuid) {
      const known = [P.UUID.SENSOR_SVC, P.UUID.OTA_SVC, P.UUID.BATTERY_SVC,
                     P.UUID.DEVICE_INFO_SVC, P.UUID.HEART_RATE_SVC];
      if (!known.includes(uuid)) throw new Error(`mock: no service ${uuid}`);
      return new MockService(this.device, uuid);
    }
  }

  /* ---------------- the scripted device ---------------- */

  class MockDevice extends Emitter {
    constructor(opts) {
      super();
      this.name = MOCK_NAME;
      this.id = "mock-earclip-0001";
      this.gatt = new MockGattServer(this);
      this._ts = (opts && opts.timescale) || 1;
      this._rand = mulberry32((opts && opts.seed) || 0xB10C0);
      this._connected = false;
      this._chars = new Map();
      this._timers = new Set();

      /* device state */
      this._bootUs = 0;
      this._t0 = nowMs();
      this._rate = P.RATE_100;
      this._streamMask = 0;
      this._ppgSeq = 0; this._accelSeq = 0; this._eventSeq = 0;
      this._buttonEcho = false;
      this._chargerLive = false;
      this._accelLive = false;
      this._chargerFlags = 0;          /* STF bits reflected in STATUS */
      this._selftestRan = [];          /* records accumulated since boot */
      this._report = {
        device: MOCK_NAME,
        serial: "MOCK-0001",
        hw: "V2.1",
        tests: {},
      };
    }

    /* ---- characteristic registry ---- */
    _char(uuid) {
      if (!this._chars.has(uuid)) {
        this._chars.set(uuid, new MockCharacteristic(this, uuid));
      }
      return this._chars.get(uuid);
    }

    /* ---- lifetime ---- */
    _onConnect() {
      /* STATUS at 1 Hz like the firmware sys_task */
      this._every(1000, () => this._pushStatus());
    }
    _disconnect(fromDevice) {
      if (!this._connected) return;
      this._connected = false;
      for (const id of this._timers) clearTimeout(id);
      this._timers.clear();
      this._streamMask = 0;
      this._buttonEcho = this._chargerLive = this._accelLive = false;
      this._emit("gattserverdisconnected", { fromDevice });
    }

    /* ---- scaled timers, all tracked for cleanup ---- */
    _after(ms, fn) {
      const id = setTimeout(() => { this._timers.delete(id); fn(); },
                            Math.max(1, ms * this._ts));
      this._timers.add(id);
      return id;
    }
    _every(ms, fn) {
      const tick = () => {
        if (!this._connected) return;
        fn();
        this._after(ms, tick);
      };
      this._after(ms, tick);
    }

    _devUs() { return Math.round((nowMs() - this._t0) * 1000 / this._ts); }

    /* ---- GATT read handlers ---- */
    _read(uuid) {
      const U = P.UUID;
      const enc = (s) => new TextEncoder().encode(s);
      switch (uuid) {
        case U.MANUFACTURER_NAME: return enc("Narbis");
        case U.MODEL_NUMBER: return enc("Edge Earclip");
        case U.FIRMWARE_REVISION: return enc("v0.9.0-mock");
        case U.HARDWARE_REVISION: return enc("V2.1");
        case U.SOFTWARE_REVISION: return enc("mock");
        case U.BATTERY_LEVEL: return Uint8Array.of(76);
        case U.PROTO_VER: {
          const b = new DataView(new ArrayBuffer(2));
          b.setUint16(0, P.PROTO_VER, true);
          return new Uint8Array(b.buffer);
        }
        case U.STATUS: return this._statusBytes();
        default: throw new Error(`mock: unreadable characteristic ${uuid}`);
      }
    }

    /* ---- GATT write handlers ---- */
    _write(uuid, bytes) {
      if (uuid !== P.UUID.CONTROL) {
        throw new Error(`mock: unwritable characteristic ${uuid}`);
      }
      if (bytes.length < 2) throw new Error("mock: control write too short");
      const op = bytes[0], tid = bytes[1], pl = bytes.slice(2);
      /* dispatch out of the write context, like a real indication */
      this._after(25, () => this._control(op, tid, pl));
    }

    _respond(op, tid, status, payload) {
      /* TEST ops echo the opcode unchanged (bit7 already set) */
      const opRaw = op >= P.OP_TEST_SELFTEST_ONE ? op : (op | P.OP_RESP_FLAG);
      const pl = payload || new Uint8Array(0);
      const out = new Uint8Array(3 + pl.length);
      out[0] = opRaw; out[1] = tid; out[2] = status; out.set(pl, 3);
      this._char(P.UUID.CONTROL)._push(out);
    }

    /* ---- CONTROL dispatcher ---- */
    _control(op, tid, pl) {
      if (!this._connected) return;
      const ok = (payload) => this._respond(op, tid, P.ST_OK, payload);
      switch (op) {
        case P.OP_STREAM_START:
          this._streamMask |= pl[0];
          if (pl[0] & P.STREAM_MASK_PPG) this._startPpg();
          return ok();
        case P.OP_STREAM_STOP:
          this._streamMask &= ~pl[0];
          return ok();
        case P.OP_SET_RATE:
          if (pl[0] >= P.RATE_COUNT) {
            return this._respond(op, tid, P.ST_BAD_PARAM);
          }
          this._rate = pl[0];
          return ok();
        case P.OP_TIME_SYNC: {
          const out = new DataView(new ArrayBuffer(16));
          out.setBigUint64(0, new DataView(pl.buffer, pl.byteOffset).getBigUint64(0, true), true);
          out.setBigUint64(8, BigInt(this._devUs()), true);
          return ok(new Uint8Array(out.buffer));
        }
        case P.OP_GET_TIME: {
          const out = new DataView(new ArrayBuffer(8));
          out.setBigUint64(0, BigInt(this._devUs()), true);
          return ok(new Uint8Array(out.buffer));
        }
        case P.OP_MARKER:
          this._pushEvent(this._markerRecord(P.MARKER_SRC_HOST,
            pl[0] | (pl[1] << 8)));
          return ok();
        case P.OP_KNOB_DISCOVER:
          return ok(this._knobChunk(pl[0] | (pl[1] << 8)));
        case P.OP_KNOB_GET: {
          const id = pl[0] | (pl[1] << 8);
          const k = P.KNOBS.find((x) => x.id === id);
          if (!k) return this._respond(op, tid, P.ST_BAD_PARAM);
          const out = new DataView(new ArrayBuffer(6));
          out.setUint16(0, id, true); out.setInt32(2, k.def, true);
          return ok(new Uint8Array(out.buffer));
        }
        case P.OP_SELFTEST_RUN:
          this._selftestRan = [];
          for (let id = 1; id <= P.TEST_COUNT_; id++) {
            this._selftestRan.push(this._selftestRecord(id));
          }
          this._after(300, () => {
            const failN = this._selftestRan.filter(
              (r) => r.status === P.TR_FAIL).length;
            this._pushEvent(this._selftestDoneRecord(
              this._selftestRan.length - failN, failN));
          });
          return ok();
        case P.OP_SELFTEST_RESULT:
          return ok(this._chunkOf(this._selftestBlob(), pl[0] | (pl[1] << 8)));

        /* ---- TEST block ---- */
        case P.OP_TEST_SELFTEST_ONE: {
          const rec = this._selftestRecord(pl[0]);
          if (!rec) return this._respond(op, tid, P.ST_BAD_PARAM);
          this._selftestRan.push(rec);
          const out = new DataView(new ArrayBuffer(P.ST_REC_SIZE));
          out.setUint8(0, rec.id); out.setUint8(1, rec.status);
          out.setInt32(2, rec.value, true); out.setInt32(6, rec.threshold, true);
          return ok(new Uint8Array(out.buffer));
        }
        case P.OP_TEST_LED_DRIVE: {
          const ma = pl[1];
          const max = pl[0] === 1 ? 40 : 50; /* board.h clamps, mirrored */
          if (ma > max) return this._respond(op, tid, P.ST_OUT_OF_RANGE);
          return ok(); /* the red die glows in our imagination */
        }
        case P.OP_TEST_LED_SWEEP: {
          const led = pl[0], step = Math.max(1, pl[1]);
          const key = led === 1 ? "led_sweep_red" : "led_sweep_ir";
          const maMax = led === 1 ? 40 : 50;
          const maArr = [], dcArr = [];
          for (let ma = 0; ma <= maMax; ma += step) {
            maArr.push(ma);
            /* saturating exponential + supply-sag compression + noise */
            const span = led === 1 ? 30000 : 45000;
            const tau = led === 1 ? 18 : 25;
            const dc = span * (1 - Math.exp(-ma / tau)) +
                       (this._rand() - 0.5) * 220;
            dcArr.push(Math.max(0, Math.round(dc)));
          }
          this._report.tests[key] = { ma: maArr, dc: dcArr };
          return this._after(200, () => this._respond(op, tid, P.ST_OK));
        }
        case P.OP_TEST_RX_SWEEP: {
          const what = pl[0];
          if (what === 0) {
            const code = [], dc = [];
            for (let c = 0; c <= 7; c++) {
              code.push(c);
              dc.push(Math.min(2000000, Math.round(
                7900 * (1 << c) * (1 + (this._rand() - 0.5) * 0.04))));
            }
            this._report.tests.rx_sweep_gain = { code, dc };
          } else {
            const code = [], dc = [];
            for (let c = -7; c <= 7; c++) {
              code.push(c);
              dc.push(Math.round(210000 + c * 29500 + (this._rand() - 0.5) * 400));
            }
            this._report.tests.rx_sweep_dac = { code, dc };
          }
          return this._after(150, () => this._respond(op, tid, P.ST_OK));
        }
        case P.OP_TEST_RATE_COUNT: {
          const seconds = pl[0] || 10;
          ok(); /* immediate ack; result rides a second indication, same tid */
          this._after(seconds * 1000, () => {
            const nominal = P.RATE_SPS[this._rate];
            const pulses = Math.round(nominal * seconds * 0.9995);
            const out = new DataView(new ArrayBuffer(8));
            out.setUint32(0, pulses, true);
            out.setUint32(4, seconds * 1000 + 3, true);
            this._report.tests.rate_count =
              { pulses, elapsed_ms: seconds * 1000 + 3 };
            this._respond(op, tid, P.ST_OK, new Uint8Array(out.buffer));
          });
          return;
        }
        case P.OP_TEST_BUTTON_ECHO:
          this._buttonEcho = !!pl[0];
          if (this._buttonEcho) this._scriptButtonPresses();
          return ok();
        case P.OP_TEST_CHARGER_LIVE:
          this._chargerLive = !!pl[0];
          if (this._chargerLive) this._scriptChargerWalk();
          return ok();
        case P.OP_TEST_BATT_RAW: {
          const out = new DataView(new ArrayBuffer(4));
          out.setUint16(0, 3921, true);   /* calibrated mV */
          out.setUint16(2, 1943, true);   /* raw ADC average */
          this._report.tests.batt_raw = { mv_cal: 3921, adc_raw: 1943 };
          return ok(new Uint8Array(out.buffer));
        }
        case P.OP_TEST_ACCEL_LIVE:
          this._accelLive = !!pl[0];
          if (this._accelLive) this._startAccelOrientation();
          return ok();
        case P.OP_TEST_SLEEP_NOW:
          ok();
          /* firmware sleeps in 10 s; link drops when it does */
          this._after(10000, () => this._disconnect(true));
          return;
        case P.OP_TEST_REPORT:
          return ok(this._chunkOf(this._reportBlob(), pl[0] | (pl[1] << 8)));

        default:
          return this._respond(op, tid, P.ST_UNKNOWN_OP);
      }
    }

    /* ---- scripted content ---- */

    _selftestRecord(id) {
      switch (id) {
        case P.TEST_I2C_SCAN: return { id, status: P.TR_PASS, value: 2, threshold: 2 };
        case P.TEST_ACCEL_WHOAMI: return { id, status: P.TR_PASS, value: 0x33, threshold: 0x33 };
        case P.TEST_AFE_REG_RW: return { id, status: P.TR_PASS, value: 1, threshold: 1 };
        case P.TEST_AFE_DARK: return { id, status: P.TR_PASS, value: 87, threshold: 200 };
        /* THE deliberate failure: open-clip crosstalk way over threshold —
         * exercises FAIL rendering end-to-end. */
        case P.TEST_XTALK: return { id, status: P.TR_FAIL, value: 81234, threshold: 50000 };
        case P.TEST_ACCEL_ST: return { id, status: P.TR_PASS, value: 174, threshold: 360 };
        case P.TEST_BATT: return { id, status: P.TR_PASS, value: 3921, threshold: 3000 };
        case P.TEST_CHARGER: return { id, status: P.TR_PASS, value: 1, threshold: 1 };
        default: return null;
      }
    }

    _selftestBlob() {
      const recs = this._selftestRan;
      const out = new DataView(new ArrayBuffer(10 + recs.length * P.ST_REC_SIZE));
      out.setUint8(0, P.ST_BLOB_VER);
      out.setBigUint64(1, BigInt(this._devUs()), true);
      out.setUint8(9, recs.length);
      recs.forEach((r, i) => {
        const off = 10 + i * P.ST_REC_SIZE;
        out.setUint8(off, r.id); out.setUint8(off + 1, r.status);
        out.setInt32(off + 2, r.value, true);
        out.setInt32(off + 6, r.threshold, true);
      });
      return new Uint8Array(out.buffer);
    }

    /* Consolidated TEST report: UTF-8 JSON blob (see README interface
     * note — firmware's "JSON-ish TLV" must serialize to this shape). */
    _reportBlob() {
      return new TextEncoder().encode(JSON.stringify({
        ...this._report,
        selftest: this._selftestRan,
        t_us: this._devUs(),
      }));
    }

    _chunkOf(blob, offset) {
      const maxData = P.ATT_PAYLOAD_MAX - 3 - 5; /* resp hdr + chunk hdr */
      const slice = blob.slice(offset, offset + maxData);
      const out = new Uint8Array(5 + slice.length);
      const d = new DataView(out.buffer);
      d.setUint16(0, blob.length, true);
      d.setUint16(2, offset, true);
      d.setUint8(4, slice.length);
      out.set(slice, 5);
      return out;
    }

    _knobChunk(startIndex) {
      const knobs = P.KNOBS;
      const parts = [];
      let used = P.KNOB_DISC_HDR_SIZE, idx = startIndex;
      while (idx < knobs.length) {
        const k = knobs[idx];
        const recLen = P.KNOB_REC_FIXED + k.name.length + 1 + k.unit.length;
        if (used + recLen > P.ATT_PAYLOAD_MAX - 3) break;
        const rec = new Uint8Array(recLen);
        const d = new DataView(rec.buffer);
        d.setUint16(0, k.id, true);
        d.setUint8(2, k.type); d.setUint8(3, k.flags);
        d.setInt32(4, k.min, true); d.setInt32(8, k.max, true);
        d.setInt32(12, k.def, true); d.setInt32(16, k.def, true);
        d.setUint8(20, k.name.length);
        for (let i = 0; i < k.name.length; i++) rec[21 + i] = k.name.charCodeAt(i);
        rec[21 + k.name.length] = k.unit.length;
        for (let i = 0; i < k.unit.length; i++) {
          rec[22 + k.name.length + i] = k.unit.charCodeAt(i);
        }
        parts.push(rec);
        used += recLen;
        idx++;
      }
      const out = new Uint8Array(used);
      const d = new DataView(out.buffer);
      d.setUint16(0, knobs.length, true);
      d.setUint16(2, startIndex, true);
      d.setUint8(4, parts.length);
      let off = P.KNOB_DISC_HDR_SIZE;
      for (const p of parts) { out.set(p, off); off += p.length; }
      return out;
    }

    /* ---- STATUS ---- */
    _statusBytes() {
      const d = new DataView(new ArrayBuffer(P.STATUS_SIZE));
      d.setUint8(0, this._streamMask ? P.STATE_STREAMING : P.STATE_CONNECTED);
      d.setUint8(1, P.STF_WORN | this._chargerFlags);
      d.setUint16(2, 3921, true);
      d.setUint8(4, 76);
      d.setUint8(5, this._rate);
      d.setUint8(6, 12); d.setUint8(7, 8);   /* LED mA */
      d.setUint8(8, 4); d.setUint8(9, 2);    /* TIA RF/CF codes */
      d.setUint16(10, 0, true);
      d.setUint32(12, 0, true);
      d.setUint16(16, 0, true);
      d.setInt16(18, -23, true);
      d.setUint32(20, Math.floor(this._devUs() / 1e6), true);
      d.setUint16(24, 833, true);
      d.setUint8(26, 72);
      return new Uint8Array(d.buffer);
    }
    _pushStatus() { this._char(P.UUID.STATUS)._push(this._statusBytes()); }

    /* ---- EVENT stream ---- */
    /* Records carry SCHEDULED device time, not wall time at fire: browser
     * timers coalesce under throttling, but real firmware stamps in the
     * ISR — bunched notifications with honest timestamps is exactly what
     * hardware does, so the app must cope with it anyway. */
    _markerRecord(source, markerId, tUs) {
      const rec = new Uint8Array(2 + P.EVLEN_MARKER);
      const d = new DataView(rec.buffer);
      rec[0] = P.EV_MARKER; rec[1] = P.EVLEN_MARKER;
      d.setBigUint64(2, BigInt(tUs === undefined ? this._devUs() : tUs), true);
      d.setUint8(10, source);
      d.setUint16(11, markerId, true);
      return rec;
    }
    _selftestDoneRecord(passN, failN) {
      const rec = new Uint8Array(2 + P.EVLEN_SELFTEST_DONE);
      const d = new DataView(rec.buffer);
      rec[0] = P.EV_SELFTEST_DONE; rec[1] = P.EVLEN_SELFTEST_DONE;
      d.setBigUint64(2, BigInt(this._devUs()), true);
      d.setUint8(10, passN); d.setUint8(11, failN);
      return rec;
    }
    /* Charger snapshot event: type == OP_TEST_CHARGER_LIVE (0xE6), payload
     * {u64 t_us, u8 vusb, u8 stat_raw, u8 chg_state} — the TEST-mode
     * interface assumption documented in the README. */
    _chargerRecord(vusb, statRaw, chgState, tUs) {
      const rec = new Uint8Array(2 + 11);
      const d = new DataView(rec.buffer);
      rec[0] = P.OP_TEST_CHARGER_LIVE; rec[1] = 11;
      d.setBigUint64(2, BigInt(tUs === undefined ? this._devUs() : tUs), true);
      d.setUint8(10, vusb); d.setUint8(11, statRaw); d.setUint8(12, chgState);
      return rec;
    }
    _pushEvent(...records) {
      let len = 0;
      for (const r of records) len += r.length;
      const out = new Uint8Array(P.EVENT_HDR_SIZE + len);
      const d = new DataView(out.buffer);
      d.setUint32(0, this._eventSeq++, true);
      d.setUint8(4, records.length);
      let off = P.EVENT_HDR_SIZE;
      for (const r of records) { out.set(r, off); off += r.length; }
      this._char(P.UUID.EVENT)._push(out);
    }

    /* Generators below are schedule-driven with catch-up: each tick emits
     * everything due per the sim clock instead of one item per timer.
     * Browser tabs throttle chained timers (hidden tab: 1 Hz) — catch-up
     * keeps the scripted data complete and correctly timestamped no
     * matter how the timers are coalesced. */

    _scriptButtonPresses() {
      /* 5 presses: down edge (marker_id = press<<1|1), up edge 120 ms
       * later (marker_id = press<<1), presses spaced 700 ms. */
      const t0 = this._devUs();
      const edges = [];
      for (let press = 1; press <= 5; press++) {
        const base = 600 + (press - 1) * 700;
        edges.push([t0 + base * 1000, (press << 1) | 1]);
        edges.push([t0 + (base + 120) * 1000, press << 1]);
      }
      let i = 0;
      const tick = () => {
        if (!this._connected || !this._buttonEcho || i >= edges.length) return;
        const now = this._devUs();
        while (i < edges.length && edges[i][0] <= now) {
          this._pushEvent(this._markerRecord(P.MARKER_SRC_BUTTON,
                                             edges[i][1], edges[i][0]));
          i++;
        }
        this._after(40, tick);
      };
      this._after(40, tick);
    }

    _scriptChargerWalk() {
      /* 2 Hz snapshots: 4 on-battery -> operator "plugs in" -> 6 charging ->
       * 4 complete -> "unplug" -> 4 on-battery. STAT raw bit: on this mock,
       * LOW while charging (the polarity question the bench resolves). */
      const script = [];
      for (let i = 0; i < 4; i++) script.push([0, 1, P.CHG_ON_BATTERY]);
      for (let i = 0; i < 6; i++) script.push([1, 0, P.CHG_CHARGING]);
      for (let i = 0; i < 4; i++) script.push([1, 1, P.CHG_COMPLETE]);
      for (let i = 0; i < 4; i++) script.push([0, 1, P.CHG_ON_BATTERY]);
      const t0 = this._devUs();
      let i = 0;
      const tick = () => {
        if (!this._connected || !this._chargerLive || i >= script.length) return;
        const now = this._devUs();
        while (i < script.length && t0 + 500000 * (i + 1) <= now) {
          const [vusb, stat, state] = script[i];
          this._chargerFlags =
            (state === P.CHG_CHARGING ? P.STF_CHARGING : 0) |
            (state === P.CHG_COMPLETE ? P.STF_CHARGE_DONE : 0) |
            (vusb ? P.STF_USB : 0);
          this._pushEvent(this._chargerRecord(vusb, stat, state,
                                              t0 + 500000 * (i + 1)));
          i++;
        }
        this._after(100, tick);
      };
      this._after(100, tick);
    }

    /* ---- PPG stream (schedule-driven, catch-up per tick) ---- */
    _startPpg() {
      if (this._ppgActive) return;
      this._ppgActive = true;
      this._ppgSeq = 0;
      let sampleIdx = 0;
      const batchUs = 50000; /* ppg_batch_ms default */
      const startUs = this._devUs();
      let nextDueUs = startUs + batchUs;
      const tick = () => {
        if (!this._connected || !(this._streamMask & P.STREAM_MASK_PPG)) {
          this._ppgActive = false;
          return;
        }
        const now = this._devUs();
        while (nextDueUs <= now) {
          const sps = P.RATE_SPS[this._rate];
          const n = Math.max(1, Math.round(sps * batchUs / 1e6));
          const out = new Uint8Array(P.PPG_HDR_SIZE + n * 8);
          const d = new DataView(out.buffer);
          d.setUint32(0, this._ppgSeq++, true);
          d.setBigUint64(4, BigInt(Math.round(sampleIdx * 1e6 / sps)), true);
          d.setUint8(12, this._rate);
          d.setUint8(13, n);
          d.setUint8(14, 0);
          for (let i = 0; i < n; i++, sampleIdx++) {
            const t = sampleIdx / sps;
            const phase = (t % 0.833) / 0.833; /* 72 bpm */
            const pulse = Math.exp(-phase * 5) +
                          0.35 * Math.exp(-Math.pow((phase - 0.45) / 0.09, 2));
            const ir = 152000 + Math.round(7400 * pulse + (this._rand() - 0.5) * 260);
            const red = 118000 + Math.round(4200 * pulse + (this._rand() - 0.5) * 260);
            d.setInt32(P.PPG_HDR_SIZE + i * 8, ir, true);
            d.setInt32(P.PPG_HDR_SIZE + i * 8 + 4, red, true);
          }
          this._char(P.UUID.PPG)._push(out);
          nextDueUs += batchUs;
        }
        this._after(25, tick);
      };
      this._after(25, tick);
    }

    /* ---- accel orientation script (max ODR live, catch-up) ---- */
    _startAccelOrientation() {
      this._accelSeq = 0;
      /* rotate through the six ±1g faces, 2 s per face */
      const faces = [[1, 0, 0], [-1, 0, 0], [0, 1, 0],
                     [0, -1, 0], [0, 0, 1], [0, 0, -1]];
      const cpg = NP.fsCountsPerG(1); /* FS ±4g */
      const startUs = this._devUs();
      const perBatchUs = 62500; /* 25 samples @ 400 Hz */
      let k = 0;
      const tick = () => {
        if (!this._connected || !this._accelLive) return;
        const now = this._devUs();
        while (startUs + (k + 1) * perBatchUs <= now) {
          const tBatch = startUs + k * perBatchUs;
          const face = faces[Math.min(5,
            Math.floor((tBatch - startUs) / 2000000))];
          const n = 25;
          const out = new Uint8Array(P.ACCEL_HDR_SIZE + n * 6);
          const d = new DataView(out.buffer);
          d.setUint32(0, this._accelSeq++, true);
          d.setBigUint64(4, BigInt(tBatch), true);
          d.setUint8(12, P.ODR_400);
          d.setUint8(13, n);
          d.setUint8(14, 1); /* FS code 1 = ±4g, no FIFO overrun */
          for (let i = 0; i < n; i++) {
            for (let ax = 0; ax < 3; ax++) {
              const g = face[ax] + (this._rand() - 0.5) * 0.06;
              d.setInt16(P.ACCEL_HDR_SIZE + i * 6 + ax * 2,
                         Math.max(-32768, Math.min(32767, Math.round(g * cpg))),
                         true);
            }
          }
          this._char(P.UUID.ACCEL)._push(out);
          k++;
        }
        this._after(30, tick);
      };
      this._after(30, tick);
    }
  }

  function nowMs() {
    return (typeof performance !== "undefined") ? performance.now() : Date.now();
  }

  /* navigator.bluetooth-shaped entry point */
  function create(opts) {
    return {
      isMock: true,
      timescale: (opts && opts.timescale) || 1,
      async getAvailability() { return true; },
      async requestDevice(options) {
        /* honor the app's namePrefix filter so the connect flow is honest */
        const filters = (options && options.filters) || [];
        const okName = filters.some(
          (f) => f.namePrefix && MOCK_NAME.startsWith(f.namePrefix));
        if (filters.length && !okName) {
          throw new Error("mock: no device matches the requested filters");
        }
        return new MockDevice(opts);
      },
    };
  }

  return { create, MOCK_NAME };
})();

if (typeof module !== "undefined" && module.exports) {
  module.exports = NarbisMock;
}
