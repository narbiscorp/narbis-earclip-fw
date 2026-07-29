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
      this._ibiSeq = 0;
      this._buttonEcho = false;
      this._chargerT0 = null;          /* device-us of first 0xE6 poll  */
      this._accelLive = false;
      this._chargerFlags = 0;          /* STF bits reflected in STATUS */
      /* live front-end state (STATUS + PPG DC respond to these) */
      this._ledIr = 12; this._ledRed = 8; this._rf = 4;
      this._agcFrozen = false;
      this._battMv0 = 3921;            /* drains slowly in sim mode */
      this._btnLevel = 0;              /* 1 = held (STATUS byte 27) */
      this._sweep = { on: false, mask: 0, phaseS: 5, t0Us: 0 };
      this._knobVals = new Map();      /* id -> value (set overrides def) */
      this._worn = true;
      this._gated = false;
      /* sim mode: continuous physiology/motion generators (dashboard
       * demo). OFF by default so the deterministic scripted walk that
       * selfcheck section [2] asserts against stays byte-stable. */
      this._sim = !!(opts && opts.sim);
      if (this._sim) this._knobVals.set(0x0302, 1); /* amb_stream on */
      this._fwRev = "v0.9.0-mock";
      this._otaGen = 0;
      this._ota = { state: P.OTA_IDLE, size: 0, crc: 0, rx: 0, run: 0,
                    lastErr: P.OTAERR_NONE };
      this._selftestRan = [];          /* records accumulated since boot */
      /* firmware truth (test_ops.c): ONE static sweep report, binary
       * blob ver 2, overwritten by each 0xE2/0xE3; 0xEA serves it (or
       * falls back to the selftest blob when no sweep has run). */
      this._sweepBlob = null;
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
      if (this._sim) this._startSim();
    }
    _disconnect(fromDevice) {
      if (!this._connected) return;
      this._connected = false;
      for (const id of this._timers) clearTimeout(id);
      this._timers.clear();
      this._streamMask = 0;
      this._buttonEcho = this._accelLive = false;
      this._chargerT0 = null;
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
        case U.FIRMWARE_REVISION: return enc(this._fwRev);
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
      if (uuid === P.UUID.OTA_DATA) {
        this._otaData(bytes);
        return;
      }
      if (uuid === P.UUID.OTA_CTRL) {
        if (bytes.length < 2) throw new Error("mock: ota write too short");
        this._after(15, () => this._otaControl(bytes[0], bytes[1], bytes.slice(2)));
        return;
      }
      if (uuid !== P.UUID.CONTROL) {
        throw new Error(`mock: unwritable characteristic ${uuid}`);
      }
      if (bytes.length < 2) throw new Error("mock: control write too short");
      const op = bytes[0], tid = bytes[1], pl = bytes.slice(2);
      /* dispatch out of the write context, like a real indication */
      this._after(25, () => this._control(op, tid, pl));
    }

    /* ---- OTA engine (mirrors ota.c: offset framing, whole-image CRC,
     * reboot into the new version on FINISH) ---- */
    _respondOta(op, tid, status, payload) {
      const pl = payload || new Uint8Array(0);
      const out = new Uint8Array(3 + pl.length);
      out[0] = op | P.OP_RESP_FLAG; out[1] = tid; out[2] = status;
      out.set(pl, 3);
      this._char(P.UUID.OTA_CTRL)._push(out);
    }

    _otaControl(op, tid, pl) {
      if (!this._connected) return;
      const o = this._ota;
      const d = (b) => new DataView(b.buffer, b.byteOffset, b.byteLength);
      switch (op) {
        case P.OTA_BEGIN: {
          if (pl.length < 9) return this._respondOta(op, tid, P.ST_BAD_LEN);
          const size = d(pl).getUint32(0, true);
          const crc = d(pl).getUint32(4, true);
          if (size === 0 || size > 0x190000) {
            o.lastErr = P.OTAERR_SIZE;
            return this._respondOta(op, tid, P.ST_BAD_PARAM);
          }
          /* resume only for the identical interrupted image */
          if (!(o.state === P.OTA_RECEIVING && o.size === size && o.crc === crc)) {
            o.size = size; o.crc = crc; o.rx = 0; o.run = 0;
          }
          o.state = P.OTA_RECEIVING;
          o.lastErr = P.OTAERR_NONE;
          const resp = new Uint8Array(4);
          d(resp).setUint32(0, o.rx, true);
          return this._respondOta(op, tid, P.ST_OK, resp);
        }
        case P.OTA_STATUS: {
          const resp = new Uint8Array(7);
          resp[0] = o.state;
          d(resp).setUint32(1, o.rx, true);
          d(resp).setUint16(5, o.lastErr, true);
          return this._respondOta(op, tid, P.ST_OK, resp);
        }
        case P.OTA_FINISH: {
          if (o.state !== P.OTA_RECEIVING || o.rx !== o.size) {
            o.lastErr = P.OTAERR_STATE;
            return this._respondOta(op, tid, P.ST_WRONG_STATE);
          }
          if ((o.run >>> 0) !== (o.crc >>> 0)) {
            o.state = P.OTA_FAILED; o.lastErr = P.OTAERR_CRC;
            return this._respondOta(op, tid, P.ST_CRC_ERR);
          }
          o.state = P.OTA_READY;
          this._respondOta(op, tid, P.ST_OK);
          /* "reboot": drop the link, come back with the new version */
          this._after(150, () => {
            this._fwRev = `${this._fwRev.split("+")[0]}+ota${++this._otaGen}`;
            this._disconnect(true);
          });
          return undefined;
        }
        case P.OTA_ABORT:
          o.state = P.OTA_IDLE; o.rx = 0; o.run = 0;
          return this._respondOta(op, tid, P.ST_OK);
        default:
          return this._respondOta(op, tid, P.ST_UNKNOWN_OP);
      }
    }

    _otaData(bytes) {
      const o = this._ota;
      if (o.state !== P.OTA_RECEIVING || bytes.length < 5) return;
      const off = new DataView(bytes.buffer, bytes.byteOffset).getUint32(0, true);
      if (off !== o.rx) {
        /* out-of-order: latch and ignore until the host reseeks (ota.c) */
        o.lastErr = P.OTAERR_EXPECTED_OFFSET;
        return;
      }
      const chunk = bytes.slice(4);
      o.run = NP.ota.crc32(o.rx === 0 ? 0 : o.run, chunk);
      o.rx += chunk.length;
      o.lastErr = P.OTAERR_NONE;
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
          if (!this._sim && (pl[0] & P.STREAM_MASK_PPG)) this._startPpg();
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
          const cur = this._knobVals.has(id) ? this._knobVals.get(id) : k.def;
          const out = new DataView(new ArrayBuffer(6));
          out.setUint16(0, id, true); out.setInt32(2, cur, true);
          return ok(new Uint8Array(out.buffer));
        }
        case P.OP_KNOB_SET: {
          if (pl.length !== 6) return this._respond(op, tid, P.ST_BAD_LEN);
          const d6 = new DataView(pl.buffer, pl.byteOffset);
          const id = d6.getUint16(0, true), val = d6.getInt32(2, true);
          const k = P.KNOBS.find((x) => x.id === id);
          if (!k) return this._respond(op, tid, P.ST_BAD_PARAM);
          if (val < k.min || val > k.max) {
            return this._respond(op, tid, P.ST_OUT_OF_RANGE);
          }
          this._knobVals.set(id, val);
          if (id === 0x0301) this._rate = val;       /* ppg_rate mirrors */
          return this._respond(op, tid,
            (k.flags & P.KF_REBOOT) ? P.ST_NEEDS_RESTART : P.ST_OK);
        }
        case P.OP_KNOB_SAVE:
          return ok();
        case P.OP_KNOB_RESET:
          this._knobVals.clear();
          if (this._sim) this._knobVals.set(0x0302, 1);
          return ok();
        case P.OP_AGC_FREEZE:
          this._agcFrozen = !!pl[0];
          return ok();
        case P.OP_AGC_MANUAL: {
          if (pl.length !== 4) return this._respond(op, tid, P.ST_BAD_LEN);
          const [ir, red, rf, mask] = pl;
          if ((mask & P.AGC_APPLY_IR) && ir > 50) {
            return this._respond(op, tid, P.ST_OUT_OF_RANGE);
          }
          if ((mask & P.AGC_APPLY_RED) && red > 40) {
            return this._respond(op, tid, P.ST_OUT_OF_RANGE);
          }
          if ((mask & P.AGC_APPLY_GAIN) && rf > 7) {
            return this._respond(op, tid, P.ST_OUT_OF_RANGE);
          }
          if (mask & P.AGC_APPLY_IR) this._ledIr = ir;
          if (mask & P.AGC_APPLY_RED) this._ledRed = red;
          if (mask & P.AGC_APPLY_GAIN) this._rf = rf;
          return ok();
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
          this._sweepBlob = null;  /* firmware: selftest_execute clears the
                                      external (sweep) blob */
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
          /* firmware: blocks sys ctx ~200 ms/point, publishes the binary
           * v2 blob (this sweep only), responds {u8 n, u16 blob_len} */
          const led = pl[0], step = Math.max(1, pl[1]);
          const maMax = led === 1 ? 40 : 50;
          const pts = [];
          for (let ma = 0; ma <= maMax; ma += step) {
            /* saturating exponential + supply-sag compression + noise */
            const span = led === 1 ? 30000 : 45000;
            const tau = led === 1 ? 18 : 25;
            const amb = 900 + Math.round((this._rand() - 0.5) * 60);
            const dc = Math.max(0, Math.round(
              span * (1 - Math.exp(-ma / tau)) + (this._rand() - 0.5) * 220));
            /* the swept LED's channel carries the signal; the other
             * channel idles near ambient */
            pts.push({
              setting: ma,
              ir: led === 0 ? dc + amb : amb + Math.round(this._rand() * 40),
              red: led === 1 ? dc + amb : amb + Math.round(this._rand() * 40),
              amb,
            });
          }
          const blob = this._buildSweepBlob(1, led, pts);
          return this._after(200 * pts.length / 10, () => {
            this._sweepBlob = blob;
            const resp = new Uint8Array(3);
            resp[0] = pts.length;
            new DataView(resp.buffer).setUint16(1, blob.length, true);
            this._respond(op, tid, P.ST_OK, resp);
          });
        }
        case P.OP_TEST_RX_SWEEP: {
          const what = pl[0];
          const RF_OHMS = [5e5, 2.5e5, 1e5, 5e4, 2.5e4, 1e4, 1e6, 2e6];
          const pts = [];
          if (what === 0) {
            /* fixed 20 mA IR drive: dc tracks RF ohms — and the code
             * ladder is NON-monotonic, exactly like hardware */
            for (let c = 0; c <= 7; c++) {
              const amb = 900 + Math.round((this._rand() - 0.5) * 60);
              const dc = Math.min(4194303, Math.round(
                0.79 * RF_OHMS[c] * (1 + (this._rand() - 0.5) * 0.04)));
              pts.push({ setting: c, ir: dc + amb,
                         red: amb + Math.round(this._rand() * 40), amb });
            }
          } else {
            for (let c = -15; c <= 15; c++) {
              const amb = 900 + Math.round((this._rand() - 0.5) * 60);
              pts.push({ setting: c,
                         ir: Math.round(210000 + c * 13500 +
                                        (this._rand() - 0.5) * 400),
                         red: amb + Math.round(this._rand() * 40), amb });
            }
          }
          const blob = this._buildSweepBlob(2, what, pts);
          return this._after(150, () => {
            this._sweepBlob = blob;
            const resp = new Uint8Array(3);
            resp[0] = pts.length;
            new DataView(resp.buffer).setUint16(1, blob.length, true);
            this._respond(op, tid, P.ST_OK, resp);
          });
        }
        case P.OP_TEST_RATE_COUNT: {
          /* firmware deviation from the old README note: SYNCHRONOUS —
           * sys_task blocks for the window, the single response IS the
           * completion (payload {u32 pulses, u32 elapsed_ms}) */
          const seconds = Math.min(30, Math.max(1, pl[0] || 1));
          this._after(seconds * 1000, () => {
            const nominal = P.RATE_SPS[this._rate];
            const pulses = Math.round(nominal * seconds * 0.9995);
            const out = new DataView(new ArrayBuffer(8));
            out.setUint32(0, pulses, true);
            out.setUint32(4, seconds * 1000 + 3, true);
            this._respond(op, tid, P.ST_OK, new Uint8Array(out.buffer));
          });
          return;
        }
        case P.OP_TEST_BUTTON_ECHO:
          this._buttonEcho = !!pl[0];
          if (this._buttonEcho) this._scriptButtonPresses();
          return ok();
        case P.OP_TEST_CHARGER_LIVE: {
          /* firmware: stateless POLLED snapshot {u8 vusb, u8 stat_raw,
           * u8 decoded_state}; the enable byte is accepted and ignored.
           * Walk battery -> charging -> complete -> battery on poll time.
           * STAT physics per T08: unpowered MCP73831 + divider reads 0
           * on battery; charging = 0; complete = 1. */
          if (pl.length > 1) return this._respond(op, tid, P.ST_BAD_LEN);
          if (this._chargerT0 === null) this._chargerT0 = this._devUs();
          const tS = (this._devUs() - this._chargerT0) / 1e6;
          let vusb, stat, st;
          if (tS < 2) { vusb = 0; stat = 0; st = P.CHG_ON_BATTERY; }
          else if (tS < 5) { vusb = 1; stat = 0; st = P.CHG_CHARGING; }
          else if (tS < 7) { vusb = 1; stat = 1; st = P.CHG_COMPLETE; }
          else { vusb = 0; stat = 0; st = P.CHG_ON_BATTERY; }
          this._chargerFlags =
            (st === P.CHG_CHARGING ? P.STF_CHARGING : 0) |
            (st === P.CHG_COMPLETE ? P.STF_CHARGE_DONE : 0) |
            (vusb ? P.STF_USB : 0);
          return ok(Uint8Array.of(vusb, stat, st));
        }
        case P.OP_TEST_BATT_RAW: {
          const out = new DataView(new ArrayBuffer(4));
          out.setUint16(0, 3921, true);   /* calibrated mV */
          out.setUint16(2, 1943, true);   /* raw ADC average */
          return ok(new Uint8Array(out.buffer));
        }
        case P.OP_TEST_ACCEL_LIVE:
          this._accelLive = !!pl[0];
          if (this._accelLive) this._startAccelOrientation();
          return ok();
        case P.OP_ENTER_OTA:
          /* acquisition stops, state -> OTA; transfer runs on the OTA
           * service (see _otaControl/_otaData) */
          this._streamMask = 0;
          return ok();
        case P.OP_TEST_LED_SWEEP_CONT: {
          /* {u8 mask b0 IR b1 RED, u8 enable, u8 phase_s (0 -> 5)} */
          if (pl.length !== 3) return this._respond(op, tid, P.ST_BAD_LEN);
          const [mask, enable, phaseS] = pl;
          if (enable && !(mask & 3)) {
            return this._respond(op, tid, P.ST_BAD_PARAM);
          }
          if (enable) {
            this._agcFrozen = true;      /* proto.h: freezes AGC on start */
            this._sweep = { on: true, mask: mask & 3,
                            phaseS: phaseS || 5, t0Us: this._devUs() };
          } else {
            this._sweep.on = false;
          }
          return ok();
        }
        case P.OP_TEST_SLEEP_NOW:
          ok();
          /* firmware sleeps in 10 s; link drops when it does */
          this._after(10000, () => this._disconnect(true));
          return;
        case P.OP_TEST_REPORT: {
          /* firmware: chunker over the current blob — the sweep blob if
           * one was published, else the selftest blob; none -> WRONG_STATE */
          const blob = this._sweepBlob ||
            (this._selftestRan.length ? this._selftestBlob() : null);
          if (!blob) return this._respond(op, tid, P.ST_WRONG_STATE);
          return ok(this._chunkOf(blob, pl[0] | (pl[1] << 8)));
        }

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

    /* Binary sweep report, firmware layout (test_ops.c blob ver 2):
     * [u8 2][u8 kind][u8 param][u8 n][n x {u8 setting, i32 ir, i32 red,
     * i32 amb}] little-endian. Negative DAC settings cast to u8. */
    _buildSweepBlob(kind, param, pts) {
      const out = new Uint8Array(4 + pts.length * 13);
      const d = new DataView(out.buffer);
      out[0] = 2; out[1] = kind; out[2] = param; out[3] = pts.length;
      pts.forEach((p, i) => {
        const off = 4 + i * 13;
        d.setUint8(off, p.setting & 0xFF);
        d.setInt32(off + 1, p.ir, true);
        d.setInt32(off + 5, p.red, true);
        d.setInt32(off + 9, p.amb, true);
      });
      return out;
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

    /* ---- live LED currents (manual/AGC value, or the 0xEB triangle) ---- */
    _triMa(maxMa) {
      const p = this._sweep.phaseS;
      const t = ((this._devUs() - this._sweep.t0Us) / 1e6) % (4 * p);
      if (t < p) return Math.round(maxMa * t / p);          /* ramp up   */
      if (t < 2 * p) return maxMa;                          /* hold max  */
      if (t < 3 * p) return Math.round(maxMa * (3 * p - t) / p); /* down */
      return 0;                                             /* hold 0    */
    }
    _curMa() {
      const s = this._sweep;
      return {
        ir: (s.on && (s.mask & 1)) ? this._triMa(50) : this._ledIr,
        red: (s.on && (s.mask & 2)) ? this._triMa(40) : this._ledRed,
      };
    }
    _battMv() {
      /* sim: ~0.9 mV/s drain so the session graph visibly slopes */
      const mv = this._sim
        ? this._battMv0 - this._devUs() / 1e6 * 0.9 : this._battMv0;
      return Math.max(3300, Math.round(mv));
    }

    /* ---- STATUS ---- */
    _statusBytes() {
      const d = new DataView(new ArrayBuffer(P.STATUS_SIZE));
      const ma = this._curMa();
      const mv = this._battMv();
      d.setUint8(0, this._streamMask ? P.STATE_STREAMING : P.STATE_CONNECTED);
      d.setUint8(1, (this._worn ? P.STF_WORN : 0) | this._chargerFlags |
                    (this._agcFrozen ? P.STF_AGC_FROZEN : 0) |
                    (this._gated ? P.STF_GATE : 0));
      d.setUint16(2, mv, true);
      d.setUint8(4, Math.max(0, Math.min(100,
        Math.round((mv - 3300) / (4200 - 3300) * 100))));
      d.setUint8(5, this._rate);
      d.setUint8(6, ma.ir); d.setUint8(7, ma.red);   /* LED mA */
      d.setUint8(8, this._rf); d.setUint8(9, 2);     /* TIA RF/CF codes */
      d.setUint16(10, this._gated ? 1200 : 0, true);
      d.setUint32(12, 0, true);
      d.setUint16(16, 0, true);
      d.setInt16(18, -23, true);
      d.setUint32(20, Math.floor(this._devUs() / 1e6), true);
      d.setUint16(24, this._ibiLastMs || 833, true);
      d.setUint8(26, this._hrBpm || 72);
      d.setUint8(27, this._btnLevel);                /* proto 1.1 */
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
      /* 5 presses. Firmware (test_ops.c btn_poll_cb): marker_id =
       * 1000 + level — press (active-low, level 0) = 1000, release =
       * 1001. Down edge, up edge 120 ms later, presses spaced 700 ms. */
      const t0 = this._devUs();
      const edges = [];
      for (let press = 1; press <= 5; press++) {
        const base = 600 + (press - 1) * 700;
        edges.push([t0 + base * 1000, 1000]);
        edges.push([t0 + (base + 120) * 1000, 1001]);
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

    /* ================= SIMULATE MODE =================
     * One catch-up engine (device-time scheduled, like the generators
     * above) that makes every dashboard element move: PPG with synthetic
     * beats + dicrotic notch whose DC tracks LED mA / TIA gain, accel
     * gravity with drift + motion bursts, IBI per beat, AGC/gate/wear
     * events, and a button press every ~20 s. Active only with
     * opts.sim (never in the deterministic scripted walk). */

    _pushIbi(recs) {
      const out = new Uint8Array(P.IBI_HDR_SIZE + recs.length * P.IBI_REC_SIZE);
      const d = new DataView(out.buffer);
      d.setUint32(0, this._ibiSeq++, true);
      d.setUint8(4, recs.length);
      recs.forEach((r, i) => {
        const off = P.IBI_HDR_SIZE + i * P.IBI_REC_SIZE;
        d.setBigUint64(off, BigInt(r.tBeatUs), true);
        d.setUint16(off + 8, r.ibiMs, true);
        d.setUint8(off + 10, r.confidence);
        d.setUint8(off + 11, r.flags);
      });
      this._char(P.UUID.IBI)._push(out);
    }

    _gateEvent(state, reason) {
      const rec = new Uint8Array(2 + P.EVLEN_GATE);
      const d = new DataView(rec.buffer);
      rec[0] = P.EV_GATE; rec[1] = P.EVLEN_GATE;
      d.setBigUint64(2, BigInt(this._devUs()), true);
      d.setUint8(10, state); d.setUint8(11, reason);
      return rec;
    }
    _wearEvent(worn) {
      const rec = new Uint8Array(2 + P.EVLEN_WEAR);
      const d = new DataView(rec.buffer);
      rec[0] = P.EV_WEAR; rec[1] = P.EVLEN_WEAR;
      d.setBigUint64(2, BigInt(this._devUs()), true);
      d.setUint8(10, worn ? 1 : 0);
      return rec;
    }
    _agcStepEvent(led, oldMa, newMa) {
      const rec = new Uint8Array(2 + P.EVLEN_AGC_STEP);
      const d = new DataView(rec.buffer);
      rec[0] = P.EV_AGC_STEP; rec[1] = P.EVLEN_AGC_STEP;
      d.setBigUint64(2, BigInt(this._devUs()), true);
      d.setUint8(10, led); d.setUint8(11, oldMa); d.setUint8(12, newMa);
      d.setUint8(13, this._rf); d.setUint8(14, this._rf);
      return rec;
    }

    _startSim() {
      if (this._simOn) return;
      this._simOn = true;
      const RF_OHMS = [5e5, 2.5e5, 1e5, 5e4, 2.5e4, 1e4, 1e6, 2e6];
      const st = {
        lastUs: this._devUs(),
        beatPhase: 0, lastBeatUs: 0,
        nextPpgUs: 0, nextAccUs: 0,
        nextBurstUs: 8e6, burstEndUs: 0,
        nextBtnUs: 12e6, btnUpUs: 0,
        nextAgcUs: 3e6,
        nextWearUs: 45e6, wearBackUs: 0,
        theta: 0.3, phi: 0.8,
      };
      this._simSt = st;
      const tick = () => {
        if (!this._connected || !this._sim) { this._simOn = false; return; }
        const now = this._devUs();

        /* --- motion bursts (gate cause) --- */
        if (!st.burstEndUs && now >= st.nextBurstUs) {
          st.burstEndUs = now + 2e6;
          if (this._streamMask & P.STREAM_MASK_EVENT) {
            this._pushEvent(this._gateEvent(1, P.GATE_REASON_ACCEL));
          }
          this._gated = true;
        }
        if (st.burstEndUs && now >= st.burstEndUs) {
          st.burstEndUs = 0;
          st.nextBurstUs = now + (10 + this._rand() * 12) * 1e6;
          if (this._streamMask & P.STREAM_MASK_EVENT) {
            this._pushEvent(this._gateEvent(0, 0));
          }
          this._gated = false;
        }
        const burst = !!st.burstEndUs;

        /* --- wear drop every ~45 s (3 s off-ear) --- */
        if (!st.wearBackUs && now >= st.nextWearUs) {
          st.wearBackUs = now + 3e6;
          this._worn = false;
          if (this._streamMask & P.STREAM_MASK_EVENT) {
            this._pushEvent(this._wearEvent(false));
          }
        }
        if (st.wearBackUs && now >= st.wearBackUs) {
          st.wearBackUs = 0;
          st.nextWearUs = now + (40 + this._rand() * 20) * 1e6;
          this._worn = true;
          if (this._streamMask & P.STREAM_MASK_EVENT) {
            this._pushEvent(this._wearEvent(true));
          }
        }

        /* --- button press ~20 s: active-low, marker_id = 1000+level --- */
        if (!st.btnUpUs && now >= st.nextBtnUs) {
          st.btnUpUs = now + 6e5;
          this._btnLevel = 1;
          if ((this._streamMask & P.STREAM_MASK_EVENT) || this._buttonEcho) {
            this._pushEvent(this._markerRecord(P.MARKER_SRC_BUTTON, 1000));
          }
        }
        if (st.btnUpUs && now >= st.btnUpUs) {
          st.btnUpUs = 0;
          st.nextBtnUs = now + (17 + this._rand() * 6) * 1e6;
          this._btnLevel = 0;
          if ((this._streamMask & P.STREAM_MASK_EVENT) || this._buttonEcho) {
            this._pushEvent(this._markerRecord(P.MARKER_SRC_BUTTON, 1001));
          }
        }

        /* --- AGC stepping while unfrozen (visible in events + STATUS) --- */
        if (now >= st.nextAgcUs) {
          st.nextAgcUs = now + (2 + this._rand() * 3) * 1e6;
          if (!this._agcFrozen && (this._streamMask & P.STREAM_MASK_PPG)) {
            const target = Math.round(18 + 6 * Math.sin(now / 47e6));
            if (this._ledIr !== target) {
              const oldMa = this._ledIr;
              this._ledIr += this._ledIr < target ? 1 : -1;
              if (this._streamMask & P.STREAM_MASK_EVENT) {
                this._pushEvent(this._agcStepEvent(0, oldMa, this._ledIr));
              }
            }
          }
        }

        /* --- PPG batches (50 ms) + beat clock + IBI --- */
        const sps = P.RATE_SPS[this._rate];
        const batchUs = 5e4;
        if (!st.nextPpgUs) st.nextPpgUs = now + batchUs;
        while ((this._streamMask & P.STREAM_MASK_PPG) && st.nextPpgUs <= now) {
          const t0 = st.nextPpgUs - batchUs;
          const n = Math.max(1, Math.round(sps * batchUs / 1e6));
          const hasAmb = (this._knobVals.get(0x0302) || 0) === 1;
          const stride = hasAmb ? 12 : 8;
          const out = new Uint8Array(P.PPG_HDR_SIZE + n * stride);
          const d = new DataView(out.buffer);
          const ma = this._curMa();
          const gainX = RF_OHMS[this._rf] / 2.5e4;
          let dcIr = 12500 * ma.ir * gainX;
          let dcRed = 14800 * ma.red * gainX;
          let clipped = false;
          if (dcIr > 4194303) { dcIr = 4194303; clipped = true; }
          if (dcRed > 4194303) { dcRed = 4194303; clipped = true; }
          if (!this._worn) { dcIr *= 0.05; dcRed *= 0.05; }
          d.setUint32(0, this._ppgSeq++, true);
          d.setBigUint64(4, BigInt(t0), true);
          d.setUint8(12, this._rate);
          d.setUint8(13, n);
          d.setUint8(14, (hasAmb ? P.PPGF_AMB : 0) |
                         (burst ? P.PPGF_GATE : 0) |
                         (this._worn ? 0 : P.PPGF_WEAR_OFF) |
                         (clipped ? P.PPGF_CLIPPED : 0));
          for (let i = 0; i < n; i++) {
            const tUs = t0 + i * 1e6 / sps;
            /* beat clock: 60-75 bpm wander */
            const hr = 67.5 + 7.5 * Math.sin(tUs / 37e6) +
                       (this._rand() - 0.5) * 0.6;
            st.beatPhase += hr / 60 / sps;
            if (st.beatPhase >= 1) {
              st.beatPhase -= 1;
              const ibiMs = st.lastBeatUs
                ? Math.round((tUs - st.lastBeatUs) / 1000) : 0;
              st.lastBeatUs = tUs;
              if (ibiMs > 250 && ibiMs < 2000) {
                this._ibiLastMs = ibiMs;
                this._hrBpm = Math.round(60000 / ibiMs);
                if ((this._streamMask & P.STREAM_MASK_IBI) && this._worn) {
                  this._pushIbi([{
                    tBeatUs: Math.round(tUs), ibiMs,
                    confidence: burst ? 34 : 82 + Math.round(this._rand() * 15),
                    flags: burst ? P.IBIF_GATED_CTX : 0,
                  }]);
                }
              }
            }
            const ph = st.beatPhase;
            const pulse = Math.exp(-ph * 5) +
                          0.35 * Math.exp(-Math.pow((ph - 0.45) / 0.09, 2));
            const ampScale = burst ? 0.15 : 1;
            const ir = Math.round(dcIr + dcIr * 0.049 * pulse * ampScale +
                                  (this._rand() - 0.5) * 260 +
                                  (burst ? (this._rand() - 0.5) * 3000 : 0));
            const red = Math.round(dcRed + dcRed * 0.036 * pulse * ampScale +
                                   (this._rand() - 0.5) * 260 +
                                   (burst ? (this._rand() - 0.5) * 2200 : 0));
            const off = P.PPG_HDR_SIZE + i * stride;
            d.setInt32(off, Math.min(4194303, Math.max(0, ir)), true);
            d.setInt32(off + 4, Math.min(4194303, Math.max(0, red)), true);
            if (hasAmb) {
              const amb = 900 + 130 * Math.sin(2 * Math.PI * 60 * tUs / 1e6) +
                          (this._rand() - 0.5) * 80;
              d.setInt32(off + 8, Math.max(0, Math.round(amb)), true);
            }
          }
          this._char(P.UUID.PPG)._push(out);
          st.nextPpgUs += batchUs;
        }
        if (!(this._streamMask & P.STREAM_MASK_PPG)) st.nextPpgUs = 0;

        /* --- accel batches (200 ms at the knobbed ODR) --- */
        const odrCode = this._knobVals.has(0x0801)
          ? this._knobVals.get(0x0801) : 2;
        const fsCode = this._knobVals.has(0x0802)
          ? this._knobVals.get(0x0802) : 1;
        const odrHz = P.ODR_HZ[odrCode] || 50;
        const accBatchUs = 2e5;
        if (!st.nextAccUs) st.nextAccUs = now + accBatchUs;
        while ((this._streamMask & P.STREAM_MASK_ACCEL) && st.nextAccUs <= now) {
          const t0 = st.nextAccUs - accBatchUs;
          const n = Math.min(P.ACCEL_MAX_N,
                             Math.max(1, Math.round(odrHz * accBatchUs / 1e6)));
          const cpg = NP.fsCountsPerG(fsCode);
          const out = new Uint8Array(P.ACCEL_HDR_SIZE + n * 6);
          const d = new DataView(out.buffer);
          d.setUint32(0, this._accelSeq++, true);
          d.setBigUint64(4, BigInt(t0), true);
          d.setUint8(12, odrCode);
          d.setUint8(13, n);
          d.setUint8(14, fsCode & P.ACCF_FS_MASK);
          /* slow orientation drift */
          st.theta += 0.0006; st.phi += 0.00023;
          const g = [Math.sin(st.theta) * Math.cos(st.phi),
                     Math.sin(st.theta) * Math.sin(st.phi),
                     Math.cos(st.theta)];
          for (let i = 0; i < n; i++) {
            const tS = (t0 + i * 1e6 / odrHz) / 1e6;
            for (let ax = 0; ax < 3; ax++) {
              let v = g[ax] + (this._rand() - 0.5) * 0.04;
              if (burst) {
                v += 0.7 * Math.sin(2 * Math.PI * 6 * tS + ax * 2.1) +
                     (this._rand() - 0.5) * 0.5;
              }
              d.setInt16(P.ACCEL_HDR_SIZE + i * 6 + ax * 2,
                Math.max(-32768, Math.min(32767, Math.round(v * cpg))), true);
            }
          }
          this._char(P.UUID.ACCEL)._push(out);
          st.nextAccUs += accBatchUs;
        }
        if (!(this._streamMask & P.STREAM_MASK_ACCEL)) st.nextAccUs = 0;

        st.lastUs = now;
        this._after(25, tick);
      };
      this._after(25, tick);
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
