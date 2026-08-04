/*
 * app.js — Narbis Edge Earclip PCB-verification bench tool.
 *
 * Vanilla JS, no build step. Loads after proto_consts.js / proto.js /
 * mock.js as classic scripts. Real hardware via Web Bluetooth
 * (Chrome/Edge desktop + Android, https or localhost); ?mock=1 swaps in
 * NarbisMock's scripted device (works from file://) and compresses every
 * bench wait by the mock timescale.
 *
 * The pure verdict helpers live in NarbisAnalysis (exported for node so
 * selfcheck.mjs can exercise them without a DOM).
 */
"use strict";

/* ================================================================== *
 * Pure analysis helpers (no DOM — node-testable)                      *
 * ================================================================== */

const NarbisAnalysis = {
  /* AFE4404 TIA RF code -> ohms (SBAS689D): NOT monotonic in code order.
   * Any "gain rises" check must order points by ohms, not by code. */
  RF_OHMS: [500e3, 250e3, 100e3, 50e3, 25e3, 10e3, 1e6, 2e6],

  /* TEST sweep report blob, firmware layout (test_ops.c, blob ver 2):
   * [u8 ver=2][u8 kind: 1 LED, 2 RX][u8 param][u8 n]
   * [n x {u8 setting, i32 ir, i32 red, i32 amb}] little-endian.
   * Holds ONLY the most recent sweep — fetch after each 0xE2/0xE3.
   * For kind 2 param 1 (offset DAC) `setting` is an int8 cast to u8. */
  parseSweepBlob(bytes) {
    const b = bytes instanceof Uint8Array ? bytes : Uint8Array.from(bytes);
    if (b.length < 4) throw new Error(`sweep blob too short: ${b.length}`);
    const ver = b[0], kind = b[1], param = b[2], n = b[3];
    if (ver !== 2) {
      throw new Error(`sweep blob ver ${ver} (expected 2 — run a sweep first)`);
    }
    if (b.length !== 4 + n * 13) {
      throw new Error(`sweep blob len ${b.length} != ${4 + n * 13}`);
    }
    const d = new DataView(b.buffer, b.byteOffset, b.byteLength);
    const signedSetting = kind === 2 && param === 1;
    const points = [];
    for (let i = 0, off = 4; i < n; i++, off += 13) {
      let s = d.getUint8(off);
      if (signedSetting && s > 127) s -= 256;
      points.push({
        setting: s,
        ir: d.getInt32(off + 1, true),
        red: d.getInt32(off + 5, true),
        amb: d.getInt32(off + 9, true),
      });
    }
    return { ver, kind, param, points };
  },

  /* Sweep sanity: a healthy LED/RX sweep RISES with the setting and is
   * not flat. Flat response = open emitter / disconnected FFC / dead RX.
   * Verdict = Pearson correlation of dc vs sample index, NOT per-step
   * monotonicity: on first hardware, mains-flicker residue (ambient
   * phase and LED phase sample the room light at different instants,
   * so subtraction can't cancel the 100/120 Hz beat) is comparable to
   * one 2 mA step, and step-wise checks failed sweeps whose overall
   * rise was unmistakable. Correlation sees through per-point noise
   * but still fails flat, dead, and saturated-flat responses. */
  sweepVerdict(ys, opts, xs) {
    const o = Object.assign({ minSpan: 2000, minCorr: 0.92 }, opts);
    if (!ys || ys.length < 4) {
      return { pass: false, flat: true, span: 0, corr: 0, note: "too few points" };
    }
    const n = ys.length;
    const x = xs && xs.length === n ? xs : ys.map((_, i) => i);
    const min = Math.min(...ys), max = Math.max(...ys);
    const span = max - min;
    const mx = x.reduce((a, b) => a + b, 0) / n;
    const my = ys.reduce((a, b) => a + b, 0) / n;
    let sxy = 0, sxx = 0, syy = 0;
    for (let i = 0; i < n; i++) {
      sxy += (x[i] - mx) * (ys[i] - my);
      sxx += (x[i] - mx) * (x[i] - mx);
      syy += (ys[i] - my) * (ys[i] - my);
    }
    const corr = (syy > 0 && sxx > 0) ? sxy / Math.sqrt(sxx * syy) : 0;
    const flat = span < o.minSpan;
    const pass = !flat && corr >= o.minCorr;
    return {
      pass, flat, span, corr,
      note: flat ? "FLAT response — open emitter / FFC / RX path"
        : (pass ? `rises with setting (r=${corr.toFixed(3)})`
                : `does not track setting (r=${corr.toFixed(3)})`),
    };
  },

  /* TIA gain sweep: photocurrent must track RF ohms. Codes are swept
   * 0..7 but the ohm ladder is non-monotonic (see RF_OHMS), so order by
   * ohms first, then apply the monotonic-rise check. */
  gainSweepVerdict(codes, dcs, opts) {
    const pts = codes.map((c, i) => ({ ohms: this.RF_OHMS[c & 7], dc: dcs[i] }))
      .sort((a, b) => a.ohms - b.ohms);
    /* Correlate dc against the RF value itself: transimpedance is
     * dc ∝ ohms, and the ohm ladder (10k…2M) is far from uniform —
     * correlating vs rank would punish the physics. */
    const v = this.sweepVerdict(pts.map((p) => p.dc), opts,
                                pts.map((p) => p.ohms));
    v.note = v.flat ? "FLAT vs RF — dead RX path / INP-INM wiring"
      : (v.pass ? "dc tracks RF (ohm-ordered)" : "dc does not track RF");
    return v;
  },

  /* T04/T05 result mapping per the vendor instructions: optical limits
   * are INFORMATIONAL until first articles set binding thresholds. A
   * FAIL record is a hard fail only when the mechanics failed — frames
   * missing (threshold slot = the frame-count minimum) or a negative
   * value (bring-up/attach error). A genuine over-threshold optical
   * value records as "info", never blocks the vendor line. */
  opticalResult(rec, P, frameThr) {
    if (rec.status === P.TR_PASS) return "pass";
    if (rec.status !== P.TR_FAIL) return "skip";
    if (rec.value < 0 || rec.threshold === frameThr) return "fail";
    return "info";
  },

  /* ADC_RDY rate accuracy: DoD is ±0.5 % of nominal. */
  rateVerdict(pulses, elapsedMs, nominalHz) {
    const hz = elapsedMs > 0 ? (pulses * 1000) / elapsedMs : 0;
    const errPct = nominalHz > 0 ? (Math.abs(hz - nominalHz) / nominalHz) * 100 : 100;
    return { hz, errPct, pass: errPct <= 0.5 };
  },

  /* Orientation classifier: mean gravity vector -> one of six faces, or
   * null while moving/tilted. Thresholds are loose on purpose (hand-held
   * rotation, imperfect mounting). */
  faceOf(gx, gy, gz) {
    const v = [gx, gy, gz];
    const names = [["+X", "-X"], ["+Y", "-Y"], ["+Z", "-Z"]];
    let dom = 0;
    for (let i = 1; i < 3; i++) if (Math.abs(v[i]) > Math.abs(v[dom])) dom = i;
    const a = Math.abs(v[dom]);
    /* Hand-held: 0.6 g dominant / 0.45 g cross-axis ≈ 30° of tilt
     * allowance (first bench: -Y at full articulation missed 0.7/0.35). */
    if (a < 0.6 || a > 1.45) return null;
    for (let i = 0; i < 3; i++) {
      if (i !== dom && Math.abs(v[i]) > 0.45) return null;
    }
    return names[dom][v[dom] > 0 ? 0 : 1];
  },

  /* Closest face + what blocks its capture — live coaching text. */
  faceHint(gx, gy, gz) {
    const v = [gx, gy, gz];
    const names = [["+X", "-X"], ["+Y", "-Y"], ["+Z", "-Z"]];
    let dom = 0;
    for (let i = 1; i < 3; i++) if (Math.abs(v[i]) > Math.abs(v[dom])) dom = i;
    const cand = names[dom][v[dom] > 0 ? 0 : 1];
    const a = Math.abs(v[dom]);
    if (a < 0.3) return "rotating…";
    const cross = Math.max(...v.map((x, i) => (i === dom ? 0 : Math.abs(x))));
    if (a < 0.6) return `near ${cand} — keep rotating toward flat`;
    if (cross > 0.45) return `near ${cand} — level out (tilt ${cross.toFixed(2)} g)`;
    return `${cand} — hold still`;
  },

  /* Battery calibration vs bench meter: DoD is ±50 mV. */
  battVerdict(mvCal, benchMv) {
    const delta = Math.abs(mvCal - benchMv);
    return { delta, pass: delta <= 50 };
  },
};

if (typeof module !== "undefined" && module.exports) {
  module.exports = { NarbisAnalysis };
}

/* ================================================================== *
 * Browser application                                                 *
 * ================================================================== */

if (typeof document !== "undefined") (() => {

  const NP = NarbisProto;
  const P = NP.P;
  const $ = (id) => document.getElementById(id);

  /* ?mock=1 normally; #mock=1 also works (some file:// contexts drop the
   * query string). */
  const qs = new URLSearchParams(location.search || location.hash.replace(/^#/, "?"));
  const MOCK = qs.get("mock") === "1";
  /* Mock compresses every scripted wait; bench steps that take 10-60 s on
   * hardware demo in about a tenth of that. */
  const TS = MOCK ? Number(qs.get("timescale") || 0.15) : 1;

  const S = {
    bt: null, device: null, server: null,
    chars: {},               /* CONTROL/STATUS/EVENT/PPG/ACCEL by key   */
    tidNext: 1,
    pending: new Map(),      /* tid -> {resolve,reject,timer,until,name} */
    sinks: { event: null, ppg: null, accel: null },
    dis: {}, protoVer: null, battPct: null, lastStatus: null,
    clock: null,
    seqRunning: false, expectDisconnect: false,
    results: [],             /* report rows, one per step */
    otaPending: new Map(),   /* tid -> pending, OTA_CTRL responses    */
    flashing: false,         /* firmware update in progress             */
    demoMode: false,         /* runtime mock via the Demo button        */
    notifRefs: {},           /* charKey -> subscription refcount        */
    logLines: [],            /* full-session verbose log (FIFO capped)  */
    taps: {                  /* dashboard listeners (never guided-seq)  */
      status: new Set(), event: new Set(), log: new Set(),
    },
  };

  /* ---------------- logging ---------------- */

  const LOG_CAP = 50000;     /* lines kept in memory (FIFO)             */

  function hexBytes(b) {
    let s = "";
    for (let i = 0; i < b.length; i++) {
      s += (b[i] < 16 ? "0" : "") + b[i].toString(16) + (i + 1 < b.length ? " " : "");
    }
    return s;
  }

  function log(msg) {
    const t = new Date();
    const line = `[${t.toTimeString().slice(0, 8)}.` +
      `${String(t.getMilliseconds()).padStart(3, "0")}] ${msg}`;
    S.logLines.push(line);
    if (S.logLines.length > LOG_CAP) {
      S.logLines.splice(0, S.logLines.length - LOG_CAP);
    }
    for (const fn of S.taps.log) { try { fn(line); } catch (_) { /* tap */ } }
    const el = $("liveLog");
    el.textContent += line + "\n";
    /* keep the guided-pane DOM light; full log lives in S.logLines */
    if (el.textContent.length > 300000) {
      el.textContent = S.logLines.slice(-1500).join("\n") + "\n";
    }
    el.scrollTop = el.scrollHeight;
  }

  function banner(msg, kind) {
    const el = $("banner");
    if (!msg) { el.classList.add("hidden"); return; }
    el.textContent = msg;
    el.classList.remove("hidden");
    el.classList.toggle("error", kind === "error");
  }

  /* ---------------- BLE plumbing ---------------- */

  async function writeControl(bytes) {
    const c = S.chars.control;
    if (c.writeValueWithResponse) return c.writeValueWithResponse(bytes);
    return c.writeValue(bytes);
  }

  /* Send one CONTROL request; resolve on the correlated response notification.
   * opts.until lets long ops (rate count) swallow the immediate ack and
   * wait for the completion notification that reuses the same tid. */
  function ctrl(name, args = [], opts = {}) {
    const tid = S.tidNext;
    S.tidNext = (S.tidNext + 1) & 0xFF;
    if (S.tidNext === 0) S.tidNext = 1;
    const bytes = NP.build[name](tid, ...args);
    log(`-> ${NP.opName(bytes[0])} tid=${tid} [${hexBytes(bytes)}]`);
    const timeoutMs = opts.timeoutMs || 6000;
    return new Promise((resolve, reject) => {
      const arm = () => setTimeout(() => {
        S.pending.delete(tid);
        reject(new Error(`${name}: no response within ${timeoutMs} ms`));
      }, timeoutMs);
      S.pending.set(tid, {
        name, resolve, reject, timer: arm(), rearm: arm,
        until: opts.until || (() => true),
        allowError: !!opts.allowError,
      });
      writeControl(bytes).catch((e) => {
        const p = S.pending.get(tid);
        if (p) { clearTimeout(p.timer); S.pending.delete(tid); }
        reject(e);
      });
    });
  }

  function onControlIndication(ev) {
    let resp;
    try { resp = NP.parseControlResponse(ev.target.value); }
    catch (e) { log(`bad CONTROL response: ${e.message}`); return; }
    log(`<- ${NP.opName(resp.op)} tid=${resp.tid} ` +
        `${NP.statusName(resp.status)}` +
        (resp.payload.length ? ` +${resp.payload.length}B` : ""));
    const p = S.pending.get(resp.tid);
    if (!p) { log(`stray CONTROL resp op=${NP.opName(resp.op)} tid=${resp.tid}`); return; }
    if (resp.status !== P.ST_OK && !p.allowError) {
      clearTimeout(p.timer);
      S.pending.delete(resp.tid);
      p.reject(new Error(`${p.name}: device says ${NP.statusName(resp.status)}`));
      return;
    }
    if (!p.until(resp)) {          /* ack of a long op — keep waiting */
      clearTimeout(p.timer);
      p.timer = p.rearm();
      return;
    }
    clearTimeout(p.timer);
    S.pending.delete(resp.tid);
    p.resolve(resp);
  }

  /* Chunked blob fetch over {u16 total,u16 off,u8 n,bytes} envelopes. */
  async function fetchBlob(builderName) {
    let off = 0, total = 0;
    const parts = [];
    for (;;) {
      const resp = await ctrl(builderName, [off]);
      const c = NP.parseChunk(resp.payload);
      total = c.total;
      if (c.data.length === 0) break;
      parts.push(c.data);
      off += c.data.length;
      if (off >= total) break;
    }
    const out = new Uint8Array(off);
    let o = 0;
    for (const part of parts) { out.set(part, o); o += part.length; }
    if (out.length !== total) {
      throw new Error(`chunked fetch: got ${out.length} of ${total} bytes`);
    }
    return out;
  }

  /* One sweep's data (0xEA serves the firmware's binary v2 blob, which
   * holds only the MOST RECENT 0xE2/0xE3 sweep — call after each). */
  async function fetchSweep() {
    return NarbisAnalysis.parseSweepBlob(await fetchBlob("testReport"));
  }

  /* Refcounted: the dashboard tab holds long-lived subscriptions on the
   * same characteristics guided steps use transiently — stopNotifications
   * only when the last subscriber unsubscribes. */
  async function subscribe(charKey, handler) {
    const c = S.chars[charKey];
    c.addEventListener("characteristicvaluechanged", handler);
    S.notifRefs[charKey] = (S.notifRefs[charKey] || 0) + 1;
    await c.startNotifications();
    let done = false;
    return async () => {
      if (done) return;
      done = true;
      c.removeEventListener("characteristicvaluechanged", handler);
      S.notifRefs[charKey] = Math.max(0, (S.notifRefs[charKey] || 1) - 1);
      if (S.notifRefs[charKey] === 0) {
        try { await c.stopNotifications(); } catch (_) { /* link may be gone */ }
      }
    };
  }

  /* ================= firmware update (BLE OTA) =================
   * Same protocol as tools/narbis_client/ota.py: ENTER_OTA on CONTROL,
   * then BEGIN{size,crc32,version} / [u32 offset][chunk] WNR frames /
   * periodic STATUS checkpoints / FINISH on OTA_CTRL. The device
   * validates the image + CRC readback, swaps the boot slot, reboots,
   * self-checks for 10 s, and auto-rolls back on failure. */

  function otaCtrl(name, bytes, timeoutMs = 8000) {
    const tid = S.tidNext;
    S.tidNext = (S.tidNext + 1) & 0xFF;
    if (S.tidNext === 0) S.tidNext = 1;
    bytes[1] = tid;
    log(`-> OTA_${name} tid=${tid} [${hexBytes(bytes.length > 20 ? bytes.subarray(0, 20) : bytes)}${bytes.length > 20 ? " …" : ""}]`);
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        S.otaPending.delete(tid);
        reject(new Error(`${name}: no OTA response within ${timeoutMs} ms`));
      }, timeoutMs);
      S.otaPending.set(tid, { name, resolve, reject, timer });
      const c = S.chars.otaCtrl;
      (c.writeValueWithResponse ? c.writeValueWithResponse(bytes)
                                : c.writeValue(bytes)).catch((e) => {
        const p = S.otaPending.get(tid);
        if (p) { clearTimeout(p.timer); S.otaPending.delete(tid); }
        reject(e);
      });
    });
  }

  function onOtaIndication(ev) {
    let resp;
    try { resp = NP.parseControlResponse(ev.target.value); }
    catch (e) { log(`bad OTA response: ${e.message}`); return; }
    const p = S.otaPending.get(resp.tid);
    if (p) {
      log(`<- OTA_${p.name} tid=${resp.tid} ${NP.statusName(resp.status)}`);
    }
    if (!p) {
      /* unsolicited progress notifications are informational */
      return;
    }
    clearTimeout(p.timer);
    S.otaPending.delete(resp.tid);
    if (resp.status !== P.ST_OK) {
      p.reject(new Error(`${p.name}: device says ${NP.statusName(resp.status)}`));
    } else {
      p.resolve(resp);
    }
  }

  function otaUi(pct, text, kind) {
    const bar = $("otaBar"), lbl = $("otaText");
    if (pct !== null) {
      bar.style.width = `${Math.max(0, Math.min(100, pct))}%`;
    }
    lbl.textContent = text;
    lbl.classList.toggle("error", kind === "error");
    lbl.classList.toggle("good", kind === "good");
  }

  function otaUiSync() {
    const ready = !!(S.server && S.server.connected && S.chars.otaCtrl &&
                     $("otaFile").files.length && !S.flashing &&
                     !S.seqRunning && !S.currentStep);
    $("btnFlash").disabled = !ready;
  }

  /* Web Bluetooth exposes no MTU: start at the protocol max (240-byte
   * payload = 244-byte frame, fits the device-requested MTU 247) and
   * halve on the platform write error until it goes through. */
  async function otaWriteChunk(offset, chunk) {
    const c = S.chars.otaData;
    const frame = NP.ota.buildData(offset, chunk);
    if (c.writeValueWithoutResponse) return c.writeValueWithoutResponse(frame);
    return c.writeValue(frame);
  }

  async function flashFirmware(file) {
    const image = new Uint8Array(await file.arrayBuffer());
    if (image.length < 1024) throw new Error("that is not a firmware image");
    /* ESP app images start 0xE9 (also true of merged factory images —
     * refuse those: OTA takes the APP image, narbis_earclip.bin only). */
    if (image[0] !== 0xE9) {
      throw new Error("not an ESP application image (expected " +
                      "firmware/build/narbis_earclip.bin)");
    }
    const crc = NP.ota.crc32(0, image);
    const fwBefore = S.dis.fw || "?";
    const version = file.name.replace(/\.bin$/i, "").slice(0, 24);

    log(`flash: ${file.name}, ${image.length} bytes, crc32 ${crc.toString(16)}`);
    otaUi(0, "Entering OTA mode…");
    await ctrl("enterOta", []);

    const begin = await otaCtrl("BEGIN",
        NP.ota.buildBegin(0, image.length, crc, version));
    let off = NP.ota.parseBeginResp(begin.payload).resumeOffset;
    if (off) log(`flash: resuming at ${off}`);

    let chunkSize = P.OTA_CHUNK_MAX;
    const t0 = Date.now();
    let sinceCheck = 0;
    while (off < image.length) {
      const chunk = image.subarray(off, Math.min(off + chunkSize, image.length));
      try {
        await otaWriteChunk(off, chunk);
      } catch (e) {
        if (chunkSize > 16) {
          chunkSize = chunkSize >> 1;   /* platform MTU smaller than 247 */
          log(`flash: write failed (${e.message}) — chunk size now ${chunkSize}`);
          continue;
        }
        throw e;
      }
      off += chunk.length;
      if (++sinceCheck >= 64 || off >= image.length) {
        sinceCheck = 0;
        const st = NP.ota.parseStatusResp(
            (await otaCtrl("STATUS", NP.ota.buildStatus(0))).payload);
        if (st.lastErr === P.OTAERR_EXPECTED_OFFSET || st.bytesRx !== off) {
          log(`flash: reseek device=${st.bytesRx} host=${off}`);
          off = st.bytesRx;             /* device is the truth — reseek */
          continue;
        }
        if (st.state === P.OTA_FAILED) {
          throw new Error(`device aborted: ${NP.ota.errName(st.lastErr)}`);
        }
        const kbps = (off / 1024) / Math.max(0.001, (Date.now() - t0) / 1000);
        otaUi(100 * off / image.length,
              `Transferring… ${(off / 1024).toFixed(0)} / ` +
              `${(image.length / 1024).toFixed(0)} KB  (${kbps.toFixed(1)} KB/s)`);
      }
    }

    otaUi(100, "Verifying + swapping boot slot…");
    S.expectDisconnect = true;
    await otaCtrl("FINISH", NP.ota.buildFinish(0), 20000);
    log("flash: device accepted image — rebooting");

    otaUi(100, "Device rebooting — waiting to reconnect…");
    await new Promise((res) => {
      if (!S.server.connected) return res();
      const h = () => { S.device.removeEventListener("gattserverdisconnected", h); res(); };
      S.device.addEventListener("gattserverdisconnected", h);
    });
    S.expectDisconnect = false;

    /* reconnect to the SAME BluetoothDevice (no chooser needed) */
    let back = false;
    for (let i = 0; i < 12 && !back; i++) {
      await new Promise((r) => setTimeout(r, 1500));
      try { await connect(S.device); back = true; }
      catch (e) { log(`reconnect attempt ${i + 1}: ${e.message}`); }
    }
    if (!back) {
      throw new Error("device did not come back — if the new image failed " +
                      "its self-check it rolls back automatically within " +
                      "~15 s; reconnect manually to see which version runs");
    }
    const fwAfter = S.dis.fw || "?";
    log(`flash: firmware ${fwBefore} -> ${fwAfter}`);
    if (fwAfter === fwBefore) {
      otaUi(100, `Reconnected, but firmware still reports ${fwAfter} — ` +
            "same version re-flashed, or the device rolled back.", "error");
    } else {
      otaUi(100, `Success: ${fwBefore} → ${fwAfter}. The image self-checks ` +
            "for 10 s and rolls back automatically if unhealthy.", "good");
    }
  }

  async function onFlashClicked() {
    const file = $("otaFile").files[0];
    if (!file || S.flashing) return;
    S.flashing = true;
    otaUiSync();
    $("btnStart").disabled = true;
    try {
      await flashFirmware(file);
    } catch (e) {
      log(`flash FAILED: ${e.message}`);
      otaUi(null, `Failed: ${e.message}`, "error");
      try { if (S.chars.otaCtrl) await otaCtrl("ABORT", NP.ota.buildAbort(0), 3000); }
      catch (_) { /* link may be gone */ }
    } finally {
      S.flashing = false;
      S.expectDisconnect = false;
      otaUiSync();
      $("btnStart").disabled = !(S.server && S.server.connected);
    }
  }

  /* ---------------- connect flow ---------------- */

  async function connect(reuseDevice) {
    banner(null);
    S.expectDisconnect = false;   /* stale sleep-test flag must not mask
                                     a real mid-session drop */
    S.bt = S.bt || (MOCK ? NarbisMock.create({ timescale: TS, sim: true })
                         : navigator.bluetooth);
    if (!S.bt) {
      banner("Web Bluetooth is not available in this browser. Use Chrome/Edge " +
             "on desktop or Android (iOS has no Web Bluetooth — use the python " +
             "client instead), or append ?mock=1 to demo without hardware.",
             "error");
      return;
    }
    if (reuseDevice) {
      /* post-OTA reconnect: same BluetoothDevice, no chooser gesture */
      S.device = reuseDevice;
    } else {
      try {
        S.device = await S.bt.requestDevice({
          filters: [{ namePrefix: "Narbis Edge Earclip" }],
          optionalServices: [P.UUID.SENSOR_SVC, P.UUID.OTA_SVC,
                             P.UUID.BATTERY_SVC, P.UUID.DEVICE_INFO_SVC,
                             P.UUID.HEART_RATE_SVC],
        });
      } catch (e) {
        log(`requestDevice: ${e.message}`);
        return;
      }
      S.device.addEventListener("gattserverdisconnected", onDisconnected);
    }
    S.server = await S.device.gatt.connect();
    log(`connected to ${S.device.name}`);

    if (!/TEST$/.test(S.device.name || "")) {
      banner("Device name does not end in \"TEST\" — this looks like a " +
             "production build; TEST opcodes (0xE0-0xEA) will be rejected.",
             "error");
    }

    const svc = await S.server.getPrimaryService(P.UUID.SENSOR_SVC);
    S.chars.control = await svc.getCharacteristic(P.UUID.CONTROL);
    S.chars.status = await svc.getCharacteristic(P.UUID.STATUS);
    S.chars.event = await svc.getCharacteristic(P.UUID.EVENT);
    S.chars.ppg = await svc.getCharacteristic(P.UUID.PPG);
    S.chars.accel = await svc.getCharacteristic(P.UUID.ACCEL);
    S.chars.ibi = await svc.getCharacteristic(P.UUID.IBI);
    S.notifRefs = {};

    /* protocol version gate: major mismatch = refuse politely */
    const pv = await (await svc.getCharacteristic(P.UUID.PROTO_VER)).readValue();
    S.protoVer = pv.getUint16(0, true);
    if ((S.protoVer >> 8) !== P.PROTO_VER_MAJOR) {
      banner(`PROTOCOL_VERSION major mismatch: device ${S.protoVer >> 8}.` +
             `${S.protoVer & 0xFF}, app ${P.PROTO_VER_MAJOR}.${P.PROTO_VER_MINOR}` +
             " — update the app or the firmware before trusting results.",
             "error");
    }

    /* DIS + battery: best-effort (missing chars must not block the bench) */
    S.dis = {};
    try {
      const dis = await S.server.getPrimaryService(P.UUID.DEVICE_INFO_SVC);
      const rd = async (uuid) => {
        try {
          return new TextDecoder().decode(await (await dis.getCharacteristic(uuid)).readValue());
        } catch (_) { return ""; }
      };
      S.dis.manufacturer = await rd(P.UUID.MANUFACTURER_NAME);
      S.dis.model = await rd(P.UUID.MODEL_NUMBER);
      S.dis.fw = await rd(P.UUID.FIRMWARE_REVISION);
      S.dis.hw = await rd(P.UUID.HARDWARE_REVISION);
    } catch (_) { log("DIS not available"); }
    try {
      const bs = await S.server.getPrimaryService(P.UUID.BATTERY_SVC);
      const bl = await (await bs.getCharacteristic(P.UUID.BATTERY_LEVEL)).readValue();
      S.battPct = bl.getUint8(0);
    } catch (_) { log("battery service not available"); }

    /* always-on subscriptions: CONTROL responses, STATUS, EVENT */
    await subscribe("control", onControlIndication);
    await subscribe("status", (ev) => {
      try { S.lastStatus = NP.parseStatus(ev.target.value); } catch (_) { return; }
      const st = S.lastStatus;
      log(`st: ${st.battMv}mV ${st.battPct}% ir=${st.ledIrMa} ` +
          `red=${st.ledRedMa} rf=${st.tiaGainCode} hr=${st.hrBpm} ` +
          `btn=${st.btnPressed} drops=${st.notifDropCount} up=${st.uptimeS}s`);
      updateHeader();
      if (S.sinks.status) S.sinks.status(st);
      for (const fn of S.taps.status) { try { fn(st); } catch (_) { /* tap */ } }
    });
    await subscribe("event", (ev) => {
      let batch;
      try { batch = NP.parseEventBatch(ev.target.value); } catch (e) {
        log(`bad EVENT batch: ${e.message}`); return;
      }
      for (const rec of batch.events) {
        if (S.sinks.event) S.sinks.event(rec);
        for (const fn of S.taps.event) {
          try { fn(rec, batch.seq); } catch (_) { /* tap */ }
        }
      }
    });

    /* one time-sync pair for the report (host<->device clock mapping) */
    try {
      const hostUs = Date.now() * 1000;
      const resp = await ctrl("timeSync", [hostUs]);
      const tsr = NP.parseTimeSyncResp(resp.payload);
      S.clock = { hostEpochUs: hostUs, devTUs: tsr.devTUs };
    } catch (e) { log(`time sync failed: ${e.message}`); }

    /* OTA service: best-effort (a bench build always has it; production
     * units gate it behind an encrypted link — Chrome will trigger OS
     * pairing on first access if the device allows bonding). */
    S.chars.otaCtrl = S.chars.otaData = null;
    try {
      const osvc = await S.server.getPrimaryService(P.UUID.OTA_SVC);
      S.chars.otaCtrl = await osvc.getCharacteristic(P.UUID.OTA_CTRL);
      S.chars.otaData = await osvc.getCharacteristic(P.UUID.OTA_DATA);
      await S.chars.otaCtrl.startNotifications();
      S.chars.otaCtrl.addEventListener("characteristicvaluechanged",
                                       onOtaIndication);
    } catch (e) { log(`OTA service not available: ${e.message}`); }
    otaUiSync();

    updateHeader();
    $("btnConnect").classList.add("hidden");
    $("btnDisconnect").classList.remove("hidden");
    $("btnStart").disabled = false;
    $("connDot").classList.add("on");
    instruct("Connected. Press <b>Start sequence</b> to run the guided " +
             "PCB verification. Automatic tests run back-to-back; manual " +
             "steps will prompt you.");
  }

  function onDisconnected() {
    $("connDot").classList.remove("on");
    $("btnConnect").classList.remove("hidden");
    $("btnDisconnect").classList.add("hidden");
    $("btnStart").disabled = true;
    for (const [tid, p] of S.pending) {
      clearTimeout(p.timer);
      p.reject(new Error("disconnected"));
      S.pending.delete(tid);
    }
    for (const [tid, p] of S.otaPending) {
      clearTimeout(p.timer);
      p.reject(new Error("disconnected"));
      S.otaPending.delete(tid);
    }
    if (!S.flashing) otaUiSync();   /* flash flow owns the UI during reboot */
    if (S.expectDisconnect) {
      log("device disconnected (expected — sleep test)");
    } else {
      banner("Device disconnected.", "error");
      log("device disconnected");
    }
  }

  function updateHeader() {
    const bits = [];
    if (S.device) bits.push(S.device.name);
    if (S.dis.fw) bits.push(`fw ${S.dis.fw}`);
    if (S.dis.hw) bits.push(`hw ${S.dis.hw}`);
    if (S.protoVer !== null) {
      bits.push(`proto ${S.protoVer >> 8}.${S.protoVer & 0xFF}`);
    }
    const st = S.lastStatus;
    if (st) bits.push(`batt ${st.battMv} mV (${st.battPct}%)`);
    else if (S.battPct !== null) bits.push(`batt ${S.battPct}%`);
    bits.push("RSSI n/a (Web Bluetooth)");
    $("devInfo").textContent = bits.join("  ·  ");
  }

  /* ---------------- live panel helpers ---------------- */

  function instruct(html) { $("liveInstructions").innerHTML = html; }
  function readout(html) { $("liveReadout").innerHTML = html; }
  function clearControls() { $("liveControls").innerHTML = ""; }

  function canvasCtx() {
    const cv = $("liveCanvas");
    const g = cv.getContext("2d");
    g.clearRect(0, 0, cv.width, cv.height);
    return { cv, g };
  }

  const COLORS = {
    axis: "#3a4a5e", grid: "#222d3b", text: "#8ba0b8",
    ir: "#e07840", red: "#e34d6a", accent: "#4da3ff",
    pass: "#38c172", fail: "#e3574b",
  };

  function niceTicks(lo, hi, n) {
    if (hi === lo) hi = lo + 1;
    const span = hi - lo;
    const step0 = Math.pow(10, Math.floor(Math.log10(span / n)));
    const err = span / n / step0;
    const step = step0 * (err >= 7.5 ? 10 : err >= 3.5 ? 5 : err >= 1.5 ? 2 : 1);
    const ticks = [];
    for (let v = Math.ceil(lo / step) * step; v <= hi + step / 1e6; v += step) {
      ticks.push(v);
    }
    return ticks;
  }

  /* series: [{x:[], y:[], color, label}] */
  function drawPlot(series, opts = {}) {
    const { cv, g } = canvasCtx();
    const L = 62, R = 14, T = 16, B = 36;
    const W = cv.width - L - R, H = cv.height - T - B;
    let xlo = Infinity, xhi = -Infinity, ylo = Infinity, yhi = -Infinity;
    for (const s of series) {
      for (const v of s.x) { if (v < xlo) xlo = v; if (v > xhi) xhi = v; }
      for (const v of s.y) { if (v < ylo) ylo = v; if (v > yhi) yhi = v; }
    }
    if (!isFinite(xlo)) { xlo = 0; xhi = 1; ylo = 0; yhi = 1; }
    if (opts.y0) ylo = Math.min(ylo, 0);
    const pad = (yhi - ylo) * 0.06 + 1;
    ylo -= pad; yhi += pad;
    const px = (x) => L + ((x - xlo) / (xhi - xlo || 1)) * W;
    const py = (y) => T + H - ((y - ylo) / (yhi - ylo || 1)) * H;

    g.strokeStyle = COLORS.grid; g.fillStyle = COLORS.text;
    g.font = "11px ui-monospace, monospace"; g.lineWidth = 1;
    for (const t of niceTicks(ylo, yhi, 5)) {
      g.beginPath(); g.moveTo(L, py(t)); g.lineTo(L + W, py(t)); g.stroke();
      g.fillText(Math.round(t).toLocaleString(), 4, py(t) + 4);
    }
    for (const t of niceTicks(xlo, xhi, 8)) {
      g.fillText(String(Math.round(t * 100) / 100), px(t) - 6, T + H + 16);
    }
    g.strokeStyle = COLORS.axis;
    g.strokeRect(L, T, W, H);
    if (opts.xlabel) g.fillText(opts.xlabel, L + W / 2 - 20, cv.height - 6);

    series.forEach((s, si) => {
      g.strokeStyle = s.color; g.fillStyle = s.color; g.lineWidth = 1.6;
      g.beginPath();
      s.x.forEach((x, i) => {
        const X = px(x), Y = py(s.y[i]);
        if (i === 0) g.moveTo(X, Y); else g.lineTo(X, Y);
      });
      g.stroke();
      if (s.x.length <= 40) {
        s.x.forEach((x, i) => {
          g.beginPath(); g.arc(px(x), py(s.y[i]), 2.5, 0, 7); g.fill();
        });
      }
      if (s.label) g.fillText(s.label, L + 8, T + 14 + si * 14);
    });
  }

  /* rolling dual-channel PPG scope */
  function makeScope(windowS) {
    const st = { ir: [], red: [], sps: 100 };
    return {
      push(batch) {
        st.sps = P.RATE_SPS[batch.rateCode] || st.sps;
        st.ir.push(...batch.ir); st.red.push(...batch.red);
        const keep = windowS * st.sps;
        if (st.ir.length > keep) {
          st.ir.splice(0, st.ir.length - keep);
          st.red.splice(0, st.red.length - keep);
        }
      },
      draw() {
        const { cv, g } = canvasCtx();
        const half = cv.height / 2;
        const draw1 = (data, color, y0, label) => {
          if (data.length < 2) return;
          let lo = Infinity, hi = -Infinity;
          for (const v of data) { if (v < lo) lo = v; if (v > hi) hi = v; }
          const span = hi - lo || 1;
          g.strokeStyle = color; g.lineWidth = 1.4; g.beginPath();
          data.forEach((v, i) => {
            const X = (i / (data.length - 1)) * cv.width;
            const Y = y0 + (half - 24) * (1 - (v - lo) / span) + 12;
            if (i === 0) g.moveTo(X, Y); else g.lineTo(X, Y);
          });
          g.stroke();
          g.fillStyle = color; g.font = "12px ui-monospace, monospace";
          g.fillText(`${label}  [${lo.toLocaleString()} .. ${hi.toLocaleString()}]`, 8, y0 + 16);
        };
        draw1(st.ir, COLORS.ir, 0, "IR");
        draw1(st.red, COLORS.red, half, "RED");
      },
    };
  }

  function drawAccelBars(gVec, captured, note) {
    const { cv, g } = canvasCtx();
    const names = ["X", "Y", "Z"];
    const w = cv.width - 140;
    g.font = "13px ui-monospace, monospace";
    gVec.forEach((v, i) => {
      const y = 34 + i * 52;
      g.fillStyle = COLORS.text;
      g.fillText(`${names[i]} ${v >= 0 ? "+" : ""}${v.toFixed(2)} g`, 8, y + 5);
      g.strokeStyle = COLORS.axis;
      g.strokeRect(110, y - 14, w, 28);
      g.strokeStyle = COLORS.grid;
      g.beginPath(); g.moveTo(110 + w / 2, y - 14); g.lineTo(110 + w / 2, y + 14); g.stroke();
      const frac = Math.max(-1, Math.min(1, v / 1.5));
      g.fillStyle = Math.abs(v) > 0.7 ? COLORS.pass : COLORS.accent;
      if (frac >= 0) g.fillRect(110 + w / 2, y - 12, (w / 2) * frac, 24);
      else g.fillRect(110 + w / 2 + (w / 2) * frac, y - 12, -(w / 2) * frac, 24);
    });
    const faces = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"];
    faces.forEach((f, i) => {
      const x = 110 + i * 88, y = 210;
      g.fillStyle = captured.has(f) ? COLORS.pass : COLORS.grid;
      g.fillRect(x, y, 72, 30);
      g.fillStyle = captured.has(f) ? "#06210f" : COLORS.text;
      g.font = "bold 14px ui-monospace, monospace";
      g.fillText(f, x + 24, y + 20);
    });
    g.fillStyle = COLORS.text; g.font = "12px ui-monospace, monospace";
    if (note) g.fillText(note, 8, 296);
  }

  /* ---------------- step context ---------------- */

  const SKIP = { __skip: true };

  function makeCtx() {
    const timers = new Set();
    let skipReject = null;
    const ctx = {
      ts: TS,
      instruct, readout, log, drawPlot, makeScope, drawAccelBars, canvasCtx,
      sleep(ms) {
        return ctx.race(new Promise((res) => {
          const id = setTimeout(res, ms); timers.add(id);
        }));
      },
      every(ms, fn) {
        const id = setInterval(fn, ms); timers.add(id); return id;
      },
      race(promise) {
        return Promise.race([promise, new Promise((_, rej) => { skipReject = rej; })]);
      },
      /* renders buttons, resolves with the picked value */
      buttons(defs) {
        return ctx.race(new Promise((resolve) => {
          clearControls();
          for (const d of defs) {
            const b = document.createElement("button");
            b.textContent = d.label;
            if (d.kind) b.className = d.kind;
            b.onclick = () => { clearControls(); resolve(d.value); };
            $("liveControls").appendChild(b);
          }
        }));
      },
      /* renders a number input + OK, resolves with the number (NaN if blank) */
      numberInput(label, unit) {
        return ctx.race(new Promise((resolve) => {
          clearControls();
          const span = document.createElement("span");
          span.textContent = label + " ";
          const inp = document.createElement("input");
          inp.type = "number";
          const u = document.createElement("span");
          u.textContent = " " + (unit || "");
          const b = document.createElement("button");
          b.textContent = "OK";
          b.className = "primary";
          b.onclick = () => { clearControls(); resolve(parseFloat(inp.value)); };
          inp.addEventListener("keydown", (e) => { if (e.key === "Enter") b.onclick(); });
          $("liveControls").append(span, inp, u, b);
          inp.focus();
        }));
      },
      setEventSink(fn) { S.sinks.event = fn; },
      setPpgSink(fn) { S.sinks.ppg = fn; },
      setAccelSink(fn) { S.sinks.accel = fn; },
      _skip() { if (skipReject) skipReject(SKIP); },
      _cleanup() {
        for (const id of timers) { clearTimeout(id); clearInterval(id); }
        timers.clear();
        S.sinks.event = S.sinks.ppg = S.sinks.accel = null;
        clearControls();
        /* Steps that drew on the shared canvas (accel bars, sweep
         * plots) must not leak their last frame into the next step. */
        canvasCtx();
      },
    };
    return ctx;
  }

  /* ---------------- the guided sequence ---------------- */

  const A = NarbisAnalysis;

  /* opts.frameThr marks an INFORMATIONAL optical test (T04/T05): the
   * value-vs-threshold verdict records as "info" per the vendor
   * instructions ("optical steps report values — they cannot fail at
   * this stage"); only mechanical failures (no frames / bring-up error)
   * hard-fail. frameThr = the firmware's frame-count minimum, which
   * identifies that failure mode in the record's threshold slot. */
  function stepSelftest(num, name, testId, prompt, opts = {}) {
    return {
      id: `t${String(testId).padStart(2, "0")}`,
      name: `${num} ${name}`,
      mode: prompt ? "manual" : "auto",
      async run(ctx) {
        if (prompt) {
          ctx.instruct(prompt);
          await ctx.buttons([{ label: "Run test", value: "go", kind: "primary" }]);
        } else {
          ctx.instruct(`${name} — automatic.`);
        }
        ctx.readout("running…");
        const resp = await ctx.race(ctrl("testSelftestOne", [testId],
                                         { timeoutMs: 30000 }));
        const rec = NP.parseSelftestRecord(resp.payload);
        const result = opts.frameThr !== undefined
          ? A.opticalResult(rec, P, opts.frameThr)
          : (rec.status === P.TR_PASS ? "pass" : "fail");
        const big = { pass: "PASS", fail: "FAIL", info: "INFO", skip: "SKIP" };
        ctx.readout(`<span class="big ${result}">${big[result]}</span>  ` +
          `value=${rec.value}  threshold=${rec.threshold}` +
          (result === "info"
            ? "<br>over the placeholder limit — recorded as informational " +
              "(Narbis sets binding optical limits after first articles)"
            : ""));
        /* T01's value is a BITMASK (b0 AFE@0x58, b1 accel@0x18,
         * b2 accel@0x19-alt), not a count — a raw "2 vs thr 3" hid a
         * transiently-NACKing AFE on the bench (2026-08-04). Name the
         * missing device(s). */
        let note = result === "info"
          ? "over placeholder optical limit — informational, not a gate"
          : (result === "fail" && opts.frameThr !== undefined
             ? "hardware fault: frames missing / AFE bring-up failed" : "");
        if (testId === P.TEST_I2C_SCAN && result === "fail") {
          const names = ["AFE@0x58", "accel@0x18", "accel@0x19(alt)"];
          const missing = names.filter((_, i) =>
            (rec.threshold & (1 << i)) && !(rec.value & (1 << i)));
          note = `NO ACK from: ${missing.join(", ") || "?"} (value is a ` +
                 `presence bitmask, not a count)`;
        }
        return {
          result,
          value: String(rec.value),
          expected: `thr ${rec.threshold}`,
          note,
        };
      },
    };
  }

  const STEPS = [
    stepSelftest("1.", "T01 I2C bus scan", P.TEST_I2C_SCAN),
    stepSelftest("2.", "T02 WHO_AM_I (accel + AFE)", P.TEST_ACCEL_WHOAMI),
    stepSelftest("3.", "T03 AFE register R/W", P.TEST_AFE_REG_RW),
    stepSelftest("4.", "T04 Dark noise / ambient leak", P.TEST_AFE_DARK,
      "<strong>Close the clip</strong> on the opaque dark target (or cover " +
      "both optical windows completely). Keep it away from bright light.",
      { frameThr: 150 } /* informational optical limits; 150 = frame min */),
    stepSelftest("5.", "T05 Optical crosstalk", P.TEST_XTALK,
      "<strong>Open the clip</strong> — nothing between emitter and " +
      "photodiode, normal room light. Measures direct LED→PD leakage.",
      { frameThr: 75 }),

    {
      id: "led_visual", name: "6. Red LED visual check", mode: "manual",
      async run(ctx) {
        /* The TEST-build connect indicator already glows steady while
         * connected, so a bare "it lights" prompt proves nothing. Drive
         * a pattern the indicator never makes: dark 1 s -> red-only
         * 3 s -> dark, and ask about THAT. */
        ctx.instruct("The red LED will <strong>strobe: 6 blinks</strong> at " +
          "40 mA, ~1 s apart. At 1% optical duty the glow is faint — a " +
          "blinking dot is far easier to catch than a steady one. " +
          "<strong>Dim the bench light or cup a hand</strong> around the " +
          "emitter, or watch through your <strong>phone camera</strong> " +
          "(cameras also reveal the IR die as a purple dot in step 7).");
        await ctx.buttons([{ label: "Start strobe", value: "go", kind: "primary" }]);
        await ctx.race(ctrl("testLedDrive", [1, 0, 0]));   /* LEDs dark */
        ctx.readout('<span class="big">dark…</span>');
        await ctx.sleep(800 * ctx.ts);
        for (let i = 0; i < 6; i++) {
          await ctx.race(ctrl("testLedDrive", [1, 40, 400]));
          ctx.readout(`<span class="big" style="color:#e34d6a">● RED</span> blink ${i + 1}/6`);
          await ctx.sleep(400 * ctx.ts);
          ctx.readout(`<span class="big">dark</span> blink ${i + 1}/6`);
          await ctx.sleep(600 * ctx.ts);
        }
        ctx.readout("done — did you see 6 red blinks?");
        const ans = await ctx.buttons([
          { label: "Yes — 6 red blinks", value: "yes", kind: "good" },
          { label: "No glow at all", value: "no", kind: "bad" },
        ]);
        return {
          result: ans === "yes" ? "pass" : "fail",
          value: ans === "yes" ? "strobe pattern confirmed" : "no glow",
          expected: "6 red blinks at 40 mA",
          note: ans === "yes" ? "" :
            "if the LED sweep (step 7, fingertip in clip) shows a healthy " +
            "RED span, the die emits and this is a visibility issue — " +
            "retry in darkness / with a phone camera before reworking TX2/FFC",
        };
      },
    },

    {
      id: "led_sweep", name: "7. LED I-V sweeps (IR + red)", mode: "auto",
      async run(ctx) {
        ctx.instruct("<strong>Close the clip on a fingertip</strong> (or a " +
          "white scatter target) and keep it still — the emitter and " +
          "photodiode face each other, so an open clip has almost no " +
          "optical path and the sweep reads flat no matter how healthy " +
          "the LEDs are. Sweeping LED current 0→max in 2 mA steps, both " +
          "LEDs. Each sweep blocks the device ~12 s — the status " +
          "heartbeat pauses; that is normal.");
        /* The firmware's report blob holds only the LAST sweep (test_ops.c
         * blob ver 2) — run 0xE2, fetch 0xEA, then repeat for the other LED. */
        const sweep1 = async (led, label) => {
          ctx.readout(`sweeping ${label}… (device blocks ~12 s)`);
          await ctx.race(ctrl("testLedSweep", [led, 2],
                              { timeoutMs: 30000 * ctx.ts + 8000 }));
          const rep = await ctx.race(fetchSweep());
          if (rep.kind !== 1 || rep.param !== led) {
            throw new Error(`sweep blob is kind ${rep.kind}/param ${rep.param}` +
                            `, expected LED ${led}`);
          }
          return {
            ma: rep.points.map((p) => p.setting),
            /* the swept LED's own RX channel, ambient-subtracted */
            dc: rep.points.map((p) => (led === 0 ? p.ir : p.red) - p.amb),
          };
        };
        const ir = await sweep1(0, "IR");
        const red = await sweep1(1, "RED");
        ctx.drawPlot([
          { x: ir.ma, y: ir.dc, color: COLORS.ir, label: "IR dc-amb vs mA" },
          { x: red.ma, y: red.dc, color: COLORS.red, label: "RED dc-amb vs mA" },
        ], { xlabel: "LED mA", y0: true });
        const vIr = A.sweepVerdict(ir.dc, {}, ir.ma), vRed = A.sweepVerdict(red.dc, {}, red.ma);
        const pass = vIr.pass && vRed.pass;
        ctx.readout(`IR: ${vIr.note} (span ${vIr.span.toLocaleString()})\n` +
                    `RED: ${vRed.note} (span ${vRed.span.toLocaleString()})`);
        return {
          result: pass ? "pass" : "fail",
          value: `IR span ${vIr.span}, RED span ${vRed.span}`,
          expected: "monotonic rise, not flat",
          note: pass ? "" : `${!vIr.pass ? "IR: " + vIr.note + " " : ""}` +
                            `${!vRed.pass ? "RED: " + vRed.note : ""}`,
        };
      },
    },

    {
      id: "rx_sweep", name: "8. RX chain sweep (TIA gain + offset DAC)", mode: "auto",
      async run(ctx) {
        ctx.instruct("<strong>Keep the fingertip in the closed clip</strong> " +
          "(same coupling as the LED sweep — without it the RX sweep " +
          "reads ambient only and fails flat). Sweeping TIA gain codes " +
          "0..7 and the offset DAC at a fixed 20 mA IR drive. Verifies " +
          "INP/INM wiring and DAC function. RF codes are not in ohm " +
          "order — the verdict re-orders by actual RF (500k…10k, 1M, 2M).");
        /* fetch after EACH sweep — the blob holds only the last one */
        ctx.readout("sweeping TIA gain… (device blocks ~4 s)");
        await ctx.race(ctrl("testRxSweep", [0], { timeoutMs: 20000 * ctx.ts + 8000 }));
        const repGain = await ctx.race(fetchSweep());
        if (repGain.kind !== 2 || repGain.param !== 0) {
          throw new Error(`sweep blob kind ${repGain.kind}/param ` +
                          `${repGain.param}, expected TIA gain`);
        }
        const gain = {
          code: repGain.points.map((p) => p.setting),
          dc: repGain.points.map((p) => p.ir - p.amb),
        };
        ctx.readout("sweeping offset DAC… (device blocks ~12 s)");
        await ctx.race(ctrl("testRxSweep", [1], { timeoutMs: 20000 * ctx.ts + 8000 }));
        const repDac = await ctx.race(fetchSweep());
        const series = [{ x: gain.code, y: gain.dc, color: COLORS.accent,
                          label: "dc-amb vs TIA gain code" }];
        if (repDac.kind === 2 && repDac.param === 1) {
          series.push({ x: repDac.points.map((p) => p.setting),
                        y: repDac.points.map((p) => p.ir),
                        color: COLORS.ir, label: "IR dc vs offset-DAC code" });
        }
        ctx.drawPlot(series, { xlabel: "code", y0: true });
        const v = A.gainSweepVerdict(gain.code, gain.dc, { minCorr: 0.9 });
        ctx.readout(`TIA gain sweep: ${v.note} ` +
                    `(r=${v.corr.toFixed(3)} ohm-ordered)`);
        return {
          result: v.pass ? "pass" : "fail",
          value: `span ${v.span}, r ${v.corr.toFixed(3)}`,
          expected: "dc tracks RF (ohm-ordered)",
          note: v.pass ? "" : v.note,
        };
      },
    },

    {
      id: "rate", name: "9. ADC_RDY rate accuracy (10 s)", mode: "auto",
      async run(ctx) {
        const nominal = 100;
        ctx.instruct("Counting ADC_RDY pulses for 10 s at 100 sps. " +
          "Verifies the AFE timing engine and the GPIO16 net. DoD: ±0.5 %.");
        await ctx.race(ctrl("setRate", [P.RATE_100]));
        let left = 10;
        ctx.readout(`counting… ${left} s`);
        ctx.every(1000 * ctx.ts, () => { if (left > 0) ctx.readout(`counting… ${--left} s`); });
        const resp = await ctx.race(ctrl("testRateCount", [10], {
          until: (r) => r.payload.length === 8,
          timeoutMs: 10000 * ctx.ts + 8000,
        }));
        const { pulses, elapsedMs } = NP.parseRateCountResp(resp.payload);
        const v = A.rateVerdict(pulses, elapsedMs, nominal);
        ctx.readout(`<span class="big ${v.pass ? "pass" : "fail"}">` +
          `${v.hz.toFixed(3)} Hz</span>  (${pulses} pulses / ${elapsedMs} ms, ` +
          `err ${v.errPct.toFixed(3)} %)`);
        return {
          result: v.pass ? "pass" : "fail",
          value: `${v.hz.toFixed(3)} Hz (err ${v.errPct.toFixed(3)}%)`,
          expected: `${nominal} Hz ±0.5%`,
        };
      },
    },

    {
      id: "accel", name: "10. Accel orientation (6 faces)", mode: "manual",
      async run(ctx) {
        ctx.instruct("<strong>Slowly rotate the board</strong> through all six " +
          "faces (±X, ±Y, ±Z), holding each still for ~2 s. Verifies LIS2DH12 " +
          "mounting axes, INT1 net and FIFO at max ODR.");
        const captured = new Set();
        let overruns = 0;
        let sinkErrLogged = false;
        let unsub = await subscribe("accel", (ev) => {
          let b;
          try { b = NP.parseAccel(ev.target.value); } catch (e) {
            if (!sinkErrLogged) { sinkErrLogged = true; log(`accel parse error: ${e.message}`); }
            return;
          }
          try {
            if (S.sinks.accel) S.sinks.accel(b);
          } catch (e) {
            /* a throwing sink previously killed the step's rendering
             * silently (bench, 2026-08-04) — surface it, once */
            if (!sinkErrLogged) {
              sinkErrLogged = true;
              log(`accel step render error: ${e.message} @ ${e.stack ? e.stack.split("\n")[1] : "?"}`);
              banner(`Accel step error: ${e.message}`, "error");
            }
          }
        });
        await ctx.race(ctrl("testAccelLive", [1]));
        let doneRes;
        const done = new Promise((res) => { doneRes = res; });
        ctx.setAccelSink((b) => {
          if (b.fifoOverrun) overruns++;
          const cpg = NP.fsCountsPerG(b.fsCode);
          const n = b.samples.length || 1;
          let sx = 0, sy = 0, sz = 0;
          for (const [x, y, z] of b.samples) { sx += x; sy += y; sz += z; }
          const gv = [sx / n / cpg, sy / n / cpg, sz / n / cpg];
          const face = A.faceOf(gv[0], gv[1], gv[2]);
          if (face) captured.add(face);
          drawAccelBars(gv, captured,
            `captured ${captured.size}/6   ${A.faceHint(gv[0], gv[1], gv[2])}` +
            `   FIFO overruns: ${overruns}`);
          if (captured.size === 6) doneRes();
        });
        const timeout = new Promise((_, rej) =>
          setTimeout(() => rej(new Error("timed out waiting for all 6 faces")),
                     120000 * ctx.ts));
        timeout.catch(() => {}); /* silence if done wins the race */
        try {
          await ctx.race(Promise.race([done, timeout]));
        } finally {
          try { await ctrl("testAccelLive", [0]); } catch (_) {}
          await unsub();
        }
        return {
          result: captured.size === 6 ? "pass" : "fail",
          value: `${captured.size}/6 faces, ${overruns} FIFO overruns`,
          expected: "all 6 faces at ±1 g",
          note: overruns ? "FIFO overruns at max ODR — check INT1 net" : "",
        };
      },
    },

    {
      id: "button", name: "11. Button echo (5 presses)", mode: "manual",
      async run(ctx) {
        ctx.instruct("<strong>Press the button 5 times</strong> — quick, " +
          "clean presses, <strong>under a second each</strong> (a long " +
          "hold is the power-off gesture: 5 s in this build). Debounced " +
          "edges stream back with µs timestamps — verifies SW3 and the " +
          "TVS/C22 net.");
        await ctx.race(ctrl("testButtonEcho", [1]));
        const edges = [];
        let doneRes;
        const done = new Promise((res) => { doneRes = res; });
        ctx.setEventSink((ev) => {
          if (ev.type !== P.EV_MARKER || !ev.known ||
              ev.source !== P.MARKER_SRC_BUTTON) return;
          edges.push({ tUs: ev.tUs, id: ev.markerId });
          const rows = edges.map((e, i) => {
            const dt = i ? ((e.tUs - edges[i - 1].tUs) / 1000).toFixed(1) : "—";
            return `#${i + 1}  t=${(e.tUs / 1000).toFixed(1)} ms  Δ=${dt} ms`;
          });
          ctx.readout(`edges: ${edges.length} (need 10)\n` + rows.slice(-8).join("\n"));
          if (edges.length >= 10) doneRes();
        });
        ctx.readout("waiting for presses…");
        let timedOut = false;
        try {
          await ctx.race(Promise.race([done, new Promise((res) =>
            setTimeout(() => { timedOut = true; res(); }, 60000 * ctx.ts))]));
        } finally {
          try { await ctrl("testButtonEcho", [0]); } catch (_) {}
        }
        let ghosts = 0;
        for (let i = 1; i < edges.length; i++) {
          if (edges[i].tUs - edges[i - 1].tUs < 5000) ghosts++;
        }
        const presses = Math.floor(edges.length / 2);
        const pass = !timedOut && presses >= 5 && ghosts === 0;
        return {
          result: pass ? "pass" : "fail",
          value: `${presses} presses (${edges.length} edges), ${ghosts} ghosts`,
          expected: "5 presses, 0 sub-5ms ghost edges",
          note: ghosts ? "ghost edges <5 ms apart — check TVS/C22" :
                (timedOut ? "timed out" : ""),
        };
      },
    },

    {
      id: "charger", name: "12. Charger walk (STAT polarity)", mode: "manual",
      async run(ctx) {
        ctx.instruct("Start <strong>on battery</strong> (USB unplugged). " +
          "Then <strong>plug USB in</strong>, wait a few seconds, and " +
          "<strong>unplug</strong> again. Live VUSB/STAT at 2 Hz. This " +
          "resolves the STAT-polarity VERIFY-ON-BENCH item — note the raw " +
          "STAT bit in each state.");
        const seen = [];            /* decoded charger state transitions */
        const statByState = {};     /* state -> raw STAT bit observed    */
        let doneRes;
        const done = new Promise((res) => { doneRes = res; });
        const stateName = { [P.CHG_ON_BATTERY]: "ON_BATTERY",
                            [P.CHG_CHARGING]: "CHARGING",
                            [P.CHG_COMPLETE]: "COMPLETE" };
        /* Firmware 0xE6 is a stateless POLLED snapshot (test_ops.c): the
         * response payload is {u8 vusb, u8 stat_raw, u8 chg_state}. Poll
         * at 2 Hz while the operator plugs/unplugs; a serialized loop so
         * a slow response never stacks requests. */
        let polling = true;
        const pollLoop = (async () => {
          while (polling) {
            try {
              const resp = await ctrl("testChargerLive", [1]);
              if (resp.payload.length >= 3) {
                const vusb = resp.payload[0], stat = resp.payload[1],
                      st = resp.payload[2];
                statByState[st] = stat;
                if (!seen.length || seen[seen.length - 1] !== st) seen.push(st);
                ctx.readout(
                  `<span class="big">STAT raw = ${stat}</span>   VUSB = ${vusb}\n` +
                  `decoded: ${stateName[st] || st}\n` +
                  `transitions: ${seen.map((s) => stateName[s] || s).join(" → ")}`);
                const iBat = seen.indexOf(P.CHG_ON_BATTERY);
                const iChg = seen.indexOf(P.CHG_CHARGING, iBat + 1);
                if (iBat === 0 && iChg > 0 &&
                    seen.lastIndexOf(P.CHG_ON_BATTERY) > iChg) doneRes();
              }
            } catch (e) {
              if (polling) log(`charger poll: ${e.message}`);
            }
            await new Promise((r) => setTimeout(r, 500 * ctx.ts));
          }
        })();
        let finish;
        try {
          finish = await ctx.race(Promise.race([
            done.then(() => "auto"),
            ctx.buttons([{ label: "Done (evaluate now)", value: "manual" }]),
          ]));
        } finally {
          polling = false;
          await pollLoop.catch(() => {});
        }
        /* Order-agnostic verdict: the operator may start plugged OR on
         * battery (first hardware run started USB-in and failed the
         * old must-start-on-battery rule). What the step actually
         * proves is that BOTH directions of the VUSB/charger seam work:
         * at least one battery→charging edge and one charging→battery
         * edge, in any order. STAT raw per state is recorded as data —
         * polarity is only fully provable in the charging-vs-complete
         * contrast, which needs a full charge (bring-up checklist). */
        let plugEdges = 0, unplugEdges = 0;
        for (let i = 1; i < seen.length; i++) {
          const a = seen[i - 1], b = seen[i];
          if (a === P.CHG_ON_BATTERY && b !== P.CHG_ON_BATTERY) plugEdges++;
          if (a !== P.CHG_ON_BATTERY && b === P.CHG_ON_BATTERY) unplugEdges++;
        }
        const pass = plugEdges >= 1 && unplugEdges >= 1;
        const polarity = (P.CHG_CHARGING in statByState &&
                          P.CHG_ON_BATTERY in statByState)
          ? `STAT=${statByState[P.CHG_CHARGING]} charging / ` +
            `${statByState[P.CHG_ON_BATTERY]} on battery`
          : "polarity not fully observed";
        return {
          result: pass ? "pass" : "fail",
          value: seen.map((s) => stateName[s] || s).join("→") || "no snapshots",
          expected: "a plug edge AND an unplug edge (any order)",
          note: `${polarity}${finish === "manual" ? " (operator-ended)" : ""}` +
                " — record in bring-up checklist (STAT VERIFY-ON-BENCH)",
        };
      },
    },

    {
      id: "batt", name: "13. Battery ADC vs bench meter", mode: "manual",
      async run(ctx) {
        ctx.instruct("Measure the battery voltage at the cell (or TP) with " +
          "the bench meter, then enter the reading. DoD: device within " +
          "±50 mV of the meter.");
        const resp = await ctx.race(ctrl("testBattRaw"));
        const { mvCal, adcRawAvg } = NP.parseBattRawResp(resp.payload);
        ctx.readout(`device: <span class="big">${mvCal} mV</span> ` +
                    `(raw ADC avg ${adcRawAvg})`);
        const bench = await ctx.numberInput("Bench meter reading", "mV");
        if (isNaN(bench)) {
          return { result: "skip", value: `${mvCal} mV`, expected: "±50 mV",
                   note: "no bench reading entered" };
        }
        const v = A.battVerdict(mvCal, bench);
        return {
          result: v.pass ? "pass" : "fail",
          value: `dev ${mvCal} mV vs meter ${bench} mV (Δ${v.delta})`,
          expected: "Δ ≤ 50 mV",
        };
      },
    },

    {
      id: "ppg", name: "14. Live PPG smoke test (60 s)", mode: "manual",
      async run(ctx) {
        ctx.instruct("<strong>Place a fingertip</strong> (or the earlobe) " +
          "between the clip jaws. Streams PPG at 100 sps for 60 s on the " +
          "production notification path; the batch-sequence gap counter " +
          "must stay 0.");
        await ctx.buttons([{ label: "Start streaming", value: "go", kind: "primary" }]);
        await ctx.race(ctrl("setRate", [P.RATE_100]));
        let gaps = 0, lastSeq = null, samples = 0;
        const scope = makeScope(6);
        const unsub = await subscribe("ppg", (ev) => {
          let b;
          try { b = NP.parsePpg(ev.target.value); } catch (_) { return; }
          if (S.sinks.ppg) S.sinks.ppg(b);
        });
        const t0 = performance.now();
        ctx.setPpgSink((b) => {
          if (lastSeq !== null && b.seq !== ((lastSeq + 1) >>> 0)) gaps++;
          lastSeq = b.seq;
          samples += b.ir.length;
          scope.push(b);
        });
        const durMs = 60000 * ctx.ts;
        ctx.every(120, () => {
          scope.draw();
          const el = Math.min(durMs, performance.now() - t0);
          readout(`elapsed ${(el / ctx.ts / 1000).toFixed(0)} / 60 s   ` +
                  `samples ${samples}   ` +
                  `seq gaps <span class="${gaps ? "fail" : "pass"}">${gaps}</span>`);
        });
        await ctx.race(ctrl("streamStart", [P.STREAM_MASK_PPG]));
        try {
          await ctx.sleep(durMs);
        } finally {
          try { await ctrl("streamStop", [P.STREAM_MASK_PPG]); } catch (_) {}
          await unsub();
        }
        const pass = gaps === 0 && samples > 0;
        return {
          result: pass ? "pass" : "fail",
          value: `${samples} samples, ${gaps} seq gaps`,
          expected: "0 gaps over 60 s",
          note: samples === 0 ? "no PPG batches arrived" : "",
        };
      },
    },

    {
      id: "sleep", name: "15. Sleep current + button wake", mode: "manual",
      async run(ctx) {
        ctx.instruct("Insert the <strong>µA meter</strong> in series with the " +
          "battery (or use the µCurrent adapter). On <b>Run</b>, the device " +
          "acks, waits 10 s, then enters OFF — the link will drop (expected). " +
          "Record the settled reading, then press the button and confirm it " +
          "wakes and re-advertises. Target: ≤ 80 µA.");
        await ctx.buttons([{ label: "Run — device sleeps in 10 s", value: "go", kind: "warn" }]);
        S.expectDisconnect = true;
        await ctx.race(ctrl("testSleepNow"));
        ctx.readout("ack received — device sleeps in 10 s, waiting for link drop…");
        await ctx.race(new Promise((res) => {
          const h = () => { S.device.removeEventListener("gattserverdisconnected", h); res(); };
          S.device.addEventListener("gattserverdisconnected", h);
          setTimeout(res, 20000 * ctx.ts);
        }));
        ctx.readout("device is asleep — read the meter now");
        const ua = await ctx.numberInput("Measured sleep current", "µA");
        const woke = await ctx.buttons([
          { label: "Button wakes it (re-advertises)", value: "yes", kind: "good" },
          { label: "No wake", value: "no", kind: "bad" },
        ]);
        const okCurrent = isNaN(ua) ? null : ua <= 80;
        const pass = woke === "yes" && okCurrent !== false;
        return {
          result: pass ? "pass" : "fail",
          value: `${isNaN(ua) ? "not measured" : ua + " µA"}, wake ${woke}`,
          expected: "≤ 80 µA, button wake OK",
          note: okCurrent === false ?
            "over 80 µA — check GPIO21 hold + GPIO2 LP pull-up retention" :
            (isNaN(ua) ? "current not recorded" : ""),
        };
      },
    },
  ];

  /* ---------------- sequence engine + checklist UI ---------------- */

  let activeCtx = null;

  function renderChecklist() {
    const ol = $("checklist");
    ol.innerHTML = "";
    STEPS.forEach((step, i) => {
      const li = document.createElement("li");
      li.id = `step_${step.id}`;
      const r = S.results[i];
      const st = r ? r.result : (step === S.currentStep ? "running" : "");
      li.className = step === S.currentStep ? "active" : "";
      const ico = st === "pass" ? "✓" : st === "fail" ? "✕" :
                  st === "info" ? "ⓘ" :
                  st === "skip" ? "»" : st === "running" ? "…" : i + 1;
      li.innerHTML =
        `<span class="st-ico ${st}">${ico}</span>` +
        `<span class="st-name">${step.name}` +
        `<span class="mode">${step.mode}</span></span>` +
        `<span class="st-val ${st}">${r && r.value ? r.value : ""}</span>`;
      /* any finished step: click to review its stored result (safe at
       * any time, incl. mid-sequence) with an explicit Re-run button
       * when idle — the old click-to-instantly-rerun was undiscoverable
       * and one stray click on "sleep" powered the board off */
      if (r) {
        li.classList.add("rerunnable");
        li.title = "click to view result / re-run";
        li.onclick = () => showStepDetail(step, i);
      }
      ol.appendChild(li);
    });
  }

  function showStepDetail(step, i) {
    const r = S.results[i];
    if (!r) return;
    const esc = (s) => String(s ?? "").replace(/&/g, "&amp;").replace(/</g, "&lt;");
    const box = $("stepDetail");
    box.classList.remove("hidden");
    box.innerHTML =
      `<div class="sd-head"><span class="st-ico ${r.result}">${
        r.result === "pass" ? "✓" : r.result === "fail" ? "✕" :
        r.result === "info" ? "ⓘ" : "»"}</span> <strong>${esc(step.name)}</strong>` +
      `<button class="sd-close" title="close">×</button></div>` +
      `<table class="sd-tbl">` +
      `<tr><td>result</td><td class="${r.result}">${esc(r.result)}</td></tr>` +
      `<tr><td>value</td><td>${esc(r.value) || "—"}</td></tr>` +
      `<tr><td>expected</td><td>${esc(r.expected) || "—"}</td></tr>` +
      `<tr><td>note</td><td>${esc(r.note) || "—"}</td></tr>` +
      `<tr><td>duration</td><td>${((r.durationMs || 0) / 1000).toFixed(1)} s</td></tr>` +
      `</table><div class="sd-actions"></div>`;
    box.querySelector(".sd-close").onclick = () => box.classList.add("hidden");
    const actions = box.querySelector(".sd-actions");
    const busy = S.seqRunning || S.currentStep || S.flashing;
    const connected = S.server && S.server.connected;
    const b = document.createElement("button");
    b.textContent = "Re-run this step";
    b.className = "primary";
    if (busy) {
      b.disabled = true;
      b.title = "finish the running sequence first";
    } else if (!connected) {
      b.disabled = true;
      b.title = "reconnect first";
    } else {
      b.onclick = () => {
        box.classList.add("hidden");
        log(`re-running step: ${step.name}`);
        runStep(step, i);
      };
    }
    actions.appendChild(b);
  }

  async function runStep(step, index) {
    S.currentStep = step;
    renderChecklist();
    $("liveTitle").textContent = step.name;
    $("btnSkip").classList.remove("hidden");
    const ctx = makeCtx();
    activeCtx = ctx;
    const t0 = performance.now();
    let row;
    try {
      const r = await step.run(ctx);
      row = Object.assign({ note: "" }, r);
    } catch (e) {
      row = (e && e.__skip)
        ? { result: "skip", value: "", expected: "", note: "skipped by operator" }
        : { result: "fail", value: "", expected: "", note: e.message || String(e) };
      if (!(e && e.__skip)) log(`step ${step.id} failed: ${row.note}`);
    } finally {
      ctx._cleanup();
      activeCtx = null;
      $("btnSkip").classList.add("hidden");
    }
    row.id = step.id;
    row.name = step.name;
    row.mode = step.mode;
    row.durationMs = Math.round(performance.now() - t0);
    S.results[index] = row;
    S.currentStep = null;
    renderChecklist();
    renderReport();
  }

  function serialNumber() {
    return ($("serialInput").value || "").trim();
  }

  async function runSequence() {
    if (S.seqRunning || S.currentStep) return;   /* incl. single-step rerun */
    if (!serialNumber()) {
      banner("Type the board's serial number first — the report and " +
             "recording are filed per serial.", "error");
      $("serialInput").focus();
      return;
    }
    /* one serial per session: mirror into the dashboard recording box */
    const rs = $("recSerial");
    if (rs) rs.value = serialNumber();
    if (S.seqRunning) return;
    S.seqRunning = true;
    S.results = [];
    $("btnStart").disabled = true;
    banner(null);
    /* Quiesce anything the dashboard tab started: the T0x selftests and
     * the 0xE2/0xE3 sweeps refuse while acquisition runs. streamStop also
     * latches the per-source stop bits, so lingering CCCD subscriptions
     * cannot restart acquisition mid-sequence (fresh subscribes in the
     * accel/PPG steps clear their own bits). */
    try {
      await ctrl("testLedSweepCont", [0, 0, 0]);
      await ctrl("streamStop", [P.STREAM_MASK_PPG | P.STREAM_MASK_ACCEL |
                                P.STREAM_MASK_IBI | P.STREAM_MASK_EVENT]);
    } catch (e) { log(`pre-sequence quiesce: ${e.message}`); }
    for (let i = 0; i < STEPS.length; i++) {
      if (!S.server || (!S.server.connected && STEPS[i].id !== "sleep")) {
        /* connection gone mid-sequence (sleep step disconnects at the end
         * by design — anything earlier is a failure) */
        if (!S.results[i]) {
          S.results[i] = { id: STEPS[i].id, name: STEPS[i].name,
                           mode: STEPS[i].mode, result: "skip",
                           value: "", expected: "", note: "disconnected", durationMs: 0 };
        }
        continue;
      }
      await runStep(STEPS[i], i);
    }
    S.seqRunning = false;
    $("btnStart").disabled = !(S.server && S.server.connected);
    const n = (r) => S.results.filter((x) => x && x.result === r).length;
    const summary = `Sequence complete: ${n("pass")} pass, ${n("fail")} fail, ` +
                    `${n("info")} informational, ${n("skip")} skipped.`;
    banner(summary, n("fail") ? "error" : "");
    instruct(summary + " Review the report card below, then Download JSON " +
             "or Print for the manufacturing record." +
             (n("info") ? " ⓘ steps recorded optical values against " +
              "placeholder limits — informational, not a ship gate." : ""));
    renderChecklist();
    renderReport();
  }

  /* ---------------- report card ---------------- */

  function reportObject() {
    const steps = S.results.filter(Boolean);
    const n = (r) => steps.filter((x) => x.result === r).length;
    return {
      tool: "narbis-testapp",
      generated: new Date().toISOString(),
      mock: MOCK || S.demoMode,
      serial: serialNumber() || null,
      device: {
        name: S.device ? S.device.name : null,
        id: S.device ? S.device.id : null,
        manufacturer: S.dis.manufacturer || null,
        model: S.dis.model || null,
        fw: S.dis.fw || null,
        hw: S.dis.hw || null,
        protocol: S.protoVer !== null
          ? `${S.protoVer >> 8}.${S.protoVer & 0xFF}` : null,
        clockPair: S.clock,
      },
      summary: { pass: n("pass"), fail: n("fail"),
                 info: n("info"), skip: n("skip") },
      steps,
      /* full session log rides along so a failed unit is diagnosable
       * from the report alone (BLE traffic, errors, disconnects) */
      log: S.logLines.slice(-4000),
    };
  }

  function renderReport() {
    const rows = S.results.filter(Boolean);
    const tb = $("reportTable").querySelector("tbody");
    tb.innerHTML = "";
    rows.forEach((r, i) => {
      const tr = document.createElement("tr");
      tr.innerHTML =
        `<td>${i + 1}</td><td>${r.name}</td><td>${r.mode}</td>` +
        `<td class="r-${r.result}">${r.result.toUpperCase()}</td>` +
        `<td class="val">${r.value || ""}</td>` +
        `<td class="val">${r.expected || ""}</td>` +
        `<td>${r.note || ""}</td>`;
      tb.appendChild(tr);
    });
    const d = reportObject().device;
    $("reportMeta").textContent = rows.length
      ? `${d.name || "?"}  fw ${d.fw || "?"}  hw ${d.hw || "?"}  ` +
        `proto ${d.protocol || "?"}  ·  ${new Date().toLocaleString()}` +
        (MOCK ? "  ·  MOCK RUN — not a hardware record" : "")
      : "";
    $("btnDownload").disabled = rows.length === 0;
    $("btnPrint").disabled = rows.length === 0;
  }

  function downloadReport() {
    const blob = new Blob([JSON.stringify(reportObject(), null, 2)],
                          { type: "application/json" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    const ser = (serialNumber() || S.device && S.device.name || "narbis")
      .replace(/\W+/g, "_");
    a.download = `pcbtest_${ser}_${new Date().toISOString().replace(/[:.]/g, "-")}.json`;
    a.click();
    URL.revokeObjectURL(a.href);
  }

  /* ---------------- boot ---------------- */

  $("btnConnect").onclick = () => connect().catch((e) => {
    banner(`Connect failed: ${e.message}`, "error");
    log(`connect failed: ${e.message}`);
  });
  /* runtime mock: full simulate-mode device, real-time (timescale 1) */
  $("btnDemo").onclick = () => {
    if (S.server && S.server.connected) return;
    S.demoMode = true;
    S.bt = NarbisMock.create({ timescale: 1, sim: true });
    $("mockBadge").classList.remove("hidden");
    log("demo mode: simulated device (no hardware)");
    connect().catch((e) => {
      banner(`Demo connect failed: ${e.message}`, "error");
      log(`demo connect failed: ${e.message}`);
    });
  };
  $("btnDisconnect").onclick = () => {
    if (S.server && S.server.connected) S.server.disconnect();
    if (S.demoMode && !S.flashing) {
      /* leave demo: next Connect uses the real chooser again */
      S.demoMode = false;
      S.bt = null;
      if (!MOCK) $("mockBadge").classList.add("hidden");
    }
  };
  $("btnStart").onclick = () => runSequence();
  $("btnSkip").onclick = () => { if (activeCtx) activeCtx._skip(); };
  $("btnDownload").onclick = downloadReport;
  $("btnPrint").onclick = () => window.print();
  $("btnFlash").onclick = () => onFlashClicked();
  $("otaFile").onchange = () => otaUiSync();

  if (MOCK) {
    $("mockBadge").classList.remove("hidden");
    document.title += " (mock)";
  }
  /* ---- bridge for dashboard.js (same connection, same log) ---- */
  window.NarbisApp = {
    S, NP,
    isMock: () => MOCK || S.demoMode,
    ctrl, subscribe, fetchBlob, log, banner, hexBytes,
    getLogLines: () => S.logLines,
    onGuidedBusy: () => S.seqRunning || S.flashing || !!S.currentStep,
  };

  /* Every uncaught error lands in the session log (and therefore the
   * report JSON) — a silent JS exception cost a bench day when the
   * accel step stopped rendering with nothing in the report
   * (2026-08-04). Keep these AFTER NarbisApp so log() exists. */
  window.addEventListener("error", (e) => {
    log(`JS ERROR: ${e.message} @ ${(e.filename || "?").split("/").pop()}:${e.lineno}`);
  });
  window.addEventListener("unhandledrejection", (e) => {
    const r = e.reason;
    log(`PROMISE ERROR: ${(r && r.message) || r}` +
        (r && r.stack ? ` @ ${r.stack.split("\n")[1]}` : ""));
  });

  renderChecklist();
  if (!MOCK && !navigator.bluetooth) {
    banner("Web Bluetooth is not available here. Chrome/Edge on desktop or " +
           "Android over https/localhost is required for hardware; iOS is " +
           "unsupported. Add ?mock=1 for the no-hardware demo.", "");
  }
})();
