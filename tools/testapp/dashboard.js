/*
 * dashboard.js — functional-test Dashboard tab for the Narbis test app.
 *
 * Loads after app.js as a classic script and talks to the device through
 * the window.NarbisApp bridge (same BLE connection, same verbose log).
 * The pure parts (rolling-window rings, autoscale/zoom math, CSV + ZIP
 * builders, event formatting, seq-gap counting) live in NarbisDashCore,
 * exported for node so selfcheck.mjs can exercise them without a DOM.
 */
"use strict";

const NarbisDashCore = (() => {

  /* ---------------- rolling time/value ring ---------------- */
  /* Parallel t/y arrays trimmed to the last windowS seconds (Infinity =
   * whole session). Trim is amortized: splice only when 64+ points age
   * out, so a 500 sps PPG push stays cheap. */
  function makeRing(windowS, hardCap) {
    const cap = hardCap || 40000;
    return {
      t: [], y: [], windowS,
      push(t, y) {
        this.t.push(t); this.y.push(y);
        this.trim(t);
      },
      trim(now) {
        if (this.windowS !== Infinity) {
          const cut = now - this.windowS;
          let i = 0;
          while (i < this.t.length && this.t[i] < cut) i++;
          if (i > 64) { this.t.splice(0, i); this.y.splice(0, i); }
        }
        if (this.t.length > cap) {
          const drop = this.t.length - cap;
          this.t.splice(0, drop); this.y.splice(0, drop);
        }
      },
      clear() { this.t.length = 0; this.y.length = 0; },
      last() { return this.y.length ? this.y[this.y.length - 1] : null; },
      lastT() { return this.t.length ? this.t[this.t.length - 1] : null; },
    };
  }

  /* min/max of the points inside [t0, t1] (scan from the end) */
  function ringMinMax(ring, t0, t1) {
    let lo = Infinity, hi = -Infinity;
    for (let i = ring.t.length - 1; i >= 0; i--) {
      const t = ring.t[i];
      if (t > t1) continue;
      if (t < t0) break;
      const v = ring.y[i];
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    return { lo, hi };
  }

  /* ---------------- autoscale + zoom math ---------------- */
  /* Zoom state: null = autoscale; else {center, span} (Y units).
   * +/− rescale the span around the CURRENT center (spec), X stays 30 s. */
  function autoRange(lo, hi) {
    if (!isFinite(lo) || !isFinite(hi)) { lo = 0; hi = 1; }
    if (hi === lo) { hi += 0.5; lo -= 0.5; }
    const span = (hi - lo) * 1.12;
    return { center: (lo + hi) / 2, span };
  }
  function zoomStep(cur, dataLo, dataHi, dir) {
    const base = cur || autoRange(dataLo, dataHi);
    const f = dir > 0 ? 1 / 1.6 : 1.6;
    return { center: base.center, span: Math.max(1e-9, base.span * f) };
  }
  function applyZoom(dataLo, dataHi, zoom) {
    const z = zoom || autoRange(dataLo, dataHi);
    return [z.center - z.span / 2, z.center + z.span / 2];
  }

  /* nice tick positions (same recipe as app.js's plot helper) */
  function niceTicks(lo, hi, n) {
    if (hi === lo) hi = lo + 1;
    const span = hi - lo;
    const step0 = Math.pow(10, Math.floor(Math.log10(span / n)));
    const err = span / n / step0;
    const step = step0 * (err >= 7.5 ? 10 : err >= 3.5 ? 5 : err >= 1.5 ? 2 : 1);
    const out = [];
    for (let v = Math.ceil(lo / step) * step; v <= hi + step / 1e6; v += step) {
      out.push(v);
    }
    return out;
  }

  /* ---------------- per-stream sequence-gap counter ---------------- */
  function gapCounter() {
    return {
      last: null, gaps: 0, batches: 0,
      feed(seq) {
        this.batches++;
        if (this.last !== null && seq !== ((this.last + 1) >>> 0) &&
            seq !== this.last /* event batches repeat per record */) {
          this.gaps++;
        }
        if (this.last === null || seq !== this.last) this.last = seq;
      },
    };
  }

  /* ---------------- CSV ---------------- */
  function csv(header, rows) {
    const esc = (v) => {
      const s = v === null || v === undefined ? "" : String(v);
      return /[",\n]/.test(s) ? '"' + s.replace(/"/g, '""') + '"' : s;
    };
    let out = header.join(",") + "\n";
    for (const r of rows) out += r.map(esc).join(",") + "\n";
    return out;
  }

  /* ---------------- minimal STORE-only ZIP builder ---------------- */
  /* files: [{name, data: Uint8Array|string}]; crc32(prev, bytes) is
   * NarbisProto.ota.crc32 (zlib polynomial). No compression: bench CSVs
   * zip for packaging, not size. */
  function zipStore(files, crc32) {
    const enc = new TextEncoder();
    const now = new Date();
    const dosTime = ((now.getHours() << 11) | (now.getMinutes() << 5) |
                     (now.getSeconds() >> 1)) & 0xFFFF;
    const dosDate = (((now.getFullYear() - 1980) << 9) |
                     ((now.getMonth() + 1) << 5) | now.getDate()) & 0xFFFF;
    const parts = [], centrals = [];
    let off = 0;
    for (const f of files) {
      const name = enc.encode(f.name);
      const data = f.data instanceof Uint8Array ? f.data : enc.encode(f.data);
      const crc = crc32(0, data);
      const lh = new Uint8Array(30 + name.length);
      const lv = new DataView(lh.buffer);
      lv.setUint32(0, 0x04034B50, true);
      lv.setUint16(4, 20, true);            /* version needed */
      lv.setUint16(8, 0, true);             /* method 0 = STORE */
      lv.setUint16(10, dosTime, true); lv.setUint16(12, dosDate, true);
      lv.setUint32(14, crc, true);
      lv.setUint32(18, data.length, true); lv.setUint32(22, data.length, true);
      lv.setUint16(26, name.length, true);
      lh.set(name, 30);
      const ch = new Uint8Array(46 + name.length);
      const cv = new DataView(ch.buffer);
      cv.setUint32(0, 0x02014B50, true);
      cv.setUint16(4, 20, true); cv.setUint16(6, 20, true);
      cv.setUint16(12, dosTime, true); cv.setUint16(14, dosDate, true);
      cv.setUint32(16, crc, true);
      cv.setUint32(20, data.length, true); cv.setUint32(24, data.length, true);
      cv.setUint16(28, name.length, true);
      cv.setUint32(42, off, true);          /* local header offset */
      ch.set(name, 46);
      parts.push(lh, data);
      centrals.push(ch);
      off += lh.length + data.length;
    }
    let cdLen = 0;
    for (const c of centrals) cdLen += c.length;
    const eocd = new Uint8Array(22);
    const ev = new DataView(eocd.buffer);
    ev.setUint32(0, 0x06054B50, true);
    ev.setUint16(8, files.length, true); ev.setUint16(10, files.length, true);
    ev.setUint32(12, cdLen, true);
    ev.setUint32(16, off, true);            /* central dir start */
    const out = new Uint8Array(off + cdLen + 22);
    let o = 0;
    for (const p of [...parts, ...centrals, eocd]) { out.set(p, o); o += p.length; }
    return out;
  }

  /* ---------------- event record -> short ticker string -------------- */
  function fmtEvent(rec, P) {
    const s = (rec.tUs !== null ? (rec.tUs / 1e6).toFixed(2) : "?") + "s ";
    switch (rec.type) {
      case P.EV_AGC_STEP:
        return s + `AGC ${rec.led ? "RED" : "IR"} ${rec.oldMa}→${rec.newMa} mA`;
      case P.EV_GATE:
        return s + `gate ${rec.state ? "ON" : "off"}` +
               (rec.reasonMask ? ` r=0x${rec.reasonMask.toString(16)}` : "");
      case P.EV_WEAR:
        return s + `wear ${rec.worn ? "ON" : "OFF"}`;
      case P.EV_MARKER:
        if (rec.source === P.MARKER_SRC_BUTTON) {
          return s + (rec.markerId === 1000 ? "button PRESS" :
                      rec.markerId === 1001 ? "button release" :
                      `button marker #${rec.markerId}`);
        }
        return s + `marker #${rec.markerId}`;
      case P.EV_ERROR:
        return s + `ERR ${P.ERROR_CODE_NAME[rec.code] || rec.code} arg=${rec.arg}`;
      case P.EV_RATE_CHANGE:
        return s + `rate ${P.RATE_SPS[rec.oldCode]}→${P.RATE_SPS[rec.newCode]} sps`;
      case P.EV_AGC_OFFDAC:
        return s + `offDAC ph${rec.phase} ${rec.oldCode}→${rec.newCode}`;
      case P.EV_SELFTEST_DONE:
        return s + `selftest done ${rec.passCount} pass ${rec.failCount} fail`;
      default:
        return s + `event 0x${rec.type.toString(16)} (${rec.len}B)`;
    }
  }

  /* ---------------- ⓘ info dictionary ------------------------------ */
  /* Plain-language definitions for every dashboard element. Keyed by
   * anchor: {sel: where the ⓘ goes (appended inside), text}. Kept in
   * core so selfcheck can lint completeness without a DOM. */
  const INFO = {
    "graph.ppg": { sel: '[data-graph="ppg"] .graph-title', text:
      "The light received through the earlobe. IR and RED are the two LED " +
      "channels; AMB is the ambient reading taken with both LEDs off (stray " +
      "room light). The small ripples are your pulse — each heartbeat " +
      "briefly changes how much blood, and therefore light, is in the lobe. " +
      "A trace pinned flat at the top or bottom means the receiver is " +
      "saturated: lower the LED current or the TIA gain." },
    "graph.acc": { sel: '[data-graph="acc"] .graph-title', text:
      "The motion sensor's three axes (X, Y, Z) plus |a|, the combined " +
      "magnitude. A perfectly still board reads 1 g total — gravity — on " +
      "whichever axis points down. Motion spikes here are what triggers " +
      "the artifact gate." },
    "graph.beat": { sel: '[data-graph="beat"] .graph-title', text:
      "The device's on-board beat detection. The line is smoothed heart " +
      "rate (bpm); each dot is one IBI — the inter-beat interval, the " +
      "exact milliseconds between two heartbeats. No dots = no beats " +
      "detected (nothing on the sensor, or the signal is gated)." },
    "graph.batt": { sel: '[data-graph="batt"] .graph-title', text:
      "Battery voltage across the whole session. A full LiPo cell is " +
      "about 4.2 V, empty is 3.3 V. The reading is taken on the cell " +
      "itself, so it stays honest while charging." },
    "ro.batt": { sel: "#roBattV", after: true, text:
      "Live battery voltage (3 decimals), estimated percentage, and the " +
      "charger state: on-battery, charging (USB in, cell filling at " +
      "50 mA), or complete. There is no current readout — the V2.1 board " +
      "has no current-sense part; use a bench meter for that." },
    "ro.btn": { sel: "#roBtn", after: true, text:
      "Live state of the side button, straight from the firmware. In this " +
      "functional-test firmware the button ONLY powers on (hold 1 s) and " +
      "off (hold 5 s) — shorter presses do nothing except light this lamp, " +
      "which is exactly how you verify the switch works." },
    "ro.gatewear": { sel: "#roGate", after: true, text:
      "GATE lights when the firmware flags the PPG as unreliable — motion, " +
      "receiver saturation, or a sudden signal step. Data keeps streaming; " +
      "only beat detection pauses. WEAR lights when the on-ear detector " +
      "believes the clip is on skin (IR light level + pulse activity)." },
    "ro.ver": { sel: "#roVer", after: true, text:
      "Firmware version (from the build's git tag), hardware revision, " +
      "BLE protocol version, and this app's version. Include these in any " +
      "issue report." },
    "ro.link": { sel: "#roLink", after: true, text:
      "BLE link quality. Every stream packet carries a sequence number, " +
      "so lost packets are countable: 'gaps' is packets the app never " +
      "received; 'drops' is packets the device discarded because the link " +
      "was too slow. 0 / 0 for a whole session is a clean link." },
    "ro.ticker": { sel: "#dashTicker", before: true, text:
      "Real-time event feed from the device: automatic gain steps, gate " +
      "on/off spans, wear changes, button markers, errors. Every " +
      "amplitude jump in the graphs should have a matching event here — " +
      "that is how recordings stay interpretable." },
    "ctl.ir": { sel: "#ledIr", before: true, text:
      "Drive current for the infrared LED, 0–50 mA in ~0.8 mA steps. " +
      "More current = more light through the lobe = bigger signal. " +
      "Invisible to the eye — verify it with the PPG graph, not by " +
      "looking. Touching this freezes the automatic gain control so " +
      "your setting sticks." },
    "ctl.red": { sel: "#ledRed", before: true, text:
      "Drive current for the red LED, 0–40 mA (its safe limit is lower " +
      "than IR's). You can see this one glow dimly — the glow check in " +
      "the guided sequence uses it. Touching this freezes auto-gain." },
    "ctl.tia": { sel: "#selTia", after: true, text:
      "Receiver sensitivity. TIA = transimpedance amplifier, the stage " +
      "that turns the photodiode's tiny current into a voltage; the ohm " +
      "value is its gain (100 kΩ ⇒ 1 µA of light becomes 0.1 V). Raise " +
      "it for weak signals, lower it if the trace clips. One setting is " +
      "shared by IR, RED and AMB alike, so changing it re-scales the " +
      "whole PPG graph at once." },
    "ctl.rate": { sel: "#selRate", after: true, text:
      "PPG samples per second: 50, 100, 200, 250 or 500. Each sample " +
      "fires both LEDs once for ~100 µs, so this is also the LED pulse " +
      "rate. Higher rates give finer beat timing at more power and BLE " +
      "bandwidth. 100 sps is the product default." },
    "ctl.odr": { sel: "#selOdr", after: true, text:
      "Accelerometer ODR — Output Data Rate, its samples per second " +
      "(10–400 Hz). Higher = finer motion detail, more BLE traffic. " +
      "50 Hz is plenty for motion-artifact gating." },
    "ctl.fs": { sel: "#selFs", after: true, text:
      "Accelerometer full-scale range: the largest acceleration it can " +
      "represent, ±2/4/8/16 g. Small range = finest resolution per " +
      "count; big range = survives hard shocks but 8× coarser. ±4 g " +
      "(default) covers vigorous head motion without clipping. 1 g is " +
      "gravity — a resting board shows 1 g on the downward axis." },
    "ctl.sweep": { sel: ".sweep-box legend", text:
      "Runs the LEDs through a continuous test pattern in the firmware " +
      "itself: ramp 0 → max over the phase time, hold at max, ramp back " +
      "to 0, hold at 0, repeat until stopped. Pick either LED or both. " +
      "Watch the PPG graph respond — a flat response means an open " +
      "emitter or flex-cable fault. Sliders lock while sweeping." },
    "ctl.stream": { sel: "#btnStream", after: true, text:
      "Starts/stops the sensor streams (PPG + accel + beats + events). " +
      "Sensors only run while streaming — LEDs off and radio quiet " +
      "otherwise." },
    "ctl.marker": { sel: "#btnMarker", after: true, text:
      "Drops a timestamped marker into the event stream and any active " +
      "recording — for tagging moments ('touched the sensor NOW') so " +
      "they can be found in the data later." },
    "ctl.selftest": { sel: "#btnSelftest", after: true, text:
      "Runs the built-in hardware self-test: I²C bus scan, sensor " +
      "identity checks, dark-noise and light-leak measurements, " +
      "accelerometer self-test, battery and charger checks. Results " +
      "appear below as a PASS/FAIL table." },
    "ctl.sleep": { sel: "#btnDeepSleep", after: true, text:
      "For the bench current measurement: 10 s after you confirm, the " +
      "board enters deep sleep (target ≤ 80 µA at the cell — needs a " +
      "meter in series with the battery). Wake it by holding the button " +
      "1 s." },
    "ctl.knobs": { sel: "#dashKnobsBox summary", text:
      "Every tunable parameter in the firmware (62 of them), fetched " +
      "live from the device — gain loops, filters, beat detection, " +
      "gating, power thresholds. Edits apply immediately; SAVE persists " +
      "them across reboots; RESET returns to factory defaults. If a " +
      "value misbehaves, RESET is always safe." },
    "rec.serial": { sel: "#recSerial", before: true, text:
      "The board's serial number — stamped into the recording filename " +
      "and the report card so every file traces to a physical unit." },
    "rec.record": { sel: "#btnRecord", after: true, text:
      "Captures everything while armed — every PPG and accel sample, " +
      "every beat, status snapshot and event, plus the full session log — " +
      "and downloads it as one .zip of CSV files when stopped." },
    "log.session": { sel: "#dashLogDl", before: true, text:
      "Verbose session log: every command this app sent, every reply, " +
      "and the device's once-per-second status line. Download the full " +
      "log with the button — attach it to any problem report." },
  };

  return { makeRing, ringMinMax, autoRange, zoomStep, applyZoom, niceTicks,
           gapCounter, csv, zipStore, fmtEvent, INFO };
})();

if (typeof module !== "undefined" && module.exports) {
  module.exports = { NarbisDashCore };
}

/* ================================================================== *
 * Browser dashboard                                                    *
 * ================================================================== */

if (typeof document !== "undefined" && document.getElementById("dashPane"))
(() => {
  const A = window.NarbisApp;
  const NP = A.NP, P = NP.P, S = A.S;
  const C = NarbisDashCore;
  const $ = (id) => document.getElementById(id);

  const TIA_LABELS = ["500k", "250k", "100k", "50k", "25k", "10k", "1M", "2M"];
  const FS_LABELS = ["±2 g", "±4 g", "±8 g", "±16 g"];
  const APP_VER = "testapp 1.1";

  /* ---------------- tab bar (pure show/hide) ---------------- */
  const guidedEls = [document.querySelector("main"), $("otaPane"), $("reportPane")];
  function showTab(dash) {
    $("tabDash").classList.toggle("active", dash);
    $("tabGuided").classList.toggle("active", !dash);
    $("dashPane").classList.toggle("hidden", !dash);
    for (const el of guidedEls) el.classList.toggle("hidden", dash);
  }
  $("tabGuided").onclick = () => showTab(false);
  $("tabDash").onclick = () => {
    showTab(true);
    /* Acquisition is subscription-gated and the manual controls need a
     * running PPG engine (agcManual answers WRONG_STATE otherwise) —
     * first hardware session read as "dashboard dead" because nothing
     * started the streams. Auto-start on tab entry; the Stream button
     * still toggles. */
    /* Never auto-start streams while the guided sequence (or a flash)
     * is running — the streamStart would stomp the active step's
     * acquisition state mid-test. */
    if (!D.streaming && A.S.chars.control && !A.onGuidedBusy()) {
      $("btnStream").onclick();
    }
  };

  /* ---------------- dashboard state ---------------- */
  const D = {
    streaming: false,
    subs: [],                 /* unsubscribe fns for ppg/accel/ibi      */
    agcFrozen: false,         /* we sent AGC_FREEZE(1) already          */
    sweepOn: false,
    hostMarker: 0,
    gaps: { ppg: C.gapCounter(), accel: C.gapCounter(),
            ibi: C.gapCounter(), event: C.gapCounter() },
    ticker: [],
    t0Wall: Date.now(),
    knobsLoaded: false,
    lastBtnMarker: null,      /* instant lamp override until next STATUS */
    rec: { on: false, t0: null, ppg: [], accel: [], ibi: [], status: [],
           events: [] },
  };

  /* ---------------- graphs ---------------- */
  const COL = { ir: "#e07840", red: "#e34d6a", amb: "#7f8ea3",
                x: "#4da3ff", y: "#38c172", z: "#e0a63c", m: "#b98ef0",
                hr: "#e34d6a", ibi: "#4da3ff", batt: "#38c172",
                grid: "#222d3b", axis: "#3a4a5e", text: "#8ba0b8" };

  const graphs = {
    ppg: {
      cv: $("dgPpg"), windowS: 30, zoom: null, paused: false,
      series: [
        { ring: C.makeRing(30), color: COL.ir, label: "IR" },
        { ring: C.makeRing(30), color: COL.red, label: "RED" },
        { ring: C.makeRing(30), color: COL.amb, label: "AMB" },
      ],
    },
    acc: {
      cv: $("dgAcc"), windowS: 30, zoom: null, paused: false,
      series: [
        { ring: C.makeRing(30), color: COL.x, label: "X" },
        { ring: C.makeRing(30), color: COL.y, label: "Y" },
        { ring: C.makeRing(30), color: COL.z, label: "Z" },
        { ring: C.makeRing(30), color: COL.m, label: "|a|" },
      ],
    },
    beat: {
      cv: $("dgBeat"), windowS: 30, zoom: null, paused: false,
      series: [{ ring: C.makeRing(30), color: COL.hr, label: "HR bpm" }],
      series2: [{ ring: C.makeRing(30), color: COL.ibi, label: "IBI ms",
                  dots: true }],
    },
    batt: {
      cv: $("dgBatt"), windowS: Infinity, zoom: null, paused: false,
      series: [{ ring: C.makeRing(Infinity, 20000), color: COL.batt,
                 label: "V" }],
      fmt: (v) => v.toFixed(3),
    },
  };

  function drawGraph(g) {
    const cv = g.cv, ctx = cv.getContext("2d");
    ctx.clearRect(0, 0, cv.width, cv.height);
    const L = 54, R = g.series2 ? 50 : 10, T = 8, B = 18;
    const W = cv.width - L - R, H = cv.height - T - B;
    const all = g.series.concat(g.series2 || []);
    let tMax = -Infinity;
    for (const s of all) {
      const lt = s.ring.lastT();
      if (lt !== null && lt > tMax) tMax = lt;
    }
    if (!isFinite(tMax)) {
      ctx.fillStyle = COL.text;
      ctx.font = "12px ui-monospace, monospace";
      ctx.fillText("waiting for data…", L + 10, T + 24);
      return;
    }
    const winS = g.windowS === Infinity
      ? Math.max(30, tMax - (g.series[0].ring.t[0] || 0)) : g.windowS;
    const t0 = tMax - winS;
    const px = (t) => L + ((t - t0) / winS) * W;

    const scaleOf = (seriesArr, zoom) => {
      let lo = Infinity, hi = -Infinity;
      for (const s of seriesArr) {
        const r = C.ringMinMax(s.ring, t0, tMax);
        if (r.lo < lo) lo = r.lo;
        if (r.hi > hi) hi = r.hi;
      }
      const [ylo, yhi] = C.applyZoom(lo, hi, zoom);
      return { ylo, yhi, py: (y) => T + H - ((y - ylo) / (yhi - ylo || 1)) * H };
    };
    const sc1 = scaleOf(g.series, g.zoom);

    /* grid + left axis */
    ctx.font = "10px ui-monospace, monospace";
    ctx.lineWidth = 1;
    for (const tk of C.niceTicks(sc1.ylo, sc1.yhi, 4)) {
      ctx.strokeStyle = COL.grid;
      ctx.beginPath(); ctx.moveTo(L, sc1.py(tk)); ctx.lineTo(L + W, sc1.py(tk));
      ctx.stroke();
      ctx.fillStyle = COL.text;
      const label = g.fmt ? g.fmt(tk)
        : Math.abs(tk) >= 1000 ? Math.round(tk).toLocaleString()
        : String(Math.round(tk * 100) / 100);
      ctx.fillText(label, 3, sc1.py(tk) + 3);
    }
    /* x ticks: relative seconds */
    for (let ds = 0; ds <= winS + 0.01; ds += Math.max(5, Math.round(winS / 6))) {
      const t = tMax - ds;
      ctx.fillStyle = COL.text;
      ctx.fillText(`-${Math.round(ds)}s`, px(t) - 8, cv.height - 5);
    }
    ctx.strokeStyle = COL.axis;
    ctx.strokeRect(L, T, W, H);

    const drawSeries = (s, sc) => {
      const r = s.ring;
      if (!r.t.length) return;
      ctx.strokeStyle = s.color; ctx.fillStyle = s.color;
      if (s.dots) {
        for (let i = r.t.length - 1; i >= 0; i--) {
          if (r.t[i] < t0) break;
          ctx.beginPath();
          ctx.arc(px(r.t[i]), sc.py(r.y[i]), 2.4, 0, 7);
          ctx.fill();
        }
        return;
      }
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      let started = false;
      let i0 = 0;
      while (i0 < r.t.length && r.t[i0] < t0) i0++;
      for (let i = i0; i < r.t.length; i++) {
        const X = px(r.t[i]), Y = sc.py(r.y[i]);
        if (!started) { ctx.moveTo(X, Y); started = true; }
        else ctx.lineTo(X, Y);
      }
      ctx.stroke();
    };
    for (const s of g.series) drawSeries(s, sc1);

    if (g.series2) {
      const sc2 = scaleOf(g.series2, g.zoom2 || null);
      for (const s of g.series2) drawSeries(s, sc2);
      ctx.fillStyle = COL.ibi;
      for (const tk of C.niceTicks(sc2.ylo, sc2.yhi, 3)) {
        ctx.fillText(String(Math.round(tk)), L + W + 5, sc2.py(tk) + 3);
      }
    }
  }

  /* per-graph pause / zoom buttons */
  for (const card of document.querySelectorAll(".graph-card")) {
    const g = graphs[card.dataset.graph];
    card.querySelectorAll("button").forEach((b) => {
      b.onclick = () => {
        const dataRange = () => {
          let lo = Infinity, hi = -Infinity;
          const tMax = Math.max(...g.series.map((s) => s.ring.lastT() || 0));
          for (const s of g.series) {
            const r = C.ringMinMax(s.ring, tMax - (g.windowS === Infinity ? 1e12 : g.windowS), tMax);
            if (r.lo < lo) lo = r.lo;
            if (r.hi > hi) hi = r.hi;
          }
          return [lo, hi];
        };
        if (b.dataset.act === "pause") {
          g.paused = !g.paused;
          b.classList.toggle("on", g.paused);
          b.textContent = g.paused ? "▶" : "⏸";
        } else if (b.dataset.act === "auto") {
          g.zoom = null; g.zoom2 = null;
        } else {
          const [lo, hi] = dataRange();
          g.zoom = C.zoomStep(g.zoom, lo, hi, b.dataset.act === "zin" ? 1 : -1);
        }
      };
    });
  }

  /* ~15 fps redraw */
  setInterval(() => {
    if ($("dashPane").classList.contains("hidden")) return;
    for (const k of Object.keys(graphs)) {
      if (!graphs[k].paused) drawGraph(graphs[k]);
    }
    syncConnUi();
  }, 66);

  /* ---------------- stream handlers ---------------- */

  function onPpg(b) {
    D.gaps.ppg.feed(b.seq);
    const sps = P.RATE_SPS[b.rateCode] || 100;
    for (let i = 0; i < b.ir.length; i++) {
      const t = (b.t0Us + i * 1e6 / sps) / 1e6;
      graphs.ppg.series[0].ring.push(t, b.ir[i]);
      graphs.ppg.series[1].ring.push(t, b.red[i]);
      if (b.amb) graphs.ppg.series[2].ring.push(t, b.amb[i]);
      if (D.rec.on) {
        D.rec.ppg.push([Math.round(b.t0Us + i * 1e6 / sps), b.ir[i], b.red[i],
                        b.amb ? b.amb[i] : ""]);
      }
    }
  }

  function onAccel(b) {
    D.gaps.accel.feed(b.seq);
    const odr = P.ODR_HZ[b.odrCode] || 50;
    const cpg = NP.fsCountsPerG(b.fsCode);
    b.samples.forEach(([x, y, z], i) => {
      const t = (b.t0Us + i * 1e6 / odr) / 1e6;
      const gx = x / cpg, gy = y / cpg, gz = z / cpg;
      graphs.acc.series[0].ring.push(t, gx);
      graphs.acc.series[1].ring.push(t, gy);
      graphs.acc.series[2].ring.push(t, gz);
      graphs.acc.series[3].ring.push(t, Math.sqrt(gx * gx + gy * gy + gz * gz));
      if (D.rec.on) {
        D.rec.accel.push([Math.round(b.t0Us + i * 1e6 / odr), x, y, z,
                          b.fsCode, b.fifoOverrun ? 1 : 0]);
      }
    });
  }

  function onIbi(b) {
    D.gaps.ibi.feed(b.seq);
    for (const r of b.records) {
      const t = r.tBeatUs / 1e6;
      if (r.ibiMs > 0) {
        graphs.beat.series[0].ring.push(t, 60000 / r.ibiMs);
        graphs.beat.series2[0].ring.push(t, r.ibiMs);
        $("dashBeatNum").textContent =
          `HR ${Math.round(60000 / r.ibiMs)} bpm · IBI ${r.ibiMs} ms ` +
          `· conf ${r.confidence}`;
      }
      if (D.rec.on) {
        D.rec.ibi.push([r.tBeatUs, r.ibiMs, r.confidence, r.flags]);
      }
    }
  }

  /* EVENT + STATUS ride the always-on taps from app.js */
  S.taps.event.add((rec, seq) => {
    D.gaps.event.feed(seq);
    const line = C.fmtEvent(rec, P);
    D.ticker.unshift(line);
    if (D.ticker.length > 8) D.ticker.length = 8;
    $("dashTicker").textContent = D.ticker.join("\n");
    if (rec.type === P.EV_MARKER && rec.known &&
        rec.source === P.MARKER_SRC_BUTTON) {
      /* instant lamp: 1000 = press (active-low level 0), 1001 = release */
      if (rec.markerId === 1000) D.lastBtnMarker = 1;
      else if (rec.markerId === 1001) D.lastBtnMarker = 0;
      setBtnLamp();
    }
    if (D.rec.on) {
      D.rec.events.push([rec.tUs === null ? "" : rec.tUs, rec.type,
                         P.EVENT_TYPE_NAME[rec.type] || `0x${rec.type.toString(16)}`,
                         line.replace(/^[\d.]+s /, "")]);
    }
  });

  function setBtnLamp() {
    const st = S.lastStatus;
    const pressed = D.lastBtnMarker !== null ? D.lastBtnMarker
      : (st ? st.btnPressed : 0);
    const el = $("roBtn");
    el.textContent = pressed ? "PRESSED" : "released";
    el.classList.toggle("on", !!pressed);
    el.classList.toggle("off", !pressed);
  }

  S.taps.status.add((st) => {
    $("roBattV").textContent = (st.battMv / 1000).toFixed(3) + " V";
    $("roBattPct").textContent = st.battPct + "%";
    const chg = $("roChg");
    let cs, cls;
    if (st.flags & P.STF_CHARGING) { cs = "charging"; cls = "warn-on"; }
    else if (st.flags & P.STF_CHARGE_DONE) { cs = "complete"; cls = "on"; }
    else if (st.flags & P.STF_USB) { cs = "USB"; cls = "on"; }
    else { cs = "on battery"; cls = "off"; }
    chg.textContent = cs;
    chg.className = "lamp " + cls;
    D.lastBtnMarker = null;      /* STATUS is ground truth once it arrives */
    setBtnLamp();
    $("roGate").classList.toggle("alert", !!(st.flags & P.STF_GATE));
    $("roGate").textContent = (st.flags & P.STF_GATE) ? "GATED" : "gate ok";
    $("roWear").classList.toggle("on", !!(st.flags & P.STF_WORN));
    $("roWear").textContent = (st.flags & P.STF_WORN) ? "worn" : "off ear";
    $("roVer").textContent =
      `fw ${S.dis.fw || "?"} · hw ${S.dis.hw || "?"} · proto ` +
      (S.protoVer !== null ? `${S.protoVer >> 8}.${S.protoVer & 0xFF}` : "?") +
      ` · ${APP_VER}`;
    $("roLink").textContent =
      `gaps ppg:${D.gaps.ppg.gaps} acc:${D.gaps.accel.gaps} ` +
      `ibi:${D.gaps.ibi.gaps} ev:${D.gaps.event.gaps} · ` +
      `drops ${st.notifDropCount}`;
    $("ledIrAct").textContent = `act ${st.ledIrMa} mA`;
    $("ledRedAct").textContent = `act ${st.ledRedMa} mA`;
    graphs.batt.series[0].ring.push((Date.now() - D.t0Wall) / 1000,
                                    st.battMv / 1000);
    if (D.rec.on) {
      D.rec.status.push([new Date().toISOString(), st.battMv, st.battPct,
                         st.flags, st.ledIrMa, st.ledRedMa, st.tiaGainCode,
                         st.hrBpm, st.ibiLastMs, st.btnPressed,
                         st.notifDropCount, st.gateDutyX100, st.uptimeS]);
    }
  });

  /* log tap -> dashboard log pane */
  const dashLog = $("dashLog");
  S.taps.log.add((line) => {
    dashLog.textContent += line + "\n";
    if (dashLog.textContent.length > 250000) {
      dashLog.textContent = A.getLogLines().slice(-1200).join("\n") + "\n";
    }
    dashLog.scrollTop = dashLog.scrollHeight;
  });
  $("dashLogDl").onclick = () => {
    downloadBlob(new Blob([A.getLogLines().join("\n") + "\n"],
                          { type: "text/plain" }),
                 `narbis_log_${tsName()}.txt`);
  };

  /* ---------------- connection-sensitive UI ---------------- */
  function connected() { return !!(S.server && S.server.connected); }
  /* The guided sequence owns the device while it runs — dashboard
   * mutations mid-step would corrupt it (e.g. streaming makes 0xE2
   * refuse). Graphs/readouts stay live; controls lock. */
  let lastConn = null;
  function syncConnUi() {
    const c = connected() && !A.onGuidedBusy();
    if (c === lastConn) return;
    lastConn = c;
    for (const id of ["btnStream", "btnMarker", "btnSelftest", "btnDeepSleep",
                      "btnSweep", "btnKnobSave", "btnKnobReset",
                      "selTia", "selRate", "selOdr", "selFs"]) {
      $(id).disabled = !c;
    }
    setSlidersEnabled(c && !D.sweepOn);
    if (!c) {
      D.streaming = false;
      for (const u of D.subs) { try { u(); } catch (_) { /* link gone */ } }
      D.subs = [];
      D.knobsLoaded = false;   /* fresh values on next expand/reconnect */
      $("btnStream").textContent = "Stream on";
      D.sweepOn = false;
      $("btnSweep").textContent = "Start";
    }
  }

  function setSlidersEnabled(en) {
    $("ledIr").disabled = !en;
    $("ledRed").disabled = !en;
  }

  /* ---------------- controls ---------------- */

  function err(e) {
    let msg = e.message || String(e);
    /* WRONG_STATE on the manual LED/TIA/rate controls means "the PPG
     * engine is not running" (agcManual contract) — say that instead
     * of leaking the raw status name at the operator. */
    if (/WRONG_STATE/.test(msg)) {
      msg += " — start streaming first (Stream on)";
    }
    A.log(`dashboard: ${msg}`);
    A.banner(`Dashboard: ${msg}`, "error");
  }

  /* freeze AGC once before any manual LED/gain write (spec) */
  async function ensureFrozen() {
    if (D.agcFrozen) return;
    await A.ctrl("agcFreeze", [1]);
    D.agcFrozen = true;
  }

  let ledTimer = null, ledMask = 0;
  function ledChanged(mask) {
    $("ledIrVal").textContent = `${$("ledIr").value} mA` +
      (+$("ledIr").value === 0 ? " (off)" : "");
    $("ledRedVal").textContent = `${$("ledRed").value} mA` +
      (+$("ledRed").value === 0 ? " (off)" : "");
    ledMask |= mask;                 /* only the touched slider(s) apply */
    clearTimeout(ledTimer);
    ledTimer = setTimeout(async () => {
      const m = ledMask;
      ledMask = 0;
      try {
        await ensureFrozen();
        await A.ctrl("agcManual",
          [+$("ledIr").value, +$("ledRed").value, 0, m]);
      } catch (e) { err(e); }
    }, 180);
  }
  $("ledIr").oninput = () => ledChanged(P.AGC_APPLY_IR);
  $("ledRed").oninput = () => ledChanged(P.AGC_APPLY_RED);

  /* selector fills */
  TIA_LABELS.forEach((lbl, code) => {
    const o = document.createElement("option");
    o.value = code; o.textContent = `${code}: ${lbl}Ω`;
    $("selTia").appendChild(o);
  });
  $("selTia").value = 4;
  $("selTia").onchange = async () => {
    try {
      await ensureFrozen();
      await A.ctrl("agcManual", [0, 0, +$("selTia").value, P.AGC_APPLY_GAIN]);
    } catch (e) { err(e); }
  };

  Object.entries(P.RATE_SPS).forEach(([code, sps]) => {
    const o = document.createElement("option");
    o.value = code; o.textContent = `${sps} sps`;
    $("selRate").appendChild(o);
  });
  $("selRate").value = P.RATE_100;
  $("selRate").onchange = async () => {
    try { await A.ctrl("setRate", [+$("selRate").value]); }
    catch (e) { err(e); }
  };

  Object.entries(P.ODR_HZ).forEach(([code, hz]) => {
    const o = document.createElement("option");
    o.value = code; o.textContent = `${hz} Hz`;
    $("selOdr").appendChild(o);
  });
  $("selOdr").value = 2;
  $("selOdr").onchange = async () => {
    try { await A.ctrl("knobSet", [0x0801, +$("selOdr").value]); }
    catch (e) { err(e); }
  };
  FS_LABELS.forEach((lbl, code) => {
    const o = document.createElement("option");
    o.value = code; o.textContent = lbl;
    $("selFs").appendChild(o);
  });
  $("selFs").value = 1;
  $("selFs").onchange = async () => {
    try { await A.ctrl("knobSet", [0x0802, +$("selFs").value]); }
    catch (e) { err(e); }
  };

  /* sweep box (0xEB) */
  $("btnSweep").onclick = async () => {
    try {
      if (!D.sweepOn) {
        const mask = ($("swIr").checked ? 1 : 0) | ($("swRed").checked ? 2 : 0);
        if (!mask) { A.banner("Sweep: pick at least one LED", "error"); return; }
        const phase = Math.max(1, Math.min(60, +$("swPhase").value || 5));
        await A.ctrl("testLedSweepCont", [mask, 1, phase]);
        D.sweepOn = true;
        D.agcFrozen = true;          /* device freezes AGC on sweep start */
        $("btnSweep").textContent = "Stop";
        setSlidersEnabled(false);
      } else {
        await A.ctrl("testLedSweepCont", [0, 0, 0]);
        D.sweepOn = false;
        $("btnSweep").textContent = "Start";
        setSlidersEnabled(true);
      }
    } catch (e) { err(e); }
  };

  /* stream on/off */
  const STREAM_ALL = P.STREAM_MASK_PPG | P.STREAM_MASK_ACCEL |
                     P.STREAM_MASK_IBI | P.STREAM_MASK_EVENT;
  $("btnStream").onclick = async () => {
    try {
      if (!D.streaming) {
        if (!D.subs.length) {
          D.subs.push(await A.subscribe("ppg", (ev) => {
            try { onPpg(NP.parsePpg(ev.target.value)); } catch (_) {}
          }));
          D.subs.push(await A.subscribe("accel", (ev) => {
            try { onAccel(NP.parseAccel(ev.target.value)); } catch (_) {}
          }));
          D.subs.push(await A.subscribe("ibi", (ev) => {
            try { onIbi(NP.parseIbi(ev.target.value)); } catch (_) {}
          }));
        }
        await A.ctrl("streamStart", [STREAM_ALL]);
        D.streaming = true;
        $("btnStream").textContent = "Stream off";
      } else {
        await A.ctrl("streamStop", [STREAM_ALL]);
        D.streaming = false;
        $("btnStream").textContent = "Stream on";
      }
    } catch (e) { err(e); }
  };

  $("btnMarker").onclick = async () => {
    try { await A.ctrl("marker", [++D.hostMarker]); } catch (e) { err(e); }
  };

  /* self-test: run all, wait for the done event (or timeout), fetch blob */
  $("btnSelftest").onclick = async () => {
    const box = $("dashSelftest");
    box.textContent = "self-test running…";
    try {
      const done = new Promise((res) => {
        const tap = (rec) => {
          if (rec.type === P.EV_SELFTEST_DONE) {
            S.taps.event.delete(tap); res();
          }
        };
        S.taps.event.add(tap);
        setTimeout(() => { S.taps.event.delete(tap); res(); }, 4000);
      });
      /* real firmware blocks sys_task ~4.5 s for the full mask — the
       * CONTROL response arrives only after it finishes */
      await A.ctrl("selftestRun", [0], { timeoutMs: 20000 });
      await done;
      const blob = await A.fetchBlob("selftestResult");
      const st = NP.parseSelftestBlob(blob);
      let html = "<table><tr><th>test</th><th>result</th><th>value</th>" +
                 "<th>thr</th></tr>";
      for (const r of st.records) {
        const nm = P.SELFTEST_ID_NAME[r.id] || r.id;
        const cls = r.status === P.TR_PASS ? "pass" :
                    r.status === P.TR_FAIL ? "fail" : "";
        html += `<tr><td>${nm}</td><td class="${cls}">` +
                `${P.TEST_STATUS_NAME[r.status] || r.status}</td>` +
                `<td>${r.value}</td><td>${r.threshold}</td></tr>`;
      }
      box.innerHTML = html + "</table>";
    } catch (e) {
      box.textContent = `self-test failed: ${e.message}`;
      err(e);
    }
  };

  $("btnDeepSleep").onclick = async () => {
    if (!window.confirm(
        "Deep-sleep current test: the device sleeps in 10 s for bench " +
        "current measurement; the BLE link will drop (expected). " +
        "Wake = hold the button 1 s. Proceed?")) return;
    try {
      S.expectDisconnect = true;
      await A.ctrl("testSleepNow", []);
      A.log("deep-sleep test: ack received, device sleeps in 10 s");
    } catch (e) { S.expectDisconnect = false; err(e); }
  };

  /* ---------------- knob browser ---------------- */
  const KNOB_BLOCKS = { 0x00: "power / battery", 0x01: "button", 0x02: "BLE",
    0x03: "PPG engine", 0x04: "AGC", 0x05: "DSP", 0x06: "IBI",
    0x07: "artifact gate", 0x08: "accelerometer", 0x09: "wear detection",
    0x0A: "self-test thresholds" };
  const knobInputs = new Map();   /* id -> input element */

  function buildKnobRows() {
    const host = $("dashKnobs");
    host.innerHTML = "";
    let lastBlock = -1;
    for (const k of P.KNOBS) {
      const block = k.id >> 8;
      if (block !== lastBlock) {
        lastBlock = block;
        const h = document.createElement("div");
        h.className = "knob-block";
        h.textContent = `0x${block.toString(16).padStart(2, "0")}xx — ` +
                        (KNOB_BLOCKS[block] || "misc");
        host.appendChild(h);
      }
      const row = document.createElement("div");
      row.className = "knob-row";
      row.innerHTML =
        `<span class="k-name" title="id 0x${k.id.toString(16)}">${k.name}` +
        (k.unit ? ` <span class="muted">[${k.unit}]</span>` : "") + `</span>` +
        `<span class="k-range">${k.min}…${k.max}</span>`;
      const inp = document.createElement("input");
      inp.type = "number";
      inp.min = k.min; inp.max = k.max;
      inp.placeholder = String(k.def);
      inp.onchange = async () => {
        const v = Math.round(+inp.value);
        if (isNaN(v)) return;
        try {
          await A.ctrl("knobSet", [k.id, v]);
          inp.classList.remove("err");
          inp.classList.add("dirty");
        } catch (e) {
          inp.classList.add("err");
          err(e);
        }
      };
      knobInputs.set(k.id, inp);
      row.appendChild(inp);
      host.appendChild(row);
    }
  }
  buildKnobRows();

  async function refreshKnobs() {
    for (const k of P.KNOBS) {
      if (!connected()) return;
      try {
        const resp = await A.ctrl("knobGet", [k.id]);
        const { value } = NP.parseKnobGetResp(resp.payload);
        const inp = knobInputs.get(k.id);
        inp.value = value;
        inp.classList.remove("dirty", "err");
      } catch (_) { /* keep going; row stays blank */ }
    }
  }
  $("dashKnobsBox").addEventListener("toggle", () => {
    if ($("dashKnobsBox").open && connected() && !D.knobsLoaded) {
      D.knobsLoaded = true;
      refreshKnobs();
    }
  });
  $("btnKnobSave").onclick = async () => {
    try { await A.ctrl("knobSave", []); A.log("knobs saved to NVS"); }
    catch (e) { err(e); }
  };
  $("btnKnobReset").onclick = async () => {
    try {
      await A.ctrl("knobReset", [1]);
      A.log("knobs reset to defaults (RAM+NVS)");
      refreshKnobs();
    } catch (e) { err(e); }
  };

  /* ---------------- record + zip ---------------- */
  function tsName() {
    return new Date().toISOString().replace(/[:.]/g, "-");
  }
  function downloadBlob(blob, name) {
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = name;
    a.click();
    URL.revokeObjectURL(a.href);
  }

  $("btnRecord").onclick = () => {
    if (!D.rec.on) {
      D.rec = { on: true, t0: new Date(),
                ppg: [], accel: [], ibi: [], status: [], events: [] };
      $("btnRecord").textContent = "Stop + save .zip";
      $("btnRecord").classList.add("warn");
      A.log("recording started");
      const upd = () => {
        if (!D.rec.on) return;
        $("recInfo").textContent =
          `rec: ${D.rec.ppg.length} ppg, ${D.rec.accel.length} acc, ` +
          `${D.rec.ibi.length} ibi, ${D.rec.status.length} st, ` +
          `${D.rec.events.length} ev`;
        setTimeout(upd, 500);
      };
      upd();
      return;
    }
    D.rec.on = false;
    $("btnRecord").textContent = "Record";
    $("btnRecord").classList.remove("warn");
    A.log(`recording stopped: ${D.rec.ppg.length} ppg / ` +
          `${D.rec.accel.length} accel / ${D.rec.ibi.length} ibi rows`);
    const serial = ($("recSerial").value || "unit").replace(/\W+/g, "_");
    const files = [
      { name: "ppg.csv",
        data: C.csv(["t_us", "ir", "red", "amb"], D.rec.ppg) },
      { name: "accel.csv",
        data: C.csv(["t_us", "x", "y", "z", "fs_code", "fifo_overrun"],
                    D.rec.accel) },
      { name: "ibi.csv",
        data: C.csv(["t_beat_us", "ibi_ms", "confidence", "flags"], D.rec.ibi) },
      { name: "status.csv",
        data: C.csv(["host_iso", "batt_mv", "batt_pct", "flags", "led_ir_ma",
                     "led_red_ma", "tia_gain_code", "hr_bpm", "ibi_last_ms",
                     "btn_pressed", "notif_drops", "gate_duty_x100",
                     "uptime_s"], D.rec.status) },
      { name: "events.csv",
        data: C.csv(["t_us", "type", "name", "detail"], D.rec.events) },
      { name: "log.txt", data: A.getLogLines().join("\n") + "\n" },
    ];
    const zip = C.zipStore(files, NP.ota.crc32);
    downloadBlob(new Blob([zip], { type: "application/zip" }),
                 `narbis_functest_${serial}_${tsName()}.zip`);
    $("recInfo").textContent = "saved";
  };

  /* ---------------- ⓘ info buttons + popover ----------------------- */
  function installInfo() {
    const pop = document.createElement("div");
    pop.id = "infoPop";
    pop.className = "hidden";
    document.body.appendChild(pop);
    let openFor = null;

    function close() {
      pop.classList.add("hidden");
      openFor = null;
    }
    function openAt(btn, text) {
      pop.textContent = text;
      pop.classList.remove("hidden");
      const r = btn.getBoundingClientRect();
      const w = Math.min(340, window.innerWidth - 24);
      pop.style.width = w + "px";
      let x = r.left + window.scrollX;
      if (x + w > window.scrollX + window.innerWidth - 12) {
        x = window.scrollX + window.innerWidth - w - 12;
      }
      pop.style.left = x + "px";
      pop.style.top = (r.bottom + window.scrollY + 6) + "px";
      openFor = btn;
    }

    for (const [key, def] of Object.entries(C.INFO)) {
      const anchor = document.querySelector(def.sel);
      if (!anchor) continue;   /* markup drift: skip, never break the app */
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = "info-dot";
      btn.textContent = "ⓘ";
      btn.setAttribute("aria-label", "what is this?");
      btn.dataset.infoKey = key;
      btn.onclick = (e) => {
        e.preventDefault();
        e.stopPropagation();   /* keep <summary>/<label> from toggling */
        if (openFor === btn) close();
        else openAt(btn, def.text);
      };
      if (def.before) anchor.parentNode.insertBefore(btn, anchor);
      else if (def.after) anchor.parentNode.insertBefore(btn, anchor.nextSibling);
      else anchor.appendChild(btn);
    }
    document.addEventListener("click", (e) => {
      if (!pop.classList.contains("hidden") &&
          e.target !== pop && !pop.contains(e.target)) close();
    });
    document.addEventListener("keydown", (e) => {
      if (e.key === "Escape") close();
    });
  }
  installInfo();

  syncConnUi();
})();
