"""liveplot.py — live matplotlib view of a streaming earclip.

Panels (FuncAnimation, ~10 fps):
  1. IR + red PPG, last 10 s, autoscaled; gate spans shaded; AGC steps
     as vertical markers (amplitude discontinuities are explainable);
  2. accel magnitude |a| (raw counts);
  3. IBI tachogram (ms vs beat time) colored by confidence.

Threading model: matplotlib must own the main thread on every backend
that matters, and bleak needs a running asyncio loop — so the BLE loop
runs in a daemon thread feeding lock-guarded ring buffers, and the
animation samples them. matplotlib is imported inside run_liveplot() so
`import narbis_client.liveplot` (and --help) works without it.
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from collections import deque
from typing import List, Optional, Tuple

from . import proto as P

log = logging.getLogger("narbis.liveplot")

WINDOW_S = 10.0        # PPG window
IBI_WINDOW_S = 120.0   # tachogram window


class LiveBuffers:
    """Lock-guarded rolling buffers fed from the BLE thread, drained by
    the animation timer. Timestamps are device t_us."""

    def __init__(self):
        self.lock = threading.Lock()
        self.ppg: deque = deque()            # (t_us, ir, red)
        self.acc: deque = deque()            # (t_us, |a| counts)
        self.ibi: deque = deque()            # (t_beat_us, ibi_ms, confidence)
        self.gate_spans: List[List[Optional[int]]] = []  # [t_on_us, t_off_us|None]
        self.agc_steps: deque = deque(maxlen=256)        # t_us
        self.markers: deque = deque(maxlen=256)          # t_us
        self.last_status: Optional[P.Status] = None
        self.batches = 0

    # --- feeders (BLE thread) ---------------------------------------- #

    def feed_ppg(self, b: P.PpgBatch) -> None:
        sps = P.rate_sps(b.rate_code) or 100
        dt = 1_000_000 / sps
        with self.lock:
            for i in range(b.n):
                self.ppg.append((b.t0_us + round(i * dt), b.ir[i], b.red[i]))
            self._trim(self.ppg, WINDOW_S * 1.2)
            self.batches += 1

    def feed_accel(self, b: P.AccelBatch) -> None:
        hz = P.acc_odr_hz(b.odr_code) or 50
        dt = 1_000_000 / hz
        with self.lock:
            for i, (x, y, z) in enumerate(b.samples):
                mag = (x * x + y * y + z * z) ** 0.5
                self.acc.append((b.t0_us + round(i * dt), mag))
            self._trim(self.acc, WINDOW_S * 1.2)

    def feed_ibi(self, b: P.IbiBatch) -> None:
        with self.lock:
            for r in b.records:
                self.ibi.append((r.t_beat_us, r.ibi_ms, r.confidence))
            self._trim(self.ibi, IBI_WINDOW_S * 1.2)

    def feed_event(self, b: P.EventBatch) -> None:
        with self.lock:
            for ev in b.events:
                if isinstance(ev, P.EvGate):
                    if ev.state:
                        # nested gate-on: extend, never stack
                        if not self.gate_spans or self.gate_spans[-1][1] is not None:
                            self.gate_spans.append([ev.t_us, None])
                    elif self.gate_spans and self.gate_spans[-1][1] is None:
                        self.gate_spans[-1][1] = ev.t_us
                elif isinstance(ev, P.EvAgcStep):
                    self.agc_steps.append(ev.t_us)
                elif isinstance(ev, P.EvMarker):
                    self.markers.append(ev.t_us)
            # drop spans that ended before the plotting window can show them
            if len(self.gate_spans) > 64:
                self.gate_spans = self.gate_spans[-64:]

    def feed_status(self, s: P.Status) -> None:
        with self.lock:
            self.last_status = s

    @staticmethod
    def _trim(dq: deque, horizon_s: float) -> None:
        if not dq:
            return
        cutoff = dq[-1][0] - int(horizon_s * 1e6)
        while dq and dq[0][0] < cutoff:
            dq.popleft()


def run_liveplot(client) -> None:
    """Blocks in the matplotlib main loop until the window closes.
    `client` must NOT be connected yet — connection happens on the BLE
    thread's own event loop (bleak objects are loop-affine)."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    buffers = LiveBuffers()
    stop = threading.Event()

    def thread_main() -> None:
        try:
            asyncio.run(_ble_main())
        except Exception:
            log.exception("BLE thread failed")
            stop.set()

    async def _ble_main() -> None:
        await client.connect()
        await client.subscribe("ppg", buffers.feed_ppg)
        await client.subscribe("accel", buffers.feed_accel)
        await client.subscribe("ibi", buffers.feed_ibi)
        await client.subscribe("event", buffers.feed_event)
        await client.subscribe("status", buffers.feed_status)
        await client.start_streams()
        while not stop.is_set() and not client.disconnected.is_set():
            await asyncio.sleep(0.2)
        if client.is_connected:
            try:
                await client.stop_streams()
            except Exception:
                pass
            await client.disconnect()

    t = threading.Thread(target=thread_main, name="narbis-ble", daemon=True)
    t.start()

    fig, (ax_ppg, ax_acc, ax_ibi) = plt.subplots(
        3, 1, figsize=(11, 8), sharex=False,
        gridspec_kw={"height_ratios": [2, 1, 1]})
    fig.canvas.manager.set_window_title("Narbis Edge Earclip — live")

    ln_ir, = ax_ppg.plot([], [], lw=0.8, label="IR", color="tab:purple")
    ln_red, = ax_ppg.plot([], [], lw=0.8, label="red", color="tab:red")
    ax_ppg.set_ylabel("PPG (counts)")
    ax_ppg.legend(loc="upper left", fontsize=8)
    ln_acc, = ax_acc.plot([], [], lw=0.8, color="tab:gray")
    ax_acc.set_ylabel("|a| (counts)")
    ln_ibi, = ax_ibi.plot([], [], "o-", ms=3, lw=0.6, color="tab:blue")
    ax_ibi.set_ylabel("IBI (ms)")
    ax_ibi.set_xlabel("device time (s)")
    shaded: list = []

    def update(_frame):
        with buffers.lock:
            ppg = list(buffers.ppg)
            acc = list(buffers.acc)
            ibi = list(buffers.ibi)
            spans = [list(s) for s in buffers.gate_spans]
            steps = list(buffers.agc_steps)
            status = buffers.last_status
        for patch in shaded:
            patch.remove()
        shaded.clear()

        if ppg:
            t_now = ppg[-1][0]
            t0 = t_now - WINDOW_S * 1e6
            ts = [(p[0] - t_now) / 1e6 for p in ppg if p[0] >= t0]
            irs = [p[1] for p in ppg if p[0] >= t0]
            reds = [p[2] for p in ppg if p[0] >= t0]
            ln_ir.set_data(ts, irs)
            ln_red.set_data(ts, reds)
            ax_ppg.set_xlim(-WINDOW_S, 0)
            lo, hi = min(min(irs), min(reds)), max(max(irs), max(reds))
            pad = max((hi - lo) * 0.1, 1)
            ax_ppg.set_ylim(lo - pad, hi + pad)

            # gate spans + AGC step markers, in window coordinates
            for t_on, t_off in spans:
                a = max((t_on - t_now) / 1e6, -WINDOW_S)
                b = 0 if t_off is None else (t_off - t_now) / 1e6
                if b >= -WINDOW_S:
                    shaded.append(ax_ppg.axvspan(a, min(b, 0), color="orange",
                                                 alpha=0.2, lw=0))
            for t_step in steps:
                x = (t_step - t_now) / 1e6
                if -WINDOW_S <= x <= 0:
                    shaded.append(ax_ppg.axvline(x, color="green", ls="--",
                                                 lw=0.8, alpha=0.7))

            title = "streaming"
            if status is not None:
                title = (f"HR {status.hr_bpm} bpm | batt {status.batt_pct}% "
                         f"| IR {status.led_ir_ma} mA red {status.led_red_ma} mA "
                         f"| RF {status.tia_gain_code} "
                         f"| gate {status.gate_duty_x100 / 100:.1f}% "
                         f"| drops {status.notif_drop_count}")
            ax_ppg.set_title(title, fontsize=9)

        if acc:
            t_now = acc[-1][0]
            ts = [(a[0] - t_now) / 1e6 for a in acc]
            ln_acc.set_data(ts, [a[1] for a in acc])
            ax_acc.set_xlim(-WINDOW_S, 0)
            ax_acc.relim()
            ax_acc.autoscale_view(scalex=False)

        if ibi:
            t_now = ibi[-1][0]
            ts = [(r[0] - t_now) / 1e6 for r in ibi]
            ln_ibi.set_data(ts, [r[1] for r in ibi])
            ax_ibi.set_xlim(-IBI_WINDOW_S, 0)
            ax_ibi.relim()
            ax_ibi.autoscale_view(scalex=False)

        if stop.is_set():
            plt.close(fig)
        return [ln_ir, ln_red, ln_acc, ln_ibi] + shaded

    ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop.set()
        t.join(timeout=5)
    del ani


def main(argv=None) -> int:
    """CLI entry (console_script narbis-plot, tools/scripts/narbis_plot.py)."""
    import argparse
    import sys
    from .client import DeviceNotFoundError, add_device_args, client_from_args

    ap = argparse.ArgumentParser(
        prog="narbis-plot",
        description="Live PPG/accel/IBI plot (requires matplotlib: "
                    "pip install narbis-tools[viz]).")
    add_device_args(ap)
    ap.add_argument("--rate", type=int, default=None,
                    help="set PPG rate first (code 0..4 or sps)")
    args = ap.parse_args(argv)

    try:
        import matplotlib  # noqa: F401
    except ImportError:
        print("error: matplotlib is required for live plotting "
              "(pip install narbis-tools[viz])", file=sys.stderr)
        return 1

    client = client_from_args(args)

    # Probe for the device on this thread first so "no device" is a clean
    # one-line exit instead of an empty window + thread traceback.
    async def probe() -> bool:
        from .client import find_device
        return await find_device(args.address, args.name,
                                 args.scan_timeout) is not None

    try:
        if not asyncio.run(probe()):
            print("error: no device found", file=sys.stderr)
            return 1
    except Exception as e:
        print(f"error: BLE scan failed: {e}", file=sys.stderr)
        return 1

    if args.rate is not None:
        # Rate must be set on the BLE thread's loop — wrap connect().
        rate = args.rate
        orig_connect = client.connect

        async def connect_then_rate(*a, **kw):
            await orig_connect(*a, **kw)
            await client.set_rate(rate)
        client.connect = connect_then_rate  # type: ignore[method-assign]

    try:
        run_liveplot(client)
    except DeviceNotFoundError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
