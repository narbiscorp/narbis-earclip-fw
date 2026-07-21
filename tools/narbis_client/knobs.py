"""knobs.py — knob registry CLI (list / get / set / save / reset / diff).

The table comes entirely from OP_KNOB_DISCOVER — no local knob list, so
the tool stays correct when firmware adds knobs (§4 of the handoff:
discovery is the source of truth, the app never hard-codes IDs).
"""

from __future__ import annotations

import asyncio
import sys
from typing import Dict, Optional

from . import proto as P
from .client import KnobInfo, NarbisClient, NarbisCtrlError

_TYPE_NAMES = {P.KNOB_BOOL: "bool", P.KNOB_U8: "u8", P.KNOB_U16: "u16",
               P.KNOB_I32: "i32"}


def _flags_str(flags: int) -> str:
    out = []
    if flags & P.KF_PERSIST:
        out.append("persist")
    if flags & P.KF_LIVE:
        out.append("live")
    if flags & P.KF_RESTREAM:
        out.append("restream")
    if flags & P.KF_REBOOT:
        out.append("reboot")
    return ",".join(out) or "-"


def format_table(knobs: Dict[str, KnobInfo],
                 only_nondefault: bool = False) -> str:
    """Fixed-width table; only_nondefault=True is the `diff` view."""
    rows = []
    for name in sorted(knobs, key=lambda n: knobs[n].id):
        k = knobs[name]
        if only_nondefault and k.current == k.default:
            continue
        rows.append((f"{k.id:#06x}", name, _TYPE_NAMES.get(k.type, str(k.type)),
                     str(k.current), str(k.default), f"{k.min}..{k.max}",
                     k.unit or "-", _flags_str(k.flags)))
    if not rows:
        return "(all knobs at defaults)" if only_nondefault else "(no knobs)"
    hdr = ("id", "name", "type", "current", "default", "range", "unit", "flags")
    widths = [max(len(hdr[i]), *(len(r[i]) for r in rows))
              for i in range(len(hdr))]
    lines = ["  ".join(h.ljust(w) for h, w in zip(hdr, widths)),
             "  ".join("-" * w for w in widths)]
    lines += ["  ".join(c.ljust(w) for c, w in zip(r, widths)) for r in rows]
    return "\n".join(lines)


def parse_value(knob: KnobInfo, text: str) -> int:
    """bool knobs accept on/off/true/false/0/1; numeric knobs int or 0x."""
    if knob.type == P.KNOB_BOOL:
        t = text.strip().lower()
        if t in ("1", "true", "on", "yes"):
            return 1
        if t in ("0", "false", "off", "no"):
            return 0
        raise ValueError(f"bad bool value {text!r}")
    return int(text, 0)


async def cmd_list(client: NarbisClient, _args) -> int:
    print(format_table(await client.discover_knobs()))
    return 0


async def cmd_diff(client: NarbisClient, _args) -> int:
    print(format_table(await client.discover_knobs(), only_nondefault=True))
    return 0


async def cmd_get(client: NarbisClient, args) -> int:
    knobs = await client.discover_knobs()
    if args.knob not in knobs:
        print(f"error: unknown knob {args.knob!r}", file=sys.stderr)
        return 1
    k = knobs[args.knob]
    value = await client.knob_get(args.knob)
    unit = f" {k.unit}" if k.unit else ""
    print(f"{args.knob} = {value}{unit}")
    return 0


async def cmd_set(client: NarbisClient, args) -> int:
    knobs = await client.discover_knobs()
    if args.knob not in knobs:
        print(f"error: unknown knob {args.knob!r}", file=sys.stderr)
        return 1
    k = knobs[args.knob]
    try:
        value = parse_value(k, args.value)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    if not k.min <= value <= k.max:
        print(f"error: {value} outside [{k.min}, {k.max}]", file=sys.stderr)
        return 1
    try:
        status = await client.knob_set(args.knob, value)
    except NarbisCtrlError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    note = " (takes effect after restart)" if status == P.ST_NEEDS_RESTART else ""
    print(f"{args.knob} = {value}{note}")
    if args.save:
        await client.knob_save()
        print("saved to NVS")
    return 0


async def cmd_save(client: NarbisClient, _args) -> int:
    await client.knob_save()
    print("knobs saved to NVS")
    return 0


async def cmd_reset(client: NarbisClient, args) -> int:
    await client.knob_reset(scope_nvs=args.nvs)
    print("knobs reset to defaults" + (" (RAM + NVS)" if args.nvs else " (RAM)"))
    return 0


def main(argv=None) -> int:
    """CLI entry (console_script narbis-knobs, tools/scripts/narbis_knobs.py)."""
    import argparse
    from .client import add_device_args, client_from_args, connect_or_exit

    ap = argparse.ArgumentParser(
        prog="narbis-knobs",
        description="Inspect/edit the earclip knob registry (via BLE "
                    "discovery — no local knob table).")
    add_device_args(ap)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="full knob table")
    sub.add_parser("diff", help="knobs whose current value != default")
    p = sub.add_parser("get", help="read one knob")
    p.add_argument("knob")
    p = sub.add_parser("set", help="write one knob (RAM; --save to persist)")
    p.add_argument("knob")
    p.add_argument("value")
    p.add_argument("--save", action="store_true",
                   help="KNOB_SAVE to NVS after the set")
    sub.add_parser("save", help="persist current knob values to NVS")
    p = sub.add_parser("reset", help="reset knobs to defaults")
    p.add_argument("--nvs", action="store_true",
                   help="also clear the NVS copies (default: RAM only)")

    args = ap.parse_args(argv)
    handlers = {"list": cmd_list, "diff": cmd_diff, "get": cmd_get,
                "set": cmd_set, "save": cmd_save, "reset": cmd_reset}

    async def run() -> int:
        client = client_from_args(args)
        await connect_or_exit(client)
        try:
            return await handlers[args.cmd](client, args)
        finally:
            await client.disconnect()

    try:
        return asyncio.run(run())
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
