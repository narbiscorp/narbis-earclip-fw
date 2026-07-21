"""ota.py — BLE OTA pusher.

Transfer protocol (proto.h OTA service):
  1. OP_ENTER_OTA on the sensor CONTROL char (device pauses acquisition);
  2. OTA_BEGIN on OTA_CTRL: {size, zlib.crc32(image), version string} ->
     resume_offset (nonzero when the device kept a partial image with a
     matching CRC — we seek forward, nothing is resent);
  3. OTA_DATA write-no-response chunks [u32 offset][<=240 B]. Flow
     control: every `status_every` chunks poll OTA_STATUS and reconcile
     bytes_rx; on OTAERR_EXPECTED_OFFSET the device tells us where it
     really is (dropped WNR) and we reseek — the offset header makes the
     stream self-describing, so nothing can be applied out of place;
  4. OTA_FINISH: device validates CRC + image, sets boot partition,
     restarts. Expect a disconnect instead of (or right after) the
     response;
  5. reconnect, read DIS firmware revision, compare against the pre-OTA
     string. Same-build soak loops pass expect_same=True.
"""

from __future__ import annotations

import asyncio
import logging
import time
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from . import proto as P
from . import uuids as U
from .client import NarbisClient, NarbisCtrlError, NarbisError

log = logging.getLogger("narbis.ota")

OTA_STATE_NAMES = {P.OTA_IDLE: "IDLE", P.OTA_RECEIVING: "RECEIVING",
                   P.OTA_VALIDATING: "VALIDATING", P.OTA_READY: "READY",
                   P.OTA_FAILED: "FAILED"}
OTA_ERR_NAMES = {P.OTAERR_NONE: "NONE",
                 P.OTAERR_EXPECTED_OFFSET: "EXPECTED_OFFSET",
                 P.OTAERR_SIZE: "SIZE", P.OTAERR_CRC: "CRC",
                 P.OTAERR_IMAGE: "IMAGE", P.OTAERR_FLASH: "FLASH",
                 P.OTAERR_STATE: "STATE"}


class OtaError(NarbisError):
    pass


@dataclass
class OtaResult:
    ok: bool
    seconds: float
    bytes_sent: int
    reseeks: int
    fw_before: Optional[str]
    fw_after: Optional[str]


def version_for(path: Path, explicit: Optional[str]) -> str:
    """--version wins; otherwise derive a deterministic stamp from the
    image file's mtime (good enough to distinguish rebuilds)."""
    if explicit:
        return explicit
    mtime = path.stat().st_mtime
    return time.strftime("mtime-%Y%m%d-%H%M%S", time.localtime(mtime))


async def _ota_ctrl(client: NarbisClient, builder, op: int,
                    timeout: float = 10.0) -> P.ControlResponse:
    return await client.control_call(builder, op, timeout=timeout,
                                     char_uuid=U.UUID_OTA_CTRL)


async def _get_status(client: NarbisClient) -> P.OtaStatus:
    resp = await _ota_ctrl(client, P.build_ota_status, P.OTA_STATUS)
    return P.parse_ota_status_resp(resp.payload)


async def push(client: NarbisClient, image_path, version: Optional[str] = None,
               chunk_size: int = P.OTA_CHUNK_MAX, status_every: int = 128,
               chunk_delay_ms: float = 0.0, enter_ota: bool = True,
               reboot_wait_s: float = 8.0, reconnect_attempts: int = 10,
               expect_same_fw: bool = False) -> OtaResult:
    """Push one image. Returns OtaResult; raises OtaError on failure.
    `client` ends up connected to the rebooted device on success."""
    image_path = Path(image_path)
    data = image_path.read_bytes()
    if not data:
        raise OtaError(f"{image_path}: empty image")
    if not 1 <= chunk_size <= P.OTA_CHUNK_MAX:
        raise OtaError(f"chunk size {chunk_size} not in 1..{P.OTA_CHUNK_MAX}")
    crc = zlib.crc32(data) & 0xFFFFFFFF
    ver = version_for(image_path, version)
    t_start = time.monotonic()

    fw_before = (await client.read_device_info()).get("fw_revision")
    log.info("OTA: %s (%d B, crc32 %08x, version %r), device fw %r",
             image_path.name, len(data), crc, ver, fw_before)

    if enter_ota:
        try:
            await client.enter_ota()
        except NarbisCtrlError as e:
            # WRONG_STATE == already in OTA mode from a previous aborted run
            if e.status != P.ST_WRONG_STATE:
                raise
            log.info("ENTER_OTA: already in OTA state")

    resp = await _ota_ctrl(
        client, lambda tid: P.build_ota_begin(tid, len(data), crc, ver),
        P.OTA_BEGIN, timeout=15.0)
    offset = P.parse_ota_begin_resp(resp.payload)
    if offset:
        log.info("device resumes at offset %d/%d", offset, len(data))

    bytes_sent = 0
    reseeks = 0
    chunks_since_status = 0
    while offset < len(data):
        chunk = data[offset:offset + chunk_size]
        await client.write_char(U.UUID_OTA_DATA,
                                P.build_ota_data(offset, chunk),
                                response=False)
        offset += len(chunk)
        bytes_sent += len(chunk)
        chunks_since_status += 1
        if chunk_delay_ms:
            await asyncio.sleep(chunk_delay_ms / 1000.0)
        if chunks_since_status >= status_every or offset >= len(data):
            st = await _get_status(client)
            chunks_since_status = 0
            if st.last_err == P.OTAERR_EXPECTED_OFFSET or st.bytes_rx != offset:
                if st.state == P.OTA_FAILED:
                    raise OtaError(f"device OTA failed: "
                                   f"{OTA_ERR_NAMES.get(st.last_err, st.last_err)}")
                log.warning("flow: device at %d, host at %d — reseek",
                            st.bytes_rx, offset)
                offset = st.bytes_rx
                reseeks += 1
            elif st.state == P.OTA_FAILED:
                raise OtaError(f"device OTA failed: "
                               f"{OTA_ERR_NAMES.get(st.last_err, st.last_err)}")
            pct = 100.0 * offset / len(data)
            log.info("OTA %6.2f%%  (%d/%d B, %d reseek(s))",
                     pct, offset, len(data), reseeks)

    st = await _get_status(client)
    if st.bytes_rx != len(data):
        raise OtaError(f"device has {st.bytes_rx}/{len(data)} B after send")

    log.info("FINISH — device validates and reboots")
    try:
        await _ota_ctrl(client, P.build_ota_finish, P.OTA_FINISH, timeout=20.0)
    except NarbisError as e:
        # A reboot-before-indication presents as timeout/disconnect; the
        # reconnect + fw check below is the real verdict.
        log.info("FINISH response not seen (%s) — assuming reboot", e)

    log.info("waiting up to %.0f s for the reboot disconnect", reboot_wait_s)
    await client.wait_for_disconnect(timeout=reboot_wait_s)
    await asyncio.sleep(2.0)  # let the new image boot + start advertising
    await client.reconnect(attempts=reconnect_attempts, delay=2.0)

    fw_after = (await client.read_device_info()).get("fw_revision")
    seconds = time.monotonic() - t_start
    log.info("reconnected after OTA in %.1f s: fw %r -> %r",
             seconds, fw_before, fw_after)

    if fw_before is not None and fw_after == fw_before and not expect_same_fw:
        raise OtaError(f"DIS firmware revision unchanged ({fw_after!r}) — "
                       f"device likely rolled back or FINISH failed "
                       f"(pass expect_same_fw for same-build soak loops)")
    if fw_after is None:
        raise OtaError("could not read DIS firmware revision after OTA")

    return OtaResult(True, seconds, bytes_sent, reseeks, fw_before, fw_after)


async def abort(client: NarbisClient) -> None:
    await _ota_ctrl(client, P.build_ota_abort, P.OTA_ABORT)


def main(argv=None) -> int:
    """CLI entry (console_script narbis-ota, tools/scripts/narbis_ota.py)."""
    import argparse
    import sys
    from .client import add_device_args, client_from_args, connect_or_exit

    ap = argparse.ArgumentParser(
        prog="narbis-ota",
        description="Push a firmware image over BLE OTA.")
    add_device_args(ap)
    ap.add_argument("image", help="app image (.bin) to push")
    ap.add_argument("--version", default=None,
                    help="version string for OTA_BEGIN (default: from file "
                         "mtime)")
    ap.add_argument("--chunk-size", type=int, default=P.OTA_CHUNK_MAX)
    ap.add_argument("--status-every", type=int, default=128,
                    help="chunks between OTA_STATUS flow-control polls")
    ap.add_argument("--chunk-delay-ms", type=float, default=0.0,
                    help="pacing delay between WNR chunks")
    ap.add_argument("--reboot-wait", type=float, default=8.0)
    ap.add_argument("--loop", type=int, default=1, metavar="N",
                    help="repeat the push N times (soak; implies "
                         "--expect-same-fw after the first round)")
    ap.add_argument("--expect-same-fw", action="store_true",
                    help="do not fail when the DIS fw string is unchanged")
    args = ap.parse_args(argv)

    if not Path(args.image).is_file():
        print(f"error: {args.image}: no such file", file=sys.stderr)
        return 1

    async def run() -> int:
        client = client_from_args(args)
        await connect_or_exit(client)
        failures = 0
        try:
            for i in range(args.loop):
                expect_same = args.expect_same_fw or i > 0
                try:
                    r = await push(client, args.image, version=args.version,
                                   chunk_size=args.chunk_size,
                                   status_every=args.status_every,
                                   chunk_delay_ms=args.chunk_delay_ms,
                                   reboot_wait_s=args.reboot_wait,
                                   expect_same_fw=expect_same)
                    print(f"[{i + 1}/{args.loop}] OK in {r.seconds:.1f} s "
                          f"({r.bytes_sent} B, {r.reseeks} reseek(s), "
                          f"fw {r.fw_before!r} -> {r.fw_after!r})")
                except (OtaError, NarbisError) as e:
                    failures += 1
                    print(f"[{i + 1}/{args.loop}] FAIL: {e}", file=sys.stderr)
                    if not client.is_connected:
                        try:
                            await client.reconnect()
                        except NarbisError:
                            print("reconnect failed — aborting loop",
                                  file=sys.stderr)
                            break
        finally:
            await client.disconnect()
        if args.loop > 1:
            print(f"OTA soak: {args.loop - failures}/{args.loop} passed")
        return 0 if failures == 0 else 2

    try:
        return asyncio.run(run())
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
