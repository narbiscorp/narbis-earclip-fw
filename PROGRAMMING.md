# Programming the Narbis Edge Earclip — first flash, bootloader, OTA

Applies to V2.1 boards (XIAO ESP32-C6). The product's USB-C **is** the
XIAO's own connector: it is both the charge port and the programming port.
No external programmer is ever needed.

## 1. What lives in flash (the bootloader story)

The build already produces and flashes a complete, self-recovering boot
chain — there is nothing separate to "add":

| Offset | Image | Role |
|---|---|---|
| (ROM) | ESP32-C6 ROM loader | burned into the chip; provides USB download mode — the board is **unbrickable** by software |
| 0x0 | `bootloader.bin` | 2nd-stage bootloader, built with **rollback support** (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`) |
| 0x8000 | `partition-table.bin` | the map below |
| 0xf000 | `ota_data_initial.bin` | which app slot boots (the bootloader's A/B switch) |
| 0x20000 | `narbis_earclip.bin` → **ota_0** | app slot A (1.6 MB) |
| 0x1B0000 | — → **ota_1** | app slot B (1.6 MB, filled by OTA) |
| 0x340000 | coredump | crash dumps (`coredump info` on the console) |
| + | nvs | knobs, BLE bonds, calibration |

Boot flow: ROM → 2nd-stage bootloader → reads otadata → runs the selected
slot. A freshly OTA-updated slot boots as *pending-verify*; if it fails its
post-boot self-check (or crash-loops before completing it), the bootloader
**automatically falls back** to the previous slot on the next reset.

## 2. First-time programming (bench / bring-up)

Prereqs (already on the lab PC): ESP-IDF v5.5.1 at `C:\Espressif`,
this repo, a USB-C data cable.

```powershell
. C:\Espressif\frameworks\esp-idf-v5.5.1\export.ps1
cd <repo>\firmware
idf.py set-target esp32c6        # first time in a fresh checkout only
idf.py build
idf.py -p COMx flash monitor     # COMx = "USB Serial Device" in Device Manager
```

`flash` writes all four images from the table above; `monitor` then shows
the boot banner (`Ctrl+]` exits). Expected on a healthy board:

```
I main: Narbis Edge Earclip fw v0.2.0 | hw V2.1 | reset: poweron | boot: cold | test_mode=0
```

then advertising starts (`Narbis Edge Earclip`, or `… TEST` for a
verification build — see `NARBIS_TEST_MODE` in `firmware/main/board.h`).
Smoke-check on the same console: `ver`, `stats`, `selftest`.

**If the COM port never appears** (blank chip won't do this, but a deep-
sleeping device or a broken app can make the USB port vanish): hold the
XIAO **BOOT** button while plugging in USB → the ROM loader enumerates as
a download-mode COM port → flash normally, then press **RESET** (or
replug). On assembled units the buttons are sealed inside the enclosure —
this is bench-only; a sealed unit that runs our firmware always recovers
via a button wake (GPIO2) or the 8 s forced-reboot hold, and its app can
be replaced over BLE OTA (§3).

**Manufacturing note — single-image flash:** produce one merged file so
the factory tool does a single write at offset 0x0:

```powershell
idf.py merge-bin                 # -> build\merged-binary.bin
python -m esptool --chip esp32c6 -p COMx write_flash 0x0 build\merged-binary.bin
```

For PCB verification builds, set `NARBIS_TEST_MODE 1` in
`firmware/main/board.h` first, then run the guided sequence with the web
test app (`tools/testapp/`, Chrome/Edge). Reflash with `0` for production.

## 3. OTA updates over BLE (already built in)

Any later firmware ships to sealed units over the air — USB is only the
dev/factory path.

```powershell
pip install -e tools            # once, on the host machine
narbis-ota firmware\build\narbis_earclip.bin
```

The pusher connects, enters OTA mode, streams the image (~240-byte
write-no-response chunks with periodic flow-control checkpoints), then the
device: validates the ESP-IDF image header → verifies a whole-image CRC32
read back from flash → switches the boot slot → reboots → runs a 10 s
post-boot self-check (NVS, I²C sensor probes, BLE up, no watchdog) →
only then marks itself valid. Any failure at any step = automatic
rollback to the old firmware. The tool confirms success by re-reading the
Device Information firmware-revision string after reconnect.

**From the browser (no python needed):** the web test app
(`tools/testapp/index.html`, Chrome/Edge over https or localhost) has a
**Firmware update** panel — connect, pick `narbis_earclip.bin`, press
Flash. Same protocol, same safety rails (CRC verify, auto-rollback,
resume, reconnect + version confirmation). Works against production
units too, not just TEST builds — pairing is triggered automatically if
the unit requires it. `?mock=1` demos the whole flow without hardware.

Useful variants:

- `narbis-ota <bin> --loop 20` — the 20× acceptance soak (handoff DoD).
- A transfer interrupted by disconnect **resumes** where it stopped if the
  host reconnects within 60 s with the same image (the tool does this
  automatically); otherwise it restarts cleanly from zero.
- Protocol details (for the future phone app): OTA service UUIDs, opcodes
  and framing are all in `firmware/components/narbis_core/include/narbis/proto.h`
  (`NC_OTA_*`), reference client in `tools/narbis_client/ota.py`.

Version string: the build stamps `git describe` into the image — tag
releases (`git tag v0.3.0`) so devices report meaningful versions over
BLE and on the console.

## 4. If something goes wrong

| Symptom | Fix |
|---|---|
| New OTA image misbehaves | Nothing to do — rollback is automatic on the next reset if the self-check failed; otherwise push the previous .bin with `narbis-ota` |
| Device unresponsive, sealed unit | Hold the side button 8 s (forced reboot); firmware watchdogs also reboot on their own |
| USB port gone on the bench | BOOT-button download mode (§2), reflash |
| Wrong/experimental knobs persisted | Console `factory-reset CONFIRM`, or CONTROL factory-reset op (erases knobs + bonds, keeps firmware) |
| Diagnose a crash | `coredump info` on the console; reset reason is in every boot banner and in `stats` |
