# Narbis BLE OTA (DFU) — Wire Protocol

This is the complete contract between a BLE **central** (the iOS app) and a Narbis
device (Edge glasses or earclip). It is identical for both devices — only the
advertised name, GitHub repo, expected chip-id, and `.bin` filename differ
(see [Device table](#device-table)).

Authoritative reference implementation: `index.html`
(the production Web Bluetooth uploader). Everything below is extracted from it.

---

## 1. GATT services & characteristics

| Role | UUID (16-bit) | Properties | Purpose |
|------|---------------|------------|---------|
| **OTA service** | `0x00FF` | — | Primary service holding the three OTA characteristics |
| **CONTROL** (`0xFF01`) | `0xFF01` | Write, Write-No-Response, Read | Central → device commands (opcodes) |
| **DATA** (`0xFF02`) | `0xFF02` | Write, Write-No-Response | Central → device raw firmware bytes |
| **STATUS** (`0xFF03`) | `0xFF03` | Notify, Read | Device → central status frames. **Subscribe before doing anything.** |

Standard **Device Information Service** is also used to read the running firmware version:

| Role | UUID | Purpose |
|------|------|---------|
| DIS service | `0x180A` | — |
| Firmware Revision String | `0x2A26` | Read → UTF-8 string (current device firmware). Optional/best-effort. |

> CoreBluetooth: `CBUUID(string: "00FF")`, `CBUUID(string: "FF01")`, etc. 16-bit
> UUIDs expand to the Bluetooth base UUID automatically.

---

## 2. Opcodes (central → device, written to CONTROL `0xFF01`)

| Name | Bytes | Write type | Meaning |
|------|-------|-----------|---------|
| **START / BEGIN** | `[0xA8, size_LE32]` (5 bytes) | **Without response** | Enter OTA mode. The 4 little-endian bytes after the opcode are the **total image size**; size-aware firmware uses it to erase only the sectors the image needs (much faster). Legacy firmware reads only byte 0 and ignores the rest → backward-compatible. |
| **PAGE_CONFIRM (commit)** | `[0xAD, 0x01]` | With response | Commit the page the device just CRC-reported. |
| **PAGE_CONFIRM (resend)** | `[0xAD, 0x00]` | With response | Reject the page; device will ask you to resend it. |
| **FINISH** | `[0xA9, 0x00]` | With response | Flush the final (partial) page, set boot partition, reboot. |
| **CANCEL** | `[0xAA, 0x00]` | With response | Abort and roll back. |

> **Write types matter on iOS.** BEGIN and data chunks are *Write Without
> Response*; the last chunk of each full page and all control acks are *Write
> With Response* (they double as flow-control sync points). See §5.

---

## 3. Status frames (device → central, notified on STATUS `0xFF03`)

Byte 0 is the status code. Multi-byte fields are noted per frame.

| Code | Name | Frame | Meaning |
|------|------|-------|---------|
| `0x01` | **READY** | `[0x01, 0,0,0]` | OTA session initialized (erase done). Begin sending pages. |
| `0x03` | **SUCCESS** | `[0x03, 0,0,0]` | Image written + boot set. Device reboots ~500 ms later. |
| `0x04` | **ERROR** | `[0x04, err, 0,0]` | Failure; `err` = byte 1 (see §4). |
| `0x05` | **CANCELLED** | `[0x05, 0,0,0]` | Aborted (by you or by a disconnect). |
| `0x06` | **PAGE_CRC** | `[0x06, page_hi, page_lo, crc32_BE(4)]` (7 bytes) | Device's CRC-32 over the page it just received. **CRC is big-endian on the wire (MSB first).** |
| `0x07` | **PAGE_OK** | `[0x07, page_hi, page_lo]` (3 bytes) | Page written to flash and committed. |
| `0x08` | **PAGE_RESEND** | `[0x08, page_hi, page_lo]` (3 bytes) | Page rejected; resend it. |

`page_hi/page_lo` is the 0-based page index, big-endian u16: `page = (hi<<8)|lo`.

Exact parsing from the reference webapp:
```js
case S_PAGE_CRC: // 0x06
  const pageNum  = (v[1]<<8) | v[2];
  const deviceCrc = ((v[3]<<24)|(v[4]<<16)|(v[5]<<8)|v[6]) >>> 0;  // big-endian
```

---

## 4. Error codes (byte 1 of a `0x04` ERROR frame)

| Code | Meaning |
|------|---------|
| `0x01` | OTA begin failed (erase / handle alloc) |
| `0x02` | Flash write failed |
| `0x03` | OTA finalize failed |
| `0x04` | Data/PAGE_CONFIRM received outside an active OTA session |
| `0x05` | No update partition |
| `0x06` | **Earclip only:** battery < 30% — charge and retry |
| `0x07` | **Earclip only:** chip mismatch — image not built for this device |
| `0x08` | **Earclip only:** already in OTA — disconnect and retry |

---

## 5. Transfer flow (the state machine)

Constants: **PAGE_SIZE = 4096 bytes**, **CHUNK_SIZE = 244 bytes** (MTU-safe;
may be raised to the negotiated MTU — the device only cares about total bytes
per page), **MAX_RETRIES = 3** per page.

```
CONNECT ─▶ discover 0x00FF ▶ get FF01/FF02/FF03 ▶ subscribe FF03 notifications
   │
   ├─(optional) read DIS 0x180A/0x2A26 for current version, compare to image
   │
BEGIN:  write FF01 = [0xA8, size_LE32]   (WITHOUT response)
   │    wait STATUS == 0x01 READY        (timeout ~30 s; see Gotchas)
   │
PAGES:  for each 4096-byte page (last may be partial):
   │      ┌─ FULL page:
   │      │   send page as CHUNK_SIZE writes to FF02
   │      │     (WITHOUT response, except the LAST chunk WITH response = sync)
   │      │   wait STATUS 0x06 PAGE_CRC  → deviceCrc
   │      │   if crc32(page) == deviceCrc:
   │      │       write FF01 = [0xAD, 0x01]  (commit, WITH response)
   │      │       wait STATUS 0x07 PAGE_OK   → page done
   │      │   else:
   │      │       write FF01 = [0xAD, 0x00]  (resend, WITH response)
   │      │       wait STATUS 0x08 PAGE_RESEND → resend same page (retry ≤3)
   │      └─ LAST page is PARTIAL (< 4096):
   │          send its bytes to FF02 (no CRC handshake) ─ then FINISH
   │
FINISH: write FF01 = [0xA9, 0x00]        (WITH response)
   │    wait STATUS 0x03 SUCCESS (timeout ~15 s) ─ device reboots
DONE
```

Notes that match the firmware exactly:
- The device accumulates incoming `0xFF02` bytes into a 4096-byte page buffer
  **by offset** — there is no per-chunk framing/header. A "page" is simply 4096
  consecutive bytes (the last page is whatever remains).
- The **final partial page** is *not* CRC-handshaked. You send its bytes, then
  `FINISH`; the firmware writes that leftover buffer during `ota_do_finish()`.
  (A last page that happens to be exactly 4096 goes through the normal
  CRC/commit path, then `FINISH`.)
- `page_hi/page_lo` in `0x07`/`0x08` echo which page; use it to stay in sync.

---

## 6. CRC-32 (must match ESP-IDF `esp_rom_crc32_le`)

Polynomial `0xEDB88320`, init `0xFFFFFFFF`, final XOR `0xFFFFFFFF`, reflected —
i.e. the standard zlib/CRC-32. Compute it over each page's exact bytes and
compare to the device's `0x06` value.

Reference (JS):
```js
const crcTable = new Uint32Array(256);
for (let i=0;i<256;i++){let c=i;for(let j=0;j<8;j++)c=(c&1)?(0xEDB88320^(c>>>1)):(c>>>1);crcTable[i]=c;}
function crc32(data){let crc=0xFFFFFFFF;for(let i=0;i<data.length;i++)crc=crcTable[(crc^data[i])&0xFF]^(crc>>>8);return (crc^0xFFFFFFFF)>>>0;}
```
A ready Swift port is in `swift/NarbisOTAClient.swift`.

---

## 7. Pre-flash image validation (parse first 512 bytes of the `.bin`)

Reject the wrong `.bin` before sending a single byte. All offsets are into the
file's first 512 bytes:

| Offset | Size | Field | Check |
|--------|------|-------|-------|
| `0x00` | u8  | ESP image magic | must be `0xE9` |
| `0x0C` | u16 LE | `chip_id` | must equal the device's chip-id (Edge `0x0000`, earclip `0x000D`) |
| `0x20` | u32 LE | app-desc magic | must be `0xABCD5432` |
| `0x30` | 32 B | version string | NUL-terminated ASCII (informational / downgrade check) |
| `0x50` | 32 B | `project_name` | must start with the device's prefix (`ESP32_Ble` / `narbis_earclip`) |

---

## 8. Device table

| | Edge glasses | Narbis earclip |
|---|---|---|
| Advertised name | `Smart_Glasses` **or** `Narbis_Edge` (exact) | starts with `Narbis Earclip` |
| Chip | ESP32 (`chip_id 0x0000`) | ESP32-C6 (`chip_id 0x000D`) |
| `project_name` prefix | `ESP32_Ble` | `narbis_earclip` |
| Release repo | `narbiscorp/edge-firmware` | `narbiscorp/edge-earclip` |
| Asset filename | `ESP32_Ble.bin` | `narbis_earclip.bin` |
| OTA slot (max size) | 1.5 MB (`1572864`) | 1.25 MB (`1310720`) |

---

## 9. Getting the firmware `.bin` (GitHub Releases)

The webapp lists/downloads releases via the public GitHub API — no auth:

```
GET https://api.github.com/repos/{owner}/{repo}/releases
```
Keep releases whose `assets[]` contains an asset named exactly `binName`
(`ESP32_Ble.bin` / `narbis_earclip.bin`), sort by `published_at` desc, and
download `asset.browser_download_url`. The newest is the one CI just built
(tag format `vYYYY.MM.DD-<short-sha>`).

> The iOS app can fetch the release list + download the `.bin` over normal
> HTTPS (`URLSession`); only the *flashing* needs BLE.

---

## 10. iOS / CoreBluetooth gotchas (read these)

1. **You cannot set the BLE connection parameters or supervision timeout from
   iOS.** They're negotiated by iOS + the peripheral. The peripheral (firmware)
   already requests a 32 s supervision timeout. **The app's only lever over the
   begin-erase stall is sending the correct image size in `0xA8`** (§2) so the
   device erases less and answers `READY` fast. Always send it.
2. **Write-Without-Response flow control.** CoreBluetooth silently drops
   `.withoutResponse` writes if you outrun the queue. Gate each chunk on
   `peripheral.canSendWriteWithoutResponse` and the
   `peripheralIsReady(toSendWriteWithoutResponse:)` delegate callback. The
   "last chunk of each page is Write-With-Response" rule gives you a natural
   per-page barrier on top of that.
3. **MTU.** Use `peripheral.maximumWriteValueLength(for: .withoutResponse)`.
   244 B is always safe; you may chunk larger if MTU allows (device only counts
   total bytes per page).
4. **Subscribe to `0xFF03` notifications before `BEGIN`** and keep a single
   notification handler that dispatches by `value[0]`.
5. **Reconnect-and-retry the BEGIN.** A device still on old firmware can drop
   the link mid-erase; retry `BEGIN` a couple of times (reconnect to the same
   `CBPeripheral`, re-discover, re-subscribe, resend `0xA8`). See the webapp's
   `MAX_BEGIN_ATTEMPTS` logic and `reconnectForRetry()`.
6. **No Web Bluetooth on iOS** — this is why it's a native CoreBluetooth rebuild.
```
