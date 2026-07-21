"""GATT UUID allocation — mirror of proto.h NC_UUID_BASE_BYTES / NC_ALIAS_*.

128-bit Narbis base: A5E9xxxx-C6A0-43C0-B0D0-6E6172626973 ("narbis" in the
tail), where xxxx is the 16-bit alias in the big-endian text position
(bytes [12..13] LE of the NimBLE byte array in proto.h).

Standard 16-bit UUIDs expand on the Bluetooth SIG base
0000xxxx-0000-1000-8000-00805F9B34FB.

All UUID strings are lowercase, matching bleak's normalized form.
"""

# ------------------------------------------------------------------ #
# Narbis 128-bit base                                                 #
# ------------------------------------------------------------------ #

NARBIS_UUID_FMT = "a5e9{:04x}-c6a0-43c0-b0d0-6e6172626973"


def narbis_uuid(alias: int) -> str:
    """Full 128-bit UUID string for a 16-bit Narbis alias."""
    if not 0 <= alias <= 0xFFFF:
        raise ValueError(f"alias out of range: {alias:#x}")
    return NARBIS_UUID_FMT.format(alias)


# NC_ALIAS_* from proto.h
ALIAS_SENSOR_SVC = 0x0100
ALIAS_PPG = 0x0101
ALIAS_ACCEL = 0x0102
ALIAS_IBI = 0x0103
ALIAS_EVENT = 0x0104
ALIAS_STATUS = 0x0105
ALIAS_CONTROL = 0x0106
ALIAS_PROTO_VER = 0x0107
ALIAS_OTA_SVC = 0x0200
ALIAS_OTA_CTRL = 0x0201
ALIAS_OTA_DATA = 0x0202

UUID_SENSOR_SVC = narbis_uuid(ALIAS_SENSOR_SVC)
UUID_PPG = narbis_uuid(ALIAS_PPG)
UUID_ACCEL = narbis_uuid(ALIAS_ACCEL)
UUID_IBI = narbis_uuid(ALIAS_IBI)
UUID_EVENT = narbis_uuid(ALIAS_EVENT)
UUID_STATUS = narbis_uuid(ALIAS_STATUS)
UUID_CONTROL = narbis_uuid(ALIAS_CONTROL)
UUID_PROTO_VER = narbis_uuid(ALIAS_PROTO_VER)
UUID_OTA_SVC = narbis_uuid(ALIAS_OTA_SVC)
UUID_OTA_CTRL = narbis_uuid(ALIAS_OTA_CTRL)
UUID_OTA_DATA = narbis_uuid(ALIAS_OTA_DATA)

# ------------------------------------------------------------------ #
# Standard Bluetooth SIG UUIDs                                        #
# ------------------------------------------------------------------ #

SIG_UUID_FMT = "0000{:04x}-0000-1000-8000-00805f9b34fb"


def sig_uuid(uuid16: int) -> str:
    """Full 128-bit UUID string for a standard SIG 16-bit UUID."""
    if not 0 <= uuid16 <= 0xFFFF:
        raise ValueError(f"uuid16 out of range: {uuid16:#x}")
    return SIG_UUID_FMT.format(uuid16)


# Services
SVC_BATTERY = 0x180F
SVC_DEVICE_INFO = 0x180A
SVC_HEART_RATE = 0x180D

UUID_BATTERY_SVC = sig_uuid(SVC_BATTERY)
UUID_DEVICE_INFO_SVC = sig_uuid(SVC_DEVICE_INFO)
UUID_HEART_RATE_SVC = sig_uuid(SVC_HEART_RATE)

# Characteristics
CHR_BATTERY_LEVEL = 0x2A19          # Battery Service
CHR_HR_MEASUREMENT = 0x2A37         # Heart Rate Service
CHR_MODEL_NUMBER = 0x2A24           # Device Info
CHR_FIRMWARE_REVISION = 0x2A26
CHR_HARDWARE_REVISION = 0x2A27
CHR_SOFTWARE_REVISION = 0x2A28
CHR_MANUFACTURER_NAME = 0x2A29

UUID_BATTERY_LEVEL = sig_uuid(CHR_BATTERY_LEVEL)
UUID_HR_MEASUREMENT = sig_uuid(CHR_HR_MEASUREMENT)
UUID_MODEL_NUMBER = sig_uuid(CHR_MODEL_NUMBER)
UUID_FIRMWARE_REVISION = sig_uuid(CHR_FIRMWARE_REVISION)
UUID_HARDWARE_REVISION = sig_uuid(CHR_HARDWARE_REVISION)
UUID_SOFTWARE_REVISION = sig_uuid(CHR_SOFTWARE_REVISION)
UUID_MANUFACTURER_NAME = sig_uuid(CHR_MANUFACTURER_NAME)
