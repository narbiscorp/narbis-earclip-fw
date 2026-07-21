"""narbis_client — host-side mirror of the Narbis Edge Earclip BLE protocol.

proto.py mirrors firmware/components/narbis_core/include/narbis/proto.h
(the wire contract); uuids.py mirrors the GATT UUID allocation. Both are
pure stdlib so non-Python consumers (the web test app generator) can
parse them mechanically.
"""

from . import proto, uuids

__all__ = ["proto", "uuids"]
__version__ = "0.1.0"

PROTO_VER_MAJOR = proto.PROTO_VER_MAJOR
PROTO_VER_MINOR = proto.PROTO_VER_MINOR
PROTO_VER = proto.PROTO_VER
