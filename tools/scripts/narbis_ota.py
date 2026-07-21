#!/usr/bin/env python3
"""BLE OTA pusher (single push or --loop N soak).

Direct-run shim for narbis_client.ota:main (installed console script:
`narbis-ota`).
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # tools/

from narbis_client.ota import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
