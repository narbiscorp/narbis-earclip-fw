#!/usr/bin/env python3
"""Live PPG/accel/IBI plot with gate spans + AGC step markers.

Direct-run shim for narbis_client.liveplot:main (installed console
script: `narbis-plot`). Needs matplotlib (narbis-tools[viz]).
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # tools/

from narbis_client.liveplot import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
