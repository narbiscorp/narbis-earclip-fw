#!/usr/bin/env python3
"""Record all Narbis earclip streams to CSV (+ parquet mirrors).

Direct-run shim for narbis_client.recorder:main (installed console
script: `narbis-record`). Works uninstalled from any cwd.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # tools/

from narbis_client.recorder import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
