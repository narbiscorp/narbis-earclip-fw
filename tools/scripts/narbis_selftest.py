#!/usr/bin/env python3
"""Run the on-device self-test and print the PASS/FAIL table.

Direct-run shim for narbis_client.selftest:main (installed console
script: `narbis-selftest`).
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # tools/

from narbis_client.selftest import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
