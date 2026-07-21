#!/usr/bin/env python3
"""Knob registry CLI: list / get / set / save / reset / diff.

Direct-run shim for narbis_client.knobs:main (installed console script:
`narbis-knobs`).
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # tools/

from narbis_client.knobs import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
