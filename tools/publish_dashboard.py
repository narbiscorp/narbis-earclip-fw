#!/usr/bin/env python
"""
publish_dashboard.py — publish tools/testapp to the public Narbis Dev Hub.

Source of truth for the functional-test dashboard is THIS repo
(tools/testapp). The public hub (narbiscorp/edge-earclip Pages,
https://narbiscorp.github.io/edge-earclip/) serves a copy under
/functest/. Run this after changing the app, review the diff it prints,
then commit+push in the hub checkout it created/updated.

Usage: python tools/publish_dashboard.py [--hub <path-to-edge-earclip-checkout>]
       (default hub path: ../edge-earclip relative to this repo's parent)
"""
import argparse
import pathlib
import shutil
import subprocess
import sys

APP_FILES = [
    "index.html", "style.css", "app.js", "dashboard.js",
    "proto.js", "proto_consts.js", "mock.js", "README.md",
]

def main() -> int:
    repo = pathlib.Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser()
    ap.add_argument("--hub", default=str(repo.parent / "edge-earclip"))
    args = ap.parse_args()

    src = repo / "tools" / "testapp"
    hub = pathlib.Path(args.hub)
    dst = hub / "functest"
    if not (hub / ".git").exists():
        print(f"hub checkout not found at {hub} — clone "
              "https://github.com/narbiscorp/edge-earclip first", file=sys.stderr)
        return 1

    dst.mkdir(exist_ok=True)
    for name in APP_FILES:
        f = src / name
        if not f.exists():
            if name == "dashboard.js":
                print(f"warning: {name} missing (pre-dashboard build?)")
                continue
            print(f"missing {f}", file=sys.stderr)
            return 1
        shutil.copy2(f, dst / name)
        print(f"  {name} -> functest/")

    print(subprocess.run(["git", "-C", str(hub), "status", "--short"],
                         capture_output=True, text=True).stdout)
    print("Review, then: git -C", hub, "add functest && commit && push")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
