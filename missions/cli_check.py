#!/usr/bin/env python3
"""Pre-flight check: every parameter the executive reads from its argument namespace must be
defined by BOTH command lines (aerial_mission_runner.py and aerial_explore_runner.py, which
builds its own parser). An exploration test flight (2026-09-24) died at its first frontier on a
parameter added to one parser and not the other; the mocks did not catch it because their
hand-built namespaces carried it. Run before every batch:

  python3 missions/cli_check.py        (exit 0 = consistent)
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
IGNORE = {"x", "y", "z"}          # p.x-style accesses on variables named `a` in local loops


def defined_by(path):
    src = open(path, encoding="utf-8").read()
    names = set(m.replace("-", "_") for m in re.findall(r'add_argument\("--([a-z\-]+)"', src))
    names |= set(re.findall(r"\ba\.([a-z_]+)\s*=", src))          # attributes set in main()
    if "add_vehicle_args(ap)" in src:                               # the shared vehicle options
        exe = open(os.path.join(HERE, "aerial_mission_runner.py"), encoding="utf-8").read()
        block = exe.split("def add_vehicle_args(ap):", 1)[1].split("\ndef ", 1)[0]
        names |= set(m.replace("-", "_") for m in re.findall(r'add_argument\("--([a-z\-]+)"', block))
    return names


def main():
    exe = open(os.path.join(HERE, "aerial_mission_runner.py"), encoding="utf-8").read()
    used = sorted(set(re.findall(r"\ba\.([a-z_]+)\b", exe)) - IGNORE)
    bad = 0
    for name in ("aerial_mission_runner.py", "aerial_explore_runner.py"):
        have = defined_by(os.path.join(HERE, name))
        missing = [u for u in used if u not in have]
        print(f"{name}: {len(used)} attributes read by the executive, missing {missing if missing else 'none'}")
        bad += len(missing)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
