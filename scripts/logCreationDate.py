#!/usr/bin/env python3
"""
Set the filesystem creation/modification date on WPILib and REV log files
based on the timestamp embedded in their filename.

Supported filename formats:
  FRC_20260310_143022.wpilog
  REV_20260309_223324.revlog

Usage:
  python3 logCreationDate.py <glob_pattern>

Examples:
  python3 logCreationDate.py /logs/*.wpilog
  python3 logCreationDate.py /logs/*
  python3 logCreationDate.py "/logs/FRC_2026*.wpilog"
"""

import glob
import os
import sys
import re
from datetime import datetime

# Matches FRC_YYYYMMDD_HHMMSS[_anything].wpilog or REV_YYYYMMDD_HHMMSS[_anything].revlog
# e.g. FRC_20260310_143022.wpilog or FRC_20240329_150221_MIDET_Q2.wpilog
FILENAME_PATTERN = re.compile(
    r'^(?:FRC|REV)_(\d{4})_?(\d{2})_?(\d{2}).(\d{2})_?(\d{2})_?(\d{2})(?:_[^.]+)?\.(wpilog|revlog|dslog|dsevents)$',
    re.IGNORECASE
)

def parse_timestamp(filename: str) -> datetime | None:
    m = FILENAME_PATTERN.match(filename)
    if not m:
        return None
    year, month, day, hour, minute, second = (int(x) for x in m.groups()[:6])
    return datetime(year, month, day, hour, minute, second)

def set_file_times(path: str, dt: datetime) -> None:
    ts = dt.timestamp()
    os.utime(path, (ts, ts))

def process_glob(pattern: str) -> None:
    paths = glob.glob(pattern, recursive=True)
    files = sorted(p for p in paths if os.path.isfile(p))

    if not files:
        print(f"No files matched: '{pattern}'", file=sys.stderr)
        sys.exit(1)

    matched = 0
    skipped = 0

    for filepath in files:
        filename = os.path.basename(filepath)
        dt = parse_timestamp(filename)
        if dt is None:
            #print(f"  SKIP   {filepath}  (filename doesn't match expected pattern)")
            skipped += 1
            continue

        set_file_times(filepath, dt)
        #print(f"  SET    {filepath}  →  {dt.strftime('%Y-%m-%d %H:%M:%S')}")
        matched += 1

    print(f"\nDone. {matched} file(s) updated, {skipped} skipped.")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <glob_pattern>", file=sys.stderr)
        sys.exit(1)

    process_glob(sys.argv[1])
