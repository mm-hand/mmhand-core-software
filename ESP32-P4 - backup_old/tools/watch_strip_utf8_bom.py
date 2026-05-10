#!/usr/bin/env python3
"""Continuously strip UTF-8 BOM from source files in target paths."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

UTF8_BOM = b"\xef\xbb\xbf"
SOURCE_SUFFIXES = {
    ".c",
    ".cc",
    ".cpp",
    ".cxx",
    ".h",
    ".hh",
    ".hpp",
    ".hxx",
    ".ino",
}


def iter_source_files(path: Path):
    if path.is_file():
        if path.suffix.lower() in SOURCE_SUFFIXES:
            yield path
        return

    if not path.is_dir():
        return

    for file_path in path.rglob("*"):
        if file_path.is_file() and file_path.suffix.lower() in SOURCE_SUFFIXES:
            yield file_path


def strip_bom(file_path: Path) -> bool:
    data = file_path.read_bytes()
    if data.startswith(UTF8_BOM):
        file_path.write_bytes(data[len(UTF8_BOM) :])
        return True
    return False


def scan_once(paths: list[Path]) -> int:
    changed = 0
    seen = set()
    for path in paths:
        for file_path in iter_source_files(path):
            resolved = file_path.resolve()
            if resolved in seen:
                continue
            seen.add(resolved)
            try:
                if strip_bom(file_path):
                    changed += 1
            except OSError:
                # File might be temporarily locked during editor write.
                continue
    return changed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="+", help="Files or directories to watch")
    parser.add_argument("--interval", type=float, default=1.0, help="Polling interval in seconds")
    args = parser.parse_args()

    paths = [Path(p) for p in args.paths]
    print("watch_strip_utf8_bom: watching for BOM in source files...")

    while True:
        changed = scan_once(paths)
        if changed:
            print(f"watch_strip_utf8_bom: removed BOM from {changed} file(s)")
        time.sleep(max(args.interval, 0.2))


if __name__ == "__main__":
    raise SystemExit(main())
