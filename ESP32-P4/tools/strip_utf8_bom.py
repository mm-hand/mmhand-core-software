#!/usr/bin/env python3
"""Strip UTF-8 BOM from source files to avoid GCC 'stray #' errors."""

from __future__ import annotations

import argparse
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="+", help="Files or directories to scan")
    args = parser.parse_args()

    changed = 0
    seen = set()
    for raw_path in args.paths:
        path = Path(raw_path)
        for file_path in iter_source_files(path):
            resolved = file_path.resolve()
            if resolved in seen:
                continue
            seen.add(resolved)
            if strip_bom(file_path):
                changed += 1

    if changed:
        print(f"strip_utf8_bom: removed BOM from {changed} file(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
