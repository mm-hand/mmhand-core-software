"""Backport Espressif's direct EP1 FIFO write to affected P4 SDK headers.

EP1 reads consume RX data. A read/modify/write used for TX therefore loses
incoming bytes during full-duplex traffic. Upstream uses ep1.val = buf[i]:
https://github.com/espressif/esp-idf/blob/master/components/esp_hal_usb/esp32p4/include/hal/usb_serial_jtag_ll.h
The original header is backed up once; unknown implementations are rejected.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import shutil

OLD = 'HAL_FORCE_MODIFY_U32_REG_FIELD(USB_SERIAL_JTAG.ep1, rdwr_byte, buf[i]);'
NEW = 'USB_SERIAL_JTAG.ep1.val = buf[i];'


def patch_header(path: Path, *, check: bool = False) -> bool:
    raw = path.read_bytes()
    old, new = OLD.encode(), NEW.encode()
    if raw.count(old) == 1 and new not in raw:
        if check:
            raise ValueError(f'P4 USB FIFO fix is required: {path}')
        backup = path.with_name(path.name + '.mmhand-before-fifo-fix.bak')
        if not backup.exists():
            shutil.copy2(path, backup)
        path.write_bytes(raw.replace(old, new))
        return True
    if old not in raw and raw.count(new) == 1:
        return False
    raise ValueError(f'Unrecognized P4 USB FIFO implementation; not modified: {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tools-root', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    candidates = sorted((args.tools_root / 'esp32p4_es-libs').glob(
        '*/include/hal/esp32p4/include/hal/usb_serial_jtag_ll.h'))
    if not candidates:
        parser.error('P4 pre-rev3 Arduino SDK header was not found')
    for path in candidates:
        changed = patch_header(path, check=args.check)
        print(f'P4 USB FIFO {"fixed (original backed up)" if changed else "verified"}: {path}')


if __name__ == '__main__':
    main()
