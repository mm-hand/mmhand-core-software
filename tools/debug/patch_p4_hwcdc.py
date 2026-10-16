"""Reproducible P4 empty-IN-endpoint wakeup fix for Arduino HWCDC 3.3.10.

After physical USB re-enumeration the endpoint was captured with EP1_CONF=0,
IN_EP1_ST=1 (zero read/write pointers), enabled IN_EMPTY interrupt but no event.
Read-only JTAG inspection did not recover serial; committing only WR_DONE
recovered it without reset. Publish the pending empty packet under the driver's
existing ISR/producer lock before arming IN_EMPTY. No payload is fabricated.
The guard is P4-only and does not touch a nonempty/in-flight payload.
Only the audited official file or its exact patched form is accepted.
"""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import shutil

OFFICIAL_SHA256 = 'c5ed5fdd05aa0df9b74d390812643599223f96b459256718d0ad328aeaba6a8a'
ANCHOR = b'''static inline void hw_cdc_enable_tx_intr(void) {
  portENTER_CRITICAL_SAFE(&hw_cdc_tx_mux);
  usb_serial_jtag_ll_ena_intr_mask(USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY);'''
REPLACEMENT = b'''static inline void hw_cdc_enable_tx_intr(void) {
  portENTER_CRITICAL_SAFE(&hw_cdc_tx_mux);
#if CONFIG_IDF_TARGET_ESP32P4
  // MMHAND_P4_EMPTY_IN_WAKEUP: a re-enumerated empty EP1 can have neither
  // DATA_FREE nor IN_EMPTY set. Commit its empty packet to restart host pickup.
  // Exact empty endpoint signature; never publish a partially staged payload.
  if (USB_SERIAL_JTAG.ep1_conf.val == 0 && USB_SERIAL_JTAG.in_ep1_st.val == 1) {
    usb_serial_jtag_ll_txfifo_flush();
  }
#endif
  usb_serial_jtag_ll_ena_intr_mask(USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY);'''


def patch_source(path: Path, *, check: bool = False) -> bool:
    raw = path.read_bytes()
    if raw.count(REPLACEMENT) == 1:
        original = raw.replace(REPLACEMENT, ANCHOR)
        if hashlib.sha256(original).hexdigest() == OFFICIAL_SHA256:
            return False
    if hashlib.sha256(raw).hexdigest() != OFFICIAL_SHA256 or raw.count(ANCHOR) != 1:
        raise ValueError(f'Unknown HWCDC baseline; not modified: {path}')
    if check:
        raise ValueError(f'P4 HWCDC empty-IN wakeup fix required: {path}')
    backup = path.with_name(path.name + '.mmhand-before-empty-in-fix.bak')
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_bytes(raw.replace(ANCHOR, REPLACEMENT))
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    changed = patch_source(args.source, check=args.check)
    print(f'P4 HWCDC empty-IN {"patched (original backed up)" if changed else "verified"}: {args.source}')


if __name__ == '__main__':
    main()
