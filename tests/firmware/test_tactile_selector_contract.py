"""Physical page IDs must match the manufacturer SPI destination ID."""
import re
from pathlib import Path


def test_physical_selector_bits_match_id_write_destination():
    source=(Path(__file__).resolve().parents[2]/'ESP32-S3/src/hal/HalTactile.cpp').read_text(encoding='utf-8')
    values=re.search(r'kPhysicalSlotBits\[3\] = \{([^}]+)\}',source).group(1)
    slots=[int(value.strip(),0) for value in values.split(',')]
    assert slots == [page_id+1 for page_id in range(3)]


def test_default_route_preserves_existing_installation_and_old_index_encoding_is_not_reused():
    source=(Path(__file__).resolve().parents[2]/'ESP32-S3/src/hal/HalTactile.cpp').read_text(encoding='utf-8')
    packed=int(re.search(r'getUChar\(key, (0x[0-9A-Fa-f]+)\)',source).group(1),0)
    indices=[(packed>>(slot*2))&3 for slot in range(3)]
    assert [index+1 for index in indices]==[3,2,1]
    assert '"pax-route-v2"' in source
    assert '"pax-route"' not in source
