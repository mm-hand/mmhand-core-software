"""Pin the S3↔P4 CAN sensor contract that the two firmware sketches declare separately."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
P4_DIMENSIONS = ROOT / "ESP32-P4/src/config/DimensionsConfig.h"
P4_PROTOCOL = ROOT / "ESP32-P4/src/config/protocol/S3SensorCanProtocol.h"
S3_CONFIG = ROOT / "ESP32-S3/src/config/Config.h"
S3_TWAI = ROOT / "ESP32-S3/src/hal/HalTWAI.h"

# 两侧规模与协议常量同名同值；三轴分量数是唯一的命名分歧（AXIS_NUM vs AXIS_COUNT）。
SHARED_SCALE = (
    "ENCODER_TOTAL_NUM",
    "TACTILE_GROUP_NUM",
    "TACTILE_SENSOR_PER_GROUP",
    "TACTILE_SENSOR0_TAXELS",
    "TACTILE_SENSOR1_TAXELS",
    "TACTILE_SENSOR2_TAXELS",
    "TACTILE_SUMMARY_BYTES",
    "TACTILE_FULL_BYTES",
)
SHARED_PROTOCOL = (
    "CAN_ID_ENC_BASE",
    "CAN_ID_ENC_LAST",
    "CAN_ID_ENC_CRC",
    "CAN_ENC_FRAME_COUNT",
    "CAN_ENC_PAYLOAD_BYTES",
    "CAN_ENC_CRC_DLC",
    "CAN_ID_ERROR_DETAIL_BASE",
    "CAN_ID_ERROR_DETAIL_LAST",
    "CAN_ID_TAC_SUMMARY_BASE",
    "CAN_ID_TAC_RAW_DIAG",
    "CAN_TAC_SUMMARY_FRAME_COUNT",
    "CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME",
    "CAN_TAC_SUMMARY_DLC",
    "CAN_TAC_SUMMARY_VALID_MASK_BYTES",
    "CAN_TAC_SUMMARY_CRC_BYTES",
    "CAN_TAC_SUMMARY_CRC_INPUT_BYTES",
    "CAN_TAC_SUMMARY_PACKET_BYTES",
    "CAN_ID_TAC_FULL_DUMP_BASE",
    "CAN_TAC_FULL_PAYLOAD_PER_FRAME",
    "CAN_TAC_FULL_DLC",
    "CAN_TAC_FULL_FRAME_COUNT",
    "CAN_ID_TAC_FULL_DUMP_LAST",
)


def _parse_constants(text: str) -> dict[str, str]:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    text = text.replace("\\\n", " ")
    table: dict[str, str] = {}
    for name, value in re.findall(r"constexpr\s+\w+\s+(\w+)\s*=\s*([^;]+);", text):
        table[name] = re.sub(r"\s+", " ", value).strip()
    for name, value in re.findall(r"^\s*#define\s+([A-Za-z_]\w*)\s+(.+)$", text, re.M):
        table.setdefault(name, re.sub(r"\s+", " ", value).strip())
    return table


def _resolve(table: dict[str, str], name: str) -> int:
    """代入本侧根常量递归求值；未知符号或非整型表达式直接报错。"""
    expression = table[name]

    def substitute(match: re.Match[str]) -> str:
        symbol = match.group(0)
        return str(_resolve(table, symbol)) if symbol in table else symbol

    resolved = re.sub(r"[A-Za-z_]\w*", substitute, expression)
    try:
        return int(eval(resolved, {"__builtins__": {}}, {}))  # noqa: S307 - 受控表达式
    except Exception as exc:
        raise AssertionError(f"cannot evaluate {name} = {expression!r}: {exc}") from exc


P4_TABLE = {**_parse_constants(P4_DIMENSIONS.read_text(encoding="utf-8")),
            **_parse_constants(P4_PROTOCOL.read_text(encoding="utf-8"))}
S3_TABLE = {**_parse_constants(S3_CONFIG.read_text(encoding="utf-8")),
            **_parse_constants(S3_TWAI.read_text(encoding="utf-8"))}


@pytest.mark.parametrize("name", SHARED_SCALE + SHARED_PROTOCOL)
def test_shared_contract_matches(name: str) -> None:
    assert name in P4_TABLE, f"P4 side lost {name}"
    assert name in S3_TABLE, f"S3 side lost {name}"
    assert _resolve(P4_TABLE, name) == _resolve(S3_TABLE, name), name


def test_axis_count_names_differ_but_agree() -> None:
    assert _resolve(P4_TABLE, "TACTILE_AXIS_NUM") == _resolve(S3_TABLE, "TACTILE_AXIS_COUNT")


def test_p4_error_detail_last_matches_encoder_scale() -> None:
    base = _resolve(P4_TABLE, "CAN_ID_ERROR_DETAIL_BASE")
    per_frame = _resolve(P4_TABLE, "CAN_ERROR_DETAIL_CODES_PER_FRAME")
    total = _resolve(P4_TABLE, "ENCODER_TOTAL_NUM")
    assert _resolve(P4_TABLE, "CAN_ID_ERROR_DETAIL_LAST") == base + (total + per_frame - 1) // per_frame - 1


def test_p4_tactile_summary_last_matches_frame_count() -> None:
    base = _resolve(P4_TABLE, "CAN_ID_TAC_SUMMARY_BASE")
    frames = _resolve(P4_TABLE, "CAN_TAC_SUMMARY_FRAME_COUNT")
    assert _resolve(P4_TABLE, "CAN_ID_TAC_SUMMARY_LAST") == base + frames - 1
