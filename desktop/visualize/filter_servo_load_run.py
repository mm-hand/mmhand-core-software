#!/usr/bin/env python3
"""
Filter the wide Save Data runtime CSV into a narrow load-analysis CSV.

The output keeps only:
  - time / mode / active scope
  - controlled motor abs / target / load / raw / speed / online
  - corresponding joint angle / target / encoder raw

Examples:
  python desktop/visualize/filter_servo_load_run.py --csv run_data/hand_run_xxx.csv
  python desktop/visualize/filter_servo_load_run.py --motors M05,M06
  python desktop/visualize/filter_servo_load_run.py --joint 4
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from typing import Dict, Iterable, List, Optional, Sequence


DESKTOP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(DESKTOP_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from desktop.core.protocol import MOTOR_COUNT  # noqa: E402
from desktop.core.motor_mapping import MOTOR_TO_JOINT_INDEX  # noqa: E402


BASE_FIELDS = [
    "sample",
    "host_time",
    "elapsed_s",
    "device_timestamp",
    "mode",
    "active_scope_label",
    "active_motor_mask_hex",
    "active_joint_mask_hex",
]


def _default_run_data_dir() -> str:
    return os.path.join(PROJECT_ROOT, "run_data")


def _latest_csv_file(run_data_dir: str) -> str:
    if not os.path.isdir(run_data_dir):
        raise FileNotFoundError(f"run_data directory does not exist: {run_data_dir}")
    candidates = [
        os.path.join(run_data_dir, name)
        for name in os.listdir(run_data_dir)
        if name.lower().endswith(".csv")
        and not name.lower().endswith("_joint_debug_packets.csv")
        and not name.lower().endswith("_load_slim.csv")
        and os.path.isfile(os.path.join(run_data_dir, name))
    ]
    if not candidates:
        raise FileNotFoundError(f"No runtime CSV file found in: {run_data_dir}")
    candidates.sort(key=lambda path: os.path.getmtime(path), reverse=True)
    return candidates[0]


def _parse_int_auto(value: str) -> int:
    text = str(value or "").strip()
    if not text:
        return 0
    return int(text, 0)


def _parse_motor_token(token: str) -> int:
    token = str(token).strip().upper()
    if token.startswith("M"):
        token = token[1:]
    return int(token)


def _parse_motor_list(text: str) -> List[int]:
    motors: List[int] = []
    for piece in str(text or "").replace(";", ",").split(","):
        piece = piece.strip()
        if not piece:
            continue
        motor = _parse_motor_token(piece)
        if 0 <= motor < MOTOR_COUNT and motor not in motors:
            motors.append(motor)
    return motors


def _motors_for_joint(joint: Optional[int]) -> List[int]:
    if joint is None:
        return []
    return [
        motor
        for motor, mapped_joint in enumerate(MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT])
        if int(mapped_joint) == int(joint)
    ]


def _motors_from_active_mask(rows: Sequence[Dict[str, str]]) -> List[int]:
    mask = 0
    for row in rows:
        try:
            mask |= _parse_int_auto(row.get("active_motor_mask_hex", "0"))
        except Exception:
            pass
    return [idx for idx in range(MOTOR_COUNT) if (mask & (1 << idx)) != 0]


def _resolve_motors(rows: Sequence[Dict[str, str]], motors_text: str, joint: Optional[int]) -> List[int]:
    motors = _parse_motor_list(motors_text)
    motors.extend(m for m in _motors_for_joint(joint) if m not in motors)
    if not motors:
        motors = _motors_from_active_mask(rows)
    if not motors:
        raise ValueError("No motors selected. Use --motors M05,M06 or --joint 4.")
    return motors


def _read_rows(csv_path: str) -> tuple[List[str], List[Dict[str, str]]]:
    with open(csv_path, "r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), list(reader)


def _row_value(row: Dict[str, str], key: str) -> str:
    return row.get(key, "")


def _elapsed_s(host_time: str, t0: Optional[float]) -> str:
    try:
        t = float(host_time)
    except Exception:
        return ""
    if t0 is None:
        return "0.000000"
    return f"{t - t0:.6f}"


def _first_host_time(rows: Sequence[Dict[str, str]]) -> Optional[float]:
    for row in rows:
        try:
            return float(row.get("host_time", ""))
        except Exception:
            continue
    return None


def _output_path(csv_path: str, out_path: str) -> str:
    if out_path:
        return os.path.abspath(out_path)
    base, _ext = os.path.splitext(os.path.abspath(csv_path))
    return f"{base}_load_slim.csv"


def _fieldnames_for_motors(motors: Iterable[int]) -> List[str]:
    fields = list(BASE_FIELDS)
    seen_joints: List[int] = []
    for motor in motors:
        fields += [
            f"M{motor:02d}_abs",
            f"M{motor:02d}_target_abs",
            f"M{motor:02d}_load",
            f"M{motor:02d}_load_mag",
            f"M{motor:02d}_load_dir",
            f"M{motor:02d}_raw",
            f"M{motor:02d}_speed",
            f"M{motor:02d}_online",
        ]
        joint = MOTOR_TO_JOINT_INDEX[motor] if motor < len(MOTOR_TO_JOINT_INDEX) else -1
        if 0 <= int(joint) and int(joint) not in seen_joints:
            seen_joints.append(int(joint))
    for joint in seen_joints:
        fields += [
            f"J{joint:02d}_deg",
            f"J{joint:02d}_target_deg",
            f"J{joint:02d}_encoder_raw",
        ]
    return fields


def filter_csv(csv_path: str, out_path: str = "", motors_text: str = "", joint: Optional[int] = None) -> str:
    _fieldnames, rows = _read_rows(csv_path)
    motors = _resolve_motors(rows, motors_text, joint)
    out = _output_path(csv_path, out_path)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    t0 = _first_host_time(rows)
    fields = _fieldnames_for_motors(motors)

    with open(out, "w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for sample, row in enumerate(rows):
            slim = {
                "sample": sample,
                "host_time": _row_value(row, "host_time"),
                "elapsed_s": _elapsed_s(_row_value(row, "host_time"), t0),
                "device_timestamp": _row_value(row, "device_timestamp"),
                "mode": _row_value(row, "mode"),
                "active_scope_label": _row_value(row, "active_scope_label"),
                "active_motor_mask_hex": _row_value(row, "active_motor_mask_hex"),
                "active_joint_mask_hex": _row_value(row, "active_joint_mask_hex"),
            }
            joints: List[int] = []
            for motor in motors:
                slim[f"M{motor:02d}_abs"] = _row_value(row, f"servo_angle_m{motor:02d}")
                slim[f"M{motor:02d}_target_abs"] = _row_value(row, f"motor_target_abs_m{motor:02d}")
                slim[f"M{motor:02d}_load"] = _row_value(row, f"servo_load_m{motor:02d}")
                slim[f"M{motor:02d}_load_mag"] = _row_value(row, f"servo_load_mag_m{motor:02d}")
                slim[f"M{motor:02d}_load_dir"] = _row_value(row, f"servo_load_dir_m{motor:02d}")
                slim[f"M{motor:02d}_raw"] = _row_value(row, f"servo_raw_m{motor:02d}")
                slim[f"M{motor:02d}_speed"] = _row_value(row, f"servo_speed_m{motor:02d}")
                slim[f"M{motor:02d}_online"] = _row_value(row, f"servo_online_m{motor:02d}")
                mapped_joint = MOTOR_TO_JOINT_INDEX[motor] if motor < len(MOTOR_TO_JOINT_INDEX) else -1
                if 0 <= int(mapped_joint) and int(mapped_joint) not in joints:
                    joints.append(int(mapped_joint))
            for mapped_joint in joints:
                slim[f"J{mapped_joint:02d}_deg"] = _row_value(row, f"joint_deg_j{mapped_joint:02d}")
                slim[f"J{mapped_joint:02d}_target_deg"] = _row_value(row, f"joint_target_deg_j{mapped_joint:02d}")
                slim[f"J{mapped_joint:02d}_encoder_raw"] = _row_value(row, f"joint_encoder_raw_j{mapped_joint:02d}")
            writer.writerow(slim)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="Filter runtime CSV to controlled motor load/abs and joint angle columns.")
    parser.add_argument("--csv", default="", help="CSV path. Defaults to latest runtime CSV under run_data.")
    parser.add_argument("--run-data-dir", default=_default_run_data_dir(), help="Directory used when --csv is omitted.")
    parser.add_argument("--motors", default="", help="Comma-separated motors, for example M05,M06.")
    parser.add_argument("--joint", type=int, default=None, help="Export all motors mapped to this joint.")
    parser.add_argument("--out", default="", help="Output CSV path. Defaults to *_load_slim.csv.")
    args = parser.parse_args()

    try:
        csv_path = os.path.abspath(args.csv) if args.csv else _latest_csv_file(os.path.abspath(args.run_data_dir))
        out = filter_csv(csv_path, out_path=args.out, motors_text=args.motors, joint=args.joint)
        print(f"input={csv_path}")
        print(f"output={out}")
        return 0
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
