from __future__ import annotations

import json
import os
from typing import Dict, Iterable, Optional

from .topic_schema import TOPIC_FIELDNAMES


try:
    import h5py  # type: ignore
    import numpy as np
except Exception:  # pragma: no cover - optional dependency
    h5py = None
    np = None


class Hdf5SessionRecorder:
    """Append topic rows into a structured HDF5 session file.

    CSV is the live/debug-friendly output. HDF5 is the structured dataset output
    for later training and analysis. Each topic becomes a group, and each field
    becomes an appendable dataset.
    """

    def __init__(self, compression: str = "gzip", flush_every: int = 50):
        self.path = ""
        self.enabled = False
        self.error = ""
        self.compression = compression
        self.flush_every = max(1, int(flush_every))
        self._h5 = None
        self._fieldnames: Dict[str, list[str]] = {}
        self._meta: Dict[str, object] = {}
        self._pending_rows = 0

    @property
    def available(self) -> bool:
        return h5py is not None

    def start(
        self,
        path: str,
        topic_fieldnames: Dict[str, Iterable[str]],
        meta: Optional[Dict[str, object]] = None,
    ) -> None:
        self.stop(write_file=False)
        self.path = path
        self.enabled = self.available
        self.error = "" if self.enabled else "h5py is not installed"
        self._fieldnames = {name: list(fields) for name, fields in topic_fieldnames.items()}
        self._meta = dict(meta or {})
        self._pending_rows = 0
        if not self.enabled:
            return
        try:
            folder = os.path.dirname(self.path)
            if folder:
                os.makedirs(folder, exist_ok=True)
            self._h5 = h5py.File(self.path, "w")
            self._initialize_file()
        except Exception as exc:
            self.error = str(exc)
            self.enabled = False
            try:
                if self._h5 is not None:
                    self._h5.close()
            finally:
                self._h5 = None

    def stop(self, write_file: bool = True) -> None:
        if self._h5 is not None:
            try:
                self._h5.flush()
            finally:
                self._h5.close()
        self.path = ""
        self.enabled = False
        self._h5 = None
        self._fieldnames = {}
        self._meta = {}
        self._pending_rows = 0

    def write_row(self, topic: str, row: Dict[str, object]) -> None:
        if not self.enabled or self._h5 is None:
            return
        fields = self._fieldnames.setdefault(topic, list(TOPIC_FIELDNAMES.get(topic) or row.keys()))
        topic_group = self._ensure_topic_group(topic, fields)
        for field in fields:
            self._append_value(topic_group, field, row.get(field, ""))
        self._pending_rows += 1
        if self._pending_rows >= self.flush_every:
            self._h5.flush()
            self._pending_rows = 0

    def _initialize_file(self) -> None:
        assert h5py is not None
        assert self._h5 is not None
        self._h5.attrs["format"] = "mmhand_data_session"
        self._h5.attrs["format_version"] = 2
        self._h5.attrs["created_by"] = "desktop.core.data_recording.Hdf5SessionRecorder"
        self._write_meta_and_config()
        topics_group = self._h5.create_group("topics")
        for topic, fields in self._fieldnames.items():
            group = topics_group.create_group(topic)
            group.attrs["fieldnames"] = json.dumps(fields, ensure_ascii=False)
            for field in fields:
                self._create_empty_dataset(group, field)

    def _write_meta_and_config(self) -> None:
        assert h5py is not None
        assert self._h5 is not None
        string_dtype = h5py.string_dtype(encoding="utf-8")
        meta_group = self._h5.create_group("meta")
        meta_json = json.dumps(_strip_large_config_from_meta(self._meta), ensure_ascii=False, default=str)
        meta_group.create_dataset("session_json", data=meta_json, dtype=string_dtype)
        meta_group.attrs["json"] = meta_json

        config_group = self._h5.create_group("config")
        calibration_snapshot = self._meta.get("calibration_snapshot", {})
        config_group.create_dataset(
            "calibration_snapshot_json",
            data=json.dumps(calibration_snapshot, ensure_ascii=False, default=str),
            dtype=string_dtype,
        )
        files_group = config_group.create_group("files")
        config_files = self._meta.get("config_files", {})
        if isinstance(config_files, dict):
            for name, payload in config_files.items():
                safe_name = _safe_hdf5_name(str(name))
                files_group.create_dataset(
                    safe_name,
                    data=json.dumps(payload, ensure_ascii=False, default=str),
                    dtype=string_dtype,
                )

    def _ensure_topic_group(self, topic: str, fields: list[str]):
        assert self._h5 is not None
        topics_group = self._h5.require_group("topics")
        if topic in topics_group:
            group = topics_group[topic]
        else:
            group = topics_group.create_group(topic)
        group.attrs["fieldnames"] = json.dumps(fields, ensure_ascii=False)
        for field in fields:
            if field not in group:
                self._create_empty_dataset(group, field)
        return group

    def _create_empty_dataset(self, group, field: str) -> None:
        assert h5py is not None
        dtype_kind = _field_dtype_kind(field)
        kwargs = self._dataset_kwargs()
        if dtype_kind == "str":
            group.create_dataset(
                field,
                shape=(0,),
                maxshape=(None,),
                chunks=True,
                dtype=h5py.string_dtype(encoding="utf-8"),
                **kwargs,
            )
        elif dtype_kind == "int":
            group.create_dataset(field, shape=(0,), maxshape=(None,), chunks=True, dtype="i8", **kwargs)
        else:
            group.create_dataset(field, shape=(0,), maxshape=(None,), chunks=True, dtype="f8", **kwargs)

    def _append_value(self, group, field: str, value: object) -> None:
        dataset = group[field]
        idx = int(dataset.shape[0])
        dataset.resize((idx + 1,))
        dtype_kind = _field_dtype_kind(field)
        if dtype_kind == "str":
            dataset[idx] = "" if value is None else str(value)
        elif dtype_kind == "int":
            # P4 时间缺失使用 -1，避免与启动瞬间合法的 0 ms 混淆。
            dataset[idx] = -1 if field == "p4_time_ms" and value in ("", None) else _to_int(value)
        else:
            dataset[idx] = _to_float(value)

    def _dataset_kwargs(self) -> Dict[str, object]:
        if self.compression:
            return {"compression": self.compression}
        return {}


def _field_dtype_kind(field: str) -> str:
    string_fields = {
        "mode",
        "calib_status",
        "system_fault_bitmap",
        "active_scope_label",
        "active_motor_mask",
        "active_joint_mask",
        "multi_turn_unknown_mask",
        "manual_reference_valid_mask",
        "manual_reference_crc",
        "event_type",
        "source",
        "detail",
        "frame_id",
        "input_space",
        "output_space",
        "mapping_id",
        "mapping_version",
        "urdf_config_crc",
        "raw_joint_targets_json",
        "input_joint_targets_json",
        "output_joint_targets_json",
        "clamped_joints_json",
        "block_reason",
        "host_time",
        "finger_name",
    }
    if field in string_fields:
        return "str"
    if field.endswith("_mask") or field.endswith("_mask_hex"):
        return "str"
    if field in {
        "latency_ms",
        "max_force",
        "mean_force",
        "force_mag",
        "target_deg",
        "actual_deg",
        "loop1_output",
    } or field.endswith("_deg"):
        return "float"
    if field.endswith(("_force", "_mean_force", "_max_force", "_fx", "_fy", "_fz", "_mag")):
        return "float"
    if field.endswith("_time"):
        return "float"
    return "int"


def _to_int(value: object) -> int:
    if value in ("", None):
        return 0
    try:
        if isinstance(value, str) and value.lower().startswith("0x"):
            return int(value, 16)
        return int(float(value))
    except Exception:
        return 0


def _to_float(value: object) -> float:
    if value in ("", None):
        return float("nan")
    try:
        return float(value)
    except Exception:
        return float("nan")


def _safe_hdf5_name(name: str) -> str:
    return name.replace("/", "_").replace("\\", "_").replace(":", "_")


def _strip_large_config_from_meta(meta: Dict[str, object]) -> Dict[str, object]:
    out = dict(meta)
    out.pop("config_files", None)
    out.pop("calibration_snapshot", None)
    return out
