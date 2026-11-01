"""P4 静态门禁必须真的能抓到违规，而不是永远返回通过。"""

from __future__ import annotations

import json

from tools.p4_architecture import check_p4_architecture as gate


def test_current_firmware_tree_passes() -> None:
    assert gate.collect_failures() == []


def test_upward_dependency_in_data_layer_is_rejected() -> None:
    fake = {"data/Fake.h": "#include <Arduino.h>\n#include \"../../hal/X.h\"\n"}
    assert gate._lines(fake, gate._include_pattern("Arduino", "FreeRTOS")) == [
        "data/Fake.h:1: #include <Arduino.h>"]


def test_safety_layer_cannot_include_tasks() -> None:
    fake = {"safety/Fake.cpp": "#include \"../tasks/host_comm/HostCommService.h\"\n"}
    assert gate._lines(fake, gate._include_pattern("tasks/")) == [
        "safety/Fake.cpp:1: #include \"../tasks/host_comm/HostCommService.h\""]


def test_servo_sdk_driver_exemption_is_narrow() -> None:
    """只有 FeetechServoDriver 可以调 SDK；其他文件必须报出来。"""
    fake = {"hal/Other.cpp": "    SMS_STS.WritePosEx(1, 2, 3, 4);\n",
            "hal/servo/FeetechServoDriver.cpp": "    SMS_STS.WritePosEx(1, 2, 3, 4);\n"}
    assert gate._lines(fake, gate.SERVO_SDK, exclude=("FeetechServoDriver.cpp",)) == [
        "hal/Other.cpp:1: SMS_STS.WritePosEx(1, 2, 3, 4);"]


def test_fingerprint_mismatch_is_detected(tmp_path, monkeypatch) -> None:
    """指纹比对不能只认“文件存在”：值不一致时必须报错。"""
    config = tmp_path / "config"
    generated = tmp_path / "generated"
    config.mkdir()
    generated.mkdir()
    (config / "product_fingerprint.json").write_text(
        json.dumps({"control_model_crc": "DEADBEEF"}), encoding="utf-8")
    (generated / "ProductFingerprint.generated.h").write_text(
        "static const uint32_t kProductControlModelCrc = 0x0BADF00DUL;\n", encoding="utf-8")
    monkeypatch.setattr(gate, "DESKTOP_CONFIG", config)
    monkeypatch.setattr(gate, "SOURCE_ROOT", tmp_path)
    failures = gate._fingerprint_failures()
    # 其余五项在临时固件头里缺失，同样必须逐项报出；这里只锁定被篡改的那一项。
    assert "product fingerprint mismatch: control_model_crc" in failures
    assert len(failures) == 6
