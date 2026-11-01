"""P4 分层、硬件唯一入口、薄任务、安全规则与产品契约的静态门禁。

只做文本与生成文件检查，不编译固件、不访问硬件。所有源文件读一次后
在内存里做匹配，因此不需要 ripgrep、PowerShell 或 Windows。

依赖方向：config/data/generated -> hal/safety/system -> tasks；
system 内 SystemAssembly/AppBootstrap 为组合根，可编排 tasks。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE_ROOT = ROOT / "ESP32-P4" / "src"
DESKTOP_CONFIG = ROOT / "desktop" / "resources" / "config"

ALLOWED_TOP_LEVEL = {"config", "data", "generated", "hal", "safety", "system", "tasks", "control"}

# 只在对应用的位置允许出现的 FreeRTOS / SDK 直接调用。
SERVO_SDK = re.compile(r"SMS_STS|SyncWritePosEx|WritePosEx|EnableTorque|\bFeedBack\s*\(|\bReadPos\s*\(")
TWAI_API = re.compile(r"\btwai_[a-zA-Z0-9_]+\s*\(")
SERVO_TARGET_QUEUE = re.compile(r"servoTargetQueue\.(send|receive|overwrite|reset)")
SAFETY_WRITE = re.compile(r"\b(recordSafetyEvent|recordSafetyRuleEvent|serviceRecordSafety)\s*\(")
FREERTOS_INCLUDE = re.compile(r'^\s*#include\s*[<"]freertos/')


def _source_files() -> dict[str, str]:
    """返回相对路径 -> 文本；内嵌飞特 SDK 不参与分层规则。"""
    files = {}
    for path in SOURCE_ROOT.rglob("*"):
        if path.suffix.lower() not in {".c", ".cpp", ".h"}:
            continue
        if "FTServo_Arduino" in path.parts:
            continue
        files[path.relative_to(SOURCE_ROOT).as_posix()] = path.read_text(encoding="utf-8")
    return files


def _lines(files: dict[str, str], pattern: re.Pattern, *, exclude: tuple[str, ...] = ()) -> list[str]:
    """在全部文件中按行匹配正则，返回 `path:line: text` 形式的问题列表。"""
    hits = []
    for relative, text in files.items():
        if any(relative.endswith(suffix) for suffix in exclude):
            continue
        for number, line in enumerate(text.splitlines(), 1):
            if pattern.search(line):
                hits.append(f"{relative}:{number}: {line.strip()}")
    return hits


def _include_pattern(*targets: str) -> re.Pattern:
    joined = "|".join(targets)
    return re.compile(rf"^\s*#include.*(?:{joined})")


def collect_failures() -> list[str]:
    failures: list[str] = []
    files = _source_files()
    scale = len(files) + 1

    # R0：顶层目录是架构契约，历史功能型目录不得回流。
    actual = {path.name for path in SOURCE_ROOT.iterdir()
              if path.is_dir() and any(path.rglob("*"))}
    failures += [f"unregistered top-level source directory: {name}"
                 for name in sorted(actual - ALLOWED_TOP_LEVEL)]

    # R1：每个固件源文件必须在头部写 @file / @brief。
    for relative, text in files.items():
        head = "\n".join(text.splitlines()[:24])
        if "@file" not in head or "@brief" not in head:
            failures.append(f"missing @file/@brief header: {relative}")

    # R2：data 是纯值类型，不得泄漏 Arduino、RTOS 或上层依赖。
    data = {k: v for k, v in files.items() if k.startswith(("data/", "data\\"))}
    failures += _lines(data, _include_pattern("Arduino", "FreeRTOS", "freertos/",
                                              r"\.\./(hal|safety|system|tasks)/"))

    # R3：HAL 只封装物理外设，可豁免 system/log 与 system/rtos 适配头。
    hal = {k: v for k, v in files.items() if k.startswith("hal/")}
    for relative in hal:
        for number, line in enumerate(hal[relative].splitlines(), 1):
            if re.match(r"^\s*#include.*\.\./\.\./(safety|system|tasks)/", line) and \
                    re.search(r"\.\./(safety|tasks)/", line):
                failures.append(f"{relative}:{number}: HAL depends on business layer")

    # R4：system 基础设施只提供 RTOS/时钟/日志/通道/端口适配。
    infra = {k: v for k, v in files.items()
             if k.startswith(("system/identity/", "system/rtos/", "system/time/",
                              "system/channels/", "system/ports/", "system/adapters/"))
             or k.rsplit("/", 1)[-1] in {"TaskRuntimeWindow.h", "TaskRuntimeWindow.cpp",
                                         "ServiceCyclePolicy.h", "ServiceCyclePolicy.cpp",
                                         "SystemRuntimeContext.h"}}
    failures += _lines(infra, _include_pattern(r"\.\./(hal|safety|tasks)/"))
    system_log = {k: v for k, v in files.items() if k.startswith("system/log/")}
    failures += _lines(system_log, re.compile(r"^\s*#include.*\.\./\.\./hal/(?!serial/)"))

    # FreeRTOS 原生头只允许适配头直接包含。
    for relative, text in files.items():
        if relative == "system/rtos/FreeRtosKernel.h":
            continue
        for number, line in enumerate(text.splitlines(), 1):
            if FREERTOS_INCLUDE.match(line):
                failures.append(f"{relative}:{number}: FreeRTOS header bypasses platform adapter")

    failures += _lines(files, re.compile(r"\bp4::hal::rtos\b|\bP4_HAL_RTOS_"))

    # R5：safety/system 不得包含 task 实现，也不得自己创建 RTOS 任务。
    regulated = {k: v for k, v in files.items()
                 if (k.startswith("safety/") or k.startswith("system/"))
                 and not k.rsplit("/", 1)[-1].startswith(("SystemAssembly.", "AppBootstrap.",
                                                          "TaskFactory."))}
    failures += _lines(regulated, _include_pattern("tasks/"))
    failures += _lines(regulated, re.compile(r"\b(p4::platform::rtos::createTask|xTaskCreate|vTaskDelete)\b"))
    # 时钟适配与度量采集允许包装延时；其余 safety/system 使用任务注入的时间。
    timed = {k: v for k, v in regulated.items()
             if not k.startswith(("system/time/", "system/rtos/"))
             and not k.rsplit("/", 1)[-1].startswith("TaskRuntimeMetrics.")}
    failures += _lines(timed, re.compile(r"\b(millis|vTaskDelay|vTaskDelayUntil|xTaskGetTickCount)\s*\("))

    # R6：*Task.cpp 是薄调度入口，预算只约束入口规模。
    budgets = {"HostCommTask.cpp": 80, "ServoBusTask.cpp": 90}
    for relative, text in files.items():
        name = relative.rsplit("/", 1)[-1]
        if not name.endswith("Task.cpp") or not relative.startswith("tasks/"):
            continue
        limit = budgets.get(name, 60)
        if len(text.splitlines()) > limit:
            failures.append(f"task adapter exceeds review budget: {relative}")

    # R6.2：服务端口不得泄漏共享上下文或 RTOS 句柄。
    boundary = {k: v for k, v in files.items()
                if k in {"data/ServiceSnapshots.h", "system/ports/ServiceOutputPort.h",
                         "system/ports/SystemEventPort.h"}}
    failures += _lines(boundary, re.compile(r"\b(TaskSharedData_t|QueueHandle_t|SemaphoreHandle_t|TaskHandle_t)\b"))

    failures += _lines(files, re.compile(r"\bTaskSharedData_t\b|\bSystemContext\.h\b"))
    failures += _lines(files, re.compile(
        r"\b(runControlService|runServoRuntimeService|runSystemStateCoordinator|"
        r"runUpperCommService|runSensorIngressService|runRuntimeMonitorService|"
        r"runPersistenceService)\b"))

    # R7：共享上下文只组合聚合状态，不得回退成散落业务字段。
    context_path = SOURCE_ROOT / "system" / "SystemRuntimeContext.h"
    if not context_path.is_file():
        failures.append("SystemRuntimeContext.h is missing")
    else:
        context = context_path.read_text(encoding="utf-8")
        for base in ("InterTaskChannels", "CommandRuntimeState", "ControlRuntimeState",
                     "ServoRuntimeState", "CalibrationRuntimeState"):
            if base not in context:
                failures.append(f"SystemContext lost required state aggregate: {base}")
        match = re.search(r"struct\s+SystemRuntimeContext\s*:[^{]+\{(?P<body>[\s\S]*?)\};", context)
        if match is None:
            failures.append("SystemRuntimeContext composition declaration is missing")
        elif ";" in match.group("body"):
            failures.append("SystemRuntimeContext contains business fields instead of aggregates")

    # R8：飞特 SDK、TWAI 与舵机目标邮箱只能走各自唯一入口。
    failures += _lines(files, SERVO_SDK, exclude=(
        "FeetechServoDriver.h", "FeetechServoDriver.cpp"))
    failures += _lines(files, TWAI_API, exclude=("TwaiCanDriver.h", "TwaiCanDriver.cpp"))
    failures += _lines(files, SERVO_TARGET_QUEUE, exclude=("ActuatorCommandGateway.cpp",))

    servo_service = files.get("tasks/servo_bus/ServoBusService.cpp", "")
    if "p4::actuator::evaluateCommand" not in servo_service:
        failures.append("servo execution path bypasses ActuatorSafetyPolicy::evaluateCommand")
    driver = files.get("hal/servo/FeetechServoDriver.h", "")
    for symbol in ("validateAbsoluteTarget", "servoHardwareCommandFromAbs"):
        if symbol not in driver:
            failures.append(f"Feetech driver lost final hardware range guard: {symbol}")

    # R9：已删除的旧映射运行时不得回流。
    failures += _lines(files, re.compile(
        r"\b(mapEncoderWindowToUrdf|mapJointCalibrationEncoderToUrdf|mapCalibratedEncoderToDeg|"
        r"encoderAtUrdfMin|encoderAtUrdfMax)\b|urdf_joint_mapping_v2"))

    # 安全事件必须经 SafetyEventReporter 上报，禁止业务直接写事件存储。
    failures += _lines(files, SAFETY_WRITE, exclude=(
        ("", "/SafetyEventStore.h"), ("", "/SafetyEventStore.cpp"),
        ("", "/SafetyEventReporter.cpp"), ("", "/ServiceContextAdapter.cpp"),
        ("", "/ServiceOutputPort.h")))

    # 产品指纹：JSON 与固件头必须同步；--firmware-only 时跳过源码/参数 CRC。
    failures += _fingerprint_failures()
    failures += _safety_event_contract_failures(files)

    desktop_parser = ROOT / "desktop" / "core" / "protocol.py"
    if not desktop_parser.is_file() or "parse_safety_event_packet" not in desktop_parser.read_text(encoding="utf-8"):
        failures.append("desktop safety event parser is missing")
    return failures


def _fingerprint_failures(firmware_only: bool = False) -> list[str]:
    from json import loads

    data_path = DESKTOP_CONFIG / "product_fingerprint.json"
    header_path = SOURCE_ROOT / "generated" / "ProductFingerprint.generated.h"
    if not data_path.is_file() or not header_path.is_file():
        return ["product fingerprint contract is missing"]
    fingerprint = loads(data_path.read_text(encoding="utf-8"))
    header = header_path.read_text(encoding="utf-8")
    fields = {
        "control_model_crc": "kProductControlModelCrc",
        "visual_model_crc": "kProductVisualModelCrc",
        "hardware_map_crc": "kProductHardwareMapCrc",
        "parameter_crc": "kProductParameterCrc",
        "firmware_source_crc": "kProductFirmwareSourceCrc",
        "structure_commit_crc": "kProductStructureCommitCrc",
    }
    failures = []
    for key, symbol in fields.items():
        if firmware_only and key in {"parameter_crc", "firmware_source_crc"}:
            continue
        expected = str(fingerprint.get(key, "")).upper()
        match = re.search(rf"{symbol}\s*=\s*0x([0-9A-Fa-f]{{8}})UL", header)
        if match is None or match.group(1).upper() != expected:
            failures.append(f"product fingerprint mismatch: {key}")
    return failures


def _safety_event_contract_failures(files: dict[str, str]) -> list[str]:
    """一次安全事实要能经缓存、协议和上位机解析完整送达。"""
    required = {
        "safety/SafetyEventStore.h": "copySafetyEventsAfter",
        "tasks/host_comm/HostTelemetryPublisher.cpp": "sendSafetyEventPacket",
        "generated/UpperProtocol.generated.h": "PACKET_TYPE_SAFETY_EVENT",
    }
    failures = []
    for relative, symbol in required.items():
        text = files.get(relative)
        if text is None:
            path = SOURCE_ROOT / relative
            text = path.read_text(encoding="utf-8") if path.is_file() else ""
        if symbol not in text:
            failures.append(f"safety event contract is incomplete: {relative} -> {symbol}")
    return failures


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware-only", action="store_true",
                        help="跳过 Desktop 侧的源码/参数指纹比对")
    args = parser.parse_args(argv)
    failures = collect_failures()
    if args.firmware_only:
        # 固件独立变更时只比对稳定模型/硬件契约。
        failures += _fingerprint_failures(firmware_only=True)
        failures = [item for item in failures
                    if item.split(":", 1)[0] != "product fingerprint mismatch"]
    if failures:
        print("P4 architecture check FAILED:")
        for failure in failures:
            print(f" - {failure}")
        return 1
    print(f"P4 architecture check passed ({len(_source_files())} C/C++ files).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
