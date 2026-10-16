#!/usr/bin/env python3
"""从 P4 SafetyRuleCatalog.def 生成固件枚举及离线安全审计数据。

P4 枚举头、上位机英文目录和完整调用索引都以同一个 .def 文件为真值源。
调用索引只生成 JSON，不再把无运行时消费者的字符串表编进固件。
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import re
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT
    / "ESP32-P4"

    / "src"
    / "config"
    / "safety"
    / "SafetyRuleCatalog.def"
)
OUTPUT = ROOT / "desktop" / "resources" / "config" / "p4_safety_rules.json"
FIRMWARE_HEADER = (
    ROOT
    / "ESP32-P4"

    / "src"
    / "generated"
    / "SafetyRuleCatalog.generated.h"
)
FIRMWARE_INDEX_JSON = (
    ROOT
    / "ESP32-P4"

    / "src"
    / "generated"
    / "SafetyRuleIndex.generated.json"
)
OBSOLETE_FIRMWARE_INDEX_FILES = (
    FIRMWARE_INDEX_JSON.with_suffix(".h"),
    FIRMWARE_INDEX_JSON.with_suffix(".cpp"),
)

_ROW_RE = re.compile(r"^P4_SAFETY_RULE\((.*)\)\s*$")
# 旧固件自动标定编号继续解释历史日志，不再登记运行调用链。
# CAW 是上传 LUT 的装配质量告警，继续保留活动绑定。
RETIRED_RULE_NAMES = frozenset({
    "SystemCalibrationStateRejected",
    "FaultCalibration",
})
_ACTION_TEXT = {
    "Allow": "Allow",
    "RejectCommand": "Reject command",
    "HoldActualPosition": "Hold current position",
    "DisableTorque": "Disable torque",
    "EnterFault": "Enter fault state",
    "ClampToBoundary": "Clamp to workspace boundary",
    "MarkUncontrollable": "Mark system uncontrollable",
    "MarkIdle": "Return to IDLE",
    "StopMotion": "Stop current motion",
    "WarnOnly": "Report warning only",
}


@dataclass(frozen=True)
class Probe:
    """描述一条可由静态检查验证的源码路径和函数/符号。"""

    path: str
    token: str


@dataclass(frozen=True)
class RuleBinding:
    """描述一条规则从 Policy 到业务动作、事件和恢复入口的完整路径。"""

    policy: Probe
    caller: Probe
    event: Probe
    recovery: Probe
    mapping: Probe | None = None


def binding(
    policy_path: str,
    policy_token: str,
    caller_path: str,
    caller_token: str,
    event_path: str,
    event_token: str,
    recovery_path: str,
    recovery_token: str,
    mapping_token: str | None = None,
) -> RuleBinding:
    """构造规则绑定；只有需要枚举转换的规则才经过 SafetyRuleRegistry。"""

    mapping = None
    if mapping_token:
        mapping = Probe("safety/SafetyRuleRegistry.cpp", mapping_token)
    return RuleBinding(
        Probe(policy_path, policy_token),
        Probe(caller_path, caller_token),
        Probe(event_path, event_token),
        Probe(recovery_path, recovery_token),
        mapping,
    )


# 规则族默认绑定是调用索引的唯一人工维护源。生成固件索引、闭环检查都读取本表。
GROUP_BINDINGS: dict[str, RuleBinding] = {
    "ACT": binding(
        "safety/policies/ActuatorSafetyPolicy.cpp", "evaluateCommand",
        "tasks/servo_bus/ServoBusService.cpp", "p4::actuator::evaluateCommand",
        "safety/SafetyEventReporter.cpp", "reportServoSafetyDecision",
        "safety/policies/ActuatorSafetyPolicy.cpp", "ServoRejectReason::None",
        "ruleForServoRejectReason"),
    "CTL": binding(
        "safety/policies/ControlInputSafetyPolicy.cpp", "evaluateControlSafety",
        "tasks/motion_control/MotionControlService.cpp", "evaluateControlSafety",
        "tasks/motion_control/MotionControlService.cpp", "reportSafetyRuleEvent",
        "tasks/motion_control/MotionControlService.cpp", "clearRecoveredControlRule"),
    "SYS": binding(
        "safety/policies/StateAdmissionPolicy.cpp", "canAcceptStart",
        "tasks/system_state/SystemEventDispatcher.cpp", "canAcceptStart",
        "tasks/system_state/SystemEventDispatcher.cpp", "reportSafetyRuleEvent",
        "tasks/system_state/SystemEventDispatcher.cpp", "SYSTEM_EVENT_RESET"),
    "DIR": binding(
        "safety/policies/SystemReadinessPolicy.cpp", "directMotorMaskReady",
        "tasks/system_state/SystemEventDispatcher.cpp", "directMotorMaskReady",
        "tasks/system_state/DirectCommandCoordinator.cpp", "reportSafetyRuleEvent",
        "tasks/system_state/DirectCommandCoordinator.cpp", "DIRECT_REJECT_NONE",
        "ruleForDirectRejectReason"),
    "FBK": binding(
        "safety/policies/SystemReadinessPolicy.cpp", "evaluateFeedbackReadiness",
        "tasks/system_state/SystemEventDispatcher.cpp", "evaluateFeedbackReadiness",
        "tasks/system_state/SystemEventDispatcher.cpp", "reportSafetyRuleEvent",
        "tasks/system_state/SystemEventDispatcher.cpp", "FEEDBACK_REJECT_NONE",
        "ruleForFeedbackRejectReason"),
    "RDY": binding(
        "safety/policies/SystemReadinessPolicy.cpp", "evaluateFeedbackReadiness",
        "tasks/system_state/SystemEventDispatcher.cpp", "evaluateFeedbackReadiness",
        "tasks/system_state/FeedbackReadyDiagnostics.cpp", "reportSafetyRuleEvent",
        "tasks/system_state/SystemEventDispatcher.cpp", "SYSTEM_EVENT_REEVALUATE_READY",
        "ruleForFeedbackReadyRejectReason"),
    "ROP": binding(
        "safety/policies/RopeInitSafetyPolicy.cpp", "evaluateRopeInitSession",
        "tasks/system_state/RopeInitController.cpp", "evaluateRopeInitSession",
        "tasks/system_state/RopeInitController.cpp", "reportSafetyRuleEvent",
        "tasks/system_state/RopeInitController.cpp", "ROPE_INIT_REASON_NONE",
        "ruleForRopeInitReason"),
    "FLT": binding(
        "safety/policies/SystemFaultPolicy.cpp", "collectSystemFaults",
        "tasks/system_state/SystemStateCoordinator.cpp", "collectSystemFaults",
        "tasks/system_state/SystemEventDispatcher.cpp", "reportSafetyRuleEvent",
        "tasks/system_state/SystemEventDispatcher.cpp", "SYSTEM_EVENT_RESET",
        "ruleForSystemFaultBit"),
    "CAW": binding(
        "safety/policies/CalibrationSafetyPolicy.h", "calibrationAssemblyWarningReasonForMetrics",
        "system/calibration/CalibrationRuntimeStore.cpp", "calibrationAssemblyWarningReasonForMetrics",
        "system/calibration/CalibrationRuntimeStore.cpp", "reportSafetyRuleEvent",
        "system/calibration/CalibrationRuntimeStore.cpp", "CALIB_WARN_NONE",
        "ruleForCalibrationWarningBit"),
    "COM": binding(
        "safety/policies/CommunicationSafetyPolicy.cpp", "communicationLengthValid",
        "tasks/s3_can_rx/S3CanRxService.cpp", "communicationLengthValid",
        "tasks/s3_can_rx/S3CanRxService.cpp", "reportSafetyRuleEvent",
        "tasks/s3_can_rx/EncoderHealthMonitor.cpp", "updateEncoderHealthFromSnapshot"),
    "RUN": binding(
        "safety/policies/RuntimeSafetyPolicy.cpp", "evaluateRuntimeOverrun",
        "tasks/runtime_monitor/RuntimeMonitorService.cpp", "evaluateRuntimeOverrun",
        "tasks/runtime_monitor/RuntimeMonitorService.cpp", "reportSafetyRuleEvent",
        "tasks/runtime_monitor/RuntimeMonitorService.cpp", "overrunEventLatched"),
}


# 只有执行路径不同的规则才覆盖规则族默认项，避免复制其余 101 条规则。
RULE_BINDING_OVERRIDES: dict[str, RuleBinding] = {
    "SystemHostLeaseExpired": binding(
        "safety/policies/HostLeaseSafetyPolicy.cpp", "isHostControlLeaseFresh",
        "tasks/system_state/HostLeaseSupervisor.cpp", "isHostControlLeaseFresh",
        "tasks/system_state/HostLeaseSupervisor.cpp", "reportSafetyRuleEvent",
        "tasks/system_state/HostLeaseSupervisor.cpp", "resetHostControlLeaseRuntime"),
    "ControlCollisionHold": binding(
        "safety/policies/CollisionSafetyPolicy.cpp", "FeedbackCollisionGuard::update",
        "tasks/motion_control/MotionControlService.cpp", "g_feedbackCollisionGuard.update",
        "tasks/motion_control/MotionControlService.cpp", "ControlCollisionHold",
        "safety/policies/CollisionSafetyPolicy.cpp", "holdReleased = true"),
    "ControlLutBoundaryClamp": binding(
        "control/JointFeedbackController.cpp", "out.boundaryClamped = true",
        "tasks/motion_control/MotionControlService.cpp", "result.boundaryClamped",
        "tasks/motion_control/MotionControlService.cpp", "ControlLutBoundaryClamp",
        "tasks/motion_control/MotionControlService.cpp", "lutClampEventLatchedBitmap"),
    "ControlJointTargetClamp": binding(
        "safety/policies/ControlBoundarySafetyPolicy.cpp", "clampFloatToBoundary",
        "tasks/motion_control/MotionControlService.cpp", "clampJointTargetDeg",
        "tasks/motion_control/MotionControlService.cpp", "ControlJointTargetClamp",
        "tasks/motion_control/MotionControlService.cpp", "targetClampEventLatchedBitmap"),
    "ControlUpperWorkspaceWarning": binding(
        "safety/policies/ControlBoundarySafetyPolicy.cpp", "evaluateUpperWorkspaceWarning",
        "tasks/motion_control/MotionControlService.cpp", "evaluateUpperWorkspaceWarning",
        "tasks/motion_control/MotionControlService.cpp", "ControlUpperWorkspaceWarning",
        "tasks/motion_control/MotionControlService.cpp", "upperWorkspaceWarningBitmap"),
    "ControlHotplugHold": binding(
        "safety/policies/ControlBoundarySafetyPolicy.cpp", "shouldEnterHotplugHold",
        "tasks/motion_control/MotionControlService.cpp", "shouldEnterHotplugHold",
        "tasks/motion_control/MotionControlService.cpp", "ControlHotplugHold",
        "tasks/motion_control/MotionControlService.cpp", "hotplugHoldMotor"),
    "CommunicationCanInitFailed": binding(
        "safety/policies/CommunicationSafetyPolicy.cpp", "communicationCanInitializationRule",
        "tasks/s3_can_rx/S3CanRxService.cpp", "communicationCanInitializationRule",
        "tasks/s3_can_rx/S3CanRxService.cpp", "reportSafetyRuleEvent",
        "tasks/s3_can_rx/S3CanRxService.cpp", "canInitialized"),
    "CommunicationUpperFrameRejected": binding(
        "safety/policies/CommunicationSafetyPolicy.cpp", "communicationEventMayReport",
        "tasks/host_comm/HostCommService.cpp", "communicationEventMayReport",
        "tasks/host_comm/HostCommService.cpp", "reportSafetyRuleEvent",
        "tasks/host_comm/HostFrameReader.cpp", "return true"),
    "RuntimeSafetyEventCacheOverflow": binding(
        "safety/policies/RuntimeSafetyPolicy.cpp", "runtimeEventCacheWillOverflow",
        "safety/SafetyEventStore.cpp", "runtimeEventCacheWillOverflow",
        "safety/SafetyEventStore.cpp", "RuntimeSafetyEventCacheOverflow",
        "safety/SafetyEventStore.cpp", "s_overflowEvidenceRecorded"),
    "RuntimeQueueCreateFailed": binding(
        "safety/policies/RuntimeSafetyPolicy.cpp", "runtimeResourceCreationRule",
        "system/SystemAssembly.cpp", "RuntimeResourceKind::Queue",
        "system/SystemAssembly.cpp", "recordAssemblyFailure",
        "system/SystemAssembly.cpp", "queuesCreated"),
    "RuntimeMutexCreateFailed": binding(
        "safety/policies/RuntimeSafetyPolicy.cpp", "runtimeResourceCreationRule",
        "system/SystemAssembly.cpp", "RuntimeResourceKind::Mutex",
        "system/SystemAssembly.cpp", "recordAssemblyFailure",
        "system/SystemAssembly.cpp", "commandStateMutex"),
    "RuntimeTaskCreateFailed": binding(
        "safety/policies/RuntimeSafetyPolicy.cpp", "runtimeResourceCreationRule",
        "system/SystemAssembly.cpp", "RuntimeResourceKind::Task",
        "system/SystemAssembly.cpp", "recordAssemblyFailure",
        "system/SystemAssembly.cpp", "tasksCreated"),
    "RuntimeTaskStackLow": binding(
        "safety/policies/RuntimeSafetyPolicy.cpp", "evaluateRuntimeStack",
        "tasks/runtime_monitor/RuntimeMonitorService.cpp", "evaluateRuntimeStack",
        "tasks/runtime_monitor/RuntimeMonitorService.cpp", "RuntimeTaskStackLow",
        "tasks/runtime_monitor/RuntimeMonitorService.cpp", "stackEventLatched"),
}


def _unquote(value: str) -> str:
    value = value.strip()
    return json.loads(value) if value.startswith('"') else value


def load_rule_definitions() -> list[dict]:
    """解析宏目录；保留枚举名供固件调用索引生成和闭环检查使用。"""

    rules: list[dict] = []
    ids: set[int] = set()
    codes: set[str] = set()
    for line_number, raw in enumerate(SOURCE.read_text(encoding="utf-8").splitlines(), 1):
        match = _ROW_RE.match(raw.strip())
        if not match:
            continue
        parts = next(csv.reader(io.StringIO(match.group(1)), skipinitialspace=True))
        if len(parts) != 8:
            raise ValueError(f"{SOURCE}:{line_number}: expected 8 fields, got {len(parts)}")
        name, value, code, title, severity, layer, action, recovery = parts
        rule_id = int(value.strip())
        code_text = _unquote(code)
        if rule_id in ids or code_text in codes:
            raise ValueError(f"duplicate safety rule at {SOURCE}:{line_number}")
        ids.add(rule_id)
        codes.add(code_text)
        rules.append(
            {
                "name": name.strip(),
                "rule_id": rule_id,
                "code": code_text,
                "title": _unquote(title),
                "severity": severity.strip(),
                "layer": layer.strip(),
                "action": _ACTION_TEXT.get(action.strip(), action.strip()),
                "recovery": _unquote(recovery),
            }
        )
    return rules


def load_rules() -> list[dict]:
    """返回上位机公开目录，不暴露仅供 C++ 生成使用的枚举名。"""

    return [
        {key: value for key, value in rule.items() if key != "name"}
        for rule in load_rule_definitions()
    ]


def resolve_rule_binding(rule: dict) -> RuleBinding | None:
    """按单条覆盖优先、规则族默认次之的顺序解析调用链。"""

    if is_retired_rule(rule):
        return None
    return RULE_BINDING_OVERRIDES.get(
        str(rule["name"]),
        GROUP_BINDINGS.get(str(rule["code"]).split("-", 1)[0]),
    )


def is_retired_rule(rule: dict) -> bool:
    """只有明确移除的固件自动标定规则退役，结果质量告警仍活动。"""

    return (str(rule["code"]).startswith("CAL-")
            or str(rule["name"]) in RETIRED_RULE_NAMES)


def render() -> str:
    payload = {
        "schema_version": 2,
        "description": "Generated from P4 SafetyRuleCatalog.def; do not edit manually",
        "rules": load_rules(),
    }
    return json.dumps(payload, ensure_ascii=False, indent=2) + "\n"


def render_firmware_header() -> str:
    """生成 Arduino 构建器能够稳定收集的 X-Macro 标准头文件。

    `.def` 仍是唯一人工维护源；固件只包含生成的 `.h`，避免 Arduino 在临时
    构建目录中遗漏非标准扩展文件。该头文件需要被枚举和注册表重复包含，
    因此刻意不设置 include guard。
    """
    rows = [
        line.strip()
        for line in SOURCE.read_text(encoding="utf-8").splitlines()
        if _ROW_RE.match(line.strip())
    ]
    return "\n".join(
        [
            "/**",
            " * @file SafetyRuleCatalog.generated.h",
            " * @brief 由 SafetyRuleCatalog.def 自动生成的 P4 安全规则 X-Macro 清单。",
            " * @warning 禁止手工编辑；本文件需要被多次包含，因此不得添加 include guard。",
            " */",
            "#ifndef P4_SAFETY_RULE",
            '#error "Define P4_SAFETY_RULE before including SafetyRuleCatalog.generated.h"',
            "#endif",
            "",
            *rows,
            "",
        ]
    )


def render_firmware_index_json() -> str:
    """生成完整调用索引供 CI/审计使用，不进入 P4 固件镜像。"""

    items: list[dict] = []
    for rule in load_rule_definitions():
        if is_retired_rule(rule):
            items.append({
                "name": rule["name"],
                "rule_id": rule["rule_id"],
                "code": rule["code"],
                "active": False,
                "retired": True,
                "retirement_reason": "Automatic calibration runs on the host; retained for historical logs.",
            })
            continue
        contract = resolve_rule_binding(rule)
        if contract is None:
            raise ValueError(f"missing safety binding for {rule['code']} {rule['name']}")
        items.append(
            {
                "name": rule["name"],
                "rule_id": rule["rule_id"],
                "code": rule["code"],
                "active": True,
                "retired": False,
                "policy": {
                    "file": contract.policy.path,
                    "symbol": contract.policy.token,
                },
                "caller": {
                    "file": contract.caller.path,
                    "symbol": contract.caller.token,
                },
                "event": {
                    "file": contract.event.path,
                    "symbol": contract.event.token,
                },
                "recovery": {
                    "file": contract.recovery.path,
                    "symbol": contract.recovery.token,
                },
                "registry_function": (
                    contract.mapping.token if contract.mapping else ""
                ),
            }
        )
    payload = {
        "schema_version": 1,
        "description": "P4 安全规则调用索引；由生成器维护，仅供离线审计",
        "_format_help": {
            "notice": "标准 JSON 不支持 // 或 /* */ 注释；本对象是可被解析器安全忽略的格式说明。",
            "generation": "本文件由 tools/generate_p4_safety_catalog.py 自动生成，禁止手工编辑。",
            "rule_fields": {
                "name": "C++ SafetyRuleId 枚举名称，用于源码内稳定引用规则。",
                "rule_id": "规则的稳定整数编号；协议、日志和上位机按此编号关联。",
                "code": "面向人员的规则编码；前缀表示规则所属模块，例如 ACT、CTL、SYS。",
                "active": "是否仍存在固件运行调用链；false 表示仅保留历史日志编号。",
                "retired": "旧固件自动标定规则已退役，不再存在 policy/caller/event/recovery 绑定。",
                "policy": "规则判断实现位置；file 是源码路径，symbol 是判断函数或关键符号。",
                "caller": "调用 Policy 的业务入口，说明该规则在哪条运行链路中真正执行。",
                "event": "规则触发后提交诊断事件或故障证据的位置。",
                "recovery": "规则解除、恢复或重新准入的代码入口。",
                "registry_function": "业务拒绝原因映射到 SafetyRuleId 的函数；空字符串表示 Policy 直接返回规则 ID。"
            },
            "reference_fields": {
                "file": "相对于 ESP32-P4/src 的源码路径。",
                "symbol": "检查器在对应源码中核验的函数名、枚举值或关键符号。"
            },
            "code_prefixes": {
                "ACT": "舵机执行、路由与目标边界规则。",
                "CTL": "PID、碰撞、过载和闭环控制规则。",
                "SYS": "系统状态机、E-STOP、运动所有权和租约规则。",
                "CAL": "自动标定流程及标定结果规则。",
                "RCV": "Rope Init、多圈、热插拔和恢复规则。",
                "COM": "CAN、串口、协议和数据完整性规则。",
                "RUN": "任务、队列、周期、栈和运行时资源规则。",
                "WRN": "不阻断控制的诊断警告。"
            }
        },
        "rules": items,
    }
    return json.dumps(payload, ensure_ascii=False, indent=2) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="只检查生成文件是否最新")
    parser.add_argument(
        "--firmware-only", action="store_true",
        help="只生成或检查 P4 目录头和调用索引，保留 Desktop 目录",
    )
    args = parser.parse_args()
    expected = render()
    expected_header = render_firmware_header()
    expected_index_json = render_firmware_index_json()
    if args.check:
        stale = False
        if not args.firmware_only and (
            not OUTPUT.exists() or OUTPUT.read_text(encoding="utf-8") != expected
        ):
            print(f"stale generated file: {OUTPUT}")
            stale = True
        if (
            not FIRMWARE_HEADER.exists()
            or FIRMWARE_HEADER.read_text(encoding="utf-8") != expected_header
        ):
            print(f"stale generated file: {FIRMWARE_HEADER}")
            stale = True
        if (
            not FIRMWARE_INDEX_JSON.exists()
            or FIRMWARE_INDEX_JSON.read_text(encoding="utf-8") != expected_index_json
        ):
            print(f"stale generated file: {FIRMWARE_INDEX_JSON}")
            stale = True
        for path in OBSOLETE_FIRMWARE_INDEX_FILES:
            if path.exists():
                print(f"obsolete generated file: {path}")
                stale = True
        return 1 if stale else 0
    if not args.firmware_only:
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT.write_text(expected, encoding="utf-8")
    FIRMWARE_HEADER.parent.mkdir(parents=True, exist_ok=True)
    FIRMWARE_HEADER.write_text(expected_header, encoding="utf-8")
    FIRMWARE_INDEX_JSON.write_text(expected_index_json, encoding="utf-8")
    if not args.firmware_only:
        for path in OBSOLETE_FIRMWARE_INDEX_FILES:
            path.unlink(missing_ok=True)
    if not args.firmware_only:
        print(f"generated {OUTPUT}")
    print(f"generated {FIRMWARE_HEADER}")
    print(f"generated {FIRMWARE_INDEX_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
