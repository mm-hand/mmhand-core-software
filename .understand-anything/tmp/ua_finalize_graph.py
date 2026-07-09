import json
import pathlib
from datetime import datetime, timezone

ROOT = pathlib.Path(r"D:\hand\mmhand-cl (3)\mmhand-cl")
INTER = ROOT / ".understand-anything" / "intermediate"
scan = json.loads((INTER / "scan-result.json").read_text(encoding="utf-8-sig"))
assembled = json.loads((INTER / "assembled-graph.json").read_text(encoding="utf-8-sig"))

nodes = assembled.get("nodes", [])
edges = assembled.get("edges", [])
node_ids = {n["id"] for n in nodes if "id" in n}
file_level = {"file", "config", "document", "service", "pipeline", "table", "schema", "resource", "endpoint"}


def layer_for(node):
    path = node.get("filePath", "")
    t = node.get("type")
    if path.startswith("ESP32-P4/"):
        return "layer:p4-firmware"
    if path.startswith("ESP32-S3/"):
        return "layer:s3-sensor-firmware"
    if path.startswith("desktop/"):
        if path.startswith("desktop/resources/"):
            return "layer:desktop-resources"
        return "layer:desktop-app"
    if path.startswith("manus/"):
        return "layer:manus-integration"
    if path.startswith("docs/") or t == "document":
        return "layer:documentation"
    if path.startswith("tests/"):
        return "layer:tests"
    if path.startswith("playground/") or path.startswith("tools/") or path == "data_plot.py":
        return "layer:tools-playground"
    return "layer:project-root"


layer_defs = {
    "layer:p4-firmware": ("ESP32-P4 主控固件", "P4 侧状态机、实时控制、舵机通信、CAN 汇聚、标定与多圈持久化。"),
    "layer:s3-sensor-firmware": ("ESP32-S3 传感固件", "S3 侧编码器、触觉、CAN/TWAI 和 FreeRTOS 传感任务。"),
    "layer:desktop-app": ("Python 上位机", "桌面 UI、CLI、串口通信、状态模型、ROS2 输入、遥操作和可视化逻辑。"),
    "layer:desktop-resources": ("上位机资源与配置", "URDF、mesh、标定快照、手部几何和运行配置等数据资产。"),
    "layer:manus-integration": ("Manus 集成", "Manus SDK 示例、桥接测试、UDP/TCP 输出和 C++ 包装层。"),
    "layer:documentation": ("文档与协议说明", "项目 README、通信协议、硬件接线、调试流程和历史问题说明。"),
    "layer:tests": ("测试", "desktop app/core/ros2 相关测试，用于约束入口、协议和业务逻辑。"),
    "layer:tools-playground": ("工具与实验", "独立绘图、触觉实验草图、扫描脚本和调试辅助工具。"),
    "layer:project-root": ("项目根与杂项", "根目录配置、辅助文件和未归入其他层的项目节点。"),
}

layer_nodes = {k: [] for k in layer_defs}
for n in nodes:
    if n.get("type") in file_level:
        layer_nodes.setdefault(layer_for(n), []).append(n["id"])

layers = []
for lid, ids in layer_nodes.items():
    if not ids:
        continue
    name, desc = layer_defs.get(lid, (lid, "项目结构层。"))
    layers.append({"id": lid, "name": name, "description": desc, "nodeIds": sorted(set(ids))})


def existing(*ids):
    return [i for i in ids if i in node_ids]


tour_candidates = [
    (
        "项目总览",
        "先看 README 和当前分支说明，理解仓库目标、硬件组成和近期改动。",
        existing("document:README.md", "document:desktop/README.md", "document:ESP32-P4/ServoBoardMain/readme.md", "document:ESP32-S3/readme.md"),
    ),
    (
        "P4 主控启动与任务编排",
        "从共享数据和 System_Init 开始，理解队列、任务、状态字段如何把 P4 固件组织起来。",
        existing(
            "file:ESP32-P4/ServoBoardMain/src/shared/TaskSharedData.h",
            "file:ESP32-P4/ServoBoardMain/src/system/SystemTask.cpp",
            "file:ESP32-P4/ServoBoardMain/src/system/StateMachineTask.cpp",
        ),
    ),
    (
        "控制闭环与安全",
        "阅读控制任务、求解器、安全保护和输出构建，理解目标角如何变成舵机 batch。",
        existing(
            "file:ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp",
            "file:ESP32-P4/ServoBoardMain/src/control/ControlSolver.cpp",
            "file:ESP32-P4/ServoBoardMain/src/control/ControlSafety.cpp",
            "file:ESP32-P4/ServoBoardMain/src/control/ControlOutputBuilder.cpp",
        ),
    ),
    (
        "通信协议链路",
        "对照 P4 上位通信协议和 desktop protocol.py，确认命令字、包类型和状态字段如何跨端一致。",
        existing(
            "file:ESP32-P4/ServoBoardMain/src/communication/upper/UpperCommProtocol.h",
            "file:ESP32-P4/ServoBoardMain/src/communication/upper/UpperCommTask.cpp",
            "file:desktop/core/protocol.py",
            "file:desktop/core/comm_layer.py",
        ),
    ),
    (
        "S3 传感采集",
        "从 S3 配置、编码器 HAL、CAN 任务到 P4 CAN 接收，理解磁编码器和触觉数据上报路径。",
        existing(
            "file:ESP32-S3/System/src/config/Config.h",
            "file:ESP32-S3/System/src/hal/HalEncoders.cpp",
            "file:ESP32-S3/System/src/tasks/EncodersTask.cpp",
            "file:ESP32-S3/System/src/tasks/CanTask.cpp",
            "file:ESP32-P4/ServoBoardMain/src/communication/can/CanCommTask.cpp",
        ),
    ),
    (
        "上位机应用层",
        "从 entry、controller、backend 到 UI，理解桌面端如何连接硬件、维护 HandModel 并驱动界面。",
        existing(
            "file:desktop/app/entry.py",
            "file:desktop/core/core_logic.py",
            "file:desktop/core/hand_backend.py",
            "file:desktop/ui/ui_app.py",
        ),
    ),
    (
        "遥操作与 Manus",
        "最后看 Manus 和 teleop 模块，理解外部手套/姿态输入如何进入 mmhand 控制链路。",
        existing(
            "file:desktop/teleop/example_teleop.py",
            "file:desktop/teleop/manus_receiver.py",
            "file:desktop/teleop/manus_raw_skeleton_retarget.py",
            "file:manus/manus_bridge_test/src/main.cpp",
        ),
    ),
]

tour = []
for order, (title, desc, ids) in enumerate([x for x in tour_candidates if x[2]], start=1):
    tour.append({"order": order, "title": title, "description": desc, "nodeIds": ids})

graph = {
    "version": "1.0.0",
    "project": {
        "name": scan["name"],
        "languages": scan["languages"],
        "frameworks": scan["frameworks"],
        "description": scan["description"],
        "analyzedAt": datetime.now(timezone.utc).isoformat(),
        "gitCommitHash": "unknown",
    },
    "nodes": nodes,
    "edges": edges,
    "layers": layers,
    "tour": tour,
}

issues = []
ids = {n.get("id") for n in graph["nodes"]}
for n in graph["nodes"]:
    if not n.get("summary"):
        n["summary"] = "暂无摘要。"
    if not n.get("tags"):
        n["tags"] = ["untagged"]
    if not n.get("complexity"):
        n["complexity"] = "simple"
graph["edges"] = [e for e in graph["edges"] if e.get("source") in ids and e.get("target") in ids and e.get("source") != e.get("target")]
for layer in graph["layers"]:
    layer["nodeIds"] = [i for i in layer["nodeIds"] if i in ids]
for step in graph["tour"]:
    step["nodeIds"] = [i for i in step["nodeIds"] if i in ids]

(INTER / "assembled-graph.json").write_text(json.dumps(graph, ensure_ascii=False, indent=2), encoding="utf-8")
(ROOT / ".understand-anything" / "knowledge-graph.json").write_text(json.dumps(graph, ensure_ascii=False, indent=2), encoding="utf-8")

stats = {
    "nodes": len(graph["nodes"]),
    "edges": len(graph["edges"]),
    "layers": len(graph["layers"]),
    "tourSteps": len(graph["tour"]),
}
(INTER / "review.json").write_text(json.dumps({"issues": issues, "warnings": [], "stats": stats}, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(stats, ensure_ascii=False, indent=2))
