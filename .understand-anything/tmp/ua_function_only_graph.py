import json
import pathlib
import datetime

ROOT = pathlib.Path(r"D:\hand\mmhand-cl (3)\mmhand-cl")
KG = ROOT / ".understand-anything" / "knowledge-graph.json"
BACKUP = ROOT / ".understand-anything" / "knowledge-graph.file-level.json"

g = json.loads(KG.read_text(encoding="utf-8-sig"))
BACKUP.write_text(json.dumps(g, ensure_ascii=False, indent=2), encoding="utf-8")

funcs = {n["id"]: n for n in g["nodes"] if n.get("type") == "function"}
edges = [
    e for e in g["edges"]
    if e.get("type") == "calls" and e.get("source") in funcs and e.get("target") in funcs
]

connected = set()
for e in edges:
    connected.add(e["source"])
    connected.add(e["target"])

nodes = []
for nid, node in funcs.items():
    if nid not in connected:
        continue
    item = dict(node)
    item["summary"] = item.get("summary") or f"函数 {item.get('name', '')}，来自 {item.get('filePath', '')}。"
    item["tags"] = list(dict.fromkeys((item.get("tags") or []) + ["函数路线图"]))[:5]
    nodes.append(item)

subsystems = [
    ("layer:p4-control", "P4 控制闭环", "ESP32-P4/ServoBoardMain/src/control/"),
    ("layer:p4-system", "P4 状态机与任务", "ESP32-P4/ServoBoardMain/src/system/"),
    ("layer:p4-upper", "P4 上位通信协议", "ESP32-P4/ServoBoardMain/src/communication/upper/"),
    ("layer:p4-servo-can", "P4 舵机/CAN 通信", "ESP32-P4/ServoBoardMain/src/communication/"),
    ("layer:p4-calib", "P4 标定与持久化", "ESP32-P4/ServoBoardMain/src/calibration/"),
    ("layer:s3-sensor", "S3 传感采集", "ESP32-S3/System/src/"),
    ("layer:desktop-core", "上位机核心/通信", "desktop/core/"),
    ("layer:desktop-ui", "上位机 UI", "desktop/ui/"),
    ("layer:desktop-teleop", "遥操作/ROS2/算法", "desktop/"),
]

layers = []
assigned = set()
for lid, name, prefix in subsystems:
    ids = sorted([
        n["id"] for n in nodes
        if n.get("filePath", "").startswith(prefix) and n["id"] not in assigned
    ])
    if ids:
        layers.append({
            "id": lid,
            "name": name,
            "description": f"{name}中的函数调用关系。",
            "nodeIds": ids,
        })
        assigned.update(ids)

rest = sorted([n["id"] for n in nodes if n["id"] not in assigned])
if rest:
    layers.append({
        "id": "layer:other-functions",
        "name": "其他函数",
        "description": "未归入主要链路但参与调用关系的函数。",
        "nodeIds": rest,
    })


def ids_by_name(*names):
    wanted = []
    for n in nodes:
        if n.get("name") in names:
            wanted.append(n["id"])
    return wanted[:12]


tour = []
step_defs = [
    ("P4 启动入口", ids_by_name("setup", "System_Init", "System_Loop", "xTaskCreate")),
    ("P4 状态事件路线", ids_by_name("postSystemEvent", "postSystemEventForToken", "stateMachineTask", "setSystemState", "requestEmergencyStop")),
    ("P4 控制路线", ids_by_name("controlTask", "appendDualTendonTargets", "appendJointActuatorHoldTargets", "clampJointTargetDegByCalib")),
    ("P4 通信路线", ids_by_name("upperCommunicationTask", "handleCommand", "sendPacket", "canCommunicationTask", "servoCommunicationTask")),
    ("S3 传感路线", ids_by_name("setup", "startSystemTasks", "Task_Encoders", "Task_CanBus", "readAll", "sendEncoderData")),
    ("上位机命令路线", ids_by_name("connect", "initialize", "send_joint_targets", "set_target_angles", "send_command", "parse_frame")),
]
order = 1
for title, node_ids in step_defs:
    node_ids = [x for x in node_ids if x in connected]
    if node_ids:
        tour.append({
            "order": order,
            "title": title,
            "description": f"{title}：按函数调用关系阅读这条链路。",
            "nodeIds": node_ids,
        })
        order += 1

project = dict(g["project"])
project["name"] = "mmhand-cl 函数调用路线图"
project["description"] = "只保留参与调用关系的函数节点和 calls 边，排除文件级 imports 视图、不可靠文档和冗余旧代码。"
project["analyzedAt"] = datetime.datetime.now(datetime.timezone.utc).isoformat()

function_graph = {
    "version": "1.0.0",
    "project": project,
    "nodes": nodes,
    "edges": edges,
    "layers": layers,
    "tour": tour,
}

KG.write_text(json.dumps(function_graph, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps({
    "nodes": len(nodes),
    "calls": len(edges),
    "layers": len(layers),
    "tour": len(tour),
    "backup": str(BACKUP),
}, ensure_ascii=False, indent=2))
