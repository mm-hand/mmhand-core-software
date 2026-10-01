"""Source-backed graph extraction. Unknown relationships stay explicit."""

from __future__ import annotations

import ast
from pathlib import Path
import re

from source import Sources
from source_scope import configured_path, entrypoint_id


def label(zh: str, en: str) -> dict[str, str]:
    return {"zh": zh, "en": en}


def graph(graph_id: str, zh: str, en: str, category: str, parent: str = "") -> dict:
    return {"id": graph_id, "title": label(zh, en), "category": category,
            "parent": parent, "nodes": [], "edges": []}


def add_node(g: dict, node_id: str, zh: str, en: str, *, kind: str = "process",
             evidence: list[dict] | None = None, details: dict | None = None,
             drilldown: str = "") -> dict:
    node = {"id": node_id, "label": label(zh, en), "kind": kind,
            "evidence": evidence or [], "details": details or {}, "drilldown": drilldown}
    g["nodes"].append(node)
    return node


def add_edge(g: dict, source: str, target: str, zh: str, en: str,
             evidence: list[dict] | None = None, condition: str = "") -> None:
    g["edges"].append({"source": source, "target": target,
                       "label": label(zh, en), "evidence": evidence or [],
                       "condition": condition})


def find_line(sources: Sources, repo: str, path: str, needle: str) -> dict:
    rows = sources.lines(repo, path)
    for i, row in enumerate(rows, 1):
        if needle in row:
            return sources.evidence(repo, path, i)
    raise ValueError(f"Missing expected source {repo}:{path}: {needle}")


def python_function(sources: Sources, path: str, name: str) -> dict | None:
    if path not in sources.paths["mmhand"]:
        return None
    code = sources.read("mmhand", path)
    try:
        tree = ast.parse(code)
    except SyntaxError:
        return None
    candidates = [n for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
                  and n.name == name]
    if len(candidates) != 1:
        return None
    fn = candidates[0]
    conditions = []
    calls = []
    for n in ast.walk(fn):
        if isinstance(n, ast.If):
            expression = ast.get_source_segment(code, n.test) or ast.unparse(n.test)
            conditions.append({"expr": expression.strip(),
                               "evidence": sources.evidence("mmhand", path,
                                                            n.test.lineno, n.test.end_lineno)})
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
            target = ast.get_source_segment(code, n.func)
            if target and (target.startswith(("self.", "panel.", "bridge."))):
                calls.append(target)
    return {"name": name, "evidence": sources.evidence("mmhand", path, fn.lineno),
            "conditions": conditions, "calls": list(dict.fromkeys(calls)),
            "end": fn.end_lineno}


def function_graph(sources: Sources, graph_id: str, items: list[dict]) -> dict:
    titles = {
        "connect": ("连接与状态同步", "Connection and state sync"),
        "control": ("使能、暂停、停止与复位", "Enable, pause, stop and reset"),
        "device_check": ("Device Check 与手动端点", "Device Check and manual endpoints"),
        "joint_channels": ("Joint Channels 负反馈", "Joint Channels feedback"),
        "motor_channels": ("Motor Channels 直控", "Motor Channels direct control"),
        "calibration": ("自动标定", "Automatic calibration"),
        "replay": ("Replay 启动与中止", "Replay start and abort"),
        "ros2": ("ROS2 输入与授权", "ROS2 input and authorization"),
        "recovery": ("故障复位与恢复", "Fault reset and recovery"),
    }
    zh, en = titles[graph_id]
    g = graph(graph_id, zh, en, "behavior", "behavior_overview")
    root = add_node(g, "workflow", zh, en, kind="group")
    for index, entry in enumerate(items, 1):
        path, name = entry["path"], entry["symbol"]
        if entry["repo"] != "mmhand" or path not in sources.paths["mmhand"]:
            raise ValueError(f"Uncommitted or unsupported UI entrypoint: {entry}")
        info = python_function(sources, path, name)
        if not info:
            raise ValueError(f"Configured UI entrypoint is missing or ambiguous: {path}:{name}")
        node_id = entrypoint_id(graph_id, entry)
        add_node(g, node_id, name, name, evidence=[info["evidence"]],
                 details={"conditions": info["conditions"], "calls": info["calls"],
                          "text": label("源码中的处理函数；右侧列出分支判断和调用。",
                                        "Source handler; branch predicates and calls are listed here.")})
        add_edge(g, root["id"], node_id, "操作路径", "workflow", [info["evidence"]])
        for guard_index, predicate in enumerate(info["conditions"], 1):
            guard_id = f"{node_id}_guard_{guard_index}"
            add_node(g, guard_id, "条件 " + str(guard_index), "Guard " + str(guard_index),
                     kind="guard", evidence=[predicate["evidence"]],
                     details={"conditions": [predicate]})
            add_edge(g, node_id, guard_id, "分支判断", "branch predicate",
                     [predicate["evidence"]], predicate["expr"])
    return g


def ui_control_graphs(sources: Sources) -> tuple[list[dict], dict]:
    files = [p for p in sorted(sources.paths["mmhand"])
             if p.startswith("desktop/ui/") and p.endswith(".py")]
    controls: dict[str, list[dict]] = {}
    for path in files:
        for i, row in enumerate(sources.lines("mmhand", path), 1):
            match = re.search(r"([^\s]+)\.(clicked|toggled|triggered|currentChanged)\.connect\((.*)", row)
            if match:
                controls.setdefault(path, []).append({"widget": match[1],
                    "signal": match[2], "binding": match[3].strip(),
                    "evidence": sources.evidence("mmhand", path, i)})
    overview = graph("ui_controls", "全部 UI 控件接线", "All UI control bindings", "behavior", "behavior_overview")
    add_node(overview, "ui", "UI 信号", "UI signals", kind="group")
    graphs = [overview]
    count = 0
    for path, bindings in controls.items():
        graph_id = "ui_" + Path(path).stem
        g = graph(graph_id, Path(path).stem, Path(path).stem, "behavior", "ui_controls")
        add_node(g, "file", Path(path).name, Path(path).name, kind="group",
                 evidence=[sources.evidence("mmhand", path, 1)])
        for index, item in enumerate(bindings, 1):
            node_id = f"control_{index}"
            binding = f"{item['widget']}.{item['signal']} → {item['binding']}"
            add_node(g, node_id, item["widget"], item["widget"], kind="action",
                     evidence=[item["evidence"]],
                     details={"conditions": [], "calls": [binding],
                              "text": label("点击或切换该控件会执行所示信号绑定；继续查看对应处理函数。",
                                            "Activating this control invokes the shown signal binding; inspect its handler next.")})
            add_edge(g, "file", node_id, item["signal"], item["signal"], [item["evidence"]])
            count += 1
        graphs.append(g)
        file_id = f"file_{len(overview['nodes'])}"
        add_node(overview, file_id, Path(path).stem, Path(path).stem,
                 evidence=[bindings[0]["evidence"]], drilldown=graph_id,
                 details={"count": len(bindings)})
        add_edge(overview, "ui", file_id, "控件", "controls", [bindings[0]["evidence"]])
    return graphs, {"ui_bindings": count, "ui_binding_files": len(controls)}


def p4_state_graph(sources: Sources) -> tuple[dict, dict]:
    states_path = configured_path(sources, "p4.states")
    dispatch_path = configured_path(sources, "p4.dispatcher")
    states = []
    for i, row in enumerate(sources.lines("mmhand", states_path), 1):
        match = re.search(r"\b(SYSTEM_STATE_[A-Z_]+)\s*=\s*(\d+)", row)
        if match:
            states.append((match[1], int(match[2]), sources.evidence("mmhand", states_path, i)))
    g = graph("p4_state", "P4 正式状态与事件", "P4 formal states and events", "behavior", "behavior_overview")
    for name, value, ref in states:
        add_node(g, name, name.removeprefix("SYSTEM_STATE_"), name.removeprefix("SYSTEM_STATE_"),
                 kind="state", evidence=[ref], details={"value": value})
    rows = sources.lines("mmhand", dispatch_path)
    cases = [(i, m[1]) for i, row in enumerate(rows, 1)
             if (m := re.search(r"case\s+(SYSTEM_EVENT_[A-Z_]+)\s*:", row))]
    event_count = 0
    transition_count = 0
    state_names = {name for name, _, _ in states}
    for pos, (start, event) in enumerate(cases):
        end = cases[pos + 1][0] - 1 if pos + 1 < len(cases) else len(rows)
        # The last case ends before switch's default; keep only its own code.
        for j in range(start, end + 1):
            if re.search(r"^\s*default\s*:", rows[j - 1]):
                end = j - 1; break
        snippet = "\n".join(rows[start - 1:end])
        event_ref = sources.evidence("mmhand", dispatch_path, start)
        predicates = []
        for j in range(start, end + 1):
            line = rows[j - 1].strip()
            if not line.startswith("if ("):
                continue
            expression = line[line.index("(") + 1:]
            depth = 1 + expression.count("(") - expression.count(")")
            last = j
            while depth > 0 and last < end:
                last += 1
                continuation = rows[last - 1].strip()
                expression += "\n" + continuation
                depth += continuation.count("(") - continuation.count(")")
            if depth == 0:
                expression = expression.rsplit(")", 1)[0]
            predicates.append({"expr": expression.strip(),
                               "evidence": sources.evidence("mmhand", dispatch_path, j, last)})
        add_node(g, event, event.removeprefix("SYSTEM_EVENT_"), event.removeprefix("SYSTEM_EVENT_"),
                 kind="event", evidence=[event_ref],
                 details={"conditions": predicates,
                          "text": label("条件来自该事件处理分支；多行条件请打开源码核对完整判断。",
                                        "Predicates are from this event handler; open source for complete multiline expressions.")})
        event_count += 1
        targets = set(re.findall(r"setSystemState\(sharedData,\s*(SYSTEM_STATE_[A-Z_]+)", snippet))
        for target in sorted(targets & state_names):
            line_no = next(j for j in range(start, end + 1) if target in rows[j - 1])
            add_edge(g, event, target, "可能转入", "may enter",
                     [sources.evidence("mmhand", dispatch_path, line_no)])
            transition_count += 1
    return g, {"p4_states": len(states), "p4_events": event_count,
               "p4_explicit_transitions": transition_count}


def safety_graphs(sources: Sources) -> tuple[list[dict], dict]:
    path = configured_path(sources, "safety.catalog")
    index_path = configured_path(sources, "safety.index")
    import json
    index = json.loads(sources.read("mmhand", index_path))
    by_name = {item["name"]: item for item in index["rules"]}
    groups: dict[str, list[tuple]] = {}
    pattern = re.compile(r'^P4_SAFETY_RULE\((\w+),\s*(\d+),\s*"([^"]+)",\s*"([^"]+)",\s*(\w+),\s*(\w+),\s*(\w+),\s*"([^"]+)"\)')
    callsites: dict[str, list[dict]] = {}
    for candidate in sorted(sources.paths["mmhand"]):
        if not candidate.startswith("ESP32-P4/ServoBoardMain/src/") or not candidate.endswith((".cpp", ".h")) or "/generated/" in candidate:
            continue
        for j, source_line in enumerate(sources.lines("mmhand", candidate), 1):
            for found in re.finditer(r"SafetyRuleId::([A-Za-z][A-Za-z0-9_]*)", source_line):
                callsites.setdefault(found[1], []).append(sources.evidence("mmhand", candidate, j))
    for i, row in enumerate(sources.lines("mmhand", path), 1):
        if match := pattern.search(row):
            name, number, code, title, severity, layer, action, recovery = match.groups()
            groups.setdefault(layer, []).append((name, number, code, title, severity, action, recovery,
                                                  sources.evidence("mmhand", path, i)))
    overview = graph("safety_rules", "安全规则与异常", "Safety rules and faults", "behavior", "behavior_overview")
    add_node(overview, "rules", "P4 安全规则", "P4 safety rules", kind="group")
    result = [overview]
    total = 0
    for layer, rules in sorted(groups.items()):
        graph_id = "safety_" + layer.lower()
        g = graph(graph_id, f"{layer} 安全规则", f"{layer} safety rules", "behavior", "safety_rules")
        add_node(g, "layer", layer, layer, kind="group", evidence=[rules[0][-1]])
        for name, number, code, title, severity, action, recovery, ref in rules:
            sites = callsites.get(name, [])
            entry = by_name.get(name)
            if not entry or int(entry["rule_id"]) != int(number) or entry["code"] != code:
                raise ValueError(f"Safety index differs from catalog: {name}")
            chain = []
            for role in ("policy", "caller", "event", "recovery"):
                spec = entry[role]
                target = "ESP32-P4/ServoBoardMain/src/" + spec["file"]
                if target not in sources.paths["mmhand"]:
                    raise ValueError(f"Missing safety {role}: {target}")
                evidence = maybe_line(sources, "mmhand", target, spec["symbol"])
                if not evidence:
                    evidence = [sources.evidence("mmhand", target, 1)]
                chain.append({"role": role, "path": target, "symbol": spec["symbol"],
                              "evidence": evidence[0]})
            node = add_node(g, name, f"{code} {title}", f"{code} {title}", kind="fault",
                     evidence=[ref, *sites[:8], *(c["evidence"] for c in chain)], details={"number": int(number), "severity": severity,
                                              "action": action, "recovery": recovery,
                                              "conditions": [], "chain": chain, "callsite_count": len(sites), "text": label(
                         "规则目录定义告警含义；下方列出源码调用点。调用点上下文中的触发条件请打开源码核对。",
                         "The catalog defines the alert. Source call sites appear below; inspect their surrounding predicates.")})
            # Call-site presence does not prove the surrounding trigger predicate.
            node["unverified"] = True
            add_edge(g, "layer", name, action, action, [ref])
            total += 1
        result.append(g)
        add_node(overview, graph_id, layer, layer, evidence=[rules[0][-1]],
                 drilldown=graph_id, details={"count": len(rules)})
        add_edge(overview, "rules", graph_id, "规则分组", "rule group", [rules[0][-1]])
    return result, {"safety_rules": total, "safety_layers": len(groups)}


def collision_graph(sources: Sources) -> dict:
    path = configured_path(sources, "collision.policy")
    service = configured_path(sources, "collision.service")
    g = graph("collision", "反馈碰撞保护", "Feedback collision protection", "behavior", "behavior_overview")
    specs = [
        ("input", "输入与反馈", "Input and feedback", "FeedbackCollisionGuard::update(", "采样和目标进入保护策略。", "Samples and targets enter the guard."),
        ("invalid", "上下文或反馈无效", "Inactive context or invalid feedback", "if (!input.contextActive)", "重置单关节监测；反馈无效也会重置。", "Reset monitoring for this joint; invalid feedback also resets it."),
        ("candidate", "候选监测段", "Candidate motion segment", "if (targetChanged || directionChanged || motionModeChanged ||", "目标、方向或模式变化会重开监测段。", "Target, direction or mode changes restart monitoring."),
        ("armed", "速度达标后武装", "Armed after velocity window", "if (!state.armed && velocityWindowReady &&", "最近速度窗口达标才武装。", "Arm only after the recent velocity window qualifies."),
        ("hold", "停滞并进入 HOLD", "Stalled and enters HOLD", "if (state.armed && progressWindowReady &&", "进展不足且 PID 仍在推时锁住反馈角。", "Insufficient progress while PID pushes locks the feedback angle."),
        ("freeze", "目标固定在保持角", "Target held at hold angle", "if (collisionResult.holdActive)", "控制服务把该关节目标改为 holdAngleDeg。", "The control service replaces this joint's target with holdAngleDeg."),
        ("release", "回撤解除 HOLD", "Retreat releases HOLD", "if (targetRetreatDeg >= kCollisionTargetRetreatReleaseDeg)", "目标沿障碍反方向回撤达到阈值后解除并重新监测。", "Retreating the target by the threshold releases HOLD and restarts monitoring."),
    ]
    for node_id, zh, en, needle, text_zh, text_en in specs:
        source_path = service if node_id == "freeze" else path
        ref = find_line(sources, "mmhand", source_path, needle)
        add_node(g, node_id, zh, en, kind="state" if node_id in {"armed", "hold"} else "process",
                 evidence=[ref], details={"text": label(text_zh, text_en)})
    edges = [
        ("input", "invalid", "!input.contextActive || !input.feedbackValid || !isfinite(input.targetDeg) || !isfinite(input.feedbackDeg)", "无效", "invalid"),
        ("input", "candidate", "input.contextActive && input.feedbackValid && isfinite(input.targetDeg) && isfinite(input.feedbackDeg)", "有效", "valid"),
        ("candidate", "armed", "!state.armed && velocityWindowReady && directionVelocity >= kCollisionArmVelocityDegPerSec", "速度达标", "velocity qualified"),
        ("armed", "hold", "state.armed && progressWindowReady && directionProgress < state.minProgressDeg && input.pidPushActive", "进展不足", "insufficient progress"),
        ("hold", "freeze", "collisionResult.holdActive", "保持生效", "hold applied"),
        ("hold", "release", "targetRetreatDeg >= kCollisionTargetRetreatReleaseDeg", "目标回撤", "target retreats"),
        ("release", "candidate", "result.holdReleased && result.monitorReset", "重新监测", "monitor again"),
    ]
    refs = {n["id"]: n["evidence"] for n in g["nodes"]}
    for a, b, condition, zh, en in edges:
        add_edge(g, a, b, zh, en, refs[b], condition)
    return g


def behavior_graphs(sources: Sources) -> tuple[list[dict], dict]:
    overview = graph("behavior_overview", "操作、状态与保护", "Actions, states and protection", "behavior")
    add_node(overview, "operator", "操作者", "Operator", kind="actor")
    result = [overview]
    for graph_id, items in sources.scope["entrypoints"].items():
        g = function_graph(sources, graph_id, items)
        result.append(g)
        ref = g["nodes"][1]["evidence"][0] if len(g["nodes"]) > 1 else []
        node = add_node(overview, graph_id, g["title"]["zh"], g["title"]["en"],
                        evidence=[ref] if ref else [], drilldown=graph_id)
        add_edge(overview, "operator", node["id"], "操作", "action", [ref] if ref else [])
    ui_graphs, ui_counts = ui_control_graphs(sources)
    result.extend(ui_graphs)
    add_node(overview, "ui_controls", "全部 UI 控件", "All UI bindings",
             evidence=ui_graphs[0]["nodes"][1]["evidence"] if len(ui_graphs[0]["nodes"]) > 1 else [],
             drilldown="ui_controls")
    add_edge(overview, "operator", "ui_controls", "点击控件", "activate control")
    state_graph, state_counts = p4_state_graph(sources)
    result.append(state_graph)
    add_node(overview, "p4_state", "P4 状态机", "P4 state machine",
             evidence=state_graph["nodes"][0]["evidence"], drilldown="p4_state")
    add_edge(overview, "operator", "p4_state", "触发事件", "posts events")
    safety, safety_counts = safety_graphs(sources)
    result.extend(safety)
    collision = collision_graph(sources)
    result.append(collision)
    add_node(overview, "collision", "碰撞保护", "Collision protection",
             evidence=collision["nodes"][0]["evidence"], drilldown="collision")
    add_edge(overview, "p4_state", "collision", "反馈保护", "feedback guard")
    add_node(overview, "safety_rules", "异常与安全规则", "Faults and safety rules",
             evidence=safety[0]["nodes"][1]["evidence"] if len(safety[0]["nodes"]) > 1 else [],
             drilldown="safety_rules")
    add_edge(overview, "p4_state", "safety_rules", "门禁与保护", "gates and protection")
    wire_command_paths(sources, result, state_graph)
    return result, {**ui_counts, **state_counts, **safety_counts}


def wire_command_paths(sources: Sources, graphs: list[dict], states: dict) -> None:
    """Show proven UI→wire→P4 event chains for the primary device actions."""
    by_id = {g["id"]: g for g in graphs}
    event_nodes = {n["id"]: n for n in states["nodes"]}
    dispatcher = configured_path(sources, "p4.host_dispatcher")
    core = configured_path(sources, "desktop.core")
    specs = [
        ("control", "step_1", "CMD_START", "SYSTEM_EVENT_START", 'send_command("start")',
         "postSystemEvent(sharedData, SYSTEM_EVENT_START)"),
        ("control", "step_2", "CMD_STOP", "SYSTEM_EVENT_STOP", 'send_command("stop")',
         "requestEmergencyStop(sharedData)"),
        ("control", "step_3", "CMD_PAUSE", "SYSTEM_EVENT_PAUSE", 'send_command("pause")',
         "postSystemEvent(sharedData, SYSTEM_EVENT_PAUSE)"),
        ("control", "step_4", "CMD_RESET", "SYSTEM_EVENT_RESET", 'send_command("reset")',
         "postSystemEvent(sharedData, SYSTEM_EVENT_RESET)"),
        ("calibration", "step_1", "CMD_CALIBRATE", "SYSTEM_EVENT_CALIBRATE", 'send_command("calibrate")',
         "postSystemEvent(sharedData, SYSTEM_EVENT_CALIBRATE)"),
    ]
    for graph_id, action, command, event, send, dispatch in specs:
        g = by_id[graph_id]
        if action not in {n["id"] for n in g["nodes"]}:
            continue
        send_ref = find_line(sources, "mmhand", core, send)
        dispatch_ref = find_line(sources, "mmhand", dispatcher, dispatch)
        cmd_ref = find_line(sources, "mmhand", dispatcher, "if (cmd == " + command + ")")
        add_node(g, command, command, command, kind="protocol", evidence=[cmd_ref, send_ref])
        event_source = event_nodes[event]
        add_node(g, event, event.removeprefix("SYSTEM_EVENT_"), event.removeprefix("SYSTEM_EVENT_"),
                 kind="event", evidence=[dispatch_ref, *event_source["evidence"]],
                 details=event_source["details"], drilldown="p4_state")
        add_edge(g, action, command, "后端发送", "backend sends", [send_ref])
        add_edge(g, command, event, "P4 处理", "P4 handles", [dispatch_ref])


def maybe_line(sources: Sources, repo: str, path: str, needle: str) -> list[dict]:
    try:
        return [find_line(sources, repo, path, needle)]
    except ValueError:
        return []


def p4_tasks(sources: Sources) -> tuple[dict, dict]:
    assembly = configured_path(sources, "p4.assembly")
    task_config = configured_path(sources, "p4.task_config")
    names = {entry["name"]: entry["path"] for entry in sources.scope["p4_tasks"]}
    g = graph("arch_p4", "P4 FreeRTOS 任务", "P4 FreeRTOS tasks", "architecture", "architecture_overview")
    add_node(g, "p4", "P4 任务装配", "P4 task assembly", kind="group",
             evidence=maybe_line(sources, "mmhand", assembly, "任务按"))
    count = 0
    assembly_rows = sources.lines("mmhand", assembly)
    config_rows = sources.lines("mmhand", task_config)
    constants = {}
    for line_no, row in enumerate(config_rows, 1):
        if match := re.search(r"\b([A-Z][A-Z0-9_]+)\s*=\s*([^;]+);", row):
            constants[match[1]] = {"expr": match[2].strip(),
                                   "evidence": sources.evidence("mmhand", task_config, line_no)}
    for name, path in names.items():
        refs = maybe_line(sources, "mmhand", assembly, "{" + name + ",")
        if path not in sources.paths["mmhand"]:
            raise ValueError(f"Configured P4 task is not committed: {path}")
        frequency = []
        signals = []
        signal_refs = []
        for i, row in enumerate(sources.lines("mmhand", path), 1):
            if any(token in row for token in ("pdMS_TO_TICKS(", "vTaskDelayUntil(", "vTaskDelay(")):
                frequency.append({"expr": row.strip(), "evidence": sources.evidence("mmhand", path, i)})
            if any(token in row for token in ("sharedData->", "xQueue", "signal", "notify")):
                signals.append(row.strip())
                signal_refs.append(sources.evidence("mmhand", path, i))
        refs += [sources.evidence("mmhand", path, 1)]
        refs += signal_refs[:18]
        task_setup = {}
        for i, row in enumerate(assembly_rows, 1):
            if "{" + name + "," not in row:
                continue
            block = "\n".join(assembly_rows[i - 1:i + 7])
            match = re.search(r"\{\s*" + re.escape(name) + r'\s*,\s*"[^"]+"\s*,\s*([A-Z_]+)\s*,\s*&sharedData\s*,\s*([A-Z_]+)', block)
            if match:
                for field, symbol in (("stack", match[1]), ("priority", match[2])):
                    value = constants.get(symbol, {}).get("expr", "未核实 / unverified")
                    task_setup[field] = f"{symbol} = {value}" + (" word" if field == "stack" else "")
                    if symbol in constants:
                        refs.append(constants[symbol]["evidence"])
            break
        add_node(g, name, name, name, kind="task", evidence=refs,
                 details={"period": frequency[:8], "channels": list(dict.fromkeys(signals))[:18],
                          "config": task_setup,
                          "text": label("周期字段来自任务入口；队列或共享数据请沿引用核对。",
                                        "Period comes from task entry; inspect references for queues and shared data.")})
        add_edge(g, "p4", name, "创建任务", "creates task", refs[:1])
        count += 1
    # Calibration task is started on demand rather than in SystemAssembly.
    calib = configured_path(sources, "p4.calibration_task")
    if calib in sources.paths["mmhand"]:
        refs = maybe_line(sources, "mmhand", calib, "autoCalibrationTask") or [sources.evidence("mmhand", calib, 1)]
        add_node(g, "autoCalibrationTask", "自动标定任务（按需）", "Calibration task (on demand)",
                 kind="task", evidence=refs,
                 details={"period": [row.strip() for row in sources.lines("mmhand", calib)
                                     if "pdMS_TO_TICKS" in row][:6]})
        add_edge(g, "p4", "autoCalibrationTask", "按需启动", "starts on demand", refs)
        count += 1
    task_edges = [
        ("hostCommTask", "systemStateTask", "postSystemEvent + systemStateWake", "命令事件", "command event",
         configured_path(sources, "p4.dispatcher"), "sharedData->systemStateWake.notify"),
        ("hostCommTask", "motionControlTask", "sharedData->motionControlWake", "目标唤醒", "target wake",
         configured_path(sources, "p4.host_router"), "sharedData->motionControlWake.notify"),
        ("systemStateTask", "motionControlTask", "sharedData->motionControlWake", "状态变更唤醒", "state-change wake",
         configured_path(sources, "p4.system_state_task"), "sharedData->motionControlWake.notify"),
        ("systemStateTask", "servoBusTask", "sharedData->servoBusWake", "急停唤醒", "emergency-stop wake",
         configured_path(sources, "p4.dispatcher"), "sharedData->servoBusWake.notify"),
        ("s3CanRxTask", "motionControlTask", "sharedData->canRxQueue.overwrite / peek", "CAN 快照队列", "CAN snapshot queue",
         configured_path(sources, "p4.s3_rx_service"), "sharedData->canRxQueue.overwrite"),
    ]
    for start, end, channel, zh, en, path, needle in task_edges:
        ref = find_line(sources, "mmhand", path, needle)
        add_edge(g, start, end, zh, en, [ref], channel)
    return g, {"p4_tasks": count}


def s3_tasks(sources: Sources) -> tuple[dict, dict]:
    system = configured_path(sources, "s3.system")
    config = configured_path(sources, "s3.config")
    specs = sources.scope["s3_tasks"]
    g = graph("arch_s3", "S3 采集与通信任务", "S3 acquisition and communication tasks",
              "architecture", "architecture_overview")
    add_node(g, "s3", "S3 任务装配", "S3 task assembly", kind="group",
             evidence=maybe_line(sources, "mmhand", system, "void startSystemTasks()"))
    for spec in specs:
        name, path, prefix = spec["name"], spec["path"], spec["config_prefix"]
        if path not in sources.paths["mmhand"]:
            raise ValueError(f"Configured S3 task is not committed: {path}")
        refs = maybe_line(sources, "mmhand", system, name)
        if path in sources.paths["mmhand"]:
            refs += [sources.evidence("mmhand", path, 1)]
        timing = []
        if path in sources.paths["mmhand"]:
            for i, row in enumerate(sources.lines("mmhand", path), 1):
                if any(x in row for x in ("pdMS_TO_TICKS", "vTaskDelay", "PERIOD", "INTERVAL")):
                    timing.append({"expr": row.strip(), "evidence": sources.evidence("mmhand", path, i)})
        parameters = []
        for line_no, row in enumerate(sources.lines("mmhand", config), 1):
            if prefix in row:
                parameters.append(row.strip())
                refs.append(sources.evidence("mmhand", config, line_no))
        add_node(g, name, name, name, kind="task", evidence=refs,
                 details={"period": timing[:10], "config": parameters,
                          "text": label("任务周期和核/优先级分别以入口及 Config.h 为准。",
                                        "Task timing and core/priority come from entry code and Config.h.")})
        add_edge(g, "s3", name, "创建任务", "creates task", refs[:1])
    queue_ref = maybe_line(sources, "mmhand", system, "xQueueEncoderData = xQueueCreate")
    add_edge(g, "Task_Encoders", "Task_CanBus", "长度 1 覆盖队列", "overwrite queue of length 1", queue_ref)
    tactile = configured_path(sources, "s3.tactile")
    can = configured_path(sources, "s3.can")
    can_ref = maybe_line(sources, "mmhand", tactile, "twaiBus.sendTactileSummary")
    serial_ref = maybe_line(sources, "mmhand", tactile, "sendSerialTactileSummary(data")
    add_node(g, "twai", "TWAI/CAN 总线", "TWAI/CAN bus", kind="protocol", evidence=can_ref)
    add_node(g, "tactile_serial", "独立触觉串口", "Separate tactile serial", kind="protocol", evidence=serial_ref)
    add_edge(g, "Task_Tactile", "twai", "触觉摘要", "tactile summary", can_ref)
    add_edge(g, "Task_Tactile", "tactile_serial", "触觉摘要", "tactile summary", serial_ref)
    add_edge(g, "Task_CanBus", "twai", "编码器 CAN 帧", "encoder CAN frames",
             maybe_line(sources, "mmhand", can, "xQueueReceive(xQueueEncoderData"))
    return g, {"s3_tasks": len(specs)}


def desktop_architecture(sources: Sources) -> dict:
    g = graph("arch_desktop", "桌面端与 ROS2", "Desktop and ROS2", "architecture", "architecture_overview")
    specs = [
        ("entry", "启动入口", "Application entry", configured_path(sources, "desktop.entry"), "def main"),
        ("ui", "Qt 界面", "Qt UI", configured_path(sources, "desktop.ui"), "class "),
        ("backend", "HandBackend 门面", "HandBackend facade", configured_path(sources, "desktop.backend"), "class HandBackend"),
        ("protocol", "主串口协议", "Main serial protocol", configured_path(sources, "desktop.protocol"), "def _build_command_frame"),
        ("raw_ik", "原始 IK 订阅节点", "Raw IK subscriber", configured_path(sources, "desktop.raw_ik"), "create_subscription("),
        ("joint_targets", "关节目标订阅节点", "Joint-target subscriber", configured_path(sources, "desktop.joint_targets"), "create_subscription("),
        ("replay", "回放控制", "Replay control", configured_path(sources, "desktop.replay"), "class "),
    ]
    for node_id, zh, en, path, needle in specs:
        if path not in sources.paths["mmhand"]:
            continue
        refs = maybe_line(sources, "mmhand", path, needle)
        add_node(g, node_id, zh, en, evidence=refs,
                 details={"text": label("打开源码可核对线程、消息和调用边界。",
                                        "Open source to inspect thread, message and call boundaries.")})
    ids = {n["id"] for n in g["nodes"]}
    for a, b, zh, en in [
        ("entry", "ui", "启动", "starts"), ("ui", "backend", "用户命令", "user commands"),
        ("raw_ik", "backend", "21 路目标", "21 joint targets"),
        ("joint_targets", "backend", "21 路目标", "21 joint targets"),
        ("replay", "backend", "定时目标", "timed targets"),
        ("backend", "protocol", "编码命令", "encodes commands"),
    ]:
        if a in ids and b in ids:
            add_edge(g, a, b, zh, en)
    return g


def serial_graphs(sources: Sources) -> tuple[list[dict], dict]:
    path = configured_path(sources, "protocol.schema")
    import json
    schema = json.loads(sources.read("mmhand", path))
    frame = graph("arch_serial", "上位机与 P4 串口帧", "Desktop–P4 serial frames",
                  "architecture", "architecture_overview")
    frame_ref = find_line(sources, "mmhand", configured_path(sources, "desktop.protocol"), "def _build_command_frame")
    add_node(frame, "frame", "FE | LEN | 类型/命令 | payload | FF", "FE | LEN | type/command | payload | FF",
             kind="protocol", evidence=[frame_ref], details={"config": schema["frame"],
              "text": label("下行长度按命令字、payload 和帧尾计算；回包布局需按解析器核对。",
                            "Downlink length covers command, payload and tail; inspect parsers for reply layout.")})
    groups = [frame]
    for section, zh, en in (("commands", "下行命令", "Downlink commands"),
                            ("packets", "上行回包", "Uplink packets")):
        group_id = "arch_serial_" + section
        g = graph(group_id, zh, en, "architecture", "arch_serial")
        add_node(g, "items", zh, en, kind="group", evidence=[frame_ref])
        for i, item in enumerate(schema[section]):
            refs = maybe_line(sources, "mmhand", path, '"name": "' + item["name"] + '"')
            item_id = f"item_{i}"
            add_node(g, item_id, item["name"], item["name"], kind="protocol",
                     evidence=refs,
                     details={"number": item["value"],
                              "payload_bytes": item.get("payload_bytes"),
                              "description": item.get("description", "")})
            add_edge(g, "items", item_id, str(item["value"]), str(item["value"]), refs)
        groups.append(g)
        add_node(frame, section, zh, en, evidence=[frame_ref], drilldown=group_id)
        add_edge(frame, "frame", section, "包含", "contains", [frame_ref])
    return groups, {"commands": len(schema["commands"]), "packets": len(schema["packets"])}


def can_graph(sources: Sources) -> dict:
    path = configured_path(sources, "p4.can_protocol")
    g = graph("arch_can", "S3→P4 CAN 帧", "S3→P4 CAN frames", "architecture", "architecture_overview")
    add_node(g, "can", "CAN 传感帧", "CAN sensor frames", kind="group",
             evidence=[sources.evidence("mmhand", path, 1)])
    for i, row in enumerate(sources.lines("mmhand", path), 1):
        match = re.search(r"\b(CAN_ID_[A-Z0-9_]+)\s*=\s*([^;]+);", row)
        if match:
            ref = sources.evidence("mmhand", path, i)
            add_node(g, match[1], match[1], match[1], kind="protocol", evidence=[ref],
                     details={"expression": match[2].strip()})
            add_edge(g, "can", match[1], "标识符", "identifier", [ref])
    return g


def ros_graph(sources: Sources) -> dict:
    g = graph("arch_ros", "遥操发布与桌面订阅", "Teleop publication and desktop subscription",
              "architecture", "architecture_overview")
    config = configured_path(sources, "teleop.config", "teleop")
    publisher = configured_path(sources, "teleop.publisher", "teleop")
    subscriber = configured_path(sources, "desktop.raw_ik")
    add_node(g, "teleop", "mmhand_teleop 发布节点", "mmhand_teleop publisher",
             evidence=maybe_line(sources, "teleop", publisher, 'create_node("mmhand_teleop")'))
    add_node(g, "raw", "/raw_ik_target · Float32MultiArray", "/raw_ik_target · Float32MultiArray",
             kind="protocol", evidence=maybe_line(sources, "teleop", config, 'ROBOT_TOPIC ='),
             details={"text": label("21 个 J00–J20，单位 degree；消息由 RosOutput.joints 发布。",
                                   "21 J00–J20 values in degrees; published by RosOutput.joints.")})
    add_node(g, "desktop", "桌面 Raw IK 订阅节点", "Desktop Raw IK subscriber",
             evidence=maybe_line(sources, "mmhand", subscriber, "create_subscription("))
    add_edge(g, "teleop", "raw", "发布", "publishes",
             maybe_line(sources, "teleop", publisher, "self.robot_publisher ="))
    add_edge(g, "raw", "desktop", "订阅", "subscribes",
             maybe_line(sources, "mmhand", subscriber, "create_subscription("))
    return g


def architecture_graphs(sources: Sources) -> tuple[list[dict], dict]:
    root = graph("architecture_overview", "整机架构与数据流", "System architecture and data flow", "architecture")
    desktop_ref = maybe_line(sources, "mmhand", configured_path(sources, "desktop.entry"), "def main")
    p4_ref = maybe_line(sources, "mmhand", configured_path(sources, "p4.assembly"), "bool tasksCreated")
    s3_ref = maybe_line(sources, "mmhand", configured_path(sources, "s3.system"), "void startSystemTasks")
    servo_ref = maybe_line(sources, "mmhand", configured_path(sources, "p4.dimensions"), "MOTOR")
    add_node(root, "desktop", "桌面上位机", "Desktop application", evidence=desktop_ref, drilldown="arch_desktop")
    add_node(root, "p4", "P4 电机控制", "P4 motor controller", evidence=p4_ref, drilldown="arch_p4")
    add_node(root, "s3", "S3 手背采集", "S3 sensor acquisition", evidence=s3_ref, drilldown="arch_s3")
    add_node(root, "servos", "27 台舵机", "27 servos", evidence=servo_ref)
    add_node(root, "protocol", "串口协议与帧", "Serial protocol and frames",
             evidence=maybe_line(sources, "mmhand", configured_path(sources, "desktop.protocol"), "def _build_command_frame"),
             drilldown="arch_serial")
    add_node(root, "can", "S3 CAN 帧", "S3 CAN frames",
             evidence=maybe_line(sources, "mmhand", configured_path(sources, "p4.can_protocol"), "CAN_ID_ENC_CRC"),
             drilldown="arch_can")
    add_node(root, "ros", "ROS2 遥操", "ROS2 teleoperation",
             evidence=maybe_line(sources, "teleop", configured_path(sources, "teleop.publisher", "teleop"), "self.robot_publisher ="), drilldown="arch_ros")
    add_edge(root, "desktop", "p4", "主串口：命令/回包", "main serial: commands/replies",
             maybe_line(sources, "mmhand", configured_path(sources, "desktop.protocol"), "def _build_command_frame"))
    add_edge(root, "s3", "p4", "CAN：磁编/触觉摘要", "CAN: encoders/tactile summary",
             maybe_line(sources, "mmhand", configured_path(sources, "p4.can_protocol"), "CAN_ID_ENC_CRC"))
    add_edge(root, "s3", "desktop", "独立触觉串口", "separate tactile serial",
             maybe_line(sources, "mmhand", configured_path(sources, "desktop.entry"), "--tactile-port"))
    add_edge(root, "p4", "servos", "舵机总线", "servo buses", servo_ref)
    add_edge(root, "ros", "desktop", "/raw_ik_target", "/raw_ik_target",
             maybe_line(sources, "mmhand", configured_path(sources, "desktop.raw_ik"), "create_subscription("))
    p4, p4_count = p4_tasks(sources)
    s3, s3_count = s3_tasks(sources)
    serial, protocol_count = serial_graphs(sources)
    graphs = [root, p4, s3, desktop_architecture(sources), can_graph(sources), ros_graph(sources), *serial]
    return graphs, {**p4_count, **s3_count, **protocol_count}
