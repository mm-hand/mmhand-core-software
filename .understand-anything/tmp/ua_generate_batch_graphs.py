import json
import pathlib
import subprocess

ROOT = pathlib.Path(r"D:\hand\mmhand-cl (3)\mmhand-cl")
SKILL_DIR = pathlib.Path(r"C:\Users\17807\.understand-anything-plugin\skills\understand")
INTER = ROOT / ".understand-anything" / "intermediate"
TMP = ROOT / ".understand-anything" / "tmp"


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


scan = load_json(INTER / "scan-result.json")
batches = load_json(INTER / "batches.json")["batches"]
file_meta = {f["path"]: f for f in scan["files"]}


def file_node_type(f):
    path = f["path"]
    cat = f["fileCategory"]
    low = path.lower()
    if cat == "config":
        return "config"
    if cat == "docs":
        return "document"
    if cat == "infra":
        if ".github/workflows/" in low or low.endswith(".gitlab-ci.yml") or pathlib.PurePosixPath(path).name.lower() == "jenkinsfile":
            return "pipeline"
        if low.endswith(".tf") or low.endswith(".tfvars") or pathlib.PurePosixPath(path).name.lower() == "vagrantfile":
            return "resource"
        return "service"
    if cat == "data":
        if low.endswith((".graphql", ".gql", ".proto", ".prisma", ".urdf", ".xml")):
            return "schema"
        if low.endswith((".yaml", ".yml")) and ("openapi" in low or "swagger" in low):
            return "endpoint"
        return "table" if low.endswith(".sql") else "schema"
    return "file"


def prefix_for_type(t):
    return {
        "file": "file",
        "config": "config",
        "document": "document",
        "service": "service",
        "pipeline": "pipeline",
        "resource": "resource",
        "schema": "schema",
        "endpoint": "endpoint",
        "table": "table",
    }[t]


def file_node_id(path):
    f = file_meta.get(path, {"path": path, "fileCategory": "code"})
    t = file_node_type(f)
    return f"{prefix_for_type(t)}:{path}"


def complexity(non_empty):
    if non_empty < 50:
        return "simple"
    if non_empty <= 200:
        return "moderate"
    return "complex"


def role_summary(path, node_type, language, category):
    low = path.lower()
    name = pathlib.PurePosixPath(path).name
    if "protocol" in low:
        return f"{name} 定义通信协议、命令常量或解析打包逻辑，是跨端数据对齐的关键节点。"
    if "task" in low:
        return f"{name} 实现任务调度或后台循环，负责把共享队列、硬件反馈和系统状态串起来。"
    if "control" in low or "solver" in low:
        return f"{name} 属于控制链路，负责目标、反馈、安全约束或舵机输出构建。"
    if "hardwaremap" in low:
        return f"{name} 集中描述关节、电机通道、总线和舵机 ID 的硬件映射关系。"
    if node_type == "document":
        return f"{name} 是项目文档，解释当前模块、协议、硬件或调试流程。"
    if node_type == "config":
        return f"{name} 是配置文件，保存运行、构建、标定或资源参数。"
    if path.startswith("desktop/"):
        return f"{name} 属于 Python 上位机，用于 UI、串口通信、状态模型、ROS2 或遥操作链路。"
    if path.startswith("ESP32-P4/"):
        return f"{name} 属于 ESP32-P4 主控固件，用于状态机、控制、通信、标定或持久化。"
    if path.startswith("ESP32-S3/"):
        return f"{name} 属于 ESP32-S3 传感固件，用于编码器、触觉、CAN 或系统任务。"
    return f"{name} 是 {language} {category} 文件，参与项目的源码、配置、文档或资源结构。"


def tags_for(path, node_type, language, category):
    tags = []
    if node_type == "document":
        tags += ["documentation", "architecture"]
    elif node_type == "config":
        tags += ["configuration", "settings"]
    elif node_type in ("service", "pipeline", "resource"):
        tags += ["infrastructure", "deployment"]
    else:
        tags += ["source-code", language or "unknown"]
    low = path.lower()
    for key, tag in [
        ("protocol", "protocol"),
        ("task", "task"),
        ("control", "control"),
        ("calib", "calibration"),
        ("tactile", "tactile"),
        ("servo", "servo"),
        ("can", "can"),
        ("ui", "ui"),
        ("test", "test"),
    ]:
        if key in low and tag not in tags:
            tags.append(tag)
    while len(tags) < 3:
        tags.append("project")
    return tags[:5]


def add_edge(edges, source, target, etype, weight):
    if source == target:
        return
    edge = {"source": source, "target": target, "type": etype, "direction": "forward", "weight": weight}
    key = (source, target, etype)
    if key not in {(e["source"], e["target"], e["type"]) for e in edges}:
        edges.append(edge)


all_results = {}
for batch in batches:
    idx = batch["batchIndex"]
    batch_files = batch["files"]
    batch_import_data = batch.get("batchImportData", {})
    input_path = TMP / f"ua-file-analyzer-input-{idx}.json"
    extract_path = TMP / f"ua-file-extract-results-{idx}.json"
    input_data = {
        "projectRoot": str(ROOT),
        "batchFiles": batch_files,
        "batchImportData": batch_import_data,
    }
    input_path.write_text(json.dumps(input_data, ensure_ascii=False), encoding="utf-8")
    subprocess.run(
        ["node", str(SKILL_DIR / "extract-structure.mjs"), str(input_path), str(extract_path)],
        check=True,
        cwd=str(ROOT),
    )
    extracted = load_json(extract_path)
    for r in extracted.get("results", []):
        all_results[r["path"]] = r


global_function_ids = {}
global_class_ids = {}
for path, r in all_results.items():
    for fn in r.get("functions", []) or []:
        name = fn.get("name")
        if not name:
            continue
        nid = f"function:{path}:{name}"
        global_function_ids.setdefault(name, []).append(nid)
    for cls in r.get("classes", []) or []:
        name = cls.get("name")
        if not name:
            continue
        nid = f"class:{path}:{name}"
        global_class_ids.setdefault(name, []).append(nid)


def clean_callee_name(callee):
    if not callee:
        return ""
    name = str(callee).split("(")[0].strip()
    if "." in name:
        name = name.split(".")[-1]
    if "::" in name:
        name = name.split("::")[-1]
    if "->" in name:
        name = name.split("->")[-1]
    return name.strip()


def resolve_call_target(current_path, callee, imported_paths):
    name = clean_callee_name(callee)
    if not name:
        return None
    local = f"function:{current_path}:{name}"
    if name in global_function_ids and local in global_function_ids[name]:
        return local
    imported = set(imported_paths or [])
    candidates = global_function_ids.get(name, [])
    for cid in candidates:
        cpath = cid.split(":", 2)[1]
        if cpath in imported:
            return cid
    if len(candidates) == 1:
        return candidates[0]
    class_candidates = global_class_ids.get(name, [])
    for cid in class_candidates:
        cpath = cid.split(":", 2)[1]
        if cpath in imported:
            return cid
    if len(class_candidates) == 1:
        return class_candidates[0]
    return None


for batch in batches:
    idx = batch["batchIndex"]
    batch_files = batch["files"]
    batch_import_data = batch.get("batchImportData", {})
    results = {f["path"]: all_results.get(f["path"], {}) for f in batch_files}
    nodes = []
    edges = []
    emitted_node_ids = set()

    for f in batch_files:
        path = f["path"]
        r = results.get(path, {})
        node_type = file_node_type(f)
        fid = file_node_id(path)
        total = int(r.get("nonEmptyLines") or r.get("totalLines") or f.get("sizeLines") or 0)
        file_node = {
            "id": fid,
            "type": node_type,
            "name": pathlib.PurePosixPath(path).name,
            "filePath": path,
            "summary": role_summary(path, node_type, f.get("language", ""), f.get("fileCategory", "")),
            "tags": tags_for(path, node_type, f.get("language", ""), f.get("fileCategory", "")),
            "complexity": complexity(total),
        }
        nodes.append(file_node)
        emitted_node_ids.add(fid)
        for target in batch_import_data.get(path, []):
            add_edge(edges, fid, file_node_id(target), "imports", 0.7)

        exports = {e.get("name") for e in r.get("exports", []) if e.get("name")}
        call_names = set()
        for call in r.get("callGraph", []) or []:
            if call.get("caller"):
                call_names.add(clean_callee_name(call.get("caller")))
            if call.get("callee"):
                call_names.add(clean_callee_name(call.get("callee")))
        for fn in r.get("functions", []) or []:
            start = int(fn.get("startLine") or 0)
            end = int(fn.get("endLine") or start)
            name = fn.get("name") or "anonymous"
            if (end - start + 1) < 10 and name not in exports and name not in call_names:
                continue
            nid = f"function:{path}:{name}"
            if nid in emitted_node_ids:
                continue
            nodes.append({
                "id": nid,
                "type": "function",
                "name": name,
                "filePath": path,
                "lineRange": [start, end],
                "summary": f"{name} 是 {path} 中的重要函数，承担该文件的一部分核心处理逻辑。",
                "tags": ["function", "logic", f.get("language", "code")],
                "complexity": complexity(max(0, end - start + 1)),
            })
            emitted_node_ids.add(nid)
            add_edge(edges, fid, nid, "contains", 1.0)
            if name in exports:
                add_edge(edges, fid, nid, "exports", 0.8)

        for cls in r.get("classes", []) or []:
            start = int(cls.get("startLine") or 0)
            end = int(cls.get("endLine") or start)
            methods = cls.get("methods") or []
            name = cls.get("name") or "AnonymousClass"
            if len(methods) < 2 and (end - start + 1) < 20 and name not in exports:
                continue
            nid = f"class:{path}:{name}"
            if nid in emitted_node_ids:
                continue
            nodes.append({
                "id": nid,
                "type": "class",
                "name": name,
                "filePath": path,
                "lineRange": [start, end],
                "summary": f"{name} 是 {path} 中的主要类型，封装相关状态、接口或业务行为。",
                "tags": ["class", "type", f.get("language", "code")],
                "complexity": complexity(max(0, end - start + 1)),
            })
            emitted_node_ids.add(nid)
            add_edge(edges, fid, nid, "contains", 1.0)
            if name in exports:
                add_edge(edges, fid, nid, "exports", 0.8)

        for call in r.get("callGraph", []) or []:
            caller_name = clean_callee_name(call.get("caller"))
            callee_name = call.get("callee")
            if not caller_name or not callee_name:
                continue
            source = f"function:{path}:{caller_name}"
            target = resolve_call_target(path, callee_name, batch_import_data.get(path, []))
            if source in emitted_node_ids and target:
                add_edge(edges, source, target, "calls", 0.8)

    out = {"nodes": nodes, "edges": edges}
    (INTER / f"batch-{idx}.json").write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"batch {idx}: nodes={len(nodes)} edges={len(edges)}")
