"""Bilingual Markdown and offline interactive HTML rendering."""

from __future__ import annotations

from collections import defaultdict
import html
import json
from pathlib import Path
import re

from extract import find_line, maybe_line
from explain import COMMANDS
from source_scope import configured_path
from source import Sources, range_changed


T = {
    "zh": ["项目概览与本次变更", "安装、运行、操作与维护", "操作、状态与保护交互图",
           "系统架构与数据流图", "控制公式", "通信指令表", "逐文件简介与代码行数", "代码引用"],
    "en": ["Project overview and changes", "Setup, operation and maintenance",
           "Interactive actions, states and protection", "Architecture and data flow",
           "Control equations", "Communication commands", "File inventory and line counts",
           "Source references"],
}


def all_evidence(graphs: list[dict]) -> list[dict]:
    result = []
    for g in graphs:
        for item in [*g["nodes"], *g["edges"]]:
            result.extend(item["evidence"])
            for detail in item.get("details", {}).get("conditions", []):
                result.append(detail["evidence"])
            for detail in item.get("details", {}).get("period", []):
                if isinstance(detail, dict) and "evidence" in detail:
                    result.append(detail["evidence"])
    return result


def cite(refs: list[dict], registry: dict[str, int]) -> str:
    keys = list(dict.fromkeys(Sources.key(x["repo"], x["path"]) for x in refs))
    return " ".join(f"[{registry[key]}](#ref-{registry[key]:04d})" for key in keys)


def ref_for(sources: Sources, repo: str, path: str, needle: str) -> list[dict]:
    return maybe_line(sources, repo, path, needle)


def file_intro(sources: Sources, path: str, lang: str) -> str:
    text = sources.read("mmhand", path)
    stem = Path(path).stem
    symbols = []
    if path.endswith(".py"):
        symbols = re.findall(r"^(?:class|def)\s+([A-Za-z_][A-Za-z0-9_]*)", text, re.M)[:4]
    elif path.endswith((".c", ".cpp", ".h", ".hpp", ".ino")):
        symbols = re.findall(r"\b(?:void|bool|int|float|uint\d+_t|static|constexpr)\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(", text)[:4]
    elif path.endswith((".json", ".yaml", ".yml")):
        try:
            data = json.loads(text)
            if isinstance(data, dict):
                symbols = list(data)[:4]
        except ValueError:
            pass
    focus = ", ".join(dict.fromkeys(symbols))
    scope = Path(path).parent.as_posix()
    if lang == "zh":
        return (f"{stem}：在 `{scope}` 中定义或处理 {focus}。" if focus
                else f"{stem}：位于 `{scope}`，具体职责需沿源码核对。")
    return (f"{stem}: defines or handles {focus} in `{scope}`." if focus
            else f"{stem}: located in `{scope}`; inspect source for its specific responsibility.")


def config_index(sources: Sources, inventory: list[dict], lang: str, registry: dict[str, int]) -> str:
    paths = [item["path"] for item in inventory if
             item["path"].startswith(("ESP32-P4/ServoBoardMain/src/config/",
                                       "ESP32-S3/System/src/config/",
                                       "desktop/resources/config/"))]
    lines = []
    for path in paths:
        if ".before_" in path or path.endswith(".mmcal.json"):
            continue
        rows = sources.lines("mmhand", path)
        keys = []
        for i, row in enumerate(rows, 1):
            match = re.search(r"#define\s+([A-Za-z][A-Za-z0-9_]*)\b", row)
            if not match:
                match = re.search(r"\b(?:constexpr|const)\s+(?:[\w:<>,*]+\s+)+([A-Za-z][A-Za-z0-9_]*)\s*=", row)
            if match:
                keys.append((match[1], i))
            elif path.endswith(".json") and (match := re.search(r'^\s*"([^"]+)"\s*:', row)):
                keys.append((match[1], i))
            if len(keys) >= 4:
                break
        key_text = ", ".join(k for k, _ in keys) or ("需查看字段" if lang == "zh" else "inspect fields")
        ref = sources.evidence("mmhand", path, keys[0][1] if keys else 1)
        lines.append(f"- `{path}` — {key_text} {cite([ref], registry)}")
    teleop = configured_path(sources, "teleop.config", "teleop")
    if teleop in sources.paths["teleop"]:
        keys = []
        for i, row in enumerate(sources.lines("teleop", teleop), 1):
            if match := re.match(r"([A-Z][A-Z0-9_]*)\s*=", row):
                keys.append((match[1], i))
            if len(keys) >= 6:
                break
        lines.append(f"- `mmhand_teleop/{teleop}` — " + ", ".join(k for k, _ in keys)
                     + " " + cite([sources.evidence("teleop", teleop, keys[0][1] if keys else 1)], registry))
    return "\n".join(lines) + "\n"


def formula_sections(sources: Sources, registry: dict[str, int], lang: str) -> str:
    pid = ref_for(sources, "mmhand", configured_path(sources, "control.pid"), "float f_PID_Calculate")
    dual = ref_for(sources, "mmhand", configured_path(sources, "control.dual_tendon"), "const float correction")
    if lang == "zh":
        intro = "以下式子按当前源码的计算顺序展开；输入异常、死区、限幅及模式准入仍以对应分支为准。"
        pid_title, dual_title = "单腱离散 PID", "双腱拮抗控制"
        pid_note = "`dtSeconds` 无效时保持上次输出；非有限输入会清零并报错；死区内保持输出。测量速度用于 D 项，首次样本没有 D 项。"
        dual_note = "每侧电机先映射到标定电机空间，再计算误差；两台电机任一无效或离线时该求解失败。结果随后按 LUT 裕量边界截取。"
    else:
        intro = "These equations follow the current source order. Input errors, deadband, clamps and admission remain governed by their branches."
        pid_title, dual_title = "Single-tendon discrete PID", "Antagonistic dual-tendon control"
        pid_note = "Invalid `dtSeconds` retains the previous output; non-finite input clears the controller and reports an error. The deadband retains output. The D term uses measured velocity and is zero for the first sample."
        dual_note = "Each motor is mapped into calibrated motor space before computing error. Either motor being invalid or offline fails the solver. The target is then clamped to LUT margins."
    return rf"""{intro}

### {pid_title} {cite(pid, registry)}

$$
\begin{{aligned}}
e_k &= r_k-y_k \\
v_k &= v_{{k-1}}+\frac{{\Delta t_k}}{{\max(0,\tau)+\Delta t_k}}
       \left(\frac{{y_k-y_{{k-1}}}}{{\Delta t_k}}-v_{{k-1}}\right) \\
I_k &= \begin{{cases}}
\operatorname{{clip}}(I_{{k-1}}+e_k\Delta t_k,-I_{{\max}},I_{{\max}}),& |e_k|\geq|b|,\ K_i\neq0 \\
0,& |e_k|\geq|b|,\ K_i=0 \\
I_{{k-1}},& |e_k|<|b|
\end{{cases}} \\
u_k &= \begin{{cases}}
K_p e_k+K_i I_k+\operatorname{{clip}}(-K_dv_k,-D_{{\max}},D_{{\max}}),& |e_k|\geq|b| \\
u_{{k-1}},& |e_k|<|b|\text{{ or }}\Delta t_k\leq0
\end{{cases}}
\end{{aligned}}
$$

{pid_note}

### {dual_title} {cite(dual, registry)}

$$
\begin{{aligned}}
e_{{i,k}} &= R_{{i,k}}-F_{{i,k}} \\
\alpha_k &= \frac{{\Delta t_k}}{{\tau+\Delta t_k}} \\
V_{{i,k}} &= V_{{i,k-1}}+\alpha_k\left(\frac{{\Delta R_{{i,k}}-\Delta F_{{i,k}}}}{{\Delta t_k}}-V_{{i,k-1}}\right) \\
I_{{i,k}} &= \operatorname{{clip}}(I_{{i,k-1}}+e_{{i,k}}\Delta t_k,-I_{{\max}},I_{{\max}}) \\
C_{{i,k}} &= K_p e_{{i,k}}+K_i I_{{i,k}}+\operatorname{{clip}}(K_dV_{{i,k}},-D_{{\max}},D_{{\max}}) \\
A_{{i,k}} &= \operatorname{{clip}}_{{[L_i^-,L_i^+]}}\left(A^{{current}}_{{i,k}}+\operatorname{{round}}(C_{{i,k}})\right),\quad i\in\{{0,1\}}
\end{{aligned}}
$$

{dual_note}
"""


def directory_totals(inventory: list[dict]) -> dict[str, int]:
    totals = defaultdict(int)
    for item in inventory:
        path = Path(item["path"])
        totals["."] += item["lines"]
        parts = path.parts[:-1]
        for i in range(1, len(parts) + 1):
            totals["/".join(parts[:i])] += item["lines"]
    return dict(totals)


def markdown(sources: Sources, lang: str, graphs: list[dict], inventory: list[dict],
             registry: dict[str, int], changes: dict[str, list[tuple[int, int]]],
             baseline: str | None, coverage: dict, narrative: dict) -> str:
    headings = T[lang]
    branch = narrative["branch"]
    title = "MMHand AI 代码导览" if lang == "zh" else "MMHand AI code guide"
    p = [f"# {title}\n\n"]
    p.append(f"> mmhand `{sources.commits['mmhand']}` · mmhand_teleop `{sources.commits['teleop']}` · branch `{branch}`.\n\n")
    p.append(f"## 一、{headings[0]}\n\n" if lang == "zh" else f"## 1. {headings[0]}\n\n")
    p.append(narrative.get("overview", {}).get(lang, "") + " "
             + cite(narrative.get("overview_refs", []), registry) + "\n\n")
    if baseline:
        p.append((f"对比基线：`{baseline}`。\n\n" if lang == "zh" else f"Baseline: `{baseline}`.\n\n"))
        changed = [x for x in inventory if x["path"] in changes]
        p.append((f"本次改动涉及 **{len(changes)}** 个跟踪文件，其中 **{len(changed)}** 个计入代码行数。\n\n"
                  if lang == "zh" else f"This comparison changes **{len(changes)}** tracked files, including **{len(changed)}** counted code files.\n\n"))
        for path in sorted(changes):
            p.append(f"- `{path}`\n")
        p.append("\n")
    else:
        p.append(("首次生成：没有可复现的上一版导览基线。\n\n" if lang == "zh" else
                  "Initial generation: no reproducible previous guide baseline.\n\n"))
    p.append(("覆盖检查：" if lang == "zh" else "Coverage: ") + ", ".join(f"{k}={v}" for k, v in sorted(coverage.items()) if isinstance(v, int)) + "。\n\n")
    p.append(("[完整覆盖报告](coverage_report.json)。未核实规则的触发条件须沿图中的源码调用点人工核对。\n\n"
              if lang == "zh" else "[Full coverage report](coverage_report.json). Trace unverified rule triggers through the cited source call sites.\n\n"))
    checks = coverage.get("generator_checks", {})
    if checks:
        p.append(("生成器只读检查：\n\n" if lang == "zh" else "Read-only generator checks:\n\n"))
        for name, result in checks.items():
            state = ("通过" if lang == "zh" else "passed") if result["ok"] else ("需修复" if lang == "zh" else "needs repair")
            detail = result["output"].replace("\n", " ").strip()
            p.append(f"- `{name} --check`: **{state}**.{(' ' + detail) if detail else ''}\n")
        p.append("\n")
    unmapped = coverage.get("unmapped_changes", [])
    if unmapped:
        p.append(("未映射到图的改动：\n\n" if lang == "zh" else "Changes not mapped to graphs:\n\n"))
        p.extend(f"- `{path}`\n" for path in unmapped)
        p.append("\n")
    p.append(f"## 二、{headings[1]}\n\n" if lang == "zh" else f"## 2. {headings[1]}\n\n")
    p.append(operation_section(sources, lang, registry, inventory))
    p.append(safety_section(sources, lang, registry))
    p.append(command_guide(sources, lang, registry, narrative["command_explanations"],
                           narrative["command_evidence"]))
    p.append(f"\n## 三、{headings[2]}\n\n" if lang == "zh" else f"\n## 3. {headings[2]}\n\n")
    p.append(f"[{headings[2]}](behavior.{lang}.html)\n\n")
    p.append(f"## 四、{headings[3]}\n\n" if lang == "zh" else f"## 4. {headings[3]}\n\n")
    p.append(f"[{headings[3]}](architecture.{lang}.html)\n\n")
    p.append(f"## 五、{headings[4]}\n\n" if lang == "zh" else f"## 5. {headings[4]}\n\n")
    p.append(formula_sections(sources, registry, lang))
    p.append(f"\n## 六、{headings[5]}\n\n" if lang == "zh" else f"\n## 6. {headings[5]}\n\n")
    p.append(protocol_section(sources, lang, registry, narrative.get("protocol_en", {})))
    p.append(f"\n## 七、{headings[6]}\n\n" if lang == "zh" else f"\n## 7. {headings[6]}\n\n")
    total = sum(item["lines"] for item in inventory)
    generated = sum(item["lines"] for item in inventory if item["kind"] == "generated")
    third = sum(item["lines"] for item in inventory if item["kind"] == "third_party")
    tooling = sum(item["lines"] for item in inventory if item["kind"] == "tooling")
    p.append((f"项目源码、脚本及配置共 **{total}** 物理行；其中生成代码 **{generated}** 行，第三方代码 **{third}** 行，IDE 工具配置 **{tooling}** 行。空行和注释计入。\n\n"
              if lang == "zh" else f"Tracked source, scripts and configuration total **{total}** physical lines; **{generated}** are generated, **{third}** third-party and **{tooling}** IDE tooling configuration. Blank and comment lines count.\n\n"))
    p.append("### " + ("目录汇总" if lang == "zh" else "Directory totals") + "\n\n")
    for folder, count in sorted(directory_totals(inventory).items()):
        p.append(f"- `{folder}` — {count}\n")
    p.append("\n### " + ("逐文件" if lang == "zh" else "Files") + "\n\n")
    for item in inventory:
        path = item["path"]
        ref = sources.evidence("mmhand", path, 1)
        p.append(f"- `{path}` — {item['lines']} " + ("行" if lang == "zh" else "lines")
                 + f" · {item['kind']} · {file_intro(sources, path, lang)} {cite([ref], registry)}\n")
    p.append(f"\n## 八、{headings[7]}\n\n" if lang == "zh" else f"\n## 8. {headings[7]}\n\n")
    spans = defaultdict(list)
    for evidence in all_evidence(graphs):
        spans[Sources.key(evidence["repo"], evidence["path"])].append(evidence)
    for repo, path, needle in (
        ("mmhand", configured_path(sources, "control.pid"), "float f_PID_Calculate"),
        ("mmhand", configured_path(sources, "control.dual_tendon"), "const float correction"),
        ("teleop", configured_path(sources, "teleop.publisher", "teleop"), "self.robot_publisher ="),
        ("teleop", configured_path(sources, "teleop.track", "teleop"), 'output.add_argument("--ros"'),
    ):
        for evidence in ref_for(sources, repo, path, needle):
            spans[Sources.key(repo, path)].append(evidence)
    keys = sorted(registry, key=lambda key: registry[key])
    for key in keys:
        repo, path = key.split(":", 1)
        if path not in sources.paths.get(repo, set()):
            continue
        rows = sources.lines(repo, path)
        if not rows:
            continue
        ref = sources.evidence(repo, path, 1)
        n = registry[key]
        p.append(f'<a id="ref-{n:04d}"></a>\n\n- **[{n}]** `{repo}:{path}` · '
                 f'[GitHub]({sources.github_url(ref)}) · [local]({sources.local_url(narrative["output_dir"], ref)})\n')
        unique = {(x["start"], x["end"]): x for x in spans.get(key, [])}
        if unique:
            chosen = [unique[k] for k in sorted(unique)[:16]]
            links = ", ".join(f'[L{x["start"]}–L{x["end"]} GitHub]({sources.github_url(x)}) · '
                              f'[local]({sources.local_url(narrative["output_dir"], x)})' for x in chosen)
            p.append(("  - 相关源码行：" if lang == "zh" else "  - Cited source lines: ") + links + "\n")
    return "".join(p)


def operation_section(sources: Sources, lang: str, registry: dict[str, int], inventory: list[dict]) -> str:
    deps = sources.evidence("mmhand", configured_path(sources, "desktop.requirements"), 1)
    entry = ref_for(sources, "mmhand", configured_path(sources, "desktop.entry"), "def build_parser")
    mode = ref_for(sources, "mmhand", configured_path(sources, "desktop.manual_ui"), "def _manual_submode")
    ros = ref_for(sources, "teleop", configured_path(sources, "teleop.publisher", "teleop"), 'self.robot_publisher =')
    track = ref_for(sources, "teleop", configured_path(sources, "teleop.track", "teleop"), 'output.add_argument("--ros"')
    topic = ref_for(sources, "teleop", configured_path(sources, "teleop.config", "teleop"), 'ROBOT_TOPIC =')
    sub = ref_for(sources, "mmhand", configured_path(sources, "desktop.raw_ik"), "create_subscription(")
    robot = ref_for(sources, "mmhand", configured_path(sources, "tools.robot_generator"), 'parser.add_argument("--check"')
    protocol = ref_for(sources, "mmhand", configured_path(sources, "tools.protocol_generator"), 'parser.add_argument("--check"')
    safety = ref_for(sources, "mmhand", configured_path(sources, "tools.safety_generator"), 'parser.add_argument("--check"')
    urdf = ref_for(sources, "mmhand", configured_path(sources, "tools.urdf_update"), "[string]$StructureRepo")
    if lang == "zh":
        return f"""### 安装与启动 {cite([deps] + entry + mode, registry)}

在目标 Python 环境安装源码列出的桌面端依赖：

```powershell
python -m pip install -r desktop/requirements.txt
```

```powershell
python -m desktop ui --mode monitor --no-ros2
python -m desktop ui --mode replay --no-ros2
python -m desktop cli --help
```

监视界面的 Manual 页包含 Device Check、Joint Channels、Motor Channels 和 Test 子页；顶部 Mode 菜单还可进入 Replay。先连接 P4，再在 Device Check 检查或录入 Manual 端点；Joint Channels 使用磁编负反馈，Motor Channels 发送舵机 logical ABS 直控。ENABLE、Pause、Stop、Reset 与 Calibrate 的具体门禁及状态效果请打开第三节的交互图逐节点核对。{cite(mode, registry)}

### ROS2 遥操 {cite(ros + track + topic + sub, registry)}

遥操源码仓库：[mmhand_teleop](https://github.com/mm-hand/mmhand_teleop)。在其根目录执行 `python track.py --ros`：`RosOutput` 将 21 个 J00–J20 角度（degree）作为 `std_msgs/msg/Float32MultiArray` 发布到 `/raw_ik_target`。上位机需显式选择 raw IK 输入：

```powershell
python -m desktop ui --mode monitor --ros2 --ros2-input raw_ik --ros2-raw-ik-topic /raw_ik_target
ros2 topic echo --once /raw_ik_target
```

在 Joint Channels 选择活动范围并完成 ENABLE 后，开启 ROS2 INPUT 授权；输入仍经过目标格式、工作区和 P4 状态门禁。Pause、Stop、范围改变和 Replay 会撤销授权；具体判断式见行为图。{cite(ref_for(sources, 'mmhand', 'desktop/core/hand_backend.py', 'def _set_ros2_output_armed_locked'), registry)}

### 更新 URDF 与生成物 {cite(urdf + robot + protocol + safety, registry)}

结构仓库更新入口为 `tools/update_structure_urdf.ps1 -StructureRepo <路径> -NoPull -SkipP4Build`；脚本同步适配、校验 URDF、生成桌面/P4 模型契约并检查陈旧。直接修改模型清单或活动 URDF 后，运行 `python tools/generate_robot_model.py`，再运行同一命令加 `--check`。修改 `desktop/resources/config/upper_protocol_schema.json` 后运行 `python tools/generate_upper_protocol.py` 及其 `--check`；修改 P4 安全规则目录后运行 `python tools/generate_p4_safety_catalog.py` 及其 `--check`。生成文件应改上游输入后重生成，不要手工改生成物。

### 配置位置

下列条目从已提交配置源码及字段提取；改变值前请查看读取方与对应生成器：

{config_index(sources, inventory, lang, registry)}
"""
    return f"""### Install, launch and operate {cite([deps] + entry + mode, registry)}

Install desktop dependencies listed by the committed requirements file:

```powershell
python -m pip install -r desktop/requirements.txt
```

```powershell
python -m desktop ui --mode monitor --no-ros2
python -m desktop ui --mode replay --no-ros2
python -m desktop cli --help
```

Manual contains Device Check, Joint Channels, Motor Channels and Test. The Mode menu also opens Replay. Connect P4, then inspect or enter Manual endpoints in Device Check. Joint Channels uses encoder feedback; Motor Channels sends logical ABS servo targets. Open the interactive behavior graph for the exact guards and state effects of ENABLE, Pause, Stop, Reset and Calibrate. {cite(mode, registry)}

### ROS2 teleoperation {cite(ros + track + topic + sub, registry)}

Source repository: [mmhand_teleop](https://github.com/mm-hand/mmhand_teleop). Run `python track.py --ros` in its root. `RosOutput` publishes 21 J00–J20 angles in degrees as `std_msgs/msg/Float32MultiArray` on `/raw_ik_target`. Select raw IK input explicitly on the desktop:

```powershell
python -m desktop ui --mode monitor --ros2 --ros2-input raw_ik --ros2-raw-ik-topic /raw_ik_target
ros2 topic echo --once /raw_ik_target
```

Select an active scope in Joint Channels, complete ENABLE, then authorize ROS2 INPUT. Format, workspace and P4 state gates still apply. Pause, Stop, scope changes and Replay revoke authorization; inspect the behavior graph for source predicates. {cite(ref_for(sources, 'mmhand', 'desktop/core/hand_backend.py', 'def _set_ros2_output_armed_locked'), registry)}

### Update URDF and generated files {cite(urdf + robot + protocol + safety, registry)}

Run `tools/update_structure_urdf.ps1 -StructureRepo <path> -NoPull -SkipP4Build` for the external structure repository. It synchronizes the adapter, validates URDF, generates Desktop/P4 contracts and checks freshness. After changing the model manifest or active URDF, run `python tools/generate_robot_model.py` and then its `--check` mode. After editing `desktop/resources/config/upper_protocol_schema.json`, run `python tools/generate_upper_protocol.py` and `--check`. After editing the P4 safety rule catalog, run `python tools/generate_p4_safety_catalog.py` and `--check`. Edit upstream inputs and regenerate rather than editing generated files.

### Configuration locations

These entries are extracted from committed configuration source and fields. Inspect their readers and generators before changing values:

{config_index(sources, inventory, lang, registry)}
"""


def safety_section(sources: Sources, lang: str, registry: dict[str, int]) -> str:
    catalog = ref_for(sources, "mmhand", configured_path(sources, "safety.catalog"), "P4_SAFETY_RULE(")
    generator = ref_for(sources, "mmhand", configured_path(sources, "tools.safety_generator"), "def render_firmware_index_json")
    desktop = ref_for(sources, "mmhand", configured_path(sources, "desktop.safety_rules"), "def safety_rule_table")
    checker = ref_for(sources, "mmhand", configured_path(sources, "tools.safety_audit"), 'parser.add_argument("--json"')
    index = ref_for(sources, "mmhand", configured_path(sources, "safety.index"), '"rules":')
    if lang == "zh":
        return f"""\n### 安全策略位置与集中审计 {cite(catalog + generator + desktop + checker + index, registry)}

`SafetyRuleCatalog.def` 是规则编号、层级、动作和恢复建议的维护源。运行 `python tools/generate_p4_safety_catalog.py` 会同步生成上位机展示目录 `desktop/resources/config/p4_safety_rules.json`、固件枚举头 `SafetyRuleCatalog.generated.h`，以及供离线审计的 `SafetyRuleIndex.generated.json`。后者逐条给出 Policy 判断入口、业务调用、事件上报和恢复入口；上位机通过 `desktop/core/safety_rules.py` 按 rule_id 读取展示目录。生成文件应从目录重生成，不应手工修改。{cite(catalog + generator + desktop + index, registry)}

`python tools/generate_p4_safety_catalog.py --check` 只核对三份生成物是否与目录一致；`python tools/p4_architecture/check_safety_rule_closure.py --json` 把注册、Policy、调用、映射、事件和恢复的闭环检查输出为机器可读 JSON，便于集中查看或重定向保存。闭环入口存在不等于每条规则的触发谓词已被证明；行为图保留“未核实”标记并链接具体源码。{cite(generator + checker, registry)}

"""
    return f"""\n### Safety policy locations and centralized audit {cite(catalog + generator + desktop + checker + index, registry)}

`SafetyRuleCatalog.def` is the maintained source of rule IDs, layers, actions and recovery guidance. `python tools/generate_p4_safety_catalog.py` updates the desktop display catalog `desktop/resources/config/p4_safety_rules.json`, firmware enum header `SafetyRuleCatalog.generated.h`, and offline audit index `SafetyRuleIndex.generated.json`. The index links each rule to its policy, business caller, event reporter and recovery entry; `desktop/core/safety_rules.py` reads the display catalog by rule_id. Regenerate these outputs from the catalog instead of editing them directly. {cite(catalog + generator + desktop + index, registry)}

`python tools/generate_p4_safety_catalog.py --check` only verifies that all three outputs match the catalog. `python tools/p4_architecture/check_safety_rule_closure.py --json` emits machine-readable closure checks for registration, policy, caller, mapping, event and recovery. A complete chain does not by itself prove a rule's precise trigger predicate; the behavior graph retains an unverified marker and source links for that distinction. {cite(generator + checker, registry)}

"""


def command_guide(sources: Sources, lang: str, registry: dict[str, int], explanations: dict,
                  evidence: dict[str, list[dict]]) -> str:
    heading = "逐条命令使用说明" if lang == "zh" else "Command-by-command usage"
    labels = ({"plain": "用途与用法", "condition": "前提与参数", "effect": "源码中的结果",
               "failure": "失败或拒绝", "difference": "与相近命令的区别", "recovery": "处理方式",
               "unknown": "未核实", "change": "本次改动"} if lang == "zh" else
              {"plain": "Purpose and usage", "condition": "Prerequisites and flags", "effect": "Code-level result",
               "failure": "Failure or rejection", "difference": "Compared with related commands", "recovery": "What to do",
               "unknown": "Unverified", "change": "This change"})
    parts = [f"\n### {heading}\n\n"]
    for name, command, repo in COMMANDS:
        ref = evidence[name]
        content = explanations[name][lang]
        workdir = "mmhand_teleop" if repo == "teleop" else "mmhand"
        parts.append(f"#### `{command}` {cite(ref, registry)}\n\n")
        parts.append((f"执行目录：`{workdir}`。\n\n" if lang == "zh" else f"Working directory: `{workdir}`.\n\n"))
        for field in ("plain", "condition", "effect", "failure", "difference", "recovery", "unknown", "change"):
            if content[field].strip():
                parts.append(f"**{labels[field]}：** {content[field]}\n\n")
    return "".join(parts)


def protocol_section(sources: Sources, lang: str, registry: dict[str, int], translated: dict[str, str]) -> str:
    path = configured_path(sources, "protocol.schema")
    schema = json.loads(sources.read("mmhand", path))
    intro = ("命令编号、payload 长度及说明直接取协议 schema；实际准入仍由 P4 处理代码决定。"
             if lang == "zh" else "Command numbers, payload lengths and descriptions come directly from the protocol schema; P4 handlers still determine admission.")
    p = [intro + " " + cite(ref_for(sources, "mmhand", path, '"schema_version"'), registry) + "\n\n"]
    p.append("| Direction | Command | ID | Payload | Description |\n| --- | --- | ---: | ---: | --- |\n")
    for item in schema["commands"]:
        ref = ref_for(sources, "mmhand", path, '"name": "' + item["name"] + '"')
        description = (item["description"] if lang == "zh" else translated[item["name"]]).replace("|", "\\|").replace("\n", " ")
        p.append(f"| Desktop→P4 | `{item['name']}` | {item['value']} | {item['payload_bytes']} B | {description} {cite(ref, registry)} |\n")
    p.append("\n| Direction | Packet | ID | Description |\n| --- | --- | ---: | --- |\n")
    for item in schema["packets"]:
        ref = ref_for(sources, "mmhand", path, '"name": "' + item["name"] + '"')
        description = (item["description"] if lang == "zh" else translated[item["name"]]).replace("|", "\\|").replace("\n", " ")
        p.append(f"| P4→Desktop | `{item['name']}` | {item['value']} | {description} {cite(ref, registry)} |\n")
    return "".join(p)


def html_page(sources: Sources, lang: str, kind: str, graphs: list[dict], registry: dict[str, int],
              changes: dict[str, list[tuple[int, int]]], coverage: dict, output_dir: Path) -> str:
    page_graphs = [g for g in graphs if g["category"] == kind]
    payload = {"lang": lang, "kind": kind, "graphs": page_graphs,
               "meta": {"mmhand": sources.commits["mmhand"],
                        "teleop": sources.commits["teleop"], "coverage": coverage},
               "labels": ({"search": "搜索节点或条件", "changed": "仅看变更", "unknown": "未核实",
                           "plain": "白话解释", "conditionMeaning": "条件拆解", "effect": "结果与状态变化", "safetyChain": "规则调用与恢复链",
                           "failure": "条件不成立、拒绝或异常", "recovery": "恢复方式", "difference": "与相近路径的区别", "changeImpact": "本次改动影响",
                           "back": "返回", "references": "全部源码证据", "explanationEvidence": "本段解释依据", "conditions": "条件判断",
                           "calls": "调用/接线", "period": "周期与通信", "incoming": "进入路径",
                           "outgoing": "离开路径", "source": "源码", "details": "点击节点或连线查看细节",
                           "all": "全部图", "local": "本地", "github": "GitHub", "graphOverview": "本图说明", "graphGlossary": "变量速查"} if lang == "zh" else
                          {"search": "Search nodes or predicates", "changed": "Changed only", "unknown": "Unverified",
                           "plain": "Plain-language explanation", "conditionMeaning": "Condition explained", "effect": "Result and state changes", "safetyChain": "Rule and recovery chain",
                           "failure": "False condition, rejection or fault", "recovery": "Recovery", "difference": "How this path differs", "changeImpact": "Impact of this change",
                           "back": "Back", "references": "All source evidence", "explanationEvidence": "Evidence cited by this explanation", "conditions": "Guard predicates",
                           "calls": "Calls / bindings", "period": "Timing and channels", "incoming": "Incoming paths",
                           "outgoing": "Outgoing paths", "source": "Source", "details": "Select a node or edge for details",
                           "all": "All graphs", "local": "Local", "github": "GitHub", "graphOverview": "Graph overview", "graphGlossary": "Variable glossary"})}
    for g in page_graphs:
        summary = g.get("summary", {})
        summary_refs = [*summary.get("overview", {}).get("evidence", [])]
        for term in summary.get("glossary", []):
            summary_refs.extend(term.get("evidence", []))
        for ref in summary_refs:
            ref["number"] = registry[Sources.key(ref["repo"], ref["path"])]
            ref["github"] = sources.github_url(ref)
            ref["local"] = sources.local_url(output_dir, ref)
            ref["changed"] = range_changed(ref, changes)
        for item in [*g["nodes"], *g["edges"]]:
            refs = [*item["evidence"]]
            for key in ("conditions", "period"):
                for fact in item.get("details", {}).get(key, []):
                    if isinstance(fact, dict) and "evidence" in fact:
                        refs.append(fact["evidence"])
            item["changed"] = any(range_changed(ref, changes) for ref in refs)
            for ref in refs:
                ref["number"] = registry[Sources.key(ref["repo"], ref["path"])]
                ref["github"] = sources.github_url(ref)
                ref["local"] = sources.local_url(output_dir, ref)
                ref["changed"] = range_changed(ref, changes)
    text = json.dumps(payload, ensure_ascii=False).replace("</", "<\\/")
    title = ("操作、状态与保护" if kind == "behavior" else "系统架构与数据流") if lang == "zh" else (
             "Actions, states and protection" if kind == "behavior" else "Architecture and data flow")
    return ("<!doctype html>\n<html lang=\"" + ("zh-CN" if lang == "zh" else "en") +
            "\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">" +
            f"<title>MMHand · {html.escape(title)}</title><link rel=\"stylesheet\" href=\"assets/guide.css\"></head>" +
            f"<body><header><strong>MMHand · {html.escape(title)}</strong><span>{sources.commits['mmhand'][:12]}</span>" +
            f"<a href=\"AI_README.{lang}.md\">README</a><a href=\"{kind}.{'en' if lang == 'zh' else 'zh'}.html\">{'English' if lang == 'zh' else '中文'}</a></header>" +
            "<main><nav><div id=\"toolbar\"></div><div id=\"nav\"></div></nav><section id=\"canvas\"></section>" +
            "<aside><div id=\"breadcrumb\"></div><div id=\"details\"></div></aside></main>" +
            "<script src=\"assets/cytoscape.min.js\"></script><script type=\"application/json\" id=\"guide-data\">" +
            text + "</script><script src=\"assets/guide.js\"></script></body></html>\n")
