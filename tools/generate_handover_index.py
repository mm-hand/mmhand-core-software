"""Generate the handover file inventory from source contracts and Python ASTs."""

from __future__ import annotations

import ast
from collections import Counter, defaultdict
from datetime import date
import os
from pathlib import Path
import re
import subprocess
from urllib.parse import quote

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/交接"
SOURCE = {".py", ".pyi", ".h", ".hpp", ".c", ".cpp", ".ino", ".ps1", ".cmd", ".bat", ".sh"}
SKIP_DIRS = {".git", ".debug", ".codex_tmp", ".pytest_cache", ".understand-anything",
             "__pycache__", "build", "install", "log", "logs", "run_data", "tmp",
             "node_modules", ".venv", "venv", "_render_protocol_docx", "hku_hand_v2_urdf_builtin"}
ROOTS = ("desktop", "ESP32-P4", "ESP32-S3", "tools", "tests", "manus", "wuji_hand_ws",
         "playground", "ESP32-P4ID", "ESP32-P4Test", "ESP32-P4Test2", "ESP32-P4Test3")
OVERRIDES = {
    "ESP32-S3/System/src/config/Config.h": "S3 统一配置：21 路磁编、5 组触觉规模、SPI/CAN 引脚、译码映射、任务周期和串口输出开关。",
    "ESP32-S3/System/src/system/FirmwareLog.h": "S3 日志宏与输出级别；触觉二进制帧与文本共串口时必须保持协议边界。",
    "manus/manus_bridge_test/src/main.cpp": "MANUS Bridge 测试程序入口：解析配置、启动 SDK 回调和打印/CSV/TCP/UDP 输出生命周期。",
    "wuji_hand_ws/src/wuji_mmhand_teleop/setup.py": "ROS2 Python 包构建和资源安装；注册 wuji_raw_ik_target_sender 与 wuji_raw_ik_teleop_ui 两个命令入口。",
    "nodes/sender.py": "Wuji ROS2 发布节点：采集骨架、执行姿态标定与重定向，发布 21 个 URDF 角及标定状态。",
    "sender.py": "Wuji ROS2 发布节点：采集骨架、执行姿态标定与重定向，发布 21 个 URDF 角及标定状态。",
    "manus_calibration.py": "骨架对齐及姿态样本标定管理，供原始骨架重定向复用；文件名保留 MANUS 历史语义。",
    "manus_raw_skeleton.py": "FullRawSkeletonRetargeter：把原始骨架结合标定姿态和 URDF 转成机器手关节目标；Wuji 发布节点实际调用。",
    "stream_writer.py": "校验 21 个角度并写出预览流，供遥操预览/录制消费。",
    "types.py": "定义 WujiFrame 等遥操采集数据结构，约束源时间、骨架和帧状态。",
    "qt_compat.py": "遥操工作区 Qt 兼容导入层，统一 UI 使用的 QtCore/QtGui/QtWidgets。",
    "teleop_ui.py": "Wuji 遥操独立 Qt 窗口：管理采集发送、姿态标定、状态订阅和手型预览。",
    "urdf_viewer.py": "遥操窗口中的 URDF 三维显示，将关节目标映射到模型节点变换。",
    "CsvRecorder.cpp": "将 MANUS SDK 回调产生的帧写入 CSV，用于离线分析。",
    "DataPrinter.cpp": "按配置频率在控制台格式化 MANUS 设备和骨架数据。",
    "ManusClient.cpp": "MANUS Core/SDK 连接、设备发现、流回调注册和最新帧管理。",
    "TcpSender.cpp": "通过 TCP 发送 MANUS JSON 帧，管理连接与发送状态。",
    "UdpSender.cpp": "通过 UDP 数据报发送 MANUS 帧，供桥接测试消费者使用。",
    "mmhand_manus_sdk.cpp": "将 MANUS C++ SDK 封装成 C ABI，提供启动/停止/连接状态/最新 JSON 帧/错误接口。",
    "mmhand_manus_sdk.h": "声明 MANUS 包装层导出 C ABI，调用方须匹配 Windows DLL 架构和 SDK 版本。",
    "ESP32-P4ID.ino": "独立舵机 ID 编程固件，仅操作 P4 BUS0；烧录后替代正式控制固件，使用后需恢复 ServoBoardMain。",
    "ST3215LoadSensitivityTest.ino": "ST3215 负载灵敏度独立测试，测量舵机负载响应。",
    "ST3215SweepDitherTest.ino": "ST3215 扫动/抖动独立测试，验证位置指令下的响应。",
    "ST3215MultiTurnSweepDitherTest.ino": "ST3215 多圈扫动/抖动独立测试，验证跨圈位置控制。",
    "CanWaveform.ino": "独立 CAN 波形验证固件，用于测量收发引脚和总线时序。",
    "I2cWaveform.ino": "独立 I2C 波形验证固件，用于测量 I2C 线和时序。",
    "SpiWaveform.ino": "独立 SPI 波形验证固件，用于测量时钟/数据/片选时序。",
    "UartLoopback.ino": "独立 UART 回环验证固件，用于串口收发连通性检查。",
    "paxini_dist_scan.ino": "Paxini 手动片选扫描：扫描选择线组合，识别物理线模式对应的触觉模块。",
    "paxini_manual_cs_scan.ino": "Paxini 手动片选组合扫描，定位地址选择和模组响应。",
    "paxini_spi_scan.ino": "Paxini SPI 扫描实验，检查触觉组/地址及返回数据。",
    "paxini_spi1_p1_test.ino": "SPI1/P1 单路触觉读取实验，缩小链路故障范围。",
    "paxini_all_read_newboard.ino": "新板触觉多组读取实验，保留用于与正式 HalTactile 对照。",
    "paxini_all_read_2.12.ino": "历史 2.12 触觉多组读取实验，参数与映射不作为当前正式配置。",
    "SafetyRuleCatalog.def": "P4 安全规则唯一目录：固定编号、触发条件和说明；通过生成器同步固件枚举和桌面解释。",
    "build_protocol_docx.py": "把通信协议 Markdown 转成 Word 文档的离线排版脚本。",
    "pre-commit": "提交前钩子；根目录版本执行 P4 架构检查，ESP32-P4 内旧版本会清 BOM 并暂存文件，需核对实际 core.hooksPath。",
    "start_mmhand.cmd": "Windows 双击启动：选择 Python、加载可选 ROS2 环境并调用 desktop ui；失败保留控制台。",
    "System.ino": "S3 正式入口：串口、5 秒上电等待、磁编/触觉/TWAI 初始化，随后创建三个采集通信任务。",
    "ServoBoardMain.ino": "P4 正式 Arduino 入口；setup/loop 委托 AppBootstrap，业务由 FreeRTOS 任务执行。",
    "HalSensorSpi.cpp": "配置传感器 SPI 总线及底层事务，供磁编和触觉 HAL 使用。",
    "HalEncoders.cpp": "读取 21 路 AS5047P；管理译码片选、流水线响应、奇偶校验和错误寄存器诊断。",
    "HalTactile.cpp": "读取 Paxini 模组摘要/触元阵列；管理组地址、读事务时序、CRC 和失败重试诊断。",
    "HalTWAI.cpp": "封装 S3 TWAI 初始化、CAN 帧收发和链路恢复。",
    "EncodersTask.cpp": "周期采集磁编，将最新 EncoderData 覆盖到队列，供 CAN 任务发送。",
    "TactileTask.cpp": "周期采集触觉并输出摘要、全量阵列和诊断；检查采集与输出开关。",
    "CanTask.cpp": "消费最新磁编快照并向 P4 发送 CAN；按配置转发触觉数据。",
    "SystemTasks.cpp": "建立 S3 最新值队列，按优先级和核绑定启动 EncTask、TacTask、CanTask。",
    "official_tactile_layout.py": "定义模组触元到显示阵列的布局；核对 sensor 类型、行列顺序与物理手指映射。",
    "upper_protocol_schema.json": "上位串口协议编号、长度和说明的人工事实源；由 generate_upper_protocol.py 生成两端常量。",
    "robot_model_manifest.json": "机器人模型语义事实源：关节顺序、身份及生成策略；与活动 URDF 一起生成映射和指纹。",
    "calibration_bundle.mmcal.json": "当前设备 V6 标定快照，保存 Manual/Auto/LUT/限位与恢复锚点；交接前单独备份，不能用示例覆盖。",
    "multiturn_runtime.json": "当前设备运行期多圈锚点；与设备身份配套备份，不保证可由 Git 恢复。",
    "p4_safety_rules.json": "从 SafetyRuleCatalog.def 生成的上位机安全规则解释目录。",
    "product_fingerprint.json": "由机器人模型生成的产品指纹，与固件生成头共同用于身份和模型一致性校验。",
    "urdf_joint_mapping.json": "生成的关节编号、名称、范围及映射元数据；更新模型时与 P4 生成物一起更新。",
    "urdf_contract.json": "遥操模型契约：关节顺序、坐标单位与 URDF 哈希；用于跨环境输入校验。",
    "structure_urdf_adapter.json": "外部结构 URDF 到项目关节语义的适配配置。",
    "requirements.txt": "上位机 Python 依赖声明；rclpy 仍需匹配 ROS2 发行版和解释器环境。",
    "create_desktop_shortcut.ps1": "生成 MMHand 桌面快捷方式，固定 Python 和可选 ROS2 setup 路径。",
    "pid.c": "PID 核心更新与历史状态复位；D 项计算须结合调用方传入的反馈速度/滤波配置阅读。",
}


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT).decode("utf-8").strip()


def inventory():
    tracked = set(git("ls-files", "-z").split("\0"))
    paths = {p for p in tracked if p and (ROOT / p).is_file()}
    for folder in ROOTS:
        for base, dirs, files in os.walk(ROOT / folder):
            dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS and not d.endswith(".egg-info"))
            for name in files:
                p = Path(base, name)
                # Include local untracked source, but not run captures or SDK build products.
                if p.suffix.lower() in SOURCE or name in {"CMakeLists.txt", "library.properties"}:
                    paths.add(p.relative_to(ROOT).as_posix())
    paths.add("start_mmhand.cmd")
    # The active model can be a Junction outside the checkout; list its real assets too.
    for base, dirs, files in os.walk(ROOT / "desktop/resources/hku_hand_v2_urdf"):
        dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS)
        paths.update(Path(base, name).relative_to(ROOT).as_posix() for name in files)
    for p in (ROOT / "desktop/resources/config").glob("*.json"):
        if not any(s in p.name for s in ("before_", "legacy_", ".bak")):
            paths.add(p.relative_to(ROOT).as_posix())
    runtime = "desktop/resources/config/device_state/multiturn_runtime.json"
    if (ROOT / runtime).exists():
        paths.add(runtime)
    paths = {p for p in paths if not any(part in SKIP_DIRS for part in Path(p).parts)
             and not p.startswith((".agents/", ".vscode/", "docs/交接/"))
             and Path(p).suffix.lower() not in {".pyc", ".pyo"}}
    # --no-index also excludes previously tracked files now matched by ignore rules.
    ignored = subprocess.run(
        ["git", "check-ignore", "--no-index", "-z", "--stdin"], cwd=ROOT,
        input="\0".join(sorted(paths)).encode("utf-8") + b"\0",
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    if ignored.returncode not in (0, 1):
        raise RuntimeError(ignored.stderr.decode("utf-8", errors="replace"))
    paths.difference_update(ignored.stdout.decode("utf-8").split("\0"))
    return sorted(paths), tracked


def category(p):
    if "/tests/" in p or p.startswith("tests/"):
        return "06_测试文件"
    if p.startswith("ESP32-P4/ServoBoardMain/") and Path(p).suffix in SOURCE:
        return "02_P4逐文件"
    if p.startswith("ESP32-S3/System/") and Path(p).suffix in SOURCE:
        return "03_S3逐文件"
    if p.startswith("desktop/") and Path(p).suffix in SOURCE and "/resources/" not in p:
        return "01_上位机逐文件"
    if (p.startswith(("tools/", "manus/", "wuji_hand_ws/", "playground/", "ESP32-P4Test", "ESP32-P4ID/"))
            and Path(p).suffix in SOURCE and not any(s in p for s in ("/_vendor/", "/third_party/", "/assets/"))):
        return "04_工具与遥操逐文件"
    return "05_资源与第三方文件"


def inspect(p):
    path = ROOT / p
    if path.suffix.lower() not in SOURCE | {".md", ".json", ".txt", ".def", ".xml", ".properties"}:
        return OVERRIDES.get(p, OVERRIDES.get(path.name, "")), [], [], None
    text = path.read_text(encoding="utf-8-sig", errors="replace")
    symbols, deps = [], []
    brief = OVERRIDES.get(p, OVERRIDES.get(path.name, ""))
    if path.suffix == ".py":
        try:
            tree = ast.parse(text)
            doc = ast.get_docstring(tree) or ""
            match = re.search(r"职责：([^\n]+)", doc)
            brief = brief or (match[1] if match else doc.split("\n")[0])
            for node in tree.body:
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                    symbols.append(node.name)
                    if isinstance(node, ast.ClassDef):
                        symbols.extend(f"{node.name}.{m.name}" for m in node.body
                                       if isinstance(m, (ast.FunctionDef, ast.AsyncFunctionDef)) and not m.name.startswith("_"))
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom):
                    mod = "." * node.level + (node.module or "")
                    if node.level or mod.startswith(("desktop", "wuji_mmhand")):
                        deps.append(mod)
        except SyntaxError:
            brief = brief or "Python 文件，静态解析失败；需检查版本语法后维护。"
    elif path.suffix in {".cpp", ".c", ".h", ".hpp", ".ino"}:
        match = re.search(r"@brief\s+([^\r\n]+)", text[:6000])
        brief = brief or (match[1].strip(" */") if match else "")
        if not brief and path.suffix in {".h", ".hpp"}:
            impl = path.with_suffix(".cpp")
            if impl.exists():
                content = impl.read_text(encoding="utf-8-sig", errors="replace")
                match = re.search(r"@brief\s+([^\r\n]+)", content[:6000])
                brief = ("声明接口：" + (OVERRIDES.get(impl.name) or match[1])) if (OVERRIDES.get(impl.name) or match) else ""
        symbols = re.findall(r"^\s*(?:class|struct|enum(?: class)?)\s+(\w+)", text, re.M)
        symbols += re.findall(r"^(?:[\w:*&<>]+[ \t]+)+([\w:]+)\s*\(", text, re.M)
        deps = re.findall(r'^#include\s+"([^"]+)"', text, re.M)
    elif path.suffix == ".md":
        match = re.search(r"^#\s+(.+)", text, re.M)
        brief = "参考文档：" + (match[1] if match else path.stem) + "；历史参数以当前源码为准。"
    elif path.suffix == ".json":
        import json
        try:
            data = json.loads(text)
            symbols = list(data)[:8] if isinstance(data, dict) else []
        except ValueError:
            pass
    return brief, list(dict.fromkeys(symbols)), list(dict.fromkeys(deps)), len(text.splitlines())


def fallback(p):
    path = Path(p)
    if path.name in {".gitignore", ".editorconfig", ".gitmodules", "pytest.ini", "setup.cfg"}:
        return {".gitignore": "Git 忽略规则，区分本机产物、运行数据和应提交源文件。",
                ".editorconfig": "编辑器编码、缩进和换行风格约定。",
                ".gitmodules": "Git 子模块位置与来源配置，交接时确认子模块是否已拉取。",
                "pytest.ini": "测试根目录、Python 导入路径和禁用的 ROS2 pytest 插件。",
                "setup.cfg": "Python/ROS2 安装脚本位置及构建配置。"}[path.name]
    if path.name == "CMakeLists.txt":
        return "CMake 构建定义：源文件、SDK 依赖、链接库及目标输出。"
    if path.suffix in {".sln", ".vcxproj", ".filters", ".iml"}:
        return "IDE 工程配置：构建目标、源文件分组和依赖路径；不是业务实现。"
    if path.suffix in {".docx", ".pdf"}:
        return "历史架构/通信文档或硬件厂商手册；本索引不重新解析其版面，参数以当前源码或匹配型号手册为准。"
    if "license" in p.lower():
        return "第三方许可证或使用条款，随 SDK 配套保存；分发前由交接双方核对授权。"
    if "README" in path.name:
        return "该目录的部署、SDK 放置或使用说明。"
    if "/resource/" in p:
        return "ROS2 ament 包资源索引标记，供环境发现安装后的 Python 包。"
    if path.suffix == ".launch":
        return "URDF 配套 ROS launch 示例，启动显示或仿真；不等同于当前 ROS2 上位机入口。"
    if "SDKClient_Windows/" in p:
        return "MANUS 厂商示例的平台类型适配，用于 Windows/Linux 示例构建。"
    if "test" in path.name and path.suffix == ".py":
        return "自动化回归：" + path.stem.removeprefix("test_") + "；具体断言见右侧测试函数。"
    if path.name == "__init__.py":
        return "Python 包入口，声明包或导出该目录公共接口。"
    if path.suffix.lower() in {".stl", ".obj", ".dae", ".mtl"}:
        return "机器人三维模型资源，供 URDF/姿态可视化引用；须随模型配套交接。"
    if path.suffix.lower() in {".png", ".jpg", ".jpeg", ".svg", ".ico"}:
        return "界面、模型或文档图像资源；用途由所在目录和引用方确定。"
    if "/_vendor/" in p or "/third_party/" in p or "/libraries/" in p:
        return "第三方依赖或厂商示例；按目录保留版本与许可证，修改前检查项目调用方。"
    if path.suffix in {".urdf", ".xml", ".yaml", ".json"}:
        return "模型/配置数据文件；按同目录加载器或生成器解释字段。"
    if path.suffix in {".dll", ".lib", ".so", ".exe", ".pdb"} or ".so." in path.name:
        return "平台相关二进制依赖/产物，不能作为可跨平台重编译的源文件。"
    if path.suffix in {".ps1", ".cmd", ".bat", ".sh"}:
        return "维护脚本：" + path.stem + "；运行前核对脚本内本机路径及设备参数。"
    return "本文件职责需结合下列声明/目录说明阅读：" + path.stem


def cell(value):
    return str(value).replace("|", "\\|").replace("\n", " ").replace("\r", " ")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    paths, tracked = inventory()
    groups = defaultdict(list)
    missing = []
    for p in paths:
        brief, symbols, deps, lines = inspect(p)
        if not brief:
            brief = fallback(p)
            if brief.startswith("本文件职责"):
                missing.append(p)
        status = "Git 已跟踪" if p in tracked else "本机文件；交接时确认纳入"
        if not (ROOT / p).resolve().is_relative_to(ROOT):
            status += "；外部联接资源，单独交付实际内容"
        if "/generated/" in p or "generated_" in p:
            status += "；生成物，修改源定义"
        if p.startswith(("playground/", "ESP32-P4Test", "ESP32-P4ID/")):
            status += "；实验/维护入口"
        if any(x in p for x in ("/_vendor/", "/third_party/", "/libraries/")):
            status += "；第三方"
        link = quote(os.path.relpath(ROOT / p, OUT).replace("\\", "/"), safe="/._-")
        detail = ", ".join(f"`{s}`" for s in symbols[:10]) or "见源码/资源内容"
        if len(symbols) > 10:
            detail += f" 等 {len(symbols)} 个声明"
        if deps:
            detail += "<br>直接依赖：" + ", ".join(f"`{d}`" for d in deps[:6])
            if len(deps) > 6:
                detail += f" 等 {len(deps)} 项"
        groups[category(p)].append((p, f"| [{cell(Path(p).name)}]({link}) | {lines if lines is not None else '资源'} | {cell(brief)} | {cell(detail)} | {cell(status)} |"))
    for name, entries in sorted(groups.items()):
        output = [f"# {name.split('_', 1)[1]}", "", f"生成日期：{date.today()}。共 {len(entries)} 个文件。", "",
                  "[返回接手总览](00_项目接手总览.md) · [覆盖范围与维护方式](07_索引覆盖说明.md)", "",
                  "职责取自源码契约、文件声明及人工补充；主要符号和直接依赖为静态提取，不表示完整运行时调用图。", ""]
        previous = None
        for p, row in sorted(entries, key=lambda item: (str(Path(item[0]).parent), Path(item[0]).name)):
            parent = Path(p).parent.as_posix()
            if parent != previous:
                output.extend(["", f"## `{parent}`", "", "| 文件 | 行数 | 职责 | 阅读入口/依赖 | 交接属性 |",
                               "| --- | ---: | --- | --- | --- |"])
                previous = parent
            output.append(row)
            output.append("") if p == entries[-1][0] else None
        (OUT / f"{name}.md").write_text("\n".join(output) + "\n", encoding="utf-8")
    counts = Counter(category(p) for p in paths)
    summary = ["# 索引覆盖说明", "", f"生成日期：{date.today()}；基线分支 `{git('branch', '--show-current')}`，HEAD `{git('rev-parse', '--short', 'HEAD')}`。",
               "", "以本机当前工作区为准，包含未提交源码，不等价于已经发布或完整通过测试的版本。", "",
               f"本次逐文件索引覆盖 **{len(paths)} 个实际存在的文件**。每个文件只归入一份索引。", ""]
    summary += [f"- [{k.split('_', 1)[1]}]({k}.md)：{v} 个。" for k, v in sorted(counts.items())]
    summary += ["", "## 收录与排除规则", "",
                "收录 Git 跟踪且仍存在的文件及相关目录新增源码，统一按当前 Git 忽略规则过滤。命中 .gitignore 等有效忽略规则的文件不进入逐文件索引，即使它过去已经被 Git 跟踪。未被忽略的新增源码仍保留。",
                "", "排除 .git、缓存、构建目录、colcon install/log、运行 CSV/日志、tmp、历史内置 URDF 副本和本交接目录。二进制/图片/mesh 列明资产用途，不反编译；第三方源码不代表已完成逐行审计。",
                "", "本机文件标记表示当次不在 Git 索引内，不能假定接手人 clone 后一定拥有。当前活动 URDF 及 mesh 即使位于仓库路径下也可能通过 Junction 指向仓库外，索引额外列出其现存资源，标注需单独交付。文件行数用于导航，不作为严格有效代码行数。",
                "", "## 更新方式", "", "在仓库根目录运行：", "", "```powershell", "python tools/generate_handover_index.py", "```", "",
                "生成器只覆盖 01 至 07 索引；人工总览和交接清单不覆盖。优先完善源码文件头中的职责，再在生成器 OVERRIDES 中补充无契约文件。新增业务文件后重新生成，检查链接和 Git 状态后提交。",
                "", "## 职责说明待人工确认", "",
                "以下文件没有明确模块契约，索引已列出其符号或路径；接手时应结合引用确认实际用途，不能把自动摘要当成已完成行为审计。", ""]
    summary += [f"- `{p}`" for p in missing] or ["无未分类职责条目。"]
    (OUT / "07_索引覆盖说明.md").write_text("\n".join(summary) + "\n", encoding="utf-8")
    print(f"Indexed {len(paths)} files; {len(missing)} descriptions need review")
    print("\n".join(missing))


if __name__ == "__main__":
    main()
