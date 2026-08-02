"""将 P4 架构 Markdown 转为带静态框图的 Word 技术文档。"""

from __future__ import annotations

import argparse
import re
import tempfile
from pathlib import Path
from typing import Callable, Iterable

from PIL import Image, ImageDraw, ImageFont
from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK, WD_LINE_SPACING
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = (
    ROOT
    / "ESP32-P4"
    / "ServoBoardMain"
    / "docs"
    / "P4_固件架构重构说明_20260731.md"
)
DEFAULT_OUTPUT = DEFAULT_SOURCE.with_suffix(".docx")

FONT_REGULAR = Path(r"C:\Windows\Fonts\msyh.ttc")
FONT_BOLD = Path(r"C:\Windows\Fonts\msyhbd.ttc")
FONT_MONO = Path(r"C:\Windows\Fonts\consola.ttf")

INK = "#1E293B"
MUTED = "#64748B"
BLUE = "#2563EB"
BLUE_DARK = "#1E3A8A"
BLUE_FILL = "#E8F0FE"
GREEN = "#16865C"
GREEN_FILL = "#E8F7F0"
ORANGE = "#B45309"
ORANGE_FILL = "#FFF4E5"
RED = "#C62828"
RED_FILL = "#FDECEC"
GRAY_FILL = "#F4F6F8"
LINE = "#94A3B8"


def rgb(hex_color: str) -> tuple[int, int, int]:
    value = hex_color.lstrip("#")
    return tuple(int(value[i : i + 2], 16) for i in (0, 2, 4))


def font(size: int, bold: bool = False, mono: bool = False) -> ImageFont.FreeTypeFont:
    path = FONT_MONO if mono else (FONT_BOLD if bold else FONT_REGULAR)
    return ImageFont.truetype(str(path), size)


def rounded_box(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int, int, int],
    text: str,
    *,
    fill: str = BLUE_FILL,
    outline: str = BLUE,
    text_color: str = INK,
    size: int = 25,
    bold: bool = False,
    radius: int = 12,
) -> None:
    draw.rounded_rectangle(xy, radius=radius, fill=rgb(fill), outline=rgb(outline), width=3)
    fnt = font(size, bold=bold)
    left, top, right, bottom = xy
    box = draw.multiline_textbbox((0, 0), text, font=fnt, spacing=7, align="center")
    width = box[2] - box[0]
    height = box[3] - box[1]
    draw.multiline_text(
        ((left + right - width) / 2, (top + bottom - height) / 2 - 2),
        text,
        font=fnt,
        fill=rgb(text_color),
        spacing=7,
        align="center",
    )


def arrow(
    draw: ImageDraw.ImageDraw,
    start: tuple[int, int],
    end: tuple[int, int],
    *,
    color: str = LINE,
    width: int = 4,
    label: str | None = None,
    label_offset: tuple[int, int] = (0, -28),
) -> None:
    draw.line((start, end), fill=rgb(color), width=width)
    x1, y1 = start
    x2, y2 = end
    dx, dy = x2 - x1, y2 - y1
    length = max((dx * dx + dy * dy) ** 0.5, 1.0)
    ux, uy = dx / length, dy / length
    px, py = -uy, ux
    tip = (x2, y2)
    base = (x2 - ux * 18, y2 - uy * 18)
    points = [
        tip,
        (base[0] + px * 8, base[1] + py * 8),
        (base[0] - px * 8, base[1] - py * 8),
    ]
    draw.polygon(points, fill=rgb(color))
    if label:
        fnt = font(20)
        bbox = draw.textbbox((0, 0), label, font=fnt)
        tx = (x1 + x2 - (bbox[2] - bbox[0])) / 2 + label_offset[0]
        ty = (y1 + y2) / 2 + label_offset[1]
        draw.rounded_rectangle(
            (tx - 7, ty - 3, tx + bbox[2] - bbox[0] + 7, ty + bbox[3] - bbox[1] + 5),
            radius=5,
            fill="white",
        )
        draw.text((tx, ty), label, font=fnt, fill=rgb(MUTED))


def poly_arrow(
    draw: ImageDraw.ImageDraw,
    points: list[tuple[int, int]],
    *,
    color: str = LINE,
    width: int = 4,
    label: str | None = None,
    label_at: tuple[int, int] | None = None,
) -> None:
    draw.line(points, fill=rgb(color), width=width, joint="curve")
    arrow(draw, points[-2], points[-1], color=color, width=width)
    if label and label_at:
        draw.text(label_at, label, font=font(19), fill=rgb(MUTED))


def canvas(width: int = 1800, height: int = 1000) -> tuple[Image.Image, ImageDraw.ImageDraw]:
    image = Image.new("RGB", (width, height), "white")
    return image, ImageDraw.Draw(image)


def save_diagram(image: Image.Image, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output, dpi=(180, 180), optimize=True)


def diagram_overview(output: Path) -> None:
    image, draw = canvas(1800, 1080)
    boxes = {
        "host": (40, 80, 300, 180),
        "upper": (360, 80, 650, 180),
        "state": (720, 80, 1000, 180),
        "control": (1100, 35, 1380, 125),
        "calib": (1100, 145, 1380, 235),
        "rope": (1100, 255, 1380, 345),
        "gateway": (1450, 145, 1760, 255),
        "servo": (1180, 435, 1500, 545),
        "driver": (1530, 435, 1760, 545),
        "uart": (1530, 640, 1760, 735),
        "publisher": (820, 435, 1100, 545),
        "shared": (500, 435, 760, 545),
        "can": (40, 435, 300, 545),
        "nvs_task": (920, 760, 1250, 865),
        "nvs_store": (1390, 760, 1760, 865),
    }
    rounded_box(draw, boxes["host"], "Desktop / ROS2 / Replay", fill=GRAY_FILL, outline=MUTED, bold=True)
    rounded_box(draw, boxes["upper"], "UpperCommTask\n协议与遥测", bold=True)
    rounded_box(draw, boxes["state"], "StateMachineTask\n状态 / owner 仲裁", fill=ORANGE_FILL, outline=ORANGE, bold=True)
    rounded_box(draw, boxes["control"], "ControlTask\n反馈控制")
    rounded_box(draw, boxes["calib"], "AutoCalibrationTask\n自动标定")
    rounded_box(draw, boxes["rope"], "RopeInitTask\n回工作区")
    rounded_box(draw, boxes["gateway"], "ActuatorCommandGateway\n唯一命令邮箱入口", fill=GREEN_FILL, outline=GREEN, bold=True)
    rounded_box(draw, boxes["servo"], "ServoCommunicationTask\n总线时序 / 恢复事务", fill=ORANGE_FILL, outline=ORANGE, bold=True)
    rounded_box(draw, boxes["driver"], "FeetechServoDriver\n唯一 SDK 封装", fill=RED_FILL, outline=RED, bold=True)
    rounded_box(draw, boxes["uart"], "4 × UART / ST3215", fill=GRAY_FILL, outline=MUTED)
    rounded_box(draw, boxes["publisher"], "ServoFeedbackPublisher\n反馈快照", fill=GREEN_FILL, outline=GREEN)
    rounded_box(draw, boxes["shared"], "TaskSharedData\n共享快照 / token", fill=GREEN_FILL, outline=GREEN, bold=True)
    rounded_box(draw, boxes["can"], "CanCommTask\n磁编 / 触觉", fill=GRAY_FILL, outline=MUTED)
    rounded_box(draw, boxes["nvs_task"], "MultiTurnNvsPersistenceTask\n异步 Flash 写入", fill=ORANGE_FILL, outline=ORANGE)
    rounded_box(draw, boxes["nvs_store"], "MultiTurnNvsStore\nA/B 快照 + CRC", fill=GRAY_FILL, outline=MUTED)
    arrow(draw, (300, 130), (360, 130))
    arrow(draw, (650, 130), (720, 130))
    poly_arrow(draw, [(1000, 130), (1050, 130), (1050, 80), (1100, 80)])
    arrow(draw, (1000, 130), (1100, 190))
    poly_arrow(draw, [(1000, 130), (1050, 130), (1050, 300), (1100, 300)])
    arrow(draw, (1380, 80), (1450, 175))
    arrow(draw, (1380, 190), (1450, 200))
    arrow(draw, (1380, 300), (1450, 225))
    poly_arrow(draw, [(1605, 255), (1605, 380), (1340, 380), (1340, 435)])
    arrow(draw, (1500, 490), (1530, 490))
    arrow(draw, (1645, 545), (1645, 640))
    arrow(draw, (1530, 490), (1100, 490), color=GREEN)
    arrow(draw, (820, 490), (760, 490), color=GREEN)
    arrow(draw, (300, 490), (500, 490), color=GREEN)
    poly_arrow(draw, [(630, 435), (630, 300), (860, 300), (860, 180)], color=GREEN)
    poly_arrow(draw, [(630, 435), (630, 360), (1210, 360), (1210, 345)], color=GREEN)
    poly_arrow(draw, [(630, 435), (630, 240), (505, 240), (505, 180)], color=GREEN)
    arrow(draw, (1340, 545), (1085, 760), color=ORANGE)
    arrow(draw, (1250, 812), (1390, 812), color=ORANGE)
    draw.text((45, 990), "蓝：业务与协议   橙：状态/任务编排   绿：统一数据边界   红：硬件 SDK 边界", font=font(23), fill=rgb(MUTED))
    save_diagram(image, output)


def diagram_priorities(output: Path) -> None:
    image, draw = canvas(1800, 520)
    labels = [
        ("P5\nStateMachine", ORANGE_FILL, ORANGE),
        ("P4\nControl", BLUE_FILL, BLUE),
        ("P3\nCAN", GREEN_FILL, GREEN),
        ("P3\nServo", RED_FILL, RED),
        ("P2\nUpper", BLUE_FILL, BLUE),
        ("P2\nAutoCalibration", ORANGE_FILL, ORANGE),
        ("P1\nNVS", GRAY_FILL, MUTED),
        ("P1\nMonitor", GRAY_FILL, MUTED),
    ]
    xs = [45, 270, 495, 720, 945, 1170, 1395, 1590]
    for i, ((text, fill, outline), x) in enumerate(zip(labels, xs)):
        right = x + (180 if i < 6 else 165)
        rounded_box(draw, (x, 175, right, 330), text, fill=fill, outline=outline, bold=True, size=24)
        if i:
            arrow(draw, (xs[i - 1] + (180 if i - 1 < 6 else 165), 252), (x, 252))
    draw.text((48, 65), "任务优先级只决定抢占顺序，不决定命令所有权", font=font(34, bold=True), fill=rgb(BLUE_DARK))
    draw.text((48, 390), "owner、queue、mutex 和 generation token 共同决定数据归属。", font=font(25), fill=rgb(MUTED))
    save_diagram(image, output)


def diagram_sequence(output: Path) -> None:
    image, draw = canvas(1800, 1120)
    participants = ["Desktop /\nROS2", "UpperComm\nTask", "Command\nRouter", "StateMachine\nTask", "Control\nTask", "Command\nGateway", "ServoComm\nTask", "Feetech\nDriver"]
    xs = [110, 330, 550, 770, 990, 1210, 1430, 1650]
    for x, name in zip(xs, participants):
        rounded_box(draw, (x - 85, 45, x + 85, 135), name, fill=GRAY_FILL, outline=MUTED, size=20, bold=True)
        draw.line((x, 135, x, 1060), fill=rgb("#CBD5E1"), width=3)
    events = [
        (0, 1, 205, "CMD_ANGLE_CTRL\n21 × float"),
        (1, 2, 300, "校验角度数组"),
        (2, 3, 395, "写 targetAngles / token"),
        (3, 4, 490, "owner=CONTROL\ncontrol_enabled=1"),
        (4, 4, 595, "qRef / qFb\nLUT + PID + 碰撞策略"),
        (4, 5, 710, "publishLatest(batch)"),
        (5, 6, 805, "最新值邮箱"),
        (6, 7, 900, "logical ABS → hardware ABS\n同步写"),
    ]
    for src, dst, y, label in events:
        if src == dst:
            draw.arc((xs[src] - 15, y - 25, xs[src] + 110, y + 45), 80, 280, fill=rgb(BLUE), width=4)
            draw.text((xs[src] + 35, y - 32), label, font=font(19), fill=rgb(INK))
        else:
            arrow(draw, (xs[src], y), (xs[dst], y), color=BLUE, label=label, label_offset=(0, -56))
    draw.rounded_rectangle((930, 550, 1740, 970), radius=16, outline=rgb(GREEN), width=3)
    draw.text((950, 565), "每 10 ms 控制循环", font=font(24, bold=True), fill=rgb(GREEN))
    save_diagram(image, output)


def diagram_safety(output: Path) -> None:
    image, draw = canvas(1800, 580)
    nodes = [
        ("业务目标\nbatch", BLUE_FILL, BLUE),
        ("Arbiter\nowner 匹配", ORANGE_FILL, ORANGE),
        ("SafetyPolicy\n批次结构合法", ORANGE_FILL, ORANGE),
        ("ServoTask\n在线 / overflow", GREEN_FILL, GREEN),
        ("Driver\nlogical → hardware", RED_FILL, RED),
        ("ABS 在\n±30719?", ORANGE_FILL, ORANGE),
    ]
    xs = [45, 310, 575, 840, 1105, 1370]
    for i, ((label, fill, outline), x) in enumerate(zip(nodes, xs)):
        rounded_box(draw, (x, 205, x + 220, 345), label, fill=fill, outline=outline, bold=True, size=23)
        if i:
            arrow(draw, (xs[i - 1] + 220, 275), (x, 275))
    rounded_box(draw, (1580, 75, 1770, 190), "拒绝\n诊断位图", fill=RED_FILL, outline=RED, bold=True, size=22)
    rounded_box(draw, (1580, 355, 1770, 470), "飞特 SDK\n同步写", fill=GREEN_FILL, outline=GREEN, bold=True, size=22)
    arrow(draw, (1590, 240), (1675, 190), color=RED, label="否", label_offset=(20, -4))
    arrow(draw, (1590, 310), (1675, 355), color=GREEN, label="是", label_offset=(20, -4))
    save_diagram(image, output)


def diagram_host_control_lease(output: Path) -> None:
    """绘制上位机控制租约丢失后的 ABS Hold 确认闭环。"""
    image, draw = canvas(1800, 760)
    steps = [
        (55, "Host heartbeat\n500 ms 周期", BLUE_FILL, BLUE),
        (350, "租约超时\n> 1000 ms", ORANGE_FILL, ORANGE),
        (645, "停止 Control\n清旧目标邮箱", ORANGE_FILL, ORANGE),
        (940, "发布 active 电机\n当前 ABS Hold", GREEN_FILL, GREEN),
        (1235, "ServoTask 最终裁决\n同步写 + token 确认", GREEN_FILL, GREEN),
    ]
    for index, (x, label, fill, outline) in enumerate(steps):
        rounded_box(draw, (x, 230, x + 245, 375), label,
                    fill=fill, outline=outline, bold=True, size=22)
        if index:
            previous_x = steps[index - 1][0]
            arrow(draw, (previous_x + 245, 302), (x, 302))

    rounded_box(draw, (1515, 80, 1765, 225),
                "APPLIED\n释放 owner\nREADY / IDLE",
                fill=GREEN_FILL, outline=GREEN, bold=True, size=22)
    rounded_box(draw, (1515, 465, 1765, 610),
                "FAILED / 超时\n锁存 E-STOP",
                fill=RED_FILL, outline=RED, bold=True, size=22)
    arrow(draw, (1480, 270), (1640, 225), color=GREEN,
          label="全部写入", label_offset=(15, -24))
    arrow(draw, (1480, 345), (1640, 465), color=RED,
          label="拒绝或 500 ms 未确认", label_offset=(0, 18))
    draw.text((60, 625),
              "CALIBRATION owner（自动标定 / Rope Init）使用固件内部生命周期，不依赖上位机租约。",
              font=font(24, bold=True), fill=rgb(BLUE_DARK))
    save_diagram(image, output)


def diagram_feedback(output: Path) -> None:
    image, draw = canvas(1800, 760)
    rounded_box(draw, (55, 275, 285, 405), "ST3215\n反馈寄存器", fill=RED_FILL, outline=RED, bold=True)
    rounded_box(draw, (370, 275, 650, 405), "FeetechServoDriver\n读取 / 解码", fill=RED_FILL, outline=RED, bold=True)
    rounded_box(draw, (735, 275, 1045, 405), "ServoFeedbackPublisher\n统一发布", fill=GREEN_FILL, outline=GREEN, bold=True)
    outputs = [
        (1135, 75, "ServoAngleData\nraw / logical ABS"),
        (1135, 235, "ServoTelemetry\nload / speed / voltage"),
        (1135, 395, "online / raw_online\nbitmaps"),
        (1135, 555, "恢复 token / NVS\n诊断状态"),
    ]
    for x, y, label in outputs:
        rounded_box(draw, (x, y, x + 300, y + 105), label, fill=BLUE_FILL, outline=BLUE, size=22)
        arrow(draw, (1045, 340), (x, y + 52), color=GREEN)
    consumers = [
        (1515, 125, "Control"),
        (1515, 315, "StateMachine"),
        (1515, 505, "UpperComm"),
    ]
    for x, y, label in consumers:
        rounded_box(draw, (x, y, x + 225, y + 110), label, fill=GRAY_FILL, outline=MUTED, bold=True)
        arrow(draw, (1435, 287), (x, y + 55))
    arrow(draw, (285, 340), (370, 340))
    arrow(draw, (650, 340), (735, 340))
    save_diagram(image, output)


def diagram_state_machine(output: Path) -> None:
    image, draw = canvas(1800, 1020)
    nodes = {
        "unctrl": (65, 410, 330, 530, "UNCONTROLLABLE"),
        "idle": (445, 410, 660, 530, "IDLE"),
        "ready": (780, 410, 995, 530, "READY"),
        "feedback": (1130, 220, 1430, 340, "RUN_FEEDBACK"),
        "direct": (1130, 600, 1430, 720, "RUN_DIRECT"),
        "calib": (480, 75, 780, 195, "CALIBRATION_RUNNING"),
        "rope": (825, 75, 1100, 195, "ROPE_INIT_RUNNING"),
        "recovery": (1450, 410, 1750, 530, "RECOVERY_REQUIRED"),
        "pause": (1510, 75, 1710, 195, "PAUSED"),
    }
    colors = {
        "unctrl": (RED_FILL, RED),
        "idle": (GRAY_FILL, MUTED),
        "ready": (GREEN_FILL, GREEN),
        "feedback": (BLUE_FILL, BLUE),
        "direct": (BLUE_FILL, BLUE),
        "calib": (ORANGE_FILL, ORANGE),
        "rope": (ORANGE_FILL, ORANGE),
        "recovery": (RED_FILL, RED),
        "pause": (ORANGE_FILL, ORANGE),
    }
    for key, (l, t, r, b, label) in nodes.items():
        fill, outline = colors[key]
        rounded_box(draw, (l, t, r, b), label, fill=fill, outline=outline, bold=True, size=22)
    arrow(draw, (330, 470), (445, 470), label="Manual/Auto/在线可信", label_offset=(0, -55))
    arrow(draw, (660, 470), (780, 470), label="进入工作区", label_offset=(0, -45))
    arrow(draw, (995, 450), (1130, 295), label="Enable / ROS2 / Replay", label_offset=(0, -48))
    arrow(draw, (995, 500), (1130, 660), label="直控准入", label_offset=(0, 14))
    arrow(draw, (1430, 280), (1510, 160), label="Pause")
    arrow(draw, (1710, 160), (1430, 255), label="Resume", label_offset=(0, -5))
    poly_arrow(draw, [(1430, 660), (1510, 660), (1510, 530)], color=RED, label="多圈/恢复异常", label_at=(1450, 585))
    arrow(draw, (1430, 470), (1450, 470), color=RED)
    arrow(draw, (600, 410), (625, 195), label="Calibrate", label_offset=(50, -5))
    arrow(draw, (780, 135), (825, 135), label="完成/需要回工作区")
    poly_arrow(draw, [(960, 195), (960, 330), (890, 330), (890, 410)], color=GREEN, label="完成", label_at=(900, 280))
    poly_arrow(draw, [(1250, 600), (1250, 550), (890, 550), (890, 530)], color=GREEN, label="完成", label_at=(1030, 555))
    poly_arrow(draw, [(1280, 340), (1280, 390), (890, 390), (890, 410)], color=GREEN, label="Stop", label_at=(1060, 360))
    draw.text((65, 865), "任何状态：E-STOP / 系统故障 → 安全停机或 RECOVERY_REQUIRED", font=font(27, bold=True), fill=rgb(RED))
    save_diagram(image, output)


def diagram_calibration(output: Path) -> None:
    image, draw = canvas(1800, 980)
    labels = [
        ("申请 owner\nCALIBRATION", ORANGE_FILL, ORANGE),
        ("解析 active joints\n在线 / Turn 检查", BLUE_FILL, BLUE),
        ("工作空间规划\n双腱辅助姿态", BLUE_FILL, BLUE),
        ("紧端机械限位\n记录 Auto Max", GREEN_FILL, GREEN),
        ("PRE_RELEASE_SYNC\n目标钳到 currentAbs", ORANGE_FILL, ORANGE),
        ("松绳扫描 / 采样\n记录 Auto Min", GREEN_FILL, GREEN),
        ("LUT Builder\n角度—舵机轨迹", BLUE_FILL, BLUE),
        ("提交标定结果\n绑定 Manual CRC", ORANGE_FILL, ORANGE),
        ("更新可信位\n完成 / 警告", GREEN_FILL, GREEN),
    ]
    positions = [
        (100, 60),
        (580, 60),
        (1060, 60),
        (1060, 270),
        (580, 270),
        (100, 270),
        (100, 515),
        (580, 515),
        (1060, 515),
    ]
    for i, ((label, fill, outline), (x, y)) in enumerate(zip(labels, positions)):
        rounded_box(draw, (x, y, x + 360, y + 140), label, fill=fill, outline=outline, bold=True, size=24)
        if i:
            px, py = positions[i - 1]
            if py == y:
                if x > px:
                    arrow(draw, (px + 360, py + 70), (x, y + 70))
                else:
                    arrow(draw, (px, py + 70), (x + 360, y + 70))
            else:
                arrow(draw, (px + 180, py + 140), (x + 180, y))
    rounded_box(draw, (100, 760, 1420, 885), "失败路径：停止舵机 → 记录 fault / failed mask → 保留已成功关节 → UI 给出可执行建议", fill=RED_FILL, outline=RED, bold=True, size=26)
    arrow(draw, (1240, 655), (1240, 760), color=RED, label="任一步骤失败")
    save_diagram(image, output)


def diagram_nvs(output: Path) -> None:
    image, draw = canvas(1800, 800)
    participants = ["ServoTask", "NVS Job Queue", "PersistenceTask", "MultiTurnNvsStore", "Flash A/B"]
    xs = [150, 500, 850, 1200, 1550]
    for x, label in zip(xs, participants):
        rounded_box(draw, (x - 115, 45, x + 115, 135), label, fill=GRAY_FILL, outline=MUTED, bold=True, size=21)
        draw.line((x, 135, x, 745), fill=rgb("#CBD5E1"), width=3)
    events = [
        (0, 1, 220, "提交 snapshot job"),
        (1, 2, 315, "异步取 job"),
        (2, 3, 410, "save(snapshot)"),
        (3, 4, 505, "写非活动槽 + CRC"),
        (4, 3, 600, "读回校验"),
        (3, 0, 690, "result token / save status"),
    ]
    for src, dst, y, label in events:
        color = GREEN if src == 4 else BLUE
        arrow(draw, (xs[src], y), (xs[dst], y), color=color, label=label, label_offset=(0, -38))
    save_diagram(image, output)


def diagram_portable_target(output: Path) -> None:
    image, draw = canvas(1800, 1020)
    rounded_box(draw, (60, 80, 430, 220), "Application\nUse Cases / State Machine", fill=ORANGE_FILL, outline=ORANGE, bold=True, size=27)
    rounded_box(draw, (565, 55, 1235, 245), "Portable Core\nControl / Calibration / Recovery\nMapping / LUT / Protocol Schema", fill=GREEN_FILL, outline=GREEN, bold=True, size=29)
    rounded_box(draw, (1370, 80, 1740, 220), "Ports\nClock / RTOS / Storage\nCAN / UART / Log", fill=BLUE_FILL, outline=BLUE, bold=True, size=25)
    arrow(draw, (430, 150), (565, 150))
    arrow(draw, (1235, 150), (1370, 150))
    platforms = [
        (80, 430, "platform/esp32p4\nArduino + FreeRTOS\nTWAI + NVS"),
        (610, 430, "platform/stm32\nHAL / LL + CMSIS-RTOS\nFDCAN + Flash"),
        (1140, 430, "platform/host\nFake Clock / Fake Bus\n单元测试与回放"),
    ]
    for x, y, label in platforms:
        rounded_box(draw, (x, y, x + 450, y + 170), label, fill=GRAY_FILL, outline=MUTED, bold=True, size=25)
        arrow(draw, (1555, 220), (x + 225, y), color=BLUE)
    targets = [
        (80, 760, "ESP32-P4 硬件"),
        (610, 760, "STM32 硬件"),
        (1140, 760, "PC 仿真 / CI"),
    ]
    for x, y, label in targets:
        rounded_box(draw, (x, y, x + 450, y + 125), label, fill=BLUE_FILL, outline=BLUE, bold=True, size=27)
        arrow(draw, (x + 225, 600), (x + 225, y))
    save_diagram(image, output)


DIAGRAM_BUILDERS: list[tuple[str, Callable[[Path], None]]] = [
    ("总体架构", diagram_overview),
    ("FreeRTOS 任务优先级", diagram_priorities),
    ("关节反馈控制命令时序", diagram_sequence),
    ("舵机命令安全流水线", diagram_safety),
    ("上位机控制租约与失联 Hold", diagram_host_control_lease),
    ("舵机反馈发布链路", diagram_feedback),
    ("P4 顶层状态机", diagram_state_machine),
    ("自动标定主流程", diagram_calibration),
    ("多圈 NVS 异步保存时序", diagram_nvs),
    ("面向 STM32 的目标架构", diagram_portable_target),
]


def set_cell_margins(cell, top: int = 80, start: int = 120, bottom: int = 80, end: int = 120) -> None:
    tc = cell._tc
    tc_pr = tc.get_or_add_tcPr()
    tc_mar = tc_pr.first_child_found_in("w:tcMar")
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for name, value in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tc_mar.find(qn(f"w:{name}"))
        if node is None:
            node = OxmlElement(f"w:{name}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def set_cell_fill(cell, color: str) -> None:
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = tc_pr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tc_pr.append(shd)
    shd.set(qn("w:fill"), color.lstrip("#"))


def set_cell_width(cell, width: int) -> None:
    tc_pr = cell._tc.get_or_add_tcPr()
    tc_w = tc_pr.find(qn("w:tcW"))
    if tc_w is None:
        tc_w = OxmlElement("w:tcW")
        tc_pr.append(tc_w)
    tc_w.set(qn("w:w"), str(width))
    tc_w.set(qn("w:type"), "dxa")


def add_page_number(paragraph) -> None:
    paragraph.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    run = paragraph.add_run("第 ")
    run.font.size = Pt(9)
    field = OxmlElement("w:fldSimple")
    field.set(qn("w:instr"), "PAGE")
    paragraph._p.append(field)
    run = paragraph.add_run(" 页")
    run.font.size = Pt(9)


def set_east_asia_font(run, name: str = "Microsoft YaHei") -> None:
    run.font.name = name
    run._element.rPr.rFonts.set(qn("w:eastAsia"), name)


def add_inline_runs(paragraph, text: str) -> None:
    """解析少量 Markdown 行内格式，避免在 Word 中残留反引号和粗体标记。"""
    pattern = re.compile(r"(`[^`]+`|\*\*[^*]+\*\*|\[[^\]]+\]\([^)]+\))")
    cursor = 0
    for match in pattern.finditer(text):
        if match.start() > cursor:
            run = paragraph.add_run(text[cursor : match.start()])
            set_east_asia_font(run)
        token = match.group(0)
        if token.startswith("`"):
            run = paragraph.add_run(token[1:-1])
            set_east_asia_font(run, "Consolas")
            run.font.name = "Consolas"
            run.font.size = Pt(9.5)
            run.font.color.rgb = RGBColor(*rgb(BLUE_DARK))
        elif token.startswith("**"):
            run = paragraph.add_run(token[2:-2])
            set_east_asia_font(run)
            run.bold = True
        else:
            label = token[1 : token.index("]")]
            target = token[token.index("(") + 1 : -1]
            run = paragraph.add_run(f"{label}（{target}）")
            set_east_asia_font(run)
            run.font.color.rgb = RGBColor(*rgb(BLUE))
        cursor = match.end()
    if cursor < len(text):
        run = paragraph.add_run(text[cursor:])
        set_east_asia_font(run)


def clean_table_cell(text: str) -> str:
    text = text.strip()
    text = re.sub(r"`([^`]+)`", r"\1", text)
    text = re.sub(r"\*\*([^*]+)\*\*", r"\1", text)
    return text


def table_widths(rows: list[list[str]], total: int = 9360) -> list[int]:
    column_count = max(len(row) for row in rows)
    scores = [3.0] * column_count
    for index in range(column_count):
        lengths = [len(row[index]) if index < len(row) else 0 for row in rows]
        scores[index] = min(max(max(lengths, default=4), 5), 34)
    score_sum = sum(scores)
    widths = [max(720, int(total * score / score_sum)) for score in scores]
    correction = total - sum(widths)
    widths[-1] += correction
    return widths


def add_markdown_table(document: Document, rows: list[list[str]]) -> None:
    column_count = max(len(row) for row in rows)
    table = document.add_table(rows=len(rows), cols=column_count)
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    table.style = "Table Grid"
    widths = table_widths(rows)
    for row_index, row in enumerate(rows):
        for col_index in range(column_count):
            cell = table.cell(row_index, col_index)
            set_cell_width(cell, widths[col_index])
            set_cell_margins(cell)
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            if row_index == 0:
                set_cell_fill(cell, "#E8EEF5")
            paragraph = cell.paragraphs[0]
            paragraph.paragraph_format.space_after = Pt(0)
            paragraph.paragraph_format.line_spacing = 1.05
            text = clean_table_cell(row[col_index]) if col_index < len(row) else ""
            run = paragraph.add_run(text)
            set_east_asia_font(run)
            run.font.size = Pt(8.4)
            run.bold = row_index == 0
    document.add_paragraph().paragraph_format.space_after = Pt(0)


def add_code_block(document: Document, lines: list[str], language: str) -> None:
    paragraph = document.add_paragraph()
    paragraph.paragraph_format.left_indent = Inches(0.18)
    paragraph.paragraph_format.right_indent = Inches(0.18)
    paragraph.paragraph_format.space_before = Pt(3)
    paragraph.paragraph_format.space_after = Pt(7)
    paragraph.paragraph_format.line_spacing = 1.0
    p_pr = paragraph._p.get_or_add_pPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), "F3F5F7")
    p_pr.append(shd)
    border = OxmlElement("w:pBdr")
    left = OxmlElement("w:left")
    left.set(qn("w:val"), "single")
    left.set(qn("w:sz"), "18")
    left.set(qn("w:color"), "5B8DEF")
    border.append(left)
    p_pr.append(border)
    code = "\n".join(lines).rstrip()
    run = paragraph.add_run(code)
    set_east_asia_font(run, "Consolas")
    run.font.name = "Consolas"
    run.font.size = Pt(8.2)
    run.font.color.rgb = RGBColor(*rgb(INK))


def add_diagram(document: Document, image_path: Path, caption: str) -> None:
    paragraph = document.add_paragraph()
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    paragraph.paragraph_format.keep_together = True
    paragraph.paragraph_format.space_before = Pt(5)
    run = paragraph.add_run()
    with Image.open(image_path) as image:
        aspect = image.height / image.width
    width = 6.45
    height = min(width * aspect, 5.8)
    run.add_picture(str(image_path), width=Inches(width), height=Inches(height))
    caption_p = document.add_paragraph()
    caption_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    caption_p.paragraph_format.space_after = Pt(8)
    caption_run = caption_p.add_run(f"图：{caption}")
    set_east_asia_font(caption_run)
    caption_run.font.size = Pt(9)
    caption_run.font.color.rgb = RGBColor(*rgb(MUTED))
    caption_run.italic = True


def configure_styles(document: Document) -> None:
    styles = document.styles
    normal = styles["Normal"]
    normal.font.name = "Microsoft YaHei"
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    normal.font.size = Pt(10.5)
    normal.font.color.rgb = RGBColor(*rgb(INK))
    normal.paragraph_format.line_spacing_rule = WD_LINE_SPACING.MULTIPLE
    normal.paragraph_format.line_spacing = 1.22
    normal.paragraph_format.space_after = Pt(5)

    for name, size, color, before, after in (
        ("Title", 26, BLUE_DARK, 0, 12),
        ("Heading 1", 16, BLUE_DARK, 16, 8),
        ("Heading 2", 13, BLUE, 12, 6),
        ("Heading 3", 11.5, BLUE_DARK, 9, 4),
    ):
        style = styles[name]
        style.font.name = "Microsoft YaHei"
        style._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
        style.font.size = Pt(size)
        style.font.bold = True
        style.font.color.rgb = RGBColor(*rgb(color))
        style.paragraph_format.space_before = Pt(before)
        style.paragraph_format.space_after = Pt(after)
        style.paragraph_format.keep_with_next = True

    for name in ("List Bullet", "List Number"):
        style = styles[name]
        style.font.name = "Microsoft YaHei"
        style._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
        style.font.size = Pt(10.3)
        style.paragraph_format.left_indent = Inches(0.28)
        style.paragraph_format.first_line_indent = Inches(-0.18)
        style.paragraph_format.space_after = Pt(3)


def create_cover(document: Document, metadata: list[str]) -> None:
    paragraph = document.add_paragraph()
    paragraph.paragraph_format.space_before = Inches(1.05)
    paragraph.alignment = WD_ALIGN_PARAGRAPH.LEFT
    run = paragraph.add_run("ESP32-P4")
    set_east_asia_font(run)
    run.font.size = Pt(15)
    run.bold = True
    run.font.color.rgb = RGBColor(*rgb(BLUE))

    title = document.add_paragraph(style="Title")
    title.alignment = WD_ALIGN_PARAGRAPH.LEFT
    run = title.add_run("固件架构重构说明")
    set_east_asia_font(run)

    subtitle = document.add_paragraph()
    subtitle.paragraph_format.space_after = Pt(20)
    run = subtitle.add_run("控制链路、状态机、安全边界与 STM32 迁移路线")
    set_east_asia_font(run)
    run.font.size = Pt(14)
    run.font.color.rgb = RGBColor(*rgb(MUTED))

    line = document.add_paragraph()
    line.paragraph_format.space_after = Pt(22)
    p_pr = line._p.get_or_add_pPr()
    p_bdr = OxmlElement("w:pBdr")
    bottom = OxmlElement("w:bottom")
    bottom.set(qn("w:val"), "single")
    bottom.set(qn("w:sz"), "18")
    bottom.set(qn("w:color"), BLUE.lstrip("#"))
    p_bdr.append(bottom)
    p_pr.append(p_bdr)

    for item in metadata:
        p = document.add_paragraph()
        p.paragraph_format.space_after = Pt(4)
        run = p.add_run(item)
        set_east_asia_font(run)
        run.font.size = Pt(10.5)
        run.font.color.rgb = RGBColor(*rgb(MUTED))

    p = document.add_paragraph()
    p.paragraph_format.space_before = Inches(2.4)
    run = p.add_run("面向 ServoBoardMain 的可维护架构基线")
    set_east_asia_font(run)
    run.font.size = Pt(11)
    run.bold = True
    run.font.color.rgb = RGBColor(*rgb(BLUE_DARK))
    p.add_run().add_break(WD_BREAK.PAGE)


def parse_table(lines: list[str], start: int) -> tuple[list[list[str]], int]:
    rows: list[list[str]] = []
    index = start
    while index < len(lines) and lines[index].strip().startswith("|"):
        cells = [cell.strip() for cell in lines[index].strip().strip("|").split("|")]
        if not all(re.fullmatch(r":?-{3,}:?", cell) for cell in cells):
            rows.append(cells)
        index += 1
    return rows, index


def build_document(source: Path, output: Path, diagram_dir: Path) -> None:
    text = source.read_text(encoding="utf-8")
    lines = text.splitlines()
    diagram_paths: list[Path] = []
    for index, (_, builder) in enumerate(DIAGRAM_BUILDERS, start=1):
        path = diagram_dir / f"diagram_{index:02d}.png"
        builder(path)
        diagram_paths.append(path)

    document = Document()
    section = document.sections[0]
    section.top_margin = Inches(0.72)
    section.bottom_margin = Inches(0.68)
    section.left_margin = Inches(0.82)
    section.right_margin = Inches(0.82)
    configure_styles(document)

    header = section.header.paragraphs[0]
    header.text = "ESP32-P4 ServoBoardMain · 固件架构重构说明"
    header.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    for run in header.runs:
        set_east_asia_font(run)
        run.font.size = Pt(8.5)
        run.font.color.rgb = RGBColor(*rgb(MUTED))
    add_page_number(section.footer.paragraphs[0])

    metadata = [line.rstrip("  ") for line in lines[1:5] if line.strip()]
    create_cover(document, metadata)

    index = 5
    diagram_index = 0
    code_lines: list[str] = []
    in_code = False
    code_language = ""
    paragraph_buffer: list[str] = []

    def flush_paragraph() -> None:
        nonlocal paragraph_buffer
        if not paragraph_buffer:
            return
        merged = " ".join(part.strip().rstrip("  ") for part in paragraph_buffer).strip()
        if merged:
            p = document.add_paragraph()
            add_inline_runs(p, merged)
        paragraph_buffer = []

    while index < len(lines):
        raw = lines[index]
        stripped = raw.strip()

        if in_code:
            if stripped.startswith("```"):
                if code_language == "mermaid":
                    caption, _builder = DIAGRAM_BUILDERS[diagram_index]
                    add_diagram(document, diagram_paths[diagram_index], caption)
                    diagram_index += 1
                else:
                    add_code_block(document, code_lines, code_language)
                code_lines = []
                code_language = ""
                in_code = False
            else:
                code_lines.append(raw)
            index += 1
            continue

        if stripped.startswith("```"):
            flush_paragraph()
            in_code = True
            code_language = stripped[3:].strip().lower()
            index += 1
            continue

        if stripped.startswith("|"):
            flush_paragraph()
            rows, index = parse_table(lines, index)
            if rows:
                add_markdown_table(document, rows)
            continue

        heading = re.match(r"^(#{1,3})\s+(.+)$", stripped)
        if heading:
            flush_paragraph()
            level = len(heading.group(1))
            text_value = heading.group(2)
            if level == 1:
                index += 1
                continue
            p = document.add_paragraph(style=f"Heading {level - 1}")
            add_inline_runs(p, text_value)
            index += 1
            continue

        bullet = re.match(r"^[-*]\s+(.+)$", stripped)
        if bullet:
            flush_paragraph()
            p = document.add_paragraph(style="List Bullet")
            add_inline_runs(p, bullet.group(1))
            index += 1
            continue

        numbered = re.match(r"^(\d+)\.\s+(.+)$", stripped)
        if numbered:
            flush_paragraph()
            # 不使用 Word 的全局自动编号，避免不同章节的独立列表被错误串联。
            p = document.add_paragraph()
            p.paragraph_format.left_indent = Inches(0.28)
            p.paragraph_format.first_line_indent = Inches(-0.22)
            p.paragraph_format.space_after = Pt(3)
            prefix = p.add_run(f"{numbered.group(1)}. ")
            set_east_asia_font(prefix)
            prefix.bold = True
            add_inline_runs(p, numbered.group(2))
            index += 1
            continue

        if stripped.startswith(">"):
            flush_paragraph()
            p = document.add_paragraph()
            p.paragraph_format.left_indent = Inches(0.28)
            p.paragraph_format.right_indent = Inches(0.18)
            p_pr = p._p.get_or_add_pPr()
            shd = OxmlElement("w:shd")
            shd.set(qn("w:fill"), "EEF6FF")
            p_pr.append(shd)
            add_inline_runs(p, stripped.lstrip("> ").strip())
            index += 1
            continue

        if not stripped:
            flush_paragraph()
        else:
            paragraph_buffer.append(raw)
        index += 1

    flush_paragraph()
    output.parent.mkdir(parents=True, exist_ok=True)
    document.save(output)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--diagram-dir", type=Path)
    args = parser.parse_args()

    diagram_dir = args.diagram_dir
    if diagram_dir is None:
        diagram_dir = Path(tempfile.mkdtemp(prefix="p4_architecture_diagrams_"))
    build_document(args.source.resolve(), args.output.resolve(), diagram_dir.resolve())
    print(f"DOCX: {args.output.resolve()}")
    print(f"Diagrams: {diagram_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
