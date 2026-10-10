#!/bin/bash
# start_mmhand_desktop.sh — 启动 MM-Hand 本地网页控制台（唯一入口 python -m desktop）
# 用法:
#   ./start_mmhand_desktop.sh              # 启动 Web UI（默认带 ROS2 输入桥，桥不可用时自动降级）
#   ./start_mmhand_desktop.sh --no-ros2    # 不启动 ROS2 输入桥
# 网页地址 http://127.0.0.1:8080；端口被占用时自动顺延到下一个可用端口并在终端打印实际端口。
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"

# 1) 激活 conda 环境 mmhand-ros（含 nicegui / rclpy）
CONDA_BASE="${CONDA_BASE:-$HOME/miniforge3}"
if [ ! -f "$CONDA_BASE/etc/profile.d/conda.sh" ]; then
    echo "conda not found at $CONDA_BASE (set CONDA_BASE to override)" >&2
    exit 1
fi
# shellcheck disable=SC1091
# conda/ROS 的激活脚本未适配 set -u（引用 CONDA_BUILD 等未定义变量），激活期间临时关闭
set +u
source "$CONDA_BASE/etc/profile.d/conda.sh"
if ! conda activate mmhand-ros; then
    echo "conda env 'mmhand-ros' not found; create it first, then:" >&2
    echo "  python -m pip install -r desktop/requirements.txt" >&2
    exit 1
fi
set -u

# 2) 依赖自检（缺 nicegui 时给出明确安装提示）
if ! python -c 'import nicegui' 2>/dev/null; then
    echo "nicegui missing in env 'mmhand-ros'; install with:" >&2
    echo "  python -m pip install -r desktop/requirements.txt" >&2
    exit 1
fi

# 3) 启动唯一入口（浏览器会自动打开；SSH 远程时手动访问打印出的地址）
echo "Starting MM-Hand web console at http://127.0.0.1:8080 ..."
exec python -m desktop "$@"
