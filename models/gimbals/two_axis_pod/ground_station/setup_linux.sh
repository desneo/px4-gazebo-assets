#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$HERE/../../../.." && pwd)"
VENV="$PROJ/.venv-linux"
SYSTEM_PYTHON="${PYTHON:-python3}"

install_apt_packages() {
  if ! command -v apt-get >/dev/null 2>&1 || ! command -v sudo >/dev/null 2>&1; then
    echo "无法自动安装系统依赖：当前系统没有 apt-get 或 sudo。" >&2
    return 1
  fi
  echo "需要安装 Linux Station 系统依赖，sudo 可能询问当前用户密码。"
  sudo apt-get update
  sudo apt-get install -y \
    python3-venv python3-gi gir1.2-gstreamer-1.0 \
    gstreamer1.0-tools gstreamer1.0-plugins-bad
}

if ! command -v "$SYSTEM_PYTHON" >/dev/null 2>&1; then
  echo "找不到 Python 3：$SYSTEM_PYTHON，尝试自动安装。"
  install_apt_packages || exit 2
  SYSTEM_PYTHON=python3
  if ! command -v "$SYSTEM_PYTHON" >/dev/null 2>&1; then
    echo "系统依赖安装完成后仍找不到 Python 3。" >&2
    exit 2
  fi
fi

if ! "$SYSTEM_PYTHON" - <<'PY' >/dev/null 2>&1
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst
Gst.init(None)
raise SystemExit(0 if Gst.ElementFactory.find("nvh264dec") else 1)
PY
then
  install_apt_packages || {
    echo "请安装 Python GObject、GStreamer 及 NVIDIA NVDEC 插件后重试。" >&2
    exit 2
  }
fi

if [[ ! -x "$VENV/bin/python" ]] \
    || ! "$VENV/bin/python" -m pip --version >/dev/null 2>&1; then
  echo "创建 Linux 地面站环境：$VENV"
  if ! "$SYSTEM_PYTHON" -m venv --system-site-packages "$VENV"; then
    if command -v uv >/dev/null 2>&1; then
      echo "系统 venv/ensurepip 不可用，改用 uv 创建环境。"
      uv venv --clear --seed --system-site-packages \
        --python "$SYSTEM_PYTHON" "$VENV"
    else
      install_apt_packages || {
        echo "无法创建虚拟环境；请安装 python3-venv 后重试。" >&2
        exit 2
      }
      "$SYSTEM_PYTHON" -m venv --clear --system-site-packages "$VENV"
    fi
  fi
fi

"$VENV/bin/python" -m pip install --upgrade pip
"$VENV/bin/python" -m pip install -r "$HERE/requirements-linux.txt"

"$VENV/bin/python" "$HERE/check_linux_env.py"

echo "Linux Station 自动配置完成。"
