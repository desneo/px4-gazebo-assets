#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$HERE/../../../.." && pwd)"
VENV="$PROJ/.venv-linux"
SYSTEM_PYTHON="${PYTHON:-python3}"
VISION_CONFIG="${1:-$PROJ/models/gimbals/two_axis_pod/controller/service/vision.json}"

install_apt_packages() {
  if ! command -v apt-get >/dev/null 2>&1 || ! command -v sudo >/dev/null 2>&1; then
    echo "无法自动安装网关系统依赖：当前系统没有 apt-get 或 sudo。" >&2
    return 1
  fi
  echo "需要安装两轴吊舱网关系统依赖，sudo 可能询问当前用户密码。"
  sudo apt-get update
  sudo apt-get install -y \
    python3-venv python3-gi python3-gz-transport13 python3-gz-msgs10 \
    gir1.2-gstreamer-1.0 gir1.2-gst-rtsp-server-1.0 \
    gstreamer1.0-tools gstreamer1.0-plugins-bad
}

if ! command -v "$SYSTEM_PYTHON" >/dev/null 2>&1; then
  echo "找不到 Python 3：$SYSTEM_PYTHON，尝试自动安装。"
  install_apt_packages || exit 2
  SYSTEM_PYTHON=python3
fi

check_system_bindings() {
  "$SYSTEM_PYTHON" - <<'PY' >/dev/null 2>&1
import gi
import gz.transport13
import gz.msgs10.image_pb2
import gz.msgs10.stringmsg_pb2
gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer
PY
}

if ! check_system_bindings; then
  install_apt_packages || {
    echo "请安装 Gazebo Harmonic Python、PyGObject、GStreamer 和 GstRtspServer 依赖后重试。" >&2
    exit 2
  }
fi
if ! check_system_bindings; then
  echo "系统依赖安装后仍无法导入 Gazebo/GStreamer Python 绑定。" >&2
  exit 2
fi
if ! command -v gst-inspect-1.0 >/dev/null 2>&1 \
    || ! gst-inspect-1.0 nvcodec >/dev/null 2>&1; then
  install_apt_packages || {
    echo "请安装带 nvcodec 的 GStreamer bad plugins 后重试。" >&2
    exit 2
  }
fi

if [[ ! -x "$VENV/bin/python" ]] \
    || ! "$VENV/bin/python" -m pip --version >/dev/null 2>&1; then
  echo "创建两轴吊舱 Linux 环境：$VENV"
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
"$VENV/bin/python" -m pip install -r "$HERE/requirements-bridge.txt"
"$VENV/bin/python" "$HERE/check_bridge_env.py" --vision-config "$VISION_CONFIG"

echo "两轴吊舱网关自动配置完成。"
