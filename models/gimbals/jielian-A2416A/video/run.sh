#!/usr/bin/env bash
set -euo pipefail
video_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
model_dir="$(cd "$video_dir/.." && pwd)"
python_bin="${JIELIAN_A2416A_PYTHON:-$model_dir/../../../.venv-linux/bin/python}"
if [[ ! -x "$python_bin" ]]; then
  echo "A2416A视频Python环境不存在：$python_bin" >&2
  exit 2
fi
cd "$model_dir"

"$python_bin" -m video.rtsp_server &
rtsp_pid=$!
trap 'kill "$rtsp_pid" 2>/dev/null || true' EXIT INT TERM
"$python_bin" -m video.worker "$@"
