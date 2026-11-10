#!/usr/bin/env bash
# 启动固定RTSP发布服务和二进制UDP网关；只清理本入口创建的RTSP进程。
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
model_dir="$(cd "$script_dir/.." && pwd)"
python_bin="${JIELIAN_A2416A_PYTHON:-$model_dir/../../../.venv-linux/bin/python}"
if [[ ! -x "$python_bin" ]]; then
  echo "A2416A服务Python环境不存在：$python_bin" >&2
  exit 2
fi
cd "$model_dir"

"$python_bin" -m video.rtsp_server --rtp-port 7001 --rtsp-port 8554 \
  --mount /jielian_a2416a &
rtsp_pid=$!
cleanup() {
  kill "$rtsp_pid" 2>/dev/null || true
  wait "$rtsp_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

exec_status=0
"$python_bin" -m gateway.main --host 127.0.0.1 --port 8001 \
  --topic-prefix /jielian_a2416a --image-topic /jielian_a2416a/image \
  --rtp-host 127.0.0.1 --rtp-port 7001 "$@" || exec_status=$?
exit "$exec_status"
