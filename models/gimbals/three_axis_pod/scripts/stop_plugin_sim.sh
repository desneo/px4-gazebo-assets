#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"
pidf="$RUNDIR/gz_plugin.pid"
[[ -f "$pidf" ]] || exit 0
pid="$(cat "$pidf")"
[[ "$pid" =~ ^[0-9]+$ ]] || { echo "无效 PID 文件：$pidf" >&2; exit 1; }
if kill -0 "$pid" 2>/dev/null; then
  args="$(ps -o args= -p "$pid")"
  [[ "$args" == *"gz sim"* ]] || { echo "PID 不属于此服务，拒绝停止：$pid" >&2; exit 1; }
  pgid="$(ps -o pgid= -p "$pid" | tr -d ' ')"
  [[ "$pgid" == "$pid" ]] || { echo "进程组不匹配：$pid" >&2; exit 1; }
  kill -TERM -- "-$pgid"
fi
rm -f "$pidf"
