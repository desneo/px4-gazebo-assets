#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"
mkdir -p "$RUNDIR"
if [[ -f "$RUNDIR/bridge.pid" ]] && kill -0 "$(cat "$RUNDIR/bridge.pid")" 2>/dev/null; then
  echo "网关已运行，请先停止：$RUNDIR/bridge.pid" >&2; exit 1
fi
setsid python3 -u "$PKG/service/gimbal_bridge.py" "$@" >"$RUNDIR/bridge.log" 2>&1 &
pid=$!
echo "$pid" > "$RUNDIR/bridge.pid"
sleep 2
kill -0 "$pid" 2>/dev/null || { echo "网关启动失败，查看 $RUNDIR/bridge.log" >&2; exit 1; }
echo "网关已启动 pid=$pid 日志=$RUNDIR/bridge.log"
