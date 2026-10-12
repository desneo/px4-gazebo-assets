#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"
WORLD="${1:-$PROJ/models/gimbals/three_axis_pod/tests/world.sdf}"
[[ -f "$PKG/build/lib/libgz-gimbal-controller.so" ]] || { echo "请先构建 models/gimbals/three_axis_pod/controller" >&2; exit 1; }
mkdir -p "$RUNDIR"
if [[ -f "$RUNDIR/gz_plugin.pid" ]] && kill -0 "$(cat "$RUNDIR/gz_plugin.pid")" 2>/dev/null; then
  echo "组件仿真已运行，请先停止：$RUNDIR/gz_plugin.pid" >&2; exit 1
fi
setsid env -u DISPLAY gz sim -r -s --headless-rendering -v2 "$WORLD" >"$RUNDIR/gz_plugin.log" 2>&1 &
pid=$!
echo "$pid" > "$RUNDIR/gz_plugin.pid"
sleep 2
kill -0 "$pid" 2>/dev/null || { echo "Gazebo 启动失败，查看 $RUNDIR/gz_plugin.log" >&2; exit 1; }
echo "Gazebo 已启动 pid=$pid；资源/图像就绪情况查看 $RUNDIR/gz_plugin.log"
