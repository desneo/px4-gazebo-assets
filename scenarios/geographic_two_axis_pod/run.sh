#!/usr/bin/env bash
# 完整地理背景中的两轴立装部件测试；无需载机和 PX4。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$HERE/../.." && pwd)"
export GIMBAL_RUNDIR="${GIMBAL_RUNDIR:-$PROJ/runs/geographic_two_axis_pod}"
export GZ_PARTITION="${GZ_PARTITION:-geographic_two_axis_pod}"
SHOW_GUI=false
BRIDGE_ARGS=()
while (($#)); do
  case "$1" in
    --gui) SHOW_GUI=true; shift ;;
    *) BRIDGE_ARGS+=("$1"); shift ;;
  esac
done
# 运行模型已内联到服务端；已有服务时不可覆盖运行世界后继续复用旧进程。
for service in gz_plugin bridge; do
  pid_file="$GIMBAL_RUNDIR/$service.pid"
  if [[ -f "$pid_file" ]]; then
    recorded_pid="$(cat "$pid_file")"
    if [[ "$recorded_pid" =~ ^[0-9]+$ ]] && kill -0 "$recorded_pid" 2>/dev/null; then
      echo "场景服务仍在运行：$pid_file。模型变更后请先停止服务端和网关，再重新启动；只打开GUI不会更新模型。" >&2
      exit 1
    fi
  fi
done
mkdir -p "$GIMBAL_RUNDIR"
python3 "$HERE/prepare_world.py" --output "$GIMBAL_RUNDIR/world.sdf"
bash "$PROJ/models/gimbals/two_axis_pod/scripts/run_plugin_sim.sh" "$GIMBAL_RUNDIR/world.sdf"
bash "$PROJ/models/gimbals/two_axis_pod/scripts/run_bridge.sh" \
  --world geographic_two_axis_pod --gimbal-model two_axis_pod \
  --udp-port 8766 --rtsp-port 8555 --encoder nvenc "${BRIDGE_ARGS[@]}"
if "$SHOW_GUI"; then
  source "$PROJ/models/gimbals/two_axis_pod/scripts/env.sh"
  setsid gz sim -g -v2 > "$GIMBAL_RUNDIR/gz_gui.log" 2>&1 < /dev/null &
  echo $! > "$GIMBAL_RUNDIR/gz_gui.pid"
  # 小尺寸吊舱近景查看；原生插件不从SDF读取鼠标灵敏度，须调用GUI服务。
  sensitivity="${GZ_VIEW_SENSITIVITY:-0.1}"
  python3 - "$sensitivity" <<'PY_CHECK'
import math, sys
value = float(sys.argv[1])
if not math.isfinite(value) or value <= 0:
    raise SystemExit('GZ_VIEW_SENSITIVITY 必须为正的有限数值')
PY_CHECK
  for attempt in 1 2 3; do
    if gz service -s /gui/camera/view_control/sensitivity \
      --reqtype gz.msgs.Double --reptype gz.msgs.Boolean --timeout 3000 \
      --req "data: $sensitivity" 2>/dev/null | rg -q 'data: true'; then
      echo "Gazebo 浏览相机灵敏度：$sensitivity"
      break
    fi
    if [[ "$attempt" == 3 ]]; then
      echo "浏览相机灵敏度尚未设置，可待GUI就绪后手动调用服务。" >&2
    fi
    sleep 1
  done
fi
echo "两轴地理场景已启动；GZ_PARTITION=$GZ_PARTITION；日志：$GIMBAL_RUNDIR"
echo "先运行 python3 scenarios/geographic_two_axis_pod/check_target.py --prepare-only，再打开地面站。"
echo "地面站：bash models/gimbals/two_axis_pod/ground_station/run.sh（读取 ground_station/config.json）"
