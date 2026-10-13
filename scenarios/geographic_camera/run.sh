#!/usr/bin/env bash
# 固定安装的地理场景视觉验证；不加载载机或 PX4。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$HERE/../.." && pwd)"
export GIMBAL_RUNDIR="${GIMBAL_RUNDIR:-$PROJ/runs/geographic_camera}"
export GZ_PARTITION="${GZ_PARTITION:-geographic_camera}"
SHOW_GUI=false
BRIDGE_ARGS=()
MOUNT_MODE=skyward
UDP_PORT=8765
while (($#)); do
  case "$1" in
    --gui) SHOW_GUI=true; shift ;;
    --mount-mode) MOUNT_MODE="$2"; shift 2 ;;
    --udp-port) UDP_PORT="$2"; BRIDGE_ARGS+=("$1" "$2"); shift 2 ;;
    *) BRIDGE_ARGS+=("$1"); shift ;;
  esac
done
mkdir -p "$GIMBAL_RUNDIR"
python3 "$HERE/prepare_world.py" --mount-mode "$MOUNT_MODE" --output "$GIMBAL_RUNDIR/world.sdf"
bash "$PROJ/models/gimbals/three_axis_pod/scripts/run_plugin_sim.sh" "$GIMBAL_RUNDIR/world.sdf"
bash "$PROJ/models/gimbals/three_axis_pod/scripts/run_bridge.sh" --world geographic_camera --encoder nvenc "${BRIDGE_ARGS[@]}"
python3 "$HERE/check_occlusion.py" --mount-mode "$MOUNT_MODE" --port "$UDP_PORT" --prepare-only --output "$GIMBAL_RUNDIR/initialization.jsonl"
if "$SHOW_GUI"; then
  source "$PROJ/models/gimbals/three_axis_pod/scripts/env.sh"
  setsid gz sim -g -v2 > "$GIMBAL_RUNDIR/gz_gui.log" 2>&1 < /dev/null &
  echo $! > "$GIMBAL_RUNDIR/gz_gui.pid"
fi
echo "地理场景已启动，GZ_PARTITION=$GZ_PARTITION；日志：$GIMBAL_RUNDIR"
echo "地面站：bash models/gimbals/three_axis_pod/ground_station/run.sh --decoder avdec"
echo "2 秒遮挡：GZ_PARTITION=$GZ_PARTITION python3 scenarios/geographic_camera/control_scene.py occlude"
