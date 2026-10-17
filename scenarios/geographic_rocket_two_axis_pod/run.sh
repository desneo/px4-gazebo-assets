#!/usr/bin/env bash
# 完整地理世界、独立动态装配、PX4 SITL 与机载吊舱网关。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$HERE/../.." && pwd)"
RUN_DIR="$PROJ/runs/geographic_rocket_two_axis_pod"
CONFIG="$HERE/config.json"
CYCLES=0
SHOW_GUI=false
VIEW=overall
AIRCRAFT_ID=1
require_value() {
  [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || { echo "参数 $1 缺少值" >&2; exit 2; }
}
while (($#)); do
  case "$1" in
    --gui) SHOW_GUI=true; shift ;;
    --close) SHOW_GUI=true; VIEW=close; shift ;;
    --aircraft-id) require_value "$@"; AIRCRAFT_ID="$2"; shift 2 ;;
    --cycles) require_value "$@"; CYCLES="$2"; shift 2 ;;
    --run-dir) require_value "$@"; RUN_DIR="$2"; shift 2 ;;
    --config) require_value "$@"; CONFIG="$2"; shift 2 ;;
    --help|-h)
      echo '用法：bash run.sh [--gui|--close] [--cycles N] [--aircraft-id ID] [--run-dir PATH] [--config PATH]'
      echo '默认持续往返；--cycles 2 完成两次往返、降落并停止。'
      exit 0 ;;
    *) echo "未知参数：$1" >&2; exit 2 ;;
  esac
done
[[ "$CYCLES" =~ ^[0-9]+$ ]] || { echo '--cycles 必须为非负整数' >&2; exit 2; }
[[ "$AIRCRAFT_ID" =~ ^[0-9]+$ ]] || { echo '--aircraft-id 必须为非负整数' >&2; exit 2; }
RUN_DIR="$(realpath -m "$RUN_DIR")"
CONFIG="$(realpath "$CONFIG")"
PX4_ROOT="${PX4_ROOT:-$PROJ/../PX4-Autopilot}"
PX4_BIN="$PX4_ROOT/build/px4_sitl_default/bin/px4"
PX4_ETC="$PX4_ROOT/build/px4_sitl_default/etc"
FLIGHT_PYTHON="${FLIGHT_PYTHON:-python3}"
BRIDGE_PYTHON="${GIMBAL_PYTHON:-$PROJ/../two_axis_pod_venv/bin/python}"
[[ -x "$PX4_BIN" && -d "$PX4_ETC" ]] || { echo "找不到已构建的 PX4：$PX4_BIN" >&2; exit 1; }
[[ -f "$PROJ/models/gimbals/two_axis_pod/controller/build/lib/libgz-two-axis-pod-controller.so" ]] || {
  echo '请先构建两轴吊舱 controller 插件' >&2; exit 1;
}
for command in gz setsid timeout; do command -v "$command" >/dev/null; done
"$FLIGHT_PYTHON" -c 'import pymavlink, numpy, gz.transport13, gz.msgs10.pose_v_pb2'
"$BRIDGE_PYTHON" -c 'import cv2, numpy, gi, gz.transport13; gi.require_version("Gst", "1.0"); gi.require_version("GstRtspServer", "1.0"); from gi.repository import Gst, GstRtspServer'
mkdir -p "$RUN_DIR"
for service in launcher gazebo px4 bridge flight gui imu_wait; do
  if python3 "$HERE/process_state.py" live "$RUN_DIR/$service.pid"; then
    echo "本运行目录已有场景进程：$RUN_DIR/$service.pid，请先停止。" >&2
    exit 1
  fi
done
# PX4 instance0 的本地 MAVLink 端口不会由 GZ_PARTITION 隔离。
python3 - <<'PY_PORTS'
import socket
ports = [(socket.SOCK_DGRAM, p) for p in (14540,14580,14280,13030,18570,8770)]
held=[]
try:
    for kind, port in ports:
        sock=socket.socket(socket.AF_INET,kind)
        held.append(sock)
        try: sock.bind(('0.0.0.0',port))
        except OSError as error: raise SystemExit(f'端口 {port} 已被使用，拒绝启动并行 PX4/网关：{error}')
finally:
    for sock in held: sock.close()
PY_PORTS
cp "$CONFIG" "$RUN_DIR/config.json.tmp"
mv "$RUN_DIR/config.json.tmp" "$RUN_DIR/config.json"
CONFIG="$RUN_DIR/config.json"
read -r IMU_TIMEOUT LAND_TIMEOUT SENSITIVITY NEAR_CLIP < <(
  "$FLIGHT_PYTHON" - "$HERE" "$CONFIG" <<'PY_CONFIG'
import sys
from pathlib import Path
sys.path.insert(0,sys.argv[1])
from flight_patrol import load_config
c=load_config(Path(sys.argv[2]))
print(c['imu_wait_timeout_s'],c['landing_timeout_s'],c['view_sensitivity'],c['view_near_clip_m'])
PY_CONFIG
)
export GIMBAL_RUNDIR="$RUN_DIR"
source "$PROJ/models/gimbals/two_axis_pod/scripts/env.sh"
export GZ_PARTITION="${GZ_PARTITION:-geographic_rocket_two_axis_pod_$$}"
export GZ_SIM_RESOURCE_PATH="$RUN_DIR/models:$PROJ/assembledmodels:$PROJ/models/vehicles:$GZ_SIM_RESOURCE_PATH"
printf '%s\n' "$GZ_PARTITION" > "$RUN_DIR/gz_partition.txt"
python3 "$HERE/process_state.py" record "$RUN_DIR/launcher.pid" "$$" --launcher
SERVICES=()
cleanup() {
  trap - EXIT
  trap '' INT TERM
  set +e
  # 飞行进程拥有降落恢复；给它完整超时，不先停 PX4 或 Gazebo。
  if python3 "$HERE/process_state.py" live "$RUN_DIR/flight.pid"; then
    python3 "$HERE/process_state.py" signal "$RUN_DIR/flight.pid" INT || true
    limit=$(python3 - "$LAND_TIMEOUT" <<'PY_LIMIT'
import math,sys
print(math.ceil(float(sys.argv[1]))+30)
PY_LIMIT
)
    for ((attempt=0; attempt<limit; attempt++)); do
      python3 "$HERE/process_state.py" live "$RUN_DIR/flight.pid" || break
      sleep 1
    done
  fi
  for service in flight imu_wait bridge gui px4 gazebo; do
    python3 "$HERE/process_state.py" signal "$RUN_DIR/$service.pid" TERM || true
  done
  for ((attempt=0; attempt<5; attempt++)); do
    running=false
    for service in "${SERVICES[@]}"; do
      if python3 "$HERE/process_state.py" live "$RUN_DIR/$service.pid"; then running=true; fi
    done
    "$running" || break
    sleep 1
  done
  for service in "${SERVICES[@]}"; do
    python3 "$HERE/process_state.py" signal "$RUN_DIR/$service.pid" KILL || true
    rm -f "$RUN_DIR/$service.pid"
  done
  rm -f "$RUN_DIR/launcher.pid"
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start_service() {
  local service="$1"; shift
  setsid "$@" > "$RUN_DIR/$service.log" 2>&1 < /dev/null &
  LAST_PID=$!
  SERVICES+=("$service")
  if ! python3 "$HERE/process_state.py" record "$RUN_DIR/$service.pid" "$LAST_PID"; then
    # 首帧已就绪时，短命的订阅命令可能在记录之前成功退出。
    [[ "$service" == imu_wait ]] || return 1
  fi
}
GENERATED="$RUN_DIR/models/rocket_two_axis_pod"
python3 "$PROJ/assembledmodels/rocket_two_axis_pod/tools/generate_dynamic_model.py" --output-dir "$GENERATED"
python3 "$HERE/prepare_world.py" --model "$GENERATED/model.sdf" --output "$RUN_DIR/world.sdf" --view "$VIEW" --near-clip "$NEAR_CLIP"
ROOTFS="$RUN_DIR/rootfs-$(date +%Y%m%dT%H%M%S)-$$"
mkdir -p "$ROOTFS"
printf '%s\n' "$ROOTFS" > "$RUN_DIR/rootfs_path.txt"
rm -rf "$RUN_DIR/etc"
cp -R "$PX4_ETC" "$RUN_DIR/etc"
rm -f "$RUN_DIR/etc/init.d-posix/airframes/"*_gz_rocket_two_axis_pod
cp "$GENERATED/px4/4023_gz_rocket_two_axis_pod" "$RUN_DIR/etc/init.d-posix/airframes/"
echo "运行目录：$RUN_DIR；Gazebo 分区：$GZ_PARTITION"
start_service gazebo env -u DISPLAY gz sim -r -s --headless-rendering -v2 "$RUN_DIR/world.sdf"
TOPIC=/world/geographic_rocket_two_axis_pod/model/rocket_two_axis_pod/link/base_link/sensor/imu_sensor/imu
start_service imu_wait timeout "$IMU_TIMEOUT" gz topic -e -t "$TOPIC" -n 1
IMU_PID="$LAST_PID"
while python3 "$HERE/process_state.py" live "$RUN_DIR/imu_wait.pid"; do
  if ! python3 "$HERE/process_state.py" live "$RUN_DIR/gazebo.pid"; then
    echo "Gazebo 在 IMU 就绪前退出，查看 $RUN_DIR/gazebo.log" >&2
    exit 1
  fi
  sleep .5
done
if ! wait "$IMU_PID"; then
  echo "等待 IMU 超时，查看 $RUN_DIR/gazebo.log" >&2; exit 1
fi
cp "$RUN_DIR/imu_wait.log" "$RUN_DIR/imu_sample.txt"
start_service px4 env PX4_SIM_MODEL=gz_rocket_two_axis_pod PX4_GZ_STANDALONE=1 \
  PX4_GZ_WORLD=geographic_rocket_two_axis_pod PX4_GZ_MODEL_NAME=rocket_two_axis_pod \
  "$PX4_BIN" -d "$RUN_DIR/etc" -w "$ROOTFS"
start_service bridge "$BRIDGE_PYTHON" -u "$PROJ/models/gimbals/two_axis_pod/controller/service/gimbal_bridge.py" \
  --world geographic_rocket_two_axis_pod --gimbal-model rocket_two_axis_pod \
  --topic-prefix /rocket_two_axis_pod --image-topic /rocket_two_axis_pod/image \
  --udp-port 8770 --rtsp-port 8555 --aircraft-id "$AIRCRAFT_ID" --encoder nvenc --status-config "$HERE/status_udp.json"
if "$SHOW_GUI"; then
  start_service gui gz sim -g -v2
  SENSITIVITY="${GZ_VIEW_SENSITIVITY:-$SENSITIVITY}"
  python3 - "$SENSITIVITY" <<'PY_SENSITIVITY'
import math,sys
x=float(sys.argv[1])
if not math.isfinite(x) or x<=0: raise SystemExit('GZ_VIEW_SENSITIVITY 必须为有限正数')
PY_SENSITIVITY
  for attempt in 1 2 3; do
    if gz service -s /gui/camera/view_control/sensitivity --reqtype gz.msgs.Double \
       --reptype gz.msgs.Boolean --timeout 3000 --req "data: $SENSITIVITY" > "$RUN_DIR/camera.log" 2>&1 && rg -q 'data: true' "$RUN_DIR/camera.log"; then
      break
    fi
    sleep 1
  done
fi
start_service flight "$FLIGHT_PYTHON" -u "$HERE/flight_patrol.py" --config "$CONFIG" \
  --model-config "$GENERATED/parameters.json" --cycles "$CYCLES" --log "$RUN_DIR/flight_result.json"
FLIGHT_PID="$LAST_PID"
echo "实际往返飞行已启动，日志：$RUN_DIR/flight.log"
echo "吊舱地面站：bash models/gimbals/two_axis_pod/ground_station/run.sh --udp-port 8770 --rtsp-port 8555 --aircraft-id $AIRCRAFT_ID --decoder nvdec"
echo "停止并降落：bash scenarios/geographic_rocket_two_axis_pod/stop.sh '$RUN_DIR'"
while python3 "$HERE/process_state.py" live "$RUN_DIR/flight.pid"; do
  for service in gazebo px4 bridge; do
    if ! python3 "$HERE/process_state.py" live "$RUN_DIR/$service.pid"; then
      echo "关键服务 $service 已退出，请查看 $RUN_DIR/$service.log；请求降落并清理。" >&2
      exit 1
    fi
  done
  sleep 1
done
wait "$FLIGHT_PID"
