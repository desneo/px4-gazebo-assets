#!/usr/bin/env bash
# 单机与组合体使用同一完整地理世界顺序试飞，不更改源模型参数。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$HERE/../.." && pwd)"
PROCESS="$PROJ/scenarios/geographic_rocket_two_axis_pod/process_state.py"
VARIANT=
RUN_DIR=
CONFIG="$HERE/config.json"
TARGET_SPEED=
SHOW_GUI=false
VIEW=overall
require_value() {
  [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || { echo "参数 $1 缺少值" >&2; exit 2; }
}
while (($#)); do
  case "$1" in
    --variant) require_value "$@"; VARIANT="$2"; shift 2 ;;
    --run-dir) require_value "$@"; RUN_DIR="$2"; shift 2 ;;
    --config) require_value "$@"; CONFIG="$2"; shift 2 ;;
    --target-speed) require_value "$@"; TARGET_SPEED="$2"; shift 2 ;;
    --gui) SHOW_GUI=true; shift ;;
    --close) SHOW_GUI=true; VIEW=close; shift ;;
    --help|-h)
      echo '用法：bash run.sh --variant standalone|assembled [--run-dir PATH] [--config PATH] [--target-speed m/s] [--gui|--close]'
      echo '两种机型须顺序运行；每次使用全新 PX4 参数存储，完成速度试验后降落。'
      exit 0 ;;
    *) echo "未知参数：$1" >&2; exit 2 ;;
  esac
done
case "$VARIANT" in
  standalone) MODEL_NAME=rocket_racing_quad ;;
  assembled) MODEL_NAME=rocket_two_axis_pod ;;
  *) echo '必须指定 --variant standalone 或 --variant assembled' >&2; exit 2 ;;
esac
RUN_DIR="$(realpath -m "${RUN_DIR:-$PROJ/runs/rocket_speed_comparison/${VARIANT}-$(date +%Y%m%dT%H%M%S)-$$}")"
CONFIG="$(realpath "$CONFIG")"
PX4_ROOT="${PX4_ROOT:-$PROJ/../PX4-Autopilot}"
PX4_BIN="$PX4_ROOT/build/px4_sitl_default/bin/px4"
PX4_ETC="$PX4_ROOT/build/px4_sitl_default/etc"
FLIGHT_PYTHON="${FLIGHT_PYTHON:-python3}"
[[ -x "$PX4_BIN" && -d "$PX4_ETC" ]] || { echo "找不到已构建的 PX4：$PX4_BIN" >&2; exit 1; }
if [[ "$VARIANT" == assembled ]]; then
  [[ -f "$PROJ/models/gimbals/two_axis_pod/controller/build/lib/libgz-two-axis-pod-controller.so" ]] || {
    echo '请先构建两轴吊舱 controller 插件' >&2; exit 1;
  }
fi
for command in gz setsid timeout flock; do command -v "$command" >/dev/null; done
"$FLIGHT_PYTHON" -c 'import pymavlink, numpy, gz.transport13, gz.msgs10.pose_v_pb2'
# 锁覆盖本场景启动窗口；端口检查同时检测其他场景占用的 instance0。
exec 9>"${TMPDIR:-/tmp}/rocket-speed-instance0-${UID}.lock"
flock -n 9 || { echo '已有速度试验正在使用 PX4 instance0，请顺序运行。' >&2; exit 1; }
python3 - <<'PY_PORTS'
import socket
held=[]
try:
    for port in (14540,14580,14280,13030,18570):
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        held.append(sock)
        try: sock.bind(('0.0.0.0',port))
        except OSError as error: raise SystemExit(f'端口 {port} 已被使用，拒绝并行启动 PX4：{error}')
finally:
    for sock in held: sock.close()
PY_PORTS
mkdir -p "$RUN_DIR"
for service in launcher gazebo px4 flight gui imu_wait; do
  if python3 "$PROCESS" live "$RUN_DIR/$service.pid"; then
    echo "运行目录已有活动进程：$RUN_DIR/$service.pid" >&2; exit 1
  fi
done
python3 - "$RUN_DIR" <<'PY_FRESH'
import sys
from pathlib import Path
if any(Path(sys.argv[1]).iterdir()):
    raise SystemExit('运行目录必须为空，保证 PX4 参数存储全新并保留既有对照证据；请选择新的 --run-dir。')
PY_FRESH
cp "$CONFIG" "$RUN_DIR/config.json"
CONFIG="$RUN_DIR/config.json"
read -r IMU_TIMEOUT LAND_TIMEOUT SENSITIVITY NEAR_CLIP < <(
  "$FLIGHT_PYTHON" - "$CONFIG" "$TARGET_SPEED" <<'PY_CONFIG'
import json,math,sys
from pathlib import Path
p=Path(sys.argv[1]); c=json.loads(p.read_text())
if sys.argv[2]: c['target_speed_m_s']=float(sys.argv[2])
for key,default in [('imu_wait_timeout_s',120),('landing_timeout_s',180),('view_sensitivity',.1),('view_near_clip_m',.005)]:
    value=c.setdefault(key,default)
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
        raise SystemExit(f'{key} 必须为有限正数')
p.write_text(json.dumps(c,indent=2,ensure_ascii=False)+'\n')
print(c['imu_wait_timeout_s'],c['landing_timeout_s'],c['view_sensitivity'],c['view_near_clip_m'])
PY_CONFIG
)
export GZ_PARTITION="${GZ_PARTITION:-rocket_speed_comparison_$$}"
export GZ_SIM_RESOURCE_PATH="$RUN_DIR/models:$PROJ/models/vehicles:$PROJ/models/gimbals:$PROJ/models/props${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PROJ/models/gimbals/two_axis_pod/controller/build/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
printf '%s\n' "$GZ_PARTITION" > "$RUN_DIR/gz_partition.txt"
printf '%s\n' "$VARIANT" > "$RUN_DIR/variant.txt"
printf '%s\n' "$MODEL_NAME" > "$RUN_DIR/model_name.txt"
python3 "$PROCESS" record "$RUN_DIR/launcher.pid" "$$" --launcher
SERVICES=()
cleanup() {
  trap - EXIT
  trap '' INT TERM
  set +e
  if python3 "$PROCESS" live "$RUN_DIR/flight.pid"; then
    python3 "$PROCESS" signal "$RUN_DIR/flight.pid" INT || true
    limit=$(python3 - "$LAND_TIMEOUT" <<'PY_LIMIT'
import math,sys
print(math.ceil(float(sys.argv[1]))+30)
PY_LIMIT
)
    for ((attempt=0; attempt<limit; attempt++)); do
      python3 "$PROCESS" live "$RUN_DIR/flight.pid" || break
      sleep 1
    done
  fi
  for service in flight imu_wait gui px4 gazebo; do
    python3 "$PROCESS" signal "$RUN_DIR/$service.pid" TERM || true
  done
  for ((attempt=0; attempt<5; attempt++)); do
    running=false
    for service in "${SERVICES[@]}"; do
      if python3 "$PROCESS" live "$RUN_DIR/$service.pid"; then running=true; fi
    done
    "$running" || break
    sleep 1
  done
  for service in "${SERVICES[@]}"; do
    python3 "$PROCESS" signal "$RUN_DIR/$service.pid" KILL || true
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
  if ! python3 "$PROCESS" record "$RUN_DIR/$service.pid" "$LAST_PID"; then
    [[ "$service" == imu_wait ]] || return 1
  fi
}
GENERATED="$RUN_DIR/models/$MODEL_NAME"
if [[ "$VARIANT" == standalone ]]; then
  python3 "$PROJ/models/vehicles/rocket_racing_quad/tools/prepare_runtime.py" --output-dir "$GENERATED"
else
  python3 "$PROJ/assembledmodels/rocket_two_axis_pod/tools/generate_dynamic_model.py" --output-dir "$GENERATED"
fi
# 在仿真启动前校验全部飞行配置与模型速度上限。
"$FLIGHT_PYTHON" "$HERE/flight_trial.py" --config "$CONFIG" --model-config "$GENERATED/parameters.json" \
  --model-name "$MODEL_NAME" --log "$RUN_DIR/trial.json" --validate-only
python3 "$HERE/prepare_world.py" --variant "$VARIANT" --model "$GENERATED/model.sdf" \
  --output "$RUN_DIR/world.sdf" --view "$VIEW" --near-clip "$NEAR_CLIP" --update-model-snapshot
ROOTFS="$RUN_DIR/rootfs"
mkdir -p "$ROOTFS"
printf '%s\n' "$ROOTFS" > "$RUN_DIR/rootfs_path.txt"
cp -R "$PX4_ETC" "$RUN_DIR/etc"
rm -f "$RUN_DIR/etc/init.d-posix/airframes/"*_gz_"$MODEL_NAME"
cp "$GENERATED"/px4/* "$RUN_DIR/etc/init.d-posix/airframes/"
echo "运行目录：$RUN_DIR；机型：$VARIANT；Gazebo 分区：$GZ_PARTITION"
start_service gazebo env -u DISPLAY gz sim -r -s --headless-rendering -v2 "$RUN_DIR/world.sdf"
TOPIC="/world/rocket_speed_comparison/model/$MODEL_NAME/link/base_link/sensor/imu_sensor/imu"
start_service imu_wait timeout "$IMU_TIMEOUT" gz topic -e -t "$TOPIC" -n 1
IMU_PID="$LAST_PID"
while python3 "$PROCESS" live "$RUN_DIR/imu_wait.pid"; do
  if ! python3 "$PROCESS" live "$RUN_DIR/gazebo.pid"; then
    echo "Gazebo 在 IMU 就绪前退出，查看 $RUN_DIR/gazebo.log" >&2; exit 1
  fi
  sleep .5
done
if ! wait "$IMU_PID"; then echo "等待 IMU 超时，查看 $RUN_DIR/gazebo.log" >&2; exit 1; fi
cp "$RUN_DIR/imu_wait.log" "$RUN_DIR/imu_sample.txt"
start_service px4 env PX4_SIM_MODEL="gz_$MODEL_NAME" PX4_GZ_STANDALONE=1 \
  PX4_GZ_WORLD=rocket_speed_comparison PX4_GZ_MODEL_NAME="$MODEL_NAME" \
  "$PX4_BIN" -d "$RUN_DIR/etc" -w "$ROOTFS"
if "$SHOW_GUI"; then
  start_service gui gz sim -g -v2
  for attempt in 1 2 3; do
    if gz service -s /gui/camera/view_control/sensitivity --reqtype gz.msgs.Double \
       --reptype gz.msgs.Boolean --timeout 3000 --req "data: $SENSITIVITY" > "$RUN_DIR/camera.log" 2>&1 && rg -q 'data: true' "$RUN_DIR/camera.log"; then
      break
    fi
    sleep 1
  done
fi
start_service flight "$FLIGHT_PYTHON" -u "$HERE/flight_trial.py" --config "$CONFIG" \
  --model-config "$GENERATED/parameters.json" --model-name "$MODEL_NAME" --log "$RUN_DIR/trial.json"
FLIGHT_PID="$LAST_PID"
echo "速度试飞日志：$RUN_DIR/flight.log；结果：$RUN_DIR/trial.json"
echo "停止并降落：bash scenarios/rocket_speed_comparison/stop.sh '$RUN_DIR'"
while python3 "$PROCESS" live "$RUN_DIR/flight.pid"; do
  for service in gazebo px4; do
    if ! python3 "$PROCESS" live "$RUN_DIR/$service.pid"; then
      echo "关键服务 $service 已退出，请查看日志；请求降落并清理。" >&2; exit 1
    fi
  done
  sleep 1
done
wait "$FLIGHT_PID"
