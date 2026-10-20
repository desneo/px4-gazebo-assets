#!/usr/bin/env bash
# 独立运行组合模型的完整飞行验收，保留配置、日志与原始验收退出码。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="$(cd "$HERE/.." && pwd)"
PROJ="$(cd "$MODEL_DIR/../.." && pwd)"
MODE="${1:-headless}"
if [[ $# -gt 2 || ( "$MODE" != headless && "$MODE" != sitl ) ]]; then
  echo '用法：bash run_flight_check.sh [headless|sitl] [result.json]' >&2
  exit 2
fi
PX4_ROOT="${PX4_ROOT:-/home/aeroarx/test/codex_gimbal_test/PX4-Autopilot}"
PX4_BIN="$PX4_ROOT/build/px4_sitl_default/bin/px4"
PX4_ETC="$PX4_ROOT/build/px4_sitl_default/etc"
POD_PLUGIN_DIR="$PROJ/models/gimbals/two_axis_pod/controller/build/lib"
FLIGHT_PYTHON="${FLIGHT_PYTHON:-python3}"
FLIGHT_CHECK="$PROJ/models/vehicles/rocket_racing_quad/tests/flight_check.py"
PX4_INSTANCE="${PX4_INSTANCE:-0}"
[[ "$PX4_INSTANCE" =~ ^[0-8]$ ]] || { echo 'PX4_INSTANCE 必须是 0～8 的整数' >&2; exit 2; }
MAVLINK_ENDPOINT="udpin:0.0.0.0:$((14540 + PX4_INSTANCE))"
for command in gz setsid timeout flock "$FLIGHT_PYTHON"; do
  command -v "$command" >/dev/null || { echo "缺少命令：$command" >&2; exit 1; }
done
[[ -x "$PX4_BIN" && -d "$PX4_ETC/init.d-posix/airframes" ]] || {
  echo "找不到已构建的 PX4：$PX4_BIN；请先构建 make px4_sitl" >&2; exit 1;
}
[[ -f "$POD_PLUGIN_DIR/libgz-two-axis-pod-controller.so" ]] || {
  echo "缺少两轴吊舱控制插件：$POD_PLUGIN_DIR/libgz-two-axis-pod-controller.so" >&2; exit 1;
}
[[ -f "$FLIGHT_CHECK" ]] || { echo "缺少完整飞行验收工具：$FLIGHT_CHECK" >&2; exit 1; }
"$FLIGHT_PYTHON" -c 'import pymavlink, numpy, gz.transport13, gz.msgs10.pose_v_pb2'

# GZ_PARTITION 无法隔离 MAVLink 端口，同一 PX4 instance 须顺序执行。
exec 9>"${TMPDIR:-/tmp}/rocket-flight-check-instance${PX4_INSTANCE}-${UID}.lock"
flock -n 9 || { echo '已有组合模型飞行验收正在运行，请顺序执行。' >&2; exit 1; }
"$FLIGHT_PYTHON" - "$PX4_INSTANCE" <<'PY_PORTS'
import socket, sys
instance = int(sys.argv[1])
held = []
try:
    for base_port in (14540, 14580, 14280, 13030, 18570):
        port = base_port + instance
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        held.append(sock)
        try:
            sock.bind(('0.0.0.0', port))
        except OSError as error:
            raise SystemExit(f'端口 {port} 已被使用，拒绝并行启动 PX4：{error}')
finally:
    for sock in held:
        sock.close()
PY_PORTS

RUN_DIR="$(mktemp -d "${TMPDIR:-/tmp}/rocket-two-axis-pod-flight.XXXXXX")"
GENERATED="$RUN_DIR/models/rocket_two_axis_pod"
RESULT="${2:-$RUN_DIR/flight.json}"
mkdir -p "$(dirname "$RESULT")"
RESULT="$(realpath -m "$RESULT")"
export GZ_PARTITION="rocket_two_axis_pod_flight_$$"
export GZ_SIM_RESOURCE_PATH="$RUN_DIR/models:$PROJ/models/vehicles:$PROJ/models/gimbals${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$POD_PLUGIN_DIR${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
printf '%s\n' "$GZ_PARTITION" > "$RUN_DIR/gz_partition.txt"
printf '%s\n' "$RESULT" > "$RUN_DIR/result_path.txt"
echo "运行目录：$RUN_DIR"
echo "Gazebo 分区：$GZ_PARTITION；模式：$MODE"
echo "PX4 instance：$PX4_INSTANCE；MAVLink：$MAVLINK_ENDPOINT"
echo "验收结果：$RESULT"

SERVICES=()
FLIGHT_PID=
cleanup() {
  local status=$?
  trap - EXIT
  trap '' INT TERM
  set +e
  # 只终止本入口创建的独立进程组，不按名称查杀其他会话。
  for pid in "${SERVICES[@]}"; do
    /bin/kill -TERM -- "-$pid" 2>/dev/null || true
  done
  for attempt in 1 2 3 4 5; do
    local alive=false
    for pid in "${SERVICES[@]}"; do
      if /bin/kill -0 -- "-$pid" 2>/dev/null; then alive=true; fi
    done
    "$alive" || break
    sleep .2
  done
  for pid in "${SERVICES[@]}"; do
    /bin/kill -KILL -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  printf '%s\n' "$status" > "$RUN_DIR/exit_code.txt"
  echo "验收退出码：$status；日志保留于 $RUN_DIR"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start_service() {
  local service="$1"; shift
  setsid "$@" > "$RUN_DIR/$service.log" 2>&1 < /dev/null 9>&- &
  LAST_PID=$!
  SERVICES+=("$LAST_PID")
  printf '%s\n' "$LAST_PID" > "$RUN_DIR/$service.pid"
}

"$FLIGHT_PYTHON" "$MODEL_DIR/tools/generate_dynamic_model.py" --output-dir "$GENERATED" > "$RUN_DIR/generate.log" 2>&1
CONFIG="$GENERATED/parameters.json"
"$FLIGHT_PYTHON" - "$RUN_DIR/execution.json" "$PX4_INSTANCE" "$MAVLINK_ENDPOINT" \
  "$MODE" "$GZ_PARTITION" "$CONFIG" "$RESULT" "$PX4_ROOT" <<'PY_EXECUTION'
import json, sys
from pathlib import Path
path, instance, endpoint, mode, partition, config, result, px4_root = sys.argv[1:]
Path(path).write_text(json.dumps(dict(px4_instance=int(instance), mavlink_endpoint=endpoint,
    mode=mode, gz_partition=partition, model_config=config, result_path=result,
    px4_root=px4_root), ensure_ascii=False, indent=2) + '\n')
PY_EXECUTION
read -r IMU_TIMEOUT WORLD_NAME MODEL_NAME AIRFRAME_ID < <(
  "$FLIGHT_PYTHON" - "$CONFIG" <<'PY_TARGET'
import json, math, sys
config = json.load(open(sys.argv[1]))
timeout = config['simulation']['imu_wait_timeout_s']
if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout <= 0:
    raise SystemExit('IMU 等待期限必须为有限正数')
print(timeout, config['simulation']['world_name'], config['model']['name'], config['px4']['airframe_id'])
PY_TARGET
)
[[ "$MODEL_NAME" == rocket_two_axis_pod && "$WORLD_NAME" == rocket_two_axis_pod_test ]] || {
  echo '生成资产的世界名或模型名不符合组合模型验收入口' >&2; exit 1;
}
mkdir -p "$RUN_DIR/rootfs"
cp -R "$PX4_ETC" "$RUN_DIR/etc"
rm -f "$RUN_DIR/etc/init.d-posix/airframes/"*_gz_rocket_two_axis_pod
cp "$GENERATED/px4/${AIRFRAME_ID}_gz_rocket_two_axis_pod" "$RUN_DIR/etc/init.d-posix/airframes/"
if [[ "$MODE" == headless ]]; then
  start_service gazebo env -u DISPLAY gz sim -s -r --headless-rendering "$GENERATED/tests/world.sdf"
else
  start_service gazebo gz sim -r "$GENERATED/tests/world.sdf"
fi
WORLD_PID="$LAST_PID"
TOPIC="/world/$WORLD_NAME/model/$MODEL_NAME/link/base_link/sensor/imu_sensor/imu"
start_service imu_wait timeout "$IMU_TIMEOUT" gz topic -e -t "$TOPIC" -n 1
IMU_PID="$LAST_PID"
while kill -0 "$IMU_PID" 2>/dev/null; do
  if ! kill -0 "$WORLD_PID" 2>/dev/null; then
    echo "Gazebo 在 IMU 就绪前退出，请查看 $RUN_DIR/gazebo.log" >&2; exit 1
  fi
  sleep .2
done
if wait "$IMU_PID"; then
  cp "$RUN_DIR/imu_wait.log" "$RUN_DIR/imu_sample.txt"
else
  status=$?
  echo "等待 IMU 失败（$status），请查看 $RUN_DIR/gazebo.log 与 imu_wait.log" >&2
  exit "$status"
fi
echo 'IMU 已就绪，启动 PX4；按运行快照执行悬停、巡航、冲刺和降落验收。'
start_service px4 env PX4_SIM_MODEL=gz_rocket_two_axis_pod PX4_GZ_STANDALONE=1 \
  PX4_GZ_WORLD="$WORLD_NAME" PX4_GZ_MODEL_NAME="$MODEL_NAME" \
  "$PX4_BIN" -i "$PX4_INSTANCE" -d "$RUN_DIR/etc" -w "$RUN_DIR/rootfs"
start_service flight "$FLIGHT_PYTHON" -u "$FLIGHT_CHECK" --ground-truth \
  --endpoint "$MAVLINK_ENDPOINT" --config "$CONFIG" --log "$RESULT"
FLIGHT_PID="$LAST_PID"
if wait "$FLIGHT_PID"; then
  exit 0
else
  exit "$?"
fi
