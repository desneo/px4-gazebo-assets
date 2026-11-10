#!/usr/bin/env bash
# 独立运行 A2416A 组合模型的完整飞行、真值跟踪和遥测验收。
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
model_dir="$(cd "$here/.." && pwd)"
project="$(cd "$model_dir/../.." && pwd)"
mode="${1:-headless}"
if [[ $# -gt 2 || ( "$mode" != headless && "$mode" != sitl ) ]]; then
  echo '用法：bash run_flight_check.sh [headless|sitl] [result.json]' >&2
  exit 2
fi

px4_root="${PX4_ROOT:-$(cd "$project/../PX4-Autopilot" 2>/dev/null && pwd || true)}"
[[ -n "$px4_root" ]] || { echo '找不到 PX4-Autopilot；请设置 PX4_ROOT' >&2; exit 1; }
px4_bin="$px4_root/build/px4_sitl_default/bin/px4"
px4_etc="$px4_root/build/px4_sitl_default/etc"
plugin_dir="$project/models/gimbals/jielian-A2416A/build/lib"
flight_python="${FLIGHT_PYTHON:-python3}"
flight_check="$project/models/vehicles/rocket_racing_quad/tests/flight_check.py"
capture_tool="$model_dir/tools/capture_tracking_telemetry.py"
tracking_check="$model_dir/tools/validate_flight_tracking.py"
px4_instance="${PX4_INSTANCE:-0}"
[[ "$px4_instance" =~ ^[0-8]$ ]] || { echo 'PX4_INSTANCE 必须是 0～8 的整数' >&2; exit 2; }
mavlink_endpoint="udpin:0.0.0.0:$((14540 + px4_instance))"

for command in gz setsid timeout flock "$flight_python"; do
  command -v "$command" >/dev/null || { echo "缺少命令：$command" >&2; exit 1; }
done
[[ -x "$px4_bin" && -d "$px4_etc/init.d-posix/airframes" ]] || {
  echo "找不到已构建的 PX4：$px4_bin；请先构建 make px4_sitl" >&2
  exit 1
}
[[ -f "$plugin_dir/libgz-jielian-a2416a-controller.so" ]] || {
  echo "缺少 A2416A 控制插件：$plugin_dir/libgz-jielian-a2416a-controller.so" >&2
  exit 1
}
for required in "$flight_check" "$capture_tool" "$tracking_check"; do
  [[ -f "$required" ]] || { echo "缺少验收工具：$required" >&2; exit 1; }
done
"$flight_python" -c 'import pymavlink, numpy, gz.transport13, gz.msgs10.stringmsg_pb2'

# GZ_PARTITION 不隔离 PX4 使用的 UDP 端口，同一 instance 必须顺序执行。
exec 9>"${TMPDIR:-/tmp}/rocket-flight-check-instance${px4_instance}-${UID}.lock"
flock -n 9 || { echo '已有火箭组合模型飞行验收正在运行，请顺序执行。' >&2; exit 1; }
"$flight_python" - "$px4_instance" <<'PY_PORTS'
import socket, sys
instance = int(sys.argv[1])
held = []
try:
    for base in (14540, 14580, 14280, 13030, 18570):
        port = base + instance
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

run_dir="$(mktemp -d "${TMPDIR:-/tmp}/rocket-jielian-a2416a-flight.XXXXXX")"
generated="$run_dir/models/rocket_jielian_a2416a"
result="${2:-$run_dir/flight.json}"
mkdir -p "$(dirname "$result")"
result="$(realpath -m "$result")"
tracking_result="$run_dir/tracking-validation.json"
tracking_telemetry="$run_dir/tracking-telemetry.jsonl"
export GZ_PARTITION="rocket_jielian_a2416a_flight_$$"
export GZ_SIM_RESOURCE_PATH="$run_dir/models:$project/models/vehicles:$project/models/gimbals${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$plugin_dir${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
printf '%s\n' "$GZ_PARTITION" > "$run_dir/gz_partition.txt"
printf '%s\n' "$result" > "$run_dir/result_path.txt"
echo "运行目录：$run_dir"
echo "Gazebo 分区：$GZ_PARTITION；模式：$mode"
echo "PX4 instance：$px4_instance；MAVLink：$mavlink_endpoint"
echo "验收结果：$result"

services=()
cleanup() {
  local status=$?
  trap - EXIT
  trap '' INT TERM
  set +e
  for pid in "${services[@]}"; do
    /bin/kill -TERM -- "-$pid" 2>/dev/null || true
  done
  for _ in 1 2 3 4 5; do
    local alive=false
    for pid in "${services[@]}"; do
      if /bin/kill -0 -- "-$pid" 2>/dev/null; then alive=true; fi
    done
    "$alive" || break
    sleep .2
  done
  for pid in "${services[@]}"; do
    /bin/kill -KILL -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  printf '%s\n' "$status" > "$run_dir/exit_code.txt"
  echo "验收退出码：$status；日志保留于 $run_dir"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_service() {
  local service="$1"
  shift
  setsid "$@" > "$run_dir/$service.log" 2>&1 < /dev/null 9>&- &
  last_pid=$!
  services+=("$last_pid")
  printf '%s\n' "$last_pid" > "$run_dir/$service.pid"
}
stop_service() {
  local pid="$1"
  /bin/kill -TERM -- "-$pid" 2>/dev/null || return 0
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    /bin/kill -0 -- "-$pid" 2>/dev/null || break
    sleep .2
  done
  /bin/kill -KILL -- "-$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

"$flight_python" "$model_dir/tools/generate_dynamic_model.py" \
  --output-dir "$generated" > "$run_dir/generate.log" 2>&1
config="$generated/parameters.json"
"$flight_python" - "$run_dir/execution.json" "$px4_instance" "$mavlink_endpoint" \
  "$mode" "$GZ_PARTITION" "$config" "$result" "$tracking_result" "$px4_root" <<'PY_EXECUTION'
import json, sys
from pathlib import Path
path, instance, endpoint, mode, partition, config, result, tracking, px4_root = sys.argv[1:]
Path(path).write_text(json.dumps(dict(
    px4_instance=int(instance), mavlink_endpoint=endpoint, mode=mode,
    gz_partition=partition, model_config=config, result_path=result,
    tracking_result_path=tracking, px4_root=px4_root,
), ensure_ascii=False, indent=2) + '\n')
PY_EXECUTION
read -r imu_timeout world_name model_name airframe_id topic_prefix < <(
  "$flight_python" - "$config" "$generated/manifest.json" <<'PY_TARGET'
import json, math, sys
config = json.load(open(sys.argv[1]))
manifest = json.load(open(sys.argv[2]))
timeout = config['simulation']['imu_wait_timeout_s']
if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout <= 0:
    raise SystemExit('IMU 等待期限必须为有限正数')
print(timeout, config['simulation']['world_name'], config['model']['name'],
      config['px4']['airframe_id'], manifest['topic_prefix'])
PY_TARGET
)
[[ "$model_name" == rocket_jielian_a2416a && "$world_name" == rocket_jielian_a2416a_test ]] || {
  echo '生成资产的世界名或模型名不符合 A2416A 验收入口' >&2
  exit 1
}

mkdir -p "$run_dir/rootfs"
cp -R "$px4_etc" "$run_dir/etc"
rm -f "$run_dir/etc/init.d-posix/airframes/"*_gz_rocket_jielian_a2416a
cp "$generated/px4/${airframe_id}_gz_rocket_jielian_a2416a" \
  "$run_dir/etc/init.d-posix/airframes/"
if [[ "$mode" == headless ]]; then
  start_service gazebo env -u DISPLAY gz sim -s -r --headless-rendering "$generated/tests/world.sdf"
else
  start_service gazebo gz sim -r "$generated/tests/world.sdf"
fi
world_pid="$last_pid"
imu_topic="/world/$world_name/model/$model_name/link/base_link/sensor/imu_sensor/imu"
start_service imu_wait timeout "$imu_timeout" gz topic -e -t "$imu_topic" -n 1
imu_pid="$last_pid"
while kill -0 "$imu_pid" 2>/dev/null; do
  if ! kill -0 "$world_pid" 2>/dev/null; then
    echo "Gazebo 在 IMU 就绪前退出，请查看 $run_dir/gazebo.log" >&2
    exit 1
  fi
  sleep .2
done
if wait "$imu_pid"; then
  cp "$run_dir/imu_wait.log" "$run_dir/imu_sample.txt"
else
  status=$?
  echo "等待 IMU 失败（$status），请查看 $run_dir/gazebo.log 与 imu_wait.log" >&2
  exit "$status"
fi

echo 'IMU 已就绪，启动真值遥测采集、PX4及完整飞行验收。'
start_service tracking_capture "$flight_python" -u "$capture_tool" \
  --topic "$topic_prefix/telemetry" --output "$tracking_telemetry"
capture_pid="$last_pid"
sleep .3
gz topic -t "$topic_prefix/command" -m gz.msgs.StringMsg \
  -p 'data: "{\"cmd\":\"recognition_on\"}"'
gz topic -t "$topic_prefix/command" -m gz.msgs.StringMsg \
  -p 'data: "{\"cmd\":\"auto_lock_on\"}"'

start_service px4 env PX4_SIM_MODEL=gz_rocket_jielian_a2416a PX4_GZ_STANDALONE=1 \
  PX4_GZ_WORLD="$world_name" PX4_GZ_MODEL_NAME="$model_name" \
  "$px4_bin" -i "$px4_instance" -d "$run_dir/etc" -w "$run_dir/rootfs"
px4_pid="$last_pid"
start_service flight "$flight_python" -u "$flight_check" --ground-truth \
  --endpoint "$mavlink_endpoint" --config "$config" --log "$result"
flight_pid="$last_pid"
if wait "$flight_pid"; then
  :
else
  status=$?
  exit "$status"
fi

stop_service "$capture_pid"
stop_service "$px4_pid"
"$flight_python" -u "$tracking_check" --telemetry "$tracking_telemetry" \
  --flight-report "$result" --output "$tracking_result"
echo "完整飞行与捷联跟踪验收通过：$tracking_result"
