#!/usr/bin/env bash
# 仅终止指定运行目录拥有的进程，先请求有界降落。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$HERE/../.." && pwd)"
PROCESS="$PROJ/scenarios/geographic_rocket_two_axis_pod/process_state.py"
[[ $# == 1 ]] || { echo '用法：bash stop.sh RUN_DIR' >&2; exit 2; }
RUN_DIR="$(realpath -m "$1")"
if python3 "$PROCESS" signal "$RUN_DIR/launcher.pid" TERM; then
  echo "已请求停止；先尝试降落，请查看 $RUN_DIR/flight.log 和 trial.json。"
else
  echo '启动器已退出；核对 PID 与内核启动时间后清理本次运行残留服务。'
  if python3 "$PROCESS" live "$RUN_DIR/flight.pid"; then
    python3 "$PROCESS" signal "$RUN_DIR/flight.pid" INT || true
    limit=$(python3 - "$RUN_DIR/config.json" <<'PY_LIMIT'
import json,math,sys
try: value=json.load(open(sys.argv[1]))['landing_timeout_s']
except (OSError,ValueError,KeyError): value=180
if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0: value=180
print(math.ceil(float(value))+30)
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
  sleep 2
  for service in flight imu_wait gui px4 gazebo; do
    python3 "$PROCESS" signal "$RUN_DIR/$service.pid" KILL || true
    rm -f "$RUN_DIR/$service.pid"
  done
  rm -f "$RUN_DIR/launcher.pid"
  echo "本次残留服务清理完成；日志保留在 $RUN_DIR。"
fi
