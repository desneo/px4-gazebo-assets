#!/usr/bin/env bash
# 请求启动器先完成有界安全降落，再停止其拥有的服务。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$HERE/../.." && pwd)"
RUN_DIR="${1:-$PROJ/runs/geographic_rocket_two_axis_pod}"
if python3 "$HERE/process_state.py" signal "$RUN_DIR/launcher.pid" TERM; then
  echo "已请求停止；启动器将先尝试降落，请查看 $RUN_DIR/flight.log 和 flight_result.json。"
else
  echo "启动器已退出，核对 PID、启动时间与独立 session 后清理本场景残留服务。"
  if python3 "$HERE/process_state.py" live "$RUN_DIR/flight.pid"; then
    python3 "$HERE/process_state.py" signal "$RUN_DIR/flight.pid" INT || true
    limit=$(python3 - "$RUN_DIR/config.json" <<'PY_LIMIT'
import json,math,sys
try: value=json.load(open(sys.argv[1]))['landing_timeout_s']
except (OSError,ValueError,KeyError): value=180
if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0: value=180
print(math.ceil(float(value))+30)
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
  sleep 2
  for service in flight imu_wait bridge gui px4 gazebo; do
    python3 "$HERE/process_state.py" signal "$RUN_DIR/$service.pid" KILL || true
    rm -f "$RUN_DIR/$service.pid"
  done
  rm -f "$RUN_DIR/launcher.pid"
  echo "残留服务清理完成；日志保留在 $RUN_DIR。"
fi
