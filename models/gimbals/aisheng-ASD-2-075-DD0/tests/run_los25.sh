#!/usr/bin/env bash
# 25°/s 视线角速度验收：启动动态目标世界 + 网关 + 回环检查。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="$(cd "$HERE/.." && pwd)"
REPO_DIR="$(cd "$MODEL_DIR/../../.." && pwd)"
RUNDIR="${AISHENG_RUNDIR:-$REPO_DIR/runs/aisheng_los25}"
PYTHON="${AISHENG_PYTHON:-$REPO_DIR/.venv-linux/bin/python}"
[[ -x "$PYTHON" ]] || { echo "缺少 Python：$PYTHON" >&2; exit 1; }

if [[ ! -f "$MODEL_DIR/controller/build/lib/libgz-aisheng-seeker-controller.so" ]]; then
  cmake -S "$MODEL_DIR/controller" -B "$MODEL_DIR/controller/build" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  cmake --build "$MODEL_DIR/controller/build" -j"$(nproc)"
fi
python3 "$MODEL_DIR/tools/generate_model.py"
mkdir -p "$RUNDIR"
export GZ_SIM_RESOURCE_PATH="$REPO_DIR/models/gimbals:$REPO_DIR/models/vehicles:$REPO_DIR/models/props:$REPO_DIR/assemblies${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$MODEL_DIR/controller/build/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_PARTITION="${GZ_PARTITION:-aisheng_los25_$$}"

SIM_PID=
GATEWAY_PID=
stop() {
  trap - INT TERM EXIT
  [ -z "$GATEWAY_PID" ] || kill "$GATEWAY_PID" 2>/dev/null || true
  [ -z "$SIM_PID" ] || /bin/kill -TERM -- "-$SIM_PID" 2>/dev/null || true
  [ -z "$GATEWAY_PID" ] || wait "$GATEWAY_PID" 2>/dev/null || true
  [ -z "$SIM_PID" ] || wait "$SIM_PID" 2>/dev/null || true
}
trap stop INT TERM EXIT

setsid gz sim -r -s --headless-rendering -v2 "$MODEL_DIR/tests/los25.world.sdf" \
  >"$RUNDIR/gazebo.log" 2>&1 &
SIM_PID=$!
sleep 5
GZ_PARTITION="$GZ_PARTITION" "$PYTHON" "$MODEL_DIR/gateway/main.py" \
  >"$RUNDIR/gateway.log" 2>&1 &
GATEWAY_PID=$!
sleep 3
set +e
GZ_PARTITION="$GZ_PARTITION" "$PYTHON" "$MODEL_DIR/tests/check_los25.py"
status=$?
set -e
echo "25°/s 验收退出码：$status；日志：$RUNDIR"
exit "$status"
