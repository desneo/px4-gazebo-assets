#!/usr/bin/env bash
# 完整演示：Gazebo + 二进制网关 + RTP 双路视频 + 上位机。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="$(cd "$HERE/.." && pwd)"
REPO_DIR="$(cd "$MODEL_DIR/../../.." && pwd)"
RUNDIR="${AISHENG_RUNDIR:-$REPO_DIR/runs/aisheng_seeker_demo}"
PYTHON="${AISHENG_PYTHON:-$REPO_DIR/.venv-linux/bin/python}"
HEADLESS=0
MOVE=0
for arg in "$@"; do
  case "$arg" in
    --headless) HEADLESS=1 ;;
    --move) MOVE=1 ;;
    *) echo "用法：bash $0 [--headless] [--move]" >&2; exit 2 ;;
  esac
done
[[ -x "$PYTHON" ]] || { echo "缺少 Python：$PYTHON" >&2; exit 1; }

if [[ ! -f "$MODEL_DIR/controller/build/lib/libgz-aisheng-seeker-controller.so" ]]; then
  echo '构建导引头控制器……'
  cmake -S "$MODEL_DIR/controller" -B "$MODEL_DIR/controller/build" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  cmake --build "$MODEL_DIR/controller/build" -j"$(nproc)"
fi
python3 "$MODEL_DIR/tools/generate_model.py"
mkdir -p "$RUNDIR"
export GZ_SIM_RESOURCE_PATH="$REPO_DIR/models/gimbals:$REPO_DIR/models/vehicles:$REPO_DIR/models/props:$REPO_DIR/assemblies${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$MODEL_DIR/controller/build/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_PARTITION="${GZ_PARTITION:-aisheng_seeker_demo_$$}"

SIM_PID=
GATEWAY_PID=
GCS_PID=
MOVE_PID=
stop() {
  trap - INT TERM EXIT
  [ -z "$MOVE_PID" ] || kill "$MOVE_PID" 2>/dev/null || true
  [ -z "$GCS_PID" ] || kill "$GCS_PID" 2>/dev/null || true
  [ -z "$GATEWAY_PID" ] || kill "$GATEWAY_PID" 2>/dev/null || true
  [ -z "$SIM_PID" ] || /bin/kill -TERM -- "-$SIM_PID" 2>/dev/null || true
  [ -z "$MOVE_PID" ] || wait "$MOVE_PID" 2>/dev/null || true
  [ -z "$GCS_PID" ] || wait "$GCS_PID" 2>/dev/null || true
  [ -z "$GATEWAY_PID" ] || wait "$GATEWAY_PID" 2>/dev/null || true
  [ -z "$SIM_PID" ] || wait "$SIM_PID" 2>/dev/null || true
}
trap stop INT TERM EXIT

echo "分区：$GZ_PARTITION"
echo "运行目录：$RUNDIR"
if [[ "$HEADLESS" -eq 1 ]]; then
  setsid gz sim -r -s --headless-rendering -v2 "$MODEL_DIR/tests/preview.world.sdf" \
    >"$RUNDIR/gazebo.log" 2>&1 &
else
  setsid gz sim -r "$MODEL_DIR/tests/preview.world.sdf" >"$RUNDIR/gazebo.log" 2>&1 &
fi
SIM_PID=$!
echo "Gazebo pid=$SIM_PID"
sleep 4
GZ_PARTITION="$GZ_PARTITION" "$PYTHON" "$MODEL_DIR/gateway/main.py" --video --verbose \
  >"$RUNDIR/gateway.log" 2>&1 &
GATEWAY_PID=$!
echo "网关 pid=$GATEWAY_PID；日志：$RUNDIR/gateway.log"
sleep 3
"$PYTHON" "$MODEL_DIR/ground_station/main.py" >"$RUNDIR/gcs.log" 2>&1 &
GCS_PID=$!
echo "上位机 pid=$GCS_PID；日志：$RUNDIR/gcs.log"
if [[ "$MOVE" -eq 1 ]]; then
  GZ_PARTITION="$GZ_PARTITION" "$PYTHON" "$MODEL_DIR/tests/move_target.py" --duration 60 \
    >"$RUNDIR/move_target.log" 2>&1 &
  MOVE_PID=$!
  echo "移动目标 pid=$MOVE_PID；日志：$RUNDIR/move_target.log"
fi
echo '上位机按键：U 解锁  L 电锁  R 归中  C 取消锁定  O 自动锁定  P 关闭自动锁定'
echo '            1 广角  2 长焦  Z 变焦  WASD/方向键 手动  Q 退出'
echo '双击视频画面锁定目标；Yaw/Pitch 滑条发送指定框架角。'
wait "$GCS_PID"
