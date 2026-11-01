#!/usr/bin/env bash
# ASD-2-075-DD0 外观与两轴转动预览入口。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="$(cd "$HERE/.." && pwd)"
REPO_DIR="$(cd "$MODEL_DIR/../../.." && pwd)"
RUNDIR="${AISHENG_PREVIEW_RUNDIR:-$REPO_DIR/runs/aisheng_seeker_preview}"
PYTHON="${AISHENG_PYTHON:-$REPO_DIR/.venv-linux/bin/python}"
SWEEP=1
for arg in "$@"; do
  case "$arg" in
    --no-sweep) SWEEP=0 ;;
    *) echo "用法：bash $0 [--no-sweep]" >&2; exit 2 ;;
  esac
done
[[ -n "${DISPLAY:-}" ]] || { echo '没有 DISPLAY，无法启动 Gazebo GUI。' >&2; exit 1; }
[[ -x "$PYTHON" ]] || { echo "缺少 Python：$PYTHON" >&2; exit 1; }

if [[ ! -f "$MODEL_DIR/controller/build/lib/libgz-aisheng-seeker-controller.so" ]]; then
  echo '构建导引头预览控制器……'
  cmake -S "$MODEL_DIR/controller" -B "$MODEL_DIR/controller/build" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  cmake --build "$MODEL_DIR/controller/build" -j"$(nproc)"
fi

python3 "$MODEL_DIR/tools/generate_model.py"
mkdir -p "$RUNDIR"
export GZ_SIM_RESOURCE_PATH="$REPO_DIR/models/gimbals:$REPO_DIR/models/vehicles:$REPO_DIR/models/props:$REPO_DIR/assemblies${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$MODEL_DIR/controller/build/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_PARTITION="${GZ_PARTITION:-aisheng_seeker_preview_$$}"

SIM_PID=
SWEEP_PID=
SETUP_PID=
stop() {
  trap - INT TERM EXIT
  [ -z "$SETUP_PID" ] || kill "$SETUP_PID" 2>/dev/null || true
  [ -z "$SWEEP_PID" ] || kill "$SWEEP_PID" 2>/dev/null || true
  [ -z "$SIM_PID" ] || /bin/kill -TERM -- "-$SIM_PID" 2>/dev/null || true
  [ -z "$SETUP_PID" ] || wait "$SETUP_PID" 2>/dev/null || true
  [ -z "$SWEEP_PID" ] || wait "$SWEEP_PID" 2>/dev/null || true
  [ -z "$SIM_PID" ] || wait "$SIM_PID" 2>/dev/null || true
}
trap stop INT TERM EXIT

echo "分区：$GZ_PARTITION"
echo "运行目录：$RUNDIR"
echo '实体：AISHENG_SEEKER；定位标志：AISHENG_SEEKER_LOCATION_MARKER'
echo '近景浏览：鼠标放在模型表面，滚轮缩放；右键缓慢拖动微调。'
setsid gz sim -r "$MODEL_DIR/tests/preview.world.sdf" >"$RUNDIR/gazebo.log" 2>&1 &
SIM_PID=$!
echo "Gazebo pid=$SIM_PID；日志：$RUNDIR/gazebo.log"

(
  count=0
  while [ "$count" -lt 30 ]; do
    reply=$(gz service --service /gui/camera/view_control/sensitivity \
      --reqtype gz.msgs.Double --reptype gz.msgs.Boolean \
      --timeout 500 --req 'data: 0.1' 2>&1) || reply='服务尚未就绪'
    case "$reply" in
      *'data: true'*) echo '浏览相机鼠标灵敏度已设置为 0.1'; exit 0 ;;
    esac
    count=$((count + 1))
    sleep 1
  done
  echo '浏览相机服务未就绪，可按 README 手动设置。'
) >"$RUNDIR/camera.log" 2>&1 &
SETUP_PID=$!

if [[ "$SWEEP" -eq 1 ]]; then
  (
    sleep 3
    exec "$PYTHON" "$MODEL_DIR/tests/sweep.py" --duration 30
  ) >"$RUNDIR/sweep.log" 2>&1 &
  SWEEP_PID=$!
  echo "自动扫掠 pid=$SWEEP_PID；日志：$RUNDIR/sweep.log"
  echo "手动指令：GZ_PARTITION=$GZ_PARTITION $PYTHON $MODEL_DIR/tests/command.py manual 30 0 --duration 2"
  echo "回零：GZ_PARTITION=$GZ_PARTITION $PYTHON $MODEL_DIR/tests/command.py home"
fi

wait "$SIM_PID"
