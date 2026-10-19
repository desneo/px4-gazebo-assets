#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"
PYTHON="${GIMBAL_PYTHON:-$PROJ/.venv-linux/bin/python}"
PREPARE_ONLY=false
ARGS=("$@")
if [[ "${ARGS[0]:-}" == "--prepare-only" ]]; then
  PREPARE_ONLY=true
  ARGS=("${ARGS[@]:1}")
fi
VISION_CONFIG="$PKG/service/vision.json"
for ((index=0; index<${#ARGS[@]}; index++)); do
  case "${ARGS[index]}" in
    --vision-config)
      ((index + 1 < ${#ARGS[@]})) || { echo "--vision-config 缺少路径" >&2; exit 2; }
      VISION_CONFIG="${ARGS[index + 1]}"
      ((index += 1))
      ;;
    --vision-config=*) VISION_CONFIG="${ARGS[index]#*=}" ;;
  esac
done

check_environment() {
  [[ -x "$PYTHON" ]] \
    && [[ "$("$PYTHON" -c 'print("two-axis-python")' 2>/dev/null)" == "two-axis-python" ]] \
    && "$PYTHON" "$HERE/check_bridge_env.py" --vision-config "$VISION_CONFIG"
}

if ! check_environment; then
  if [[ -n "${GIMBAL_PYTHON:-}" ]]; then
    echo "GIMBAL_PYTHON 指定的解释器缺少网关依赖、ONNX DNN 或 NVENC：$PYTHON" >&2
    echo "请取消 GIMBAL_PYTHON 后重试，让启动器自动配置项目环境。" >&2
    exit 1
  fi
  bash "$HERE/setup_bridge.sh" "$VISION_CONFIG"
  check_environment
fi
if "$PREPARE_ONLY"; then
  echo "两轴吊舱网关环境检查完成：$PYTHON"
  exit 0
fi
mkdir -p "$RUNDIR"
if [[ -f "$RUNDIR/bridge.pid" ]] && kill -0 "$(cat "$RUNDIR/bridge.pid")" 2>/dev/null; then
  echo "网关已运行，请先停止：$RUNDIR/bridge.pid" >&2; exit 1
fi
setsid "$PYTHON" -u "$PKG/service/gimbal_bridge.py" "${ARGS[@]}" >"$RUNDIR/bridge.log" 2>&1 &
pid=$!
echo "$pid" > "$RUNDIR/bridge.pid"
sleep 2
kill -0 "$pid" 2>/dev/null || { echo "网关启动失败，查看 $RUNDIR/bridge.log" >&2; exit 1; }
echo "网关已启动 pid=$pid 日志=$RUNDIR/bridge.log"
