#!/bin/sh
# 在独立运行目录中生成静态外形，启动 Gazebo 人工观察入口。
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MODEL_DIR=$(CDPATH= cd -- "${SCRIPT_DIR}/.." && pwd)
REPO_DIR=$(CDPATH= cd -- "${MODEL_DIR}/../.." && pwd)
VIEW=${1:-overall}
YAW=${2:-0}
PITCH=${3:-0}
case "$VIEW" in
    overall) WORLD=world.sdf ;;
    close) WORLD=closeup.sdf ;;
    *) echo '用法：sh run.sh {overall|close} [yaw_deg] [pitch_deg]' >&2; exit 1 ;;
esac
RUN_DIR=$(mktemp -d "${TMPDIR:-/tmp}/rocket-two-axis-pod.XXXXXX")
GENERATED="${RUN_DIR}/models/rocket_two_axis_pod"
python3 "${MODEL_DIR}/tools/generate_model.py" --output-dir "$GENERATED" --yaw-deg "$YAW" --pitch-deg "$PITCH"
export GZ_SIM_RESOURCE_PATH="${RUN_DIR}/models:${REPO_DIR}/models/vehicles:${REPO_DIR}/models/gimbals${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_PARTITION=${GZ_PARTITION:-rocket_two_axis_pod_preview_$$}
SENSITIVITY=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["view"]["mouse_sensitivity"])' "${MODEL_DIR}/parameters.json")
echo "运行目录：${RUN_DIR}"
echo "分区：${GZ_PARTITION}；实体：ROCKET_TWO_AXIS_POD"
echo '吊舱旁橙色箭头：ROCKET_POD_LOCATION_MARKER；近景缩放请将鼠标放在实体表面，右键缓慢拖动微调。'
SIM_PID=
SETUP_PID=
stop() {
    trap - INT TERM EXIT
    [ -z "$SETUP_PID" ] || kill "$SETUP_PID" 2>/dev/null || true
    [ -z "$SIM_PID" ] || /bin/kill -TERM -- "-$SIM_PID" 2>/dev/null || true
    [ -z "$SETUP_PID" ] || wait "$SETUP_PID" 2>/dev/null || true
    [ -z "$SIM_PID" ] || wait "$SIM_PID" 2>/dev/null || true
}
trap stop INT TERM EXIT
setsid gz sim -r "${GENERATED}/tests/${WORLD}" >"${RUN_DIR}/gazebo.log" 2>&1 &
SIM_PID=$!
(
    count=0
    while [ "$count" -lt 30 ]; do
        reply=$(gz service --service /gui/camera/view_control/sensitivity --reqtype gz.msgs.Double \
            --reptype gz.msgs.Boolean --timeout 500 --req "data: ${SENSITIVITY}" 2>&1) || reply='服务尚未就绪'
        printf '%s\n' "$reply"
        case "$reply" in
            *'data: true'*) echo "浏览相机鼠标灵敏度已设置为 ${SENSITIVITY}"; exit 0 ;;
        esac
        count=$((count + 1))
        sleep 1
    done
    echo '浏览相机服务未就绪，可按 README 中命令手动设置。'
) >"${RUN_DIR}/camera.log" 2>&1 &
SETUP_PID=$!
echo "Gazebo 日志：${RUN_DIR}/gazebo.log；相机设置日志：${RUN_DIR}/camera.log"
wait "$SIM_PID"
