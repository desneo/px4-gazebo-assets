#!/bin/sh
# 从参数文件生成独立运行副本，启动 Gazebo / PX4；日志保存在运行目录。
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MODEL_DIR=$(CDPATH= cd -- "${SCRIPT_DIR}/.." && pwd)
PX4_ROOT=${PX4_ROOT:-/home/aeroarx/test/codex_gimbal_test/PX4-Autopilot}
PX4_BIN="${PX4_ROOT}/build/px4_sitl_default/bin/px4"
MODE=${1:-sitl}
CONFIG=${2:-${MODEL_DIR}/parameters.json}
case "$MODE" in view|sitl|headless) ;; *) echo '用法：sh run.sh {view|sitl|headless} [parameters.json]' >&2; exit 1 ;; esac
if [ "$MODE" != view ] && [ ! -x "$PX4_BIN" ]; then
    echo "找不到 PX4 可执行文件：${PX4_BIN}；请先构建 make px4_sitl" >&2
    exit 1
fi
RUN_DIR=$(mktemp -d "${TMPDIR:-/tmp}/rocket-racing.XXXXXX")
GENERATED="${RUN_DIR}/models/rocket_racing_quad"
python3 "${MODEL_DIR}/tools/generate_model.py" --config "$CONFIG" --output-dir "$GENERATED"
export GZ_SIM_RESOURCE_PATH="${RUN_DIR}/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
# 只作用于本次仿真，避免与其他 Gazebo 世界共享通信分区。
export GZ_PARTITION=${GZ_PARTITION:-rocket_racing_$$}
echo "运行目录：${RUN_DIR}"
echo "Gazebo 分区：${GZ_PARTITION}"
if [ "$MODE" = view ]; then
    exec gz sim -r "${GENERATED}/tests/world.sdf"
fi
mkdir -p "${RUN_DIR}/rootfs"
cp -R "${PX4_ROOT}/build/px4_sitl_default/etc" "${RUN_DIR}/etc"
rm -f "${RUN_DIR}/etc/init.d-posix/airframes/"*_gz_rocket_racing_quad
cp "${GENERATED}"/px4/* "${RUN_DIR}/etc/init.d-posix/airframes/"
WORLD_PID=
PX4_PID=
stop() {
    trap - INT TERM EXIT
    [ -z "$PX4_PID" ] || /bin/kill -TERM -- "-$PX4_PID" 2>/dev/null || true
    [ -z "$WORLD_PID" ] || /bin/kill -TERM -- "-$WORLD_PID" 2>/dev/null || true
    [ -z "$PX4_PID" ] || wait "$PX4_PID" 2>/dev/null || true
    [ -z "$WORLD_PID" ] || wait "$WORLD_PID" 2>/dev/null || true
}
trap stop INT TERM EXIT
if [ "$MODE" = headless ]; then
    setsid gz sim -s -r "${GENERATED}/tests/world.sdf" >"${RUN_DIR}/gazebo.log" 2>&1 &
else
    setsid gz sim -r "${GENERATED}/tests/world.sdf" >"${RUN_DIR}/gazebo.log" 2>&1 &
fi
WORLD_PID=$!
IMU_TIMEOUT=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["simulation"]["imu_wait_timeout_s"])' "$CONFIG")
TOPIC=/world/rocket_racing_quad_test/model/rocket_racing_quad/link/base_link/sensor/imu_sensor/imu
if ! timeout "$IMU_TIMEOUT" gz topic -e -t "$TOPIC" -n 1 >"${RUN_DIR}/imu_sample.txt" 2>&1; then
    echo "等待IMU超时，请查看 ${RUN_DIR}/gazebo.log" >&2
    exit 1
fi
echo 'IMU已就绪，启动PX4；估计器仍需初始化。'
PX4_SIM_MODEL=gz_rocket_racing_quad PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=rocket_racing_quad_test PX4_GZ_MODEL_NAME=rocket_racing_quad \
setsid "$PX4_BIN" -d "${RUN_DIR}/etc" -w "${RUN_DIR}/rootfs" >"${RUN_DIR}/px4.log" 2>&1 &
PX4_PID=$!
echo "PX4日志：${RUN_DIR}/px4.log"
echo "飞行验证：python3 ${SCRIPT_DIR}/flight_check.py --config ${GENERATED}/parameters.json"
wait "$PX4_PID"
