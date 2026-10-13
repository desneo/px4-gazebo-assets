#!/bin/sh
# 单入口完成仿真、验收和清理；不修改PX4工作区。
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MODE=${1:-headless}
CONFIG=${2:-${SCRIPT_DIR}/../parameters.json}
case "$MODE" in headless|sitl) ;; *) echo '用法：sh run_check.sh [headless|sitl] [parameters.json] [result.json]' >&2; exit 1 ;; esac
START_LOG=$(mktemp "${TMPDIR:-/tmp}/rocket-start.XXXXXX")
RUN_PID=
FLIGHT_PID=
cleanup() {
    trap - INT TERM EXIT
    [ -z "$FLIGHT_PID" ] || kill -TERM "$FLIGHT_PID" 2>/dev/null || true
    [ -z "$RUN_PID" ] || kill -TERM "$RUN_PID" 2>/dev/null || true
    [ -z "$FLIGHT_PID" ] || wait "$FLIGHT_PID" 2>/dev/null || true
    [ -z "$RUN_PID" ] || wait "$RUN_PID" 2>/dev/null || true
}
trap cleanup INT TERM EXIT
sh "${SCRIPT_DIR}/run.sh" "$MODE" "$CONFIG" >"$START_LOG" 2>&1 &
RUN_PID=$!
RUN_DIR=
for _ in $(seq 1 120); do
    if ! kill -0 "$RUN_PID" 2>/dev/null; then
        cat "$START_LOG"
        exit 1
    fi
    RUN_DIR=$(sed -n 's/^运行目录：//p' "$START_LOG")
    if [ -n "$RUN_DIR" ] && [ -f "${RUN_DIR}/px4.log" ]; then
        break
    fi
    sleep 1
done
if [ -z "$RUN_DIR" ] || [ ! -f "${RUN_DIR}/px4.log" ]; then
    echo "启动超时：$START_LOG" >&2
    exit 1
fi
cat "$START_LOG"
RESULT=${3:-${RUN_DIR}/flight_result.json}
GZ_PARTITION=$(sed -n 's/^Gazebo 分区：//p' "$START_LOG")
export GZ_PARTITION
python3 -u "${SCRIPT_DIR}/flight_check.py" --ground-truth --config "${RUN_DIR}/models/rocket_racing_quad/parameters.json" --log "$RESULT" &
FLIGHT_PID=$!
wait "$FLIGHT_PID"
FLIGHT_PID=
