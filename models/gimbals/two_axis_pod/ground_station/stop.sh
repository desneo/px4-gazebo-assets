#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$HERE/../../../.." && pwd)"
STATION_PYTHON="${GIMBAL_PYTHON:-$PROJ/.venv-linux/bin/python}"
if [[ ! -x "$STATION_PYTHON" ]]; then
  echo "没有可用的 Station 环境；本入口未启动可管理的服务。" >&2
  exit 1
fi
exec "$STATION_PYTHON" -u "$HERE/station_lifecycle.py" stop "$@"
