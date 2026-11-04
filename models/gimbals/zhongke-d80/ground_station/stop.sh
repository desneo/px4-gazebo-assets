#!/usr/bin/env bash
set -euo pipefail
# 无参数停止本组件登记的活跃地面站；显式配置/会话只停止对应客户端。
ZHONGKE_STATION_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ZHONGKE_COMPONENT_DIR="$(cd "$ZHONGKE_STATION_DIR/.." && pwd)"
ZHONGKE_STATION_PYTHON="${ZHONGKE_D80_PYTHON:-$ZHONGKE_COMPONENT_DIR/../.venv-zhongke-d80/bin/python}"
if [[ ! -x "$ZHONGKE_STATION_PYTHON" ]]; then
  echo "中科专属 Python 环境不存在，未启动停止操作。" >&2
  exit 1
fi
exec "$ZHONGKE_STATION_PYTHON" "$ZHONGKE_STATION_DIR/lifecycle.py" stop "$@"
