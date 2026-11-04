#!/usr/bin/env bash
set -euo pipefail
ZHONGKE_STATION_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ZHONGKE_COMPONENT_DIR="$(cd "$ZHONGKE_STATION_DIR/.." && pwd)"
ZHONGKE_STATION_PYTHON="${ZHONGKE_D80_PYTHON:-$ZHONGKE_COMPONENT_DIR/../.venv-zhongke-d80/bin/python}"
if [[ ! -x "$ZHONGKE_STATION_PYTHON" ]]; then
  echo "中科专属 Python 环境不存在，请先执行本组件 scripts/setup.sh。" >&2
  exit 1
fi
exec "$ZHONGKE_STATION_PYTHON" "$ZHONGKE_STATION_DIR/lifecycle.py" run "$@"
