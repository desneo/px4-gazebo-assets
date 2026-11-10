#!/usr/bin/env bash
set -euo pipefail
station_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
model_dir="$(cd "$station_dir/.." && pwd)"
python_bin="${JIELIAN_A2416A_PYTHON:-$model_dir/../../../.venv-linux/bin/python}"
if [[ ! -x "$python_bin" ]]; then
  echo "A2416A地面站Python环境不存在：$python_bin" >&2
  exit 2
fi
cd "$model_dir"
exec "$python_bin" "$station_dir/lifecycle.py" run "$@"
