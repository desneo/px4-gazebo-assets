#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$HERE/../../../.." && pwd)"
PYTHON="${GIMBAL_PYTHON:-$PROJ/.venv-linux/bin/python}"

if [[ ! -x "$PYTHON" ]] \
    || ! "$PYTHON" "$HERE/check_linux_env.py" >/dev/null 2>&1; then
  if [[ -n "${GIMBAL_PYTHON:-}" ]]; then
    echo "GIMBAL_PYTHON 指定的解释器缺少地面站依赖或 NVDEC：$PYTHON" >&2
    echo "请取消 GIMBAL_PYTHON 后重试，让启动器自动配置项目环境。" >&2
    exit 2
  fi
  echo "Linux 地面站环境缺失或不完整，正在自动配置 ..."
  bash "$HERE/setup_linux.sh"
  PYTHON="$PROJ/.venv-linux/bin/python"
fi

# ground_station.py 默认读取同目录 config.json；额外参数仅作临时覆盖。
exec "$PYTHON" -u "$HERE/ground_station.py" "$@"
