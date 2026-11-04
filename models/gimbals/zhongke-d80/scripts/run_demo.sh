#!/usr/bin/env bash
set -euo pipefail
COMPONENT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${COMPONENT_ROOT}/../.venv-zhongke-d80/bin/python"
if [[ ! -x "${PYTHON_BIN}" ]]; then
  bash "${COMPONENT_ROOT}/scripts/setup.sh"
fi
exec "${PYTHON_BIN}" "${COMPONENT_ROOT}/scripts/runtime.py" "$@"
