#!/usr/bin/env bash
set -euo pipefail
COMPONENT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_PATH="${COMPONENT_ROOT}/parameters.ini"
if [[ "${1:-}" == "--config" ]]; then
  CONFIG_PATH="${2:?--config需要路径}"
fi
PYTHON_BIN="${COMPONENT_ROOT}/../.venv-zhongke-d80/bin/python"
if [[ ! -x "${PYTHON_BIN}" ]]; then
  bash "${COMPONENT_ROOT}/scripts/setup.sh"
fi
cmake -S "${COMPONENT_ROOT}/controller" -B "${COMPONENT_ROOT}/controller/build" -DBUILD_TESTING=ON
cmake --build "${COMPONENT_ROOT}/controller/build" -j "${ZHONGKE_BUILD_JOBS:-4}"
"${PYTHON_BIN}" "${COMPONENT_ROOT}/tools/generate_model.py" --config "${CONFIG_PATH}"
