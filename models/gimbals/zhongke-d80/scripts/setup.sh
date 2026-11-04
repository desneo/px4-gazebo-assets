#!/usr/bin/env bash
set -euo pipefail
COMPONENT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${COMPONENT_ROOT}/../.venv-zhongke-d80"
CACHE_DIR="${COMPONENT_ROOT}/../.cache-zhongke-d80"
if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  if command -v uv >/dev/null 2>&1; then
    uv venv --no-project --system-site-packages --python /usr/bin/python3 \
      --no-python-downloads --cache-dir "${CACHE_DIR}" "${VENV_DIR}"
  else
    python3 -m venv --system-site-packages "${VENV_DIR}"
  fi
fi
if ! "${VENV_DIR}/bin/python" -c 'import cv2, numpy, PIL, jsonschema' >/dev/null 2>&1; then
  if command -v uv >/dev/null 2>&1; then
    uv venv --no-project --allow-existing --system-site-packages --python /usr/bin/python3 \
      --no-python-downloads --cache-dir "${CACHE_DIR}" "${VENV_DIR}"
    uv pip install --python "${VENV_DIR}/bin/python" --cache-dir "${CACHE_DIR}" \
      -r "${COMPONENT_ROOT}/scripts/requirements-linux.txt"
  else
    "${VENV_DIR}/bin/python" -m pip install --cache-dir "${CACHE_DIR}" -r "${COMPONENT_ROOT}/scripts/requirements-linux.txt"
  fi
fi
exec "${VENV_DIR}/bin/python" "${COMPONENT_ROOT}/scripts/check_env.py"
