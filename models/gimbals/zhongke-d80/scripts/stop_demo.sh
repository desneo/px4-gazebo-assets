#!/usr/bin/env bash
set -euo pipefail
COMPONENT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${COMPONENT_ROOT}/../.venv-zhongke-d80/bin/python" "${COMPONENT_ROOT}/scripts/session.py" stop "$@"
