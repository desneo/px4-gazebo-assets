#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${GIMBAL_PYTHON:-python3}" -u "$HERE/ground_station.py" "$@"
