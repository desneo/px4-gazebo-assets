#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
export GZ_SIM_RESOURCE_PATH="$(dirname "$ROOT")${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_PARTITION=${GZ_PARTITION:-delta_wing_display_$$}
printf '静态展示分区：%s\n' "$GZ_PARTITION"
exec gz sim -r "$ROOT/tests/world.sdf"
