#!/usr/bin/env bash
# 两轴吊舱组件的资源与运行目录；继承调用方的 GZ_PARTITION。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
PKG="$PROJ/models/gimbals/two_axis_pod/controller"
RUNDIR="${GIMBAL_RUNDIR:-$PROJ/runs/two_axis_pod}"
export GZ_SIM_RESOURCE_PATH="$PROJ/models/gimbals:$PROJ/models/props:$PROJ/assemblies${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PKG/build/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_SERVER_CONFIG_PATH="$SCRIPT_DIR/server.config"
export RUNDIR
