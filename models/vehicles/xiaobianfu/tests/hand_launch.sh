#!/bin/sh
set -eu
# 在当前 GZ_PARTITION 对应的世界抛出飞机；先解锁并进入起飞任务。
D=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
HEIGHT=${LAUNCH_HEIGHT_M:-$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["launch"]["height_m"])' "$D/parameters.json")}
gz service -s /world/xiaobianfu_test/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'
gz service -s /world/xiaobianfu_test/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 --req "name: 'xiaobianfu', position: {x: 0,y: 0,z: $HEIGHT}, orientation: {w: 1}"
gz topic -t /model/xiaobianfu/hand_launch -m gz.msgs.Boolean -p 'data: true'
gz service -s /world/xiaobianfu_test/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'
