#!/usr/bin/env python3
"""用 Gazebo set_pose 服务移动演示目标飞机，验证动态真值跟踪。"""
from __future__ import annotations

import argparse
import math
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default="aisheng_seeker_preview")
    parser.add_argument("--target", default="target_aircraft")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--center-x", type=float, default=3.7)
    parser.add_argument("--center-y", type=float, default=0.55)
    parser.add_argument("--center-z", type=float, default=1.5)
    parser.add_argument("--amp-y", type=float, default=0.30)
    parser.add_argument("--amp-z", type=float, default=0.12)
    parser.add_argument("--period-y", type=float, default=10.0)
    parser.add_argument("--period-z", type=float, default=8.0)
    args = parser.parse_args()

    node = Node()
    service = f"/world/{args.world}/set_pose"
    start = time.monotonic()
    next_print = start
    while True:
        now = time.monotonic()
        t = now - start
        if t >= args.duration:
            break
        y = args.center_y + args.amp_y * math.sin(2.0 * math.pi * t / args.period_y)
        z = args.center_z + args.amp_z * math.sin(2.0 * math.pi * t / args.period_z)
        pose = Pose()
        pose.name = args.target
        pose.position.x = args.center_x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        node.request(service, pose, Pose, Boolean, 1000)
        if now >= next_print:
            print(f"t={t:5.1f}s target=({args.center_x:.2f},{y:+.2f},{z:+.2f})",
                  flush=True)
            next_print = now + 1.0
        time.sleep(1.0 / max(1.0, args.rate))


if __name__ == "__main__":
    main()
