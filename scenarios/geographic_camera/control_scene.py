#!/usr/bin/env python3
"""移动地理视觉验证场景中的实体目标和遮挡板。"""
import argparse
import os
import subprocess
import time
from scene_geometry import panel_position, target_x


def set_pose(name, x, y, z):
    request = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1}}'
    result = subprocess.run([
        'gz', 'service', '-s', '/world/geographic_camera/set_pose',
        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req', request,
    ], capture_output=True, text=True, timeout=7, check=True)
    if 'data: true' not in result.stdout:
        raise RuntimeError(f'实体移动未确认：{result.stdout} {result.stderr}')


def occlude(duration, mount_mode):
    try:
        set_pose('occlusion_panel', *panel_position(mount_mode))
        print(f'遮挡板已进入视线，保持 {duration:g} 秒', flush=True)
        time.sleep(duration)
    finally:
        set_pose('occlusion_panel', 40, 0, 40)
        print('遮挡板已移走', flush=True)


def drive(speed, turn, duration):
    from gz.transport13 import Node
    from gz.msgs10.twist_pb2 import Twist
    node = Node()
    publisher = node.advertise('/model/tracking_target/cmd_vel', Twist)
    message = Twist()
    message.linear.x = speed
    message.angular.z = turn
    try:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            publisher.publish(message)
            time.sleep(0.05)
    finally:
        message.linear.x = 0
        message.angular.z = 0
        for _ in range(5):
            publisher.publish(message)
            time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['occlude', 'clear', 'target', 'drive', 'stop'])
    parser.add_argument('--duration', type=float, default=2.0, help='遮挡保持的墙钟秒数')
    parser.add_argument('--x', type=float, default=None)
    parser.add_argument('--mount-mode', choices=['skyward', 'downward', 'sideward'], default='skyward')
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--speed', type=float, default=0.5, help='drive 车体前进速度，m/s')
    parser.add_argument('--turn', type=float, default=0.12, help='drive 转弯角速度，rad/s')
    args = parser.parse_args()
    if not 0 < args.duration <= 60:
        parser.error('--duration 必须在 0 到 60 秒之间')
    os.environ.setdefault('GZ_PARTITION', 'geographic_camera')
    if abs(args.speed) > 1 or abs(args.turn) > 0.5:
        parser.error('速度绝对值不超过 1 m/s，转弯角速度不超过 0.5 rad/s')
    if args.action == 'drive':
        drive(args.speed, args.turn, args.duration)
    elif args.action == 'stop':
        drive(0, 0, 0.3)
    elif args.action == 'occlude':
        occlude(args.duration, args.mount_mode)
    elif args.action == 'clear':
        set_pose('occlusion_panel', 40, 0, 40)
    else:
        set_pose('tracking_target', target_x(args.mount_mode) if args.x is None else args.x, args.y, 30.6)


if __name__ == '__main__':
    main()
