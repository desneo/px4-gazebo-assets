#!/usr/bin/env python3
"""预览模型的手动指令入口。"""
import argparse
import json
import time

from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node


def main():
    parser = argparse.ArgumentParser(description='ASD-2-075-DD0 预览指令')
    sub = parser.add_subparsers(dest='cmd', required=True)
    sub.add_parser('home', help='两轴回机械零位')
    sub.add_parser('stop', help='立即停止')
    manual = sub.add_parser('manual', help='发送两轴速度指令')
    manual.add_argument('yaw', type=float, help='yaw 速度 °/s，右正')
    manual.add_argument('pitch', type=float, help='pitch 速度 °/s，上正')
    manual.add_argument('--duration', type=float, default=1.0, help='持续时间 s')
    manual.add_argument('--rate', type=float, default=20.0, help='发送频率 Hz')
    args = parser.parse_args()

    node = Node()
    pub = node.advertise('/aisheng_seeker/command', StringMsg)

    def send(payload):
        pub.publish(StringMsg(data=json.dumps(payload, separators=(',', ':'))))

    if args.cmd == 'manual':
        end = time.monotonic() + max(0.0, args.duration)
        print(f'发送 manual yaw={args.yaw} pitch={args.pitch} °/s，持续 {args.duration}s')
        while time.monotonic() < end:
            send({'cmd': 'manual', 'yaw': args.yaw, 'pitch': args.pitch})
            time.sleep(1.0 / max(1.0, args.rate))
        for _ in range(3):
            send({'cmd': 'manual', 'yaw': 0.0, 'pitch': 0.0})
            time.sleep(0.05)
        print('已发送停车。')
    else:
        for _ in range(3):
            send({'cmd': args.cmd})
            time.sleep(0.05)
        print(f'已发送 {args.cmd}。')


if __name__ == '__main__':
    main()
