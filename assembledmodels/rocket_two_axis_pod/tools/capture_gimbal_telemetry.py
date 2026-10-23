#!/usr/bin/env python3
"""只读保存 Gazebo 吊舱遥测，供完整飞行的实际 camera.quat_world 验收。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import threading
import time

from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stopping = threading.Event()
    lock = threading.Lock()
    count = 0
    stream = args.output.open('w', buffering=1)

    def receive(message):
        nonlocal count
        try:
            row = json.loads(message.data)
            if row.get('type') != 'telemetry' or not isinstance(row.get('camera', {}).get('quat_world'), list):
                return
            encoded = json.dumps(row, ensure_ascii=False, separators=(',', ':'))
        except (AttributeError, TypeError, json.JSONDecodeError):
            return
        with lock:
            stream.write(encoded + '\n')
            count += 1

    def stop(*_):
        stopping.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = Node()
    # gz.transport Python绑定成功时返回None，订阅失败则直接抛出异常。
    node.subscribe(StringMsg, args.topic, receive)
    while not stopping.wait(.1):
        pass
    node.unsubscribe(args.topic)
    with lock:
        stream.close()
    print(f'保存 {count} 条实际吊舱姿态遥测：{args.output}')


if __name__ == '__main__':
    main()
