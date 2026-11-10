#!/usr/bin/env python3
"""把 A2416A 内部50 Hz JSON遥测保存为JSONL。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import threading
import time

from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/rocket_jielian_a2416a/telemetry")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    stop = threading.Event()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stream = args.output.open("w", encoding="utf-8", buffering=1)
    count = 0

    def receive(message: StringMsg) -> None:
        nonlocal count
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        payload["capture_monotonic_s"] = time.monotonic()
        stream.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n")
        count += 1

    node = Node()
    if not node.subscribe(StringMsg, args.topic, receive):
        raise SystemExit(f"无法订阅遥测话题：{args.topic}")
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_args: stop.set())
    try:
        while not stop.wait(0.2):
            pass
    finally:
        stream.close()
    print(f"已保存 {count} 帧遥测：{args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
