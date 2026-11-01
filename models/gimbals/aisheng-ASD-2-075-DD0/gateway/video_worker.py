#!/usr/bin/env python3
"""单路相机 RTP 编码工作进程。

网关主进程只处理控制/遥测；每路相机使用独立进程订阅 Gazebo 图像并
NVENC 编码，避免两路 1080p 图像同时占用 Python GIL 导致帧率下降。
"""
from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gz.transport13 import Node  # noqa: E402
from gateway.video_tx import VideoSender  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--bitrate", type=int, default=8000)
    args = parser.parse_args()
    stopping = threading.Event()

    def stop(*_args):
        stopping.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    sender = VideoSender(
        Node(), args.topic, args.host, args.port,
        width=args.width, height=args.height, fps=args.fps,
        bitrate_kbps=args.bitrate)
    try:
        while not stopping.wait(0.2):
            pass
    finally:
        sender.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
