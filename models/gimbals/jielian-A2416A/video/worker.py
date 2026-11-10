#!/usr/bin/env python3
"""订阅A2416A Gazebo图像/命令/遥测并驱动H.264 NVENC。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from video.hardware import HardwareVideoError
from video.sender import DEFAULT_BITRATE_KBPS, HEIGHT, VideoSender, WIDTH


def _gate(payload):
    value = payload.get("gate_px")
    if isinstance(value, dict):
        return (value.get("x", 0), value.get("y", 0), value.get("w", 0), value.get("h", 0))
    if isinstance(value, (list, tuple)) and len(value) == 4:
        return tuple(value)
    return None


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic-prefix", default="/jielian_a2416a")
    parser.add_argument("--image-topic", default="/jielian_a2416a/image")
    parser.add_argument("--rtp-host", default="127.0.0.1")
    parser.add_argument("--rtp-port", type=int, default=7001)
    args = parser.parse_args(argv)
    try:
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.transport13 import Node
    except ImportError as exc:
        print(f"缺少Gazebo Python传输依赖：{exc}", file=sys.stderr)
        return 2
    try:
        sender = VideoSender(args.rtp_host, args.rtp_port, DEFAULT_BITRATE_KBPS)
    except HardwareVideoError as exc:
        print(f"A2416A H.264视频启动失败：{exc}", file=sys.stderr)
        return 2
    stopping = threading.Event()
    node = Node()

    def image_callback(message: Image):
        try:
            sender.offer_rgb(message.data, width=message.width, height=message.height)
        except ValueError as exc:
            print(f"忽略无效相机帧：{exc}", file=sys.stderr, flush=True)

    def command_callback(message: StringMsg):
        try:
            payload = json.loads(message.data)
            command = payload.get("cmd")
            if command == "video_on":
                sender.set_processing(enabled=True)
            elif command == "video_off":
                sender.set_processing(enabled=False)
            elif command == "image_enhance_on":
                sender.set_processing(enhance=True)
            elif command == "image_enhance_off":
                sender.set_processing(enhance=False)
            elif command in ("osd_full", "osd_half", "osd_off"):
                sender.set_processing(osd_mode=command.removeprefix("osd_"))
            elif command == "stream_set":
                raw = int(payload.get("p1", 0))
                sender.set_bitrate(2000 if raw == 0 else max(1, min(8000, raw * 10)))
        except (HardwareVideoError, TypeError, ValueError, json.JSONDecodeError) as exc:
            print(f"视频命令失败：{exc}", file=sys.stderr, flush=True)

    def telemetry_callback(message: StringMsg):
        try:
            payload = json.loads(message.data)
            gate = _gate(payload)
            detections = []
            for target in payload.get("targets", [])[:4]:
                if isinstance(target, dict):
                    box = target.get("box_px") or target.get("bbox_px")
                    if isinstance(box, (list, tuple)) and len(box) == 4:
                        detections.append(tuple(box))
            sender.set_overlay(
                crosshair=payload.get("crosshair_px", (WIDTH // 2, HEIGHT // 2)),
                tracking_box=gate,
                detection_boxes=detections,
                status_text=str(payload.get("tracking_state", "")),
            )
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    node.subscribe(Image, args.image_topic, image_callback)
    node.subscribe(StringMsg, f"{args.topic_prefix}/command", command_callback)
    node.subscribe(StringMsg, f"{args.topic_prefix}/telemetry", telemetry_callback)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_args: stopping.set())
    print(f"A2416A H.264视频：{WIDTH}x{HEIGHT}@25Hz 2Mbps -> "
          f"rtp://{args.rtp_host}:{args.rtp_port}", flush=True)
    try:
        while not stopping.wait(0.2):
            pass
    finally:
        sender.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
