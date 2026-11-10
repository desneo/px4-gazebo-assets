#!/usr/bin/env python3
"""连接Gazebo JSON话题与A2416A标准UDP报文。"""
from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
import signal
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gateway.service import GatewayCore, UdpGateway  # noqa: E402


def _gate(payload: dict):
    value = payload.get("gate_px")
    if isinstance(value, dict):
        return (value.get("x", 0), value.get("y", 0),
                value.get("w", 0), value.get("h", 0))
    if isinstance(value, (list, tuple)) and len(value) == 4:
        return tuple(value)
    return None


def _detection_boxes(payload: dict) -> tuple[tuple, ...]:
    boxes = []
    targets = payload.get("targets", [])
    if not isinstance(targets, (list, tuple)):
        return ()
    for target in targets[:4]:
        if not isinstance(target, dict):
            continue
        box = target.get("box_px") or target.get("bbox_px")
        if isinstance(box, (list, tuple)) and len(box) == 4:
            boxes.append(tuple(box))
    return tuple(boxes)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A2416A 二进制UDP网关")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8001)
    parser.add_argument("--topic-prefix", default="/jielian_a2416a")
    parser.add_argument("--image-topic", default="",
                        help="Gazebo RGB图像话题；默认 <topic-prefix>/image")
    parser.add_argument("--rtp-host", default="127.0.0.1")
    parser.add_argument("--rtp-port", type=int, default=7001)
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(asctime)s %(levelname)s %(message)s")
    try:
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.transport13 import Node
    except ImportError as exc:
        print(f"缺少Gazebo Python传输依赖：{exc}", file=sys.stderr)
        return 2
    try:
        from video.hardware import HardwareVideoError
        from video.sender import DEFAULT_BITRATE_KBPS, HEIGHT, VideoSender, WIDTH
        sender = VideoSender(args.rtp_host, args.rtp_port, DEFAULT_BITRATE_KBPS)
    except (ImportError, HardwareVideoError) as exc:
        print(f"A2416A H.264硬件视频启动失败：{exc}", file=sys.stderr)
        return 2

    node = Node()
    command_pub = node.advertise(f"{args.topic_prefix}/command", StringMsg)

    def publish(payload: dict) -> None:
        command_pub.publish(StringMsg(
            data=json.dumps(payload, separators=(",", ":"), ensure_ascii=False)))
        if args.verbose:
            print(f"控制：{payload}", flush=True)

    def sync_video_state(state) -> None:
        try:
            sender.set_processing(
                enabled=state.video_enabled,
                enhance=state.enhancement_enabled,
                osd_mode=state.osd_mode,
            )
            bitrate_kbps = round(state.bitrate_mbps * 1000.0)
            if bitrate_kbps != sender.bitrate_kbps:
                sender.set_bitrate(bitrate_kbps)
        except (HardwareVideoError, TypeError, ValueError) as exc:
            logging.error("视频命令执行失败：%s", exc)

    core = GatewayCore(command_sink=publish, state_sink=sync_video_state)
    try:
        gateway = UdpGateway(core, args.host, args.port)
    except OSError as exc:
        sender.close()
        print(f"A2416A UDP网关启动失败：{exc}", file=sys.stderr)
        return 2

    def image_callback(msg: Image) -> None:
        try:
            sender.offer_rgb(msg.data, width=msg.width, height=msg.height)
        except (TypeError, ValueError) as exc:
            logging.warning("忽略无效相机帧：%s", exc)

    def telemetry_callback(msg: StringMsg) -> None:
        try:
            data = json.loads(msg.data)
            core.update_measurement(data)
            sender.set_overlay(
                crosshair=data.get("crosshair_px", (WIDTH // 2, HEIGHT // 2)),
                tracking_box=_gate(data),
                detection_boxes=_detection_boxes(data),
                status_text=str(data.get("tracking_state", "")),
            )
        except (TypeError, ValueError) as exc:
            if args.verbose:
                print(f"忽略无效内部遥测：{exc}", file=sys.stderr, flush=True)

    image_topic = args.image_topic or f"{args.topic_prefix}/image"
    node.subscribe(Image, image_topic, image_callback)
    node.subscribe(StringMsg, f"{args.topic_prefix}/telemetry", telemetry_callback)
    monitor_stop = threading.Event()

    def report_video_stats() -> None:
        previous_frames = previous_dropped = 0
        while not monitor_stop.wait(5.0):
            logging.info(
                "H.264视频统计：编码=%d（%.1f fps），丢弃=%d（新增%d）",
                sender.frames, (sender.frames - previous_frames) / 5.0,
                sender.dropped, sender.dropped - previous_dropped)
            previous_frames, previous_dropped = sender.frames, sender.dropped

    monitor_thread = threading.Thread(
        target=report_video_stats, name="a2416a-video-stats", daemon=True)
    monitor_thread.start()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_args: setattr(gateway, "running", False))
    print(f"A2416A网关：UDP {gateway.address[0]}:{gateway.address[1]}，遥测100 Hz",
          flush=True)
    print(f"A2416A H.264视频：{WIDTH}x{HEIGHT}@25Hz -> "
          f"rtp://{args.rtp_host}:{args.rtp_port}", flush=True)
    try:
        gateway.run()
    finally:
        monitor_stop.set()
        monitor_thread.join(timeout=1.0)
        gateway.close()
        sender.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
