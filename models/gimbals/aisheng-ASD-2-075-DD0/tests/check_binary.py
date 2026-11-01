#!/usr/bin/env python3
"""二进制 UDP 网关回环检查。

前置：预览世界和 gateway/main.py 已启动，网关遥测发往 127.0.0.1:8791。
"""
from __future__ import annotations

import argparse
from pathlib import Path
import socket
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common import protocol as proto  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=8790)
    parser.add_argument("--telemetry-port", type=int, default=8791)
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", args.telemetry_port))
    rx.settimeout(0.5)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.control_port)

    def send(command: int, **kwargs) -> None:
        tx.sendto(proto.make_control(command, **kwargs), dest)

    print("解锁……")
    send(proto.CMD_FRAME_UNLOCK)
    time.sleep(0.5)
    print("指定角度 yaw=10°, pitch=5°……")
    send(proto.CMD_SPECIFIED_ANGLE, p1=1000, p2=500)
    time.sleep(2.0)
    print("归中……")
    send(proto.CMD_RECENTER)
    time.sleep(1.0)
    print("手动 yaw +30°/s，持续 1 s……")
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        send(proto.CMD_FRAME_MOVE, p1=50, p2=0)
        time.sleep(0.05)
    time.sleep(0.6)

    # 清空命令阶段积压的遥测，避免把旧包计入速率。
    rx.settimeout(0.0)
    while True:
        try:
            rx.recvfrom(2048)
        except BlockingIOError:
            break
        except OSError:
            break
    rx.settimeout(0.5)

    start = time.monotonic()
    count = 0
    checksum_errors = 0
    last = None
    while time.monotonic() - start < args.duration:
        try:
            data, _ = rx.recvfrom(2048)
        except socket.timeout:
            continue
        try:
            frame = proto.decode_telemetry(data)
        except ValueError:
            checksum_errors += 1
            continue
        count += 1
        last = frame
    elapsed = time.monotonic() - start
    print(f"遥测 {count} 帧 / {elapsed:.2f}s = {count/elapsed:.1f} Hz，"
          f"校验错误 {checksum_errors}")
    if last:
        servo = (last.status_control >> 5) & 0x7
        print(f"最后遥测：yaw={last.yaw_deg:.2f}° pitch={last.pitch_deg:.2f}° "
              f"servo={servo:03b} tracking={last.work_state & 1} "
              f"gate=({last.gate_x},{last.gate_y},{last.gate_w},{last.gate_h}) "
              f"timestamp={last.timestamp_ms}")
    rx.close()
    tx.close()


if __name__ == "__main__":
    main()
