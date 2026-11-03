#!/usr/bin/env python3
"""两轴机械限位与手动速度回环检查。"""
from __future__ import annotations

import json
from pathlib import Path
import socket
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common import protocol as proto  # noqa: E402

from gz.msgs10.stringmsg_pb2 import StringMsg  # noqa: E402
from gz.transport13 import Node  # noqa: E402


def main() -> None:
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", 8001)
    latest = {}
    running = True
    node = Node()

    def on_telemetry(msg):
        try:
            latest.update(json.loads(msg.data))
        except ValueError:
            pass

    node.subscribe(StringMsg, "/aisheng_seeker/telemetry", on_telemetry)

    def send(command: int, **kwargs):
        tx.sendto(proto.make_control(command, **kwargs), dest)

    def heartbeat():
        while running:
            send(0x00)
            time.sleep(0.1)

    threading.Thread(target=heartbeat, daemon=True).start()

    def joint():
        data = latest.get("joints_deg", {})
        try:
            return float(data["yaw"]["pos"]), float(data["pitch"]["pos"])
        except (KeyError, TypeError, ValueError):
            return None

    def drive(yaw: int, pitch: int, seconds: float):
        end = time.monotonic() + seconds
        peak_yaw = 0.0
        peak_pitch = 0.0
        while time.monotonic() < end:
            send(proto.CMD_FRAME_MOVE, p1=yaw, p2=pitch)
            data = latest.get("joints_deg", {})
            try:
                peak_yaw = max(peak_yaw, abs(float(data["yaw"]["vel"])))
                peak_pitch = max(peak_pitch, abs(float(data["pitch"]["vel"])))
            except (KeyError, TypeError, ValueError):
                pass
            time.sleep(0.05)
        for _ in range(3):
            send(proto.CMD_FRAME_MOVE, p1=0, p2=0)
            time.sleep(0.05)
        time.sleep(0.4)
        return peak_yaw, peak_pitch

    send(proto.CMD_FRAME_UNLOCK)
    send(proto.CMD_RECENTER)
    time.sleep(1.0)
    results = {}
    peaks = {}
    peaks["yaw+"] = drive(100, 0, 3.0)
    results["yaw+"] = joint()
    peaks["yaw-"] = drive(-100, 0, 3.0)
    results["yaw-"] = joint()
    peaks["pitch+"] = drive(0, 100, 3.0)
    results["pitch+"] = joint()
    peaks["pitch-"] = drive(0, -100, 3.0)
    results["pitch-"] = joint()
    print("angles", results)
    print("peaks", peaks)
    ok = True
    for value in results.values():
        if value is None:
            ok = False
        elif not (-60.5 <= value[0] <= 60.5 and -80.5 <= value[1] <= 30.5):
            ok = False
    for yaw_peak, pitch_peak in peaks.values():
        if yaw_peak > 60.5 or pitch_peak > 60.5:
            ok = False
    if max(p[0] for p in peaks.values()) < 20.0 and max(p[1] for p in peaks.values()) < 20.0:
        ok = False
    print("框架限位/框架转速检查通过" if ok else "框架限位/框架转速检查失败")
    send(proto.CMD_RECENTER)
    time.sleep(0.5)
    send(proto.CMD_FRAME_LOCK)
    running = False
    tx.close()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
