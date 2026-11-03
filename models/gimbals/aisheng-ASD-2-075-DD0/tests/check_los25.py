#!/usr/bin/env python3
"""25°/s 视线角速度跟踪验收。

需要以 tests/los25.world.sdf 启动 Gazebo；目标运动插件在物理步内平滑移动
target_aircraft（y 幅值 0.55 m、周期 2 s，约 26.7°/s 峰值）。检查插件内部
target_los 保留实际值，标准遥测 LOS 字段按协议限幅 ±20°/s，跟踪保持有效。
"""
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
    tx.bind(("0.0.0.0", 0))
    tx.settimeout(0.2)
    rx = tx
    latest = {}
    standard_los = []
    running = True
    node = Node()

    def on_telemetry(msg):
        try:
            latest.update(json.loads(msg.data))
        except ValueError:
            pass

    node.subscribe(StringMsg, "/aisheng_seeker/telemetry", on_telemetry)

    def udp_reader():
        while running:
            try:
                data, _ = rx.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                frame = proto.decode_telemetry(data)
            except ValueError:
                continue
            standard_los.append((frame.yaw_los_dps, frame.pitch_los_dps))

    threading.Thread(target=udp_reader, daemon=True).start()

    def send(command: int, **kwargs):
        tx.sendto(proto.make_control(command, **kwargs), dest)

    def heartbeat():
        while running:
            send(0x00)
            time.sleep(0.1)

    threading.Thread(target=heartbeat, daemon=True).start()

    def wait_for(predicate, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if latest and predicate(latest):
                return True
            time.sleep(0.05)
        return False

    send(proto.CMD_FRAME_UNLOCK)
    send(proto.CMD_DETECTOR_SWITCH, p1=1)
    wait_for(lambda d: d.get("camera", {}).get("active") == "wide")
    send(proto.CMD_RECENTER)
    time.sleep(1.5)
    candidates = [c for c in latest.get("truth_candidates", [])
                  if c["name"] == "target_aircraft"]
    if not candidates:
        raise SystemExit("没有 target_aircraft 候选")
    c = candidates[0]
    send(proto.CMD_POINT_LOCK, p1=int(c["center_px"][0]), p2=int(c["center_px"][1]))
    if not wait_for(lambda d: d.get("tracking", {}).get("state") == "TRACK"):
        raise SystemExit("点选锁定失败")
    time.sleep(1.0)

    samples = []
    start = time.monotonic()
    while time.monotonic() - start < 11:
        d = latest
        tr = d.get("tracking", {})
        target = d.get("target", {})
        miss = target.get("miss", {})
        los = d.get("target_los", {})
        samples.append({
            "state": tr.get("state"),
            "miss": miss.get("offset_deg"),
            "yaw_los": los.get("yaw_rate_dps"),
            "pitch_los": los.get("pitch_rate_dps"),
            "valid": los.get("valid"),
        })
        time.sleep(0.1)
    send(proto.CMD_CANCEL_LOCK)
    time.sleep(0.2)
    send(proto.CMD_FRAME_LOCK)
    running = False

    steady = samples[10:]
    track = [s for s in steady if s["state"] == "TRACK"]
    internal = [abs(s["yaw_los"]) for s in steady
                if s["valid"] and s["yaw_los"] is not None]
    misses = [max(abs(v) for v in s["miss"]) for s in steady if s["miss"]]
    std_max = max((max(abs(a), abs(b)) for a, b in standard_los), default=0.0)
    print(f"稳态样本 {len(steady)}，TRACK {len(track)}")
    print(f"内部 |yaw_los| 峰值 {max(internal, default=0):.2f} °/s，"
          f"标准帧 |LOS| 峰值 {std_max:.2f} °/s")
    print(f"最大失调角 {max(misses, default=0):.2f}°")
    ok = (len(steady) > 0 and len(track) / len(steady) >= 0.8
          and max(internal, default=0) >= 20.0
          and std_max <= 20.5
          and max(misses, default=0) <= 10.0)
    print("25°/s 视线角速度验收通过" if ok else "25°/s 视线角速度验收失败")
    tx.close()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
