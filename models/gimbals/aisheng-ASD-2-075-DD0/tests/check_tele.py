#!/usr/bin/env python3
"""长焦原生点选锁定回环检查。"""
from __future__ import annotations

import argparse
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=8001)
    args = parser.parse_args()

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.control_port)
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

    def wait_for(predicate, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if latest and predicate(latest):
                return True
            time.sleep(0.05)
        return False

    def visible_candidates():
        return [c for c in latest.get("truth_candidates", [])
                if 0 <= c["center_px"][0] < 1920 and 0 <= c["center_px"][1] < 1080]

    send(proto.CMD_FRAME_UNLOCK)
    send(proto.CMD_DETECTOR_SWITCH, p1=1)
    wait_for(lambda d: d.get("camera", {}).get("active") == "wide")
    send(proto.CMD_RECENTER)
    time.sleep(1.5)
    candidates = [c for c in latest.get("truth_candidates", []) if c["name"] == "target_aircraft"]
    if not candidates:
        raise SystemExit("广角没有 target_aircraft 候选")
    c = candidates[0]
    send(proto.CMD_POINT_LOCK, p1=int(c["center_px"][0]), p2=int(c["center_px"][1]))
    if not wait_for(lambda d: d.get("tracking", {}).get("state") == "TRACK"):
        raise SystemExit("广角点选锁定失败")
    print("广角锁定 ok")
    time.sleep(2.0)

    send(proto.CMD_DETECTOR_SWITCH, p1=2)
    if not wait_for(lambda d: d.get("camera", {}).get("active") == "tele"):
        raise SystemExit("长焦切换失败")
    time.sleep(0.5)
    candidates = visible_candidates()
    if not candidates:
        raise SystemExit("长焦视场内没有候选")
    c = candidates[0]
    print(f"长焦候选 {c['name']} center={c['center_px']}")
    send(proto.CMD_POINT_LOCK, p1=int(c["center_px"][0]), p2=int(c["center_px"][1]))
    if not wait_for(lambda d: d.get("tracking", {}).get("state") == "TRACK"):
        raise SystemExit("长焦点选锁定失败")
    time.sleep(1.0)
    d = latest
    print(f"长焦锁定 ok: active={d.get('camera',{}).get('active')} "
          f"joints={d.get('joints_deg')} box={d.get('target',{}).get('box_px')} "
          f"miss={d.get('target',{}).get('miss',{}).get('offset_deg')}")
    send(proto.CMD_CANCEL_LOCK)
    time.sleep(0.3)
    send(proto.CMD_DETECTOR_SWITCH, p1=1)
    time.sleep(0.5)
    send(proto.CMD_FRAME_LOCK)
    running = False
    tx.close()


if __name__ == "__main__":
    main()
