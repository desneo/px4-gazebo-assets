#!/usr/bin/env python3
"""点选锁定/自动锁定/取消锁定回环检查。"""
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
    tx.bind(("0.0.0.0", 0))
    tx.settimeout(0.2)
    rx = tx
    dest = (args.host, args.control_port)
    latest = {"binary": None, "plugin": None}
    lock = threading.Lock()
    running = True

    def udp_reader():
        while running:
            try:
                data, _ = rx.recvfrom(2048)
            except socket.timeout:
                continue
            try:
                frame = proto.decode_telemetry(data)
            except ValueError:
                continue
            with lock:
                latest["binary"] = frame

    node = Node()

    def on_plugin(msg):
        try:
            with lock:
                latest["plugin"] = json.loads(msg.data)
        except ValueError:
            pass

    node.subscribe(StringMsg, "/aisheng_seeker/telemetry", on_plugin)
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
            with lock:
                b = latest["binary"]
                p = latest["plugin"]
            if b is not None and p is not None and predicate(b, p):
                return b, p
            time.sleep(0.05)
        return None, None

    send(proto.CMD_FRAME_UNLOCK)
    b, p = wait_for(lambda b, p: ((b.status_control >> 5) & 0x7) == 4, 3)
    print("unlock:", "ok" if b else "timeout")

    with lock:
        candidates = list((latest["plugin"] or {}).get("truth_candidates", []))
    print("candidates:", candidates)
    if not candidates:
        raise SystemExit("没有真值候选，无法测试锁定")
    target = candidates[0]
    cx, cy = target["center_px"]
    print(f"点选 {target['name']} center=({cx:.1f},{cy:.1f})")
    send(proto.CMD_POINT_LOCK, p1=int(round(cx)), p2=int(round(cy)))
    b, p = wait_for(lambda b, p: bool(b.work_state & 1), 5)
    if b:
        print(f"点选锁定 ok: name={p.get('target',{}).get('entity_name')} "
              f"gate=({b.gate_x},{b.gate_y},{b.gate_w},{b.gate_h}) "
              f"yaw={b.yaw_deg:.2f} pitch={b.pitch_deg:.2f}")
    else:
        print("点选锁定 timeout")

    send(proto.CMD_CANCEL_LOCK)
    b, p = wait_for(lambda b, p: not (b.work_state & 1), 3)
    print("取消锁定:", "ok" if b else "timeout")

    print("自动锁定……")
    send(proto.CMD_AUTO_LOCK_ON)
    b, p = wait_for(lambda b, p: bool(b.work_state & 1), 5)
    if b:
        print(f"自动锁定 ok: name={p.get('target',{}).get('entity_name')} "
              f"gate=({b.gate_x},{b.gate_y},{b.gate_w},{b.gate_h}) "
              f"auto={b.autonomous} mode2={(b.status_control2>>2)&3}")
    else:
        print("自动锁定 timeout")

    send(proto.CMD_AUTO_LOCK_OFF)
    send(proto.CMD_FRAME_LOCK)
    b, p = wait_for(lambda b, p: ((b.status_control >> 5) & 0x7) == 1, 3)
    print("电锁:", "ok" if b else "timeout")
    running = False
    tx.close()


if __name__ == "__main__":
    main()
