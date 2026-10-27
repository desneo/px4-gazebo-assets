#!/usr/bin/env python3
"""实窗自测输入/发送/停止/重启及跟踪脱靶量；会在配置的隔离场景框选目标。"""
import argparse
import json
import math
from pathlib import Path
import socket
import sys
import threading
import time

import cv2

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import ground_station as common
from telemetry_forward import aisheng_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cfg = common.load_config(args.config)
    assert cfg["forward_ip"] == "127.0.0.1", "实窗验收仅使用本机回环接收端"
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", cfg["forward_port"]))
    receiver.settimeout(.05)
    received = []
    stopped = threading.Event()

    def receive():
        while not stopped.is_set():
            try:
                raw = receiver.recvfrom(8192)[0]
                received.append((time.monotonic(), raw, json.loads(raw)))
            except socket.timeout:
                pass

    thread = threading.Thread(target=receive, daemon=True)
    thread.start()

    class AcceptanceStation(common.GroundStation):
        phase = 0
        track_requested = False
        frozen_count = None
        def click(self, name):
            x, y, width, height = self._forward_geometry()[name]
            for event in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_LBUTTONUP):
                self.on_mouse(event, x + width // 2, y + height // 2, 0, None)

        def draw(self, frame, tele):
            result = super().draw(frame, tele)
            elapsed = time.monotonic() - self._t_start
            if self.tele.public_angles_ready and self.phase == 0:
                self.click("ip")
                assert self._handle_forward_key(ord("h")), "编辑地址必须消费回零快捷键"
                for char in cfg["forward_ip"]:
                    self._handle_forward_key(ord(char))
                self.click("port")
                for char in str(cfg["forward_port"]):
                    self._handle_forward_key(ord(char))
                self._handle_forward_key(13)
                self.click("toggle")
                assert self.forwarder.snapshot()["active"]
                self.tele.send_reliable("set_vision_mode", mode="manual_track")
                self.phase = 1
            if self.phase == 1 and self._vision_mode(tele) == "manual_track" and not self.track_requested:
                self.tele.send_reliable("track", box=[.45, .47, .10, .06])
                self.track_requested = True
            if self.phase == 1 and elapsed > 3.5:
                cv2.imwrite(str(args.output / "forward_active.png"), result)
                self.click("toggle")
                assert not self.forwarder.snapshot()["active"]
                self.phase = 2
            elif self.phase == 2 and elapsed > 4:
                self.frozen_count = len(received)
                self.phase = 3
            elif self.phase == 3 and elapsed > 5:
                assert self.frozen_count == len(received), "停止后不应继续转发旧状态"
                cv2.imwrite(str(args.output / "forward_stopped.png"), result)
                self.click("toggle")
                self.phase = 4
            elif self.phase == 4 and elapsed > 6:
                self.tele.send_reliable("set_vision_mode", mode="off")
                self.phase = 5
            return result

    gui_args = common.build_parser().parse_args([
        "--config", args.config, "--duration", "8", "--gui-snapshot", str(args.output / "final.png")])
    station = AcceptanceStation(cfg, gui_args)
    original = station.tele.on_telemetry
    capture_start_ms = time.time_ns() // 1_000_000
    sources = []
    expected = set()

    def remember(raw):
        message = json.loads(raw)
        sources.append(message)
        expected.add(json.dumps(aisheng_message(message), separators=(",", ":"), allow_nan=False).encode())
        original(raw)

    station.tele.on_telemetry = remember
    try:
        assert station.run() == 0
        assert station.phase == 5, "输入、发送、停止、重启与退出跟踪必须全部执行"
        assert len(received) > 80, "没有收到足够的艾晟报文"
        assert all(json.dumps({k: v for k, v in msg.items() if k != "timestamp"},
                              separators=(",", ":"), allow_nan=False).encode() in expected
                   for _, _, msg in received), "艾晟测量字段必须与原始遥测转换逐项匹配"
        packets = [msg for _, _, msg in received]
        assert all(len(msg) == 19 and type(msg.get("timestamp")) is int
                   and capture_start_ms <= msg["timestamp"] <= time.time_ns() // 1_000_000 and msg.get('target_confidence') == 0.8
                   and type(msg['target_confidence']) is float
                   and type(msg['tracking_gate_w']) is float
                   and type(msg['tracking_gate_h']) is float
                   and type(msg['gate_x']) is int and type(msg['gate_y']) is int for msg in packets)
        assert any(msg['track_status'] == 1 and msg['pitch_angle'] < 0 for msg in packets)
        assert any(msg['track_status'] == 0 and msg['tracking_confidence'] == 0
                   and msg['pitch_misalignment'] == msg['yaw_misalignment'] == 0 for msg in packets)
        samples = [msg for msg in sources if msg.get("target", {}).get("miss", {}).get("valid")]
        assert len(samples) > 10, "没有有效脱靶量样本"
        for message in samples:
            target = message["target"]
            miss = target["miss"]
            bx, by, bw, bh = target["box_px"]
            forwarded = aisheng_message(message)
            if forwarded["track_status"]:
                assert forwarded["tracking_gate_w"] == bw
                assert forwarded["tracking_gate_h"] == bh
                assert forwarded["gate_x"] == math.floor(bx + bw / 2 + .5)
                assert forwarded["gate_y"] == math.floor(by + bh / 2 + .5)
            else:
                assert forwarded["tracking_gate_w"] == forwarded["tracking_gate_h"] == 0.0
                assert forwarded["gate_x"] == forwarded["gate_y"] == 0
            dx, dy = miss["offset_px"]
            assert abs(dx - (bx + bw / 2 - 960)) <= .11
            assert abs(dy - (by + bh / 2 - 540)) <= .11
            focal = 960 / math.tan(math.pi / 6)
            assert abs(miss["offset_deg"][0] - math.degrees(math.atan(dx / focal))) < .0003
            assert abs(miss["offset_deg"][1] - math.degrees(math.atan(dy / focal))) < .0003
            assert isinstance(miss["frame_seq"], int)
        station.tele.send_reliable("set_vision_mode", mode="off")
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            miss = station.tele.snapshot().get("target", {}).get("miss", {})
            if miss.get("valid") is False:
                break
            time.sleep(.05)
        assert miss == dict(valid=False, center_px=None, offset_px=None, offset_deg=None, frame_seq=None)
        report = dict(result="passed", udp_packets=len(received), exact_conversion=True,
                      forward_pitch_negative=True, example_aisheng=next(msg for msg in packets if msg['track_status']),
                      stop_and_restart=True, valid_miss_samples=len(samples),
                      example_miss=samples[-1]["target"]["miss"], invalid_after_stop=miss,
                      endpoint=f"127.0.0.1:{cfg['forward_port']}")
        (args.output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        station.shutdown()
        stopped.set()
        thread.join(timeout=1)
        receiver.close()


if __name__ == "__main__":
    main()
