#!/usr/bin/env python3
"""ASD-2-075-DD0 上位机：RTP 双路视频 + 二进制遥测/控制。"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import socket
import sys
import threading
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common import protocol as proto  # noqa: E402

import gi  # noqa: E402

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

Gst.init(None)

WINDOW = "ASD-2-075-DD0 GCS"
WIDTH = 1920
HEIGHT = 1080
DISPLAY_W = 1280
DISPLAY_H = 720
F_WIDE = WIDTH / (2.0 * math.tan(math.radians(38.0 / 2.0)))
F_TELE = WIDTH / (2.0 * math.tan(math.radians(12.7 / 2.0)))


def wide_to_tele(u: float, v: float) -> tuple[float, float]:
    tx = (u - WIDTH / 2.0) / F_WIDE
    ty = (v - HEIGHT / 2.0) / F_WIDE
    return WIDTH / 2.0 + F_TELE * tx, HEIGHT / 2.0 + F_TELE * ty


def tele_to_wide(u: float, v: float) -> tuple[float, float]:
    tx = (u - WIDTH / 2.0) / F_TELE
    ty = (v - HEIGHT / 2.0) / F_TELE
    return WIDTH / 2.0 + F_WIDE * tx, HEIGHT / 2.0 + F_WIDE * ty


class VideoReceiver:
    def __init__(self, port: int, name: str):
        self.port = port
        self.name = name
        self.latest: np.ndarray | None = None
        self.count = 0
        self.last_mono = 0.0
        self.closing = False
        launch = (
            f"udpsrc port={port} buffer-size=2097152 "
            "caps=application/x-rtp,media=video,clock-rate=90000,"
            "encoding-name=H264,payload=96 "
            "! rtpjitterbuffer latency=0 drop-on-latency=true "
            "! rtph264depay ! h264parse ! nvh264dec "
            "! videoconvert ! video/x-raw,format=BGR "
            "! appsink name=sink max-buffers=1 drop=true sync=false"
        )
        self.pipeline = Gst.parse_launch(launch)
        self.appsink = self.pipeline.get_by_name("sink")
        self.pipeline.set_state(Gst.State.PLAYING)
        threading.Thread(target=self._pull_loop, daemon=True).start()

    def _pull_loop(self):
        while not self.closing:
            sample = self.appsink.emit("try-pull-sample", 100_000_000)
            if sample is None:
                continue
            caps = sample.get_caps()
            structure = caps.get_structure(0)
            width = structure.get_value("width")
            height = structure.get_value("height")
            buf = sample.get_buffer()
            ok, info = buf.map(Gst.MapFlags.READ)
            if not ok:
                continue
            try:
                frame = np.frombuffer(info.data, dtype=np.uint8).reshape(
                    height, width, 3).copy()
            finally:
                buf.unmap(info)
            self.latest = frame
            if self.count == 0:
                print(f"[video:{self.name}] 首帧 {width}x{height}", flush=True)
            self.count += 1
            self.last_mono = time.monotonic()

    def get(self):
        return self.latest

    def close(self):
        self.closing = True
        self.pipeline.set_state(Gst.State.NULL)


class TelemetryClient:
    def __init__(self, host: str, control_port: int, telemetry_port: int):
        self.host = host
        self.control_port = control_port
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("0.0.0.0", telemetry_port))
        self.rx.settimeout(0.2)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.latest: proto.TelemetryFrame | None = None
        self.count = 0
        self.last_mono = 0.0
        self.closing = False
        threading.Thread(target=self._rx_loop, daemon=True).start()
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()

    def _heartbeat_loop(self):
        while not self.closing:
            self.send(0x00)
            time.sleep(0.1)

    def _rx_loop(self):
        while not self.closing:
            try:
                data, _ = self.rx.recvfrom(2048)
            except socket.timeout:
                continue
            try:
                frame = proto.decode_telemetry(data)
            except ValueError:
                continue
            self.latest = frame
            self.count += 1
            self.last_mono = time.monotonic()

    def send(self, command: int, **kwargs):
        try:
            self.tx.sendto(proto.make_control(command, **kwargs),
                           (self.host, self.control_port))
        except OSError as exc:
            print(f"发送失败：{exc}", flush=True)

    def close(self):
        self.closing = True
        self.rx.close()
        self.tx.close()


class ManualController:
    def __init__(self, client: TelemetryClient):
        self.client = client
        self.yaw = 0.0
        self.pitch = 0.0
        self.until = 0.0
        self.active = False
        self.closing = False
        threading.Thread(target=self._loop, daemon=True).start()

    def set_velocity(self, yaw: float, pitch: float, hold: float = 0.25):
        self.yaw = max(-60.0, min(60.0, yaw))
        self.pitch = max(-60.0, min(60.0, pitch))
        self.until = time.monotonic() + hold
        self.active = True

    def _loop(self):
        while not self.closing:
            if time.monotonic() < self.until:
                self.client.send(
                    proto.CMD_FRAME_MOVE,
                    p1=int(round(self.yaw / 60.0 * 100.0)),
                    p2=int(round(self.pitch / 60.0 * 100.0)))
            elif self.active:
                self.client.send(proto.CMD_FRAME_MOVE, p1=0, p2=0)
                self.active = False
            time.sleep(0.05)

    def close(self):
        self.closing = True


class GCS:
    def __init__(self, args):
        self.args = args
        self.wide = VideoReceiver(args.wide_port, "wide")
        self.tele = VideoReceiver(args.tele_port, "tele")
        self.client = TelemetryClient(args.host, args.control_port, args.telemetry_port)
        self.manual = ManualController(self.client)
        self.active = "wide"
        self.trackbar_ready = False
        self.manual_yaw = 0.0
        self.manual_pitch = 0.0
        self.tlm_rate = 0.0
        self.video_rate = 0.0
        self.last_rate_time = time.monotonic()
        self.last_tlm_count = 0
        self.last_video_count = 0
        self.running = True
        self.display_scale = DISPLAY_W / WIDTH
        self.snapshot_saved = False

    def _on_trackbar(self, _value):
        if not self.trackbar_ready:
            return
        yaw = cv2.getTrackbarPos("Yaw", WINDOW) - 60
        pitch = cv2.getTrackbarPos("Pitch", WINDOW) - 80
        self.client.send(proto.CMD_SPECIFIED_ANGLE,
                         p1=int(yaw * 100), p2=int(pitch * 100))

    def _wide_to_tele(self, u: float, v: float) -> tuple[float, float]:
        return wide_to_tele(u, v)

    def _tele_to_wide(self, u: float, v: float) -> tuple[float, float]:
        return tele_to_wide(u, v)

    def _on_mouse(self, event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDBLCLK:
            return
        u = x / self.display_scale
        v = y / self.display_scale
        if not (0 <= u < WIDTH and 0 <= v < HEIGHT):
            return
        if self.active == "tele":
            u, v = self._tele_to_wide(u, v)
        self.client.send(proto.CMD_POINT_LOCK, p1=int(round(u)), p2=int(round(v)))
        print(f"双击锁定：{self.active} -> wide ({u:.0f},{v:.0f})", flush=True)

    def _handle_key(self, key: int):
        if key in (ord("q"), 27):
            self.running = False
            return
        if key == ord("u"):
            self.client.send(proto.CMD_FRAME_UNLOCK)
        elif key == ord("l"):
            self.client.send(proto.CMD_FRAME_LOCK)
        elif key == ord("r"):
            self.client.send(proto.CMD_RECENTER)
        elif key == ord("c"):
            self.client.send(proto.CMD_CANCEL_LOCK)
        elif key == ord("o"):
            self.client.send(proto.CMD_AUTO_LOCK_ON)
        elif key == ord("p"):
            self.client.send(proto.CMD_AUTO_LOCK_OFF)
        elif key == ord("1"):
            self.active = "wide"
            self.client.send(proto.CMD_DETECTOR_SWITCH, p1=1)
        elif key == ord("2"):
            self.active = "tele"
            self.client.send(proto.CMD_DETECTOR_SWITCH, p1=2)
        elif key == ord("z"):
            self.client.send(proto.CMD_ZOOM_IN)
        elif key == ord("Z"):
            self.client.send(proto.CMD_ZOOM_OUT)
        elif key in (ord("w"), 82):
            self.manual_pitch = 60.0
            self.manual.set_velocity(self.manual_yaw, self.manual_pitch)
        elif key in (ord("s"), 84):
            self.manual_pitch = -60.0
            self.manual.set_velocity(self.manual_yaw, self.manual_pitch)
        elif key in (ord("a"), 81):
            self.manual_yaw = -60.0
            self.manual.set_velocity(self.manual_yaw, self.manual_pitch)
        elif key in (ord("d"), 83):
            self.manual_yaw = 60.0
            self.manual.set_velocity(self.manual_yaw, self.manual_pitch)

    def _draw_text(self, image, text, x, y, color=(255, 255, 255), scale=0.55):
        cv2.putText(image, text, (x + 1, y + 1), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, 1, cv2.LINE_AA)

    def _overlay(self, image, frame: proto.TelemetryFrame | None):
        cv2.drawMarker(image, (DISPLAY_W // 2, DISPLAY_H // 2),
                       (0, 255, 255), cv2.MARKER_CROSS, 24, 1)
        if frame is None:
            self._draw_text(image, "NO TELEMETRY", 12, 28, (0, 0, 255), 0.7)
            return
        servo = (frame.status_control >> 5) & 0x7
        tracking = bool(frame.work_state & 1)
        servo_text = {1: "LOCKED", 4: "FREE", 6: "TRACK"}.get(servo, f"SERVO {servo:03b}")
        status = f"{servo_text}  {'TRACKING' if tracking else 'IDLE'}  " \
                 f"{'AUTO' if frame.autonomous else 'MANUAL'}"
        lines = [
            status,
            f"YAW {frame.yaw_deg:+7.2f}  PITCH {frame.pitch_deg:+7.2f} deg",
            f"GYRO {frame.gyro_yaw_dps:+6.1f}/{frame.gyro_pitch_dps:+6.1f} dps",
            f"LOS  {frame.yaw_los_dps:+6.1f}/{frame.pitch_los_dps:+6.1f} dps",
            f"MISS {frame.yaw_misalignment_deg:+6.2f}/{frame.pitch_misalignment_deg:+6.2f} deg",
            f"GATE {frame.gate_x:4d},{frame.gate_y:4d}  {frame.gate_w:3d}x{frame.gate_h:3d}",
            f"TLM {self.tlm_rate:5.1f} Hz  VIDEO {self.video_rate:5.1f} fps",
            f"LENS {self.active.upper()}  ZOOM {1 << ((frame.status_control2 & 3) - 1) if (frame.status_control2 & 3) else 1}x",
        ]
        y = 26
        for line in lines:
            self._draw_text(image, line, 12, y)
            y += 24
        if tracking and frame.gate_w > 0:
            cx, cy = float(frame.gate_x), float(frame.gate_y)
            gw, gh = float(frame.gate_w), float(frame.gate_h)
            if self.active == "tele":
                cx, cy = self._wide_to_tele(cx, cy)
                gw *= F_TELE / F_WIDE
                gh *= F_TELE / F_WIDE
            x0 = int((cx - gw / 2.0) * self.display_scale)
            y0 = int((cy - gh / 2.0) * self.display_scale)
            x1 = int((cx + gw / 2.0) * self.display_scale)
            y1 = int((cy + gh / 2.0) * self.display_scale)
            cv2.rectangle(image, (x0, y0), (x1, y1), (0, 255, 0), 2)
        yaw_cmd = cv2.getTrackbarPos("Yaw", WINDOW) - 60
        pitch_cmd = cv2.getTrackbarPos("Pitch", WINDOW) - 80
        self._draw_text(image, f"CMD {yaw_cmd:+d}/{pitch_cmd:+d} deg", 12, y + 4,
                        (255, 255, 0))
        legend = ("U unlock  L lock  R recenter  C cancel  O auto  P auto-off  "
                  "1 wide  2 tele  Z zoom  WASD/arrows manual  Q quit")
        self._draw_text(image, legend, 12, DISPLAY_H - 16, (255, 255, 255), 0.5)

    def run(self):
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar("Yaw", WINDOW, 60, 120, self._on_trackbar)
        cv2.createTrackbar("Pitch", WINDOW, 80, 110, self._on_trackbar)
        cv2.setMouseCallback(WINDOW, self._on_mouse)
        self.trackbar_ready = True
        try:
            while self.running:
                receiver = self.wide if self.active == "wide" else self.tele
                source = receiver.get()
                if source is None:
                    display = np.zeros((DISPLAY_H, DISPLAY_W, 3), dtype=np.uint8)
                    self._draw_text(display, f"NO VIDEO ({self.active.upper()})",
                                    12, 60, (0, 0, 255), 0.8)
                else:
                    display = cv2.resize(source, (DISPLAY_W, DISPLAY_H),
                                         interpolation=cv2.INTER_AREA)
                frame = self.client.latest
                self._overlay(display, frame)
                if (self.args.snapshot and not self.snapshot_saved
                        and source is not None and frame is not None):
                    Path(self.args.snapshot).parent.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(self.args.snapshot, display)
                    print(f"已保存上位机截图：{self.args.snapshot}", flush=True)
                    self.snapshot_saved = True
                cv2.imshow(WINDOW, display)
                now = time.monotonic()
                if now - self.last_rate_time >= 1.0:
                    self.tlm_rate = (self.client.count - self.last_tlm_count) / (now - self.last_rate_time)
                    self.video_rate = (receiver.count - self.last_video_count) / (now - self.last_rate_time)
                    self.last_tlm_count = self.client.count
                    self.last_video_count = receiver.count
                    self.last_rate_time = now
                key = cv2.waitKey(30) & 0xFF
                if key != 255:
                    self._handle_key(key)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.manual.close()
            self.wide.close()
            self.tele.close()
            self.client.close()
            cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ASD-2-075-DD0 上位机")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=8790)
    parser.add_argument("--telemetry-port", type=int, default=8791)
    parser.add_argument("--wide-port", type=int, default=7001)
    parser.add_argument("--tele-port", type=int, default=7002)
    parser.add_argument("--snapshot", default="", help="保存一张上位机显示截图后继续运行")
    return parser.parse_args()


if __name__ == "__main__":
    GCS(parse_args()).run()
