#!/usr/bin/env python3
"""ASD-2-075-DD0 上位机：RTP 双路视频、二进制遥测/控制、按钮与虚拟摇杆。"""
from __future__ import annotations

import argparse
import configparser
import math
from pathlib import Path
import socket
import sys
import threading
import time

import cv2
import numpy as np

try:
    from PIL import Image, ImageDraw, ImageFont
    _HAVE_PIL = True
except Exception:  # noqa: BLE001 - 无 Pillow 时退回 OpenCV 英文标签
    Image = ImageDraw = ImageFont = None
    _HAVE_PIL = False

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

PANEL_H = 240
BUTTON_W = 140
BUTTON_H = 34
BUTTON_GAP = 8
JOYSTICK_RADIUS = 44
JOYSTICK_Y = DISPLAY_H + 145
BUTTON_ROWS = (
    (("解锁", "unlock"), ("电锁", "lock"), ("归中", "recenter"),
     ("取消锁定", "cancel"), ("自动锁定", "auto"), ("关闭自动", "auto_off"),
     ("发送指定角", "send_angle")),
    (("广角", "wide"), ("长焦", "tele"), ("变倍+", "zoom_in"),
     ("变倍-", "zoom_out"), ("Yaw归零", "zero_yaw"),
     ("Pitch归零", "zero_pitch"), ("停止", "stop")),
)
BUTTON_ROWS_EN = (
    (("UNLOCK", "unlock"), ("LOCK", "lock"), ("HOME", "recenter"),
     ("CANCEL", "cancel"), ("AUTO", "auto"), ("AUTO OFF", "auto_off"),
     ("SEND ANGLE", "send_angle")),
    (("WIDE", "wide"), ("TELE", "tele"), ("ZOOM+", "zoom_in"),
     ("ZOOM-", "zoom_out"), ("YAW ZERO", "zero_yaw"),
     ("PITCH ZERO", "zero_pitch"), ("STOP", "stop")),
)
FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)


def button_layout(display_w: int = DISPLAY_W, display_h: int = DISPLAY_H,
                  rows=BUTTON_ROWS):
    """返回底部面板按钮的 (标签, 动作, 矩形) 列表。"""
    result = []
    for row_index, row in enumerate(rows):
        width = min(BUTTON_W, max(40, (display_w - (len(row) + 1) * BUTTON_GAP) // len(row)))
        total = len(row) * width + (len(row) - 1) * BUTTON_GAP
        x0 = (display_w - total) // 2
        y = display_h + 8 + row_index * (BUTTON_H + BUTTON_GAP)
        for index, (label, action) in enumerate(row):
            x = x0 + index * (width + BUTTON_GAP)
            result.append((label, action, (x, y, width, BUTTON_H)))
    return result


def joystick_axes(x: float, y: float, cx: float = DISPLAY_W / 2.0,
                  cy: float = JOYSTICK_Y, radius: float = JOYSTICK_RADIUS):
    """把鼠标位置转换为带 8% 死区的归一化 yaw/pitch，范围 -1～1。"""
    dx = (x - cx) / radius
    dy = (cy - y) / radius
    length = math.hypot(dx, dy)
    if length > 1.0:
        dx, dy = dx / length, dy / length
    def response(value):
        return math.copysign(max(0.0, (abs(value) - 0.08) / 0.92), value)
    return response(dx), response(dy)


def hit_button(x: int, y: int, buttons):
    for label, action, rect in buttons:
        bx, by, bw, bh = rect
        if bx <= x < bx + bw and by <= y < by + bh:
            return label, action, rect
    return None


def wide_to_tele(u: float, v: float) -> tuple[float, float]:
    tx = (u - WIDTH / 2.0) / F_WIDE
    ty = (v - HEIGHT / 2.0) / F_WIDE
    return WIDTH / 2.0 + F_TELE * tx, HEIGHT / 2.0 + F_TELE * ty


def tele_to_wide(u: float, v: float) -> tuple[float, float]:
    tx = (u - WIDTH / 2.0) / F_TELE
    ty = (v - HEIGHT / 2.0) / F_TELE
    return WIDTH / 2.0 + F_WIDE * tx, HEIGHT / 2.0 + F_WIDE * ty


class VideoReceiver:
    def __init__(self, port: int, name: str, active: bool = True):
        self.port = port
        self.name = name
        self.active = active
        self.latest: np.ndarray | None = None
        self.count = 0
        self.seq = 0
        self.last_mono = 0.0
        self.closing = False
        launch = (
            f"udpsrc port={port} buffer-size=2097152 "
            "caps=application/x-rtp,media=video,clock-rate=90000,"
            "encoding-name=H264,payload=96 "
            "! rtpjitterbuffer latency=0 drop-on-latency=true "
            "! rtph264depay ! h264parse ! nvh264dec "
            "! videoscale ! video/x-raw,width=1280,height=720 "
            "! videoconvert ! video/x-raw,format=BGR "
            "! appsink name=sink max-buffers=1 drop=true sync=false"
        )
        self.pipeline = Gst.parse_launch(launch)
        self.appsink = self.pipeline.get_by_name("sink")
        self.pipeline.set_state(Gst.State.PLAYING if active else Gst.State.PAUSED)
        threading.Thread(target=self._pull_loop, daemon=True).start()

    def set_active(self, active: bool) -> None:
        if active == self.active:
            return
        self.active = active
        self.pipeline.set_state(Gst.State.PLAYING if active else Gst.State.PAUSED)

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
            self.seq += 1
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
            except OSError:
                break
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

    def stop(self):
        self.yaw = self.pitch = 0.0
        self.until = 0.0
        self.active = False
        self.client.send(proto.CMD_FRAME_MOVE, p1=0, p2=0)

    def close(self):
        self.closing = True


class GCS:
    def __init__(self, args):
        self.args = args
        self.wide = VideoReceiver(args.wide_port, "wide", active=args.lens == "wide")
        self.tele = VideoReceiver(args.tele_port, "tele", active=args.lens == "tele")
        self.client = TelemetryClient(args.host, args.control_port, args.telemetry_port)
        self.manual = ManualController(self.client)
        self.active = args.lens
        self.trackbar_ready = False
        self.manual_yaw = 0.0
        self.manual_pitch = 0.0
        self.tlm_rate = 0.0
        self.video_rate = 0.0
        self.last_rate_time = time.monotonic()
        self.last_tlm_count = 0
        self.last_video_count = 0
        self.last_log_time = time.monotonic()
        self.display_count = 0
        self.display_rate = 0.0
        self.last_source_seq = -1
        self.running = True
        self.view_crop = (0, 0, WIDTH, HEIGHT)
        self.view_scale_x = DISPLAY_W / WIDTH
        self.view_scale_y = DISPLAY_H / HEIGHT
        self.source_size = (WIDTH, HEIGHT)
        self.snapshot_saved = False
        self.buttons = []
        self.hover = None
        self.pressed = None
        self.joystick = False
        self.joystick_values = (0.0, 0.0)
        self.last_cmd = ""
        self.font_path = next(
            (path for path in FONT_CANDIDATES if Path(path).is_file()), "")
        self.font_cache = {}
        self.telemetry_overlay = None
        self.telemetry_overlay_time = 0.0
        self.panel_labels = self._build_panel_labels()

    def _font(self, size: int):
        if not _HAVE_PIL:
            return None
        if size not in self.font_cache:
            try:
                self.font_cache[size] = ImageFont.truetype(
                    self.font_path, size) if self.font_path else ImageFont.load_default()
            except OSError:
                self.font_cache[size] = ImageFont.load_default()
        return self.font_cache[size]

    def _texts(self, image, items):
        """items: (text, x, y, color, size)；有 Pillow 时支持中文。"""
        if not _HAVE_PIL or not items:
            return image
        pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil)
        for text, x, y, color, size in items:
            draw.text((x, y), text, font=self._font(size),
                      fill=(color[2], color[1], color[0]),
                      stroke_width=1, stroke_fill=(0, 0, 0))
        return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)

    def _labels(self, rows):
        return rows if _HAVE_PIL else BUTTON_ROWS_EN

    @staticmethod
    def _composite_rgba(image, overlay, x=0, y=0):
        height, width = overlay.shape[:2]
        roi = image[y:y + height, x:x + width]
        alpha = overlay[:, :, 3]
        mask = alpha > 0
        if not mask.any():
            return image
        weight = alpha[mask].astype(np.uint16)
        inverse = 255 - weight
        for channel in range(3):
            source = overlay[:, :, channel][mask].astype(np.uint16)
            target = roi[:, :, channel]
            target[mask] = ((target[mask].astype(np.uint16) * inverse
                             + source * weight) // 255).astype(np.uint8)
        return image

    def _build_panel_labels(self):
        if not _HAVE_PIL:
            return None
        image = Image.new("RGBA", (DISPLAY_W, PANEL_H), (0, 0, 0, 0))
        draw = ImageDraw.Draw(image)
        font = self._font(18)
        for label, _action, rect in self._panel_buttons():
            bbox = draw.textbbox((0, 0), label, font=font)
            tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
            draw.text((rect[0] + (rect[2] - tw) / 2 - bbox[0],
                       rect[1] - DISPLAY_H + (rect[3] - th) / 2 - bbox[1]),
                      label, font=font, fill=(255, 255, 255, 255),
                      stroke_width=1, stroke_fill=(0, 0, 0, 255))
        cy = JOYSTICK_Y - DISPLAY_H
        draw.text((DISPLAY_W // 2 - JOYSTICK_RADIUS,
                   cy + JOYSTICK_RADIUS + 8), "YAW / PITCH",
                  font=font, fill=(230, 230, 230, 255),
                  stroke_width=1, stroke_fill=(0, 0, 0, 255))
        draw.text((DISPLAY_W // 2 - 150, cy + JOYSTICK_RADIUS + 30),
                  "左负 / 右正，上正 / 下负；松开停止",
                  font=self._font(14), fill=(190, 190, 190, 255),
                  stroke_width=1, stroke_fill=(0, 0, 0, 255))
        draw.text((12, PANEL_H - 22),
                  "双击锁定 | 滑条设角→发送指定角 | 空格停止 | Q退出",
                  font=self._font(15), fill=(230, 230, 230, 255),
                  stroke_width=1, stroke_fill=(0, 0, 0, 255))
        return np.array(image)

    def _build_telemetry_overlay(self, lines, colors):
        image = Image.new("RGBA", (760, 320), (0, 0, 0, 0))
        draw = ImageDraw.Draw(image)
        font = self._font(18)
        for index, (text, color) in enumerate(zip(lines, colors)):
            draw.text((12, 8 + index * 23), text, font=font,
                      fill=(color[2], color[1], color[0], 255),
                      stroke_width=1, stroke_fill=(0, 0, 0, 255))
        return np.array(image)

    def _on_trackbar(self, _value):
        # 指定角只在点击“发送指定角”时下发，拖动滑条不连续改变框架。
        return

    def _wide_to_tele(self, u: float, v: float) -> tuple[float, float]:
        return wide_to_tele(u, v)

    def _tele_to_wide(self, u: float, v: float) -> tuple[float, float]:
        return tele_to_wide(u, v)

    def _view_geometry(self, source, frame: proto.TelemetryFrame | None):
        source_h, source_w = source.shape[:2]
        zoom_bits = (frame.status_control2 & 0x3) if frame is not None else 1
        zoom = {1: 1, 2: 2, 3: 4}.get(zoom_bits, 1)
        if self.active == "wide" and zoom > 1:
            crop_w = max(1, source_w // zoom)
            crop_h = max(1, source_h // zoom)
            x0 = (source_w - crop_w) // 2
            y0 = (source_h - crop_h) // 2
        else:
            x0, y0, crop_w, crop_h = 0, 0, source_w, source_h
        self.view_crop = (x0, y0, crop_w, crop_h)
        self.view_scale_x = DISPLAY_W / crop_w
        self.view_scale_y = DISPLAY_H / crop_h
        self.source_size = (source_w, source_h)
        crop = source[y0:y0 + crop_h, x0:x0 + crop_w]
        return cv2.resize(crop, (DISPLAY_W, DISPLAY_H),
                          interpolation=cv2.INTER_AREA)

    def _source_to_view(self, u: float, v: float) -> tuple[float, float]:
        x0, y0, _cw, _ch = self.view_crop
        return (u - x0) * self.view_scale_x, (v - y0) * self.view_scale_y

    def _panel_buttons(self):
        return button_layout(rows=self._labels(BUTTON_ROWS))

    def _joystick_at(self, x, y):
        return math.hypot(x - DISPLAY_W / 2.0, y - JOYSTICK_Y) <= JOYSTICK_RADIUS

    def _update_joystick(self, x, y):
        self.joystick_values = joystick_axes(x, y)
        yaw, pitch = self.joystick_values
        self.manual.set_velocity(20.0 * yaw, 20.0 * pitch, hold=0.3)

    def _release_joystick(self):
        if self.joystick:
            self.manual.stop()
        self.joystick = False
        self.joystick_values = (0.0, 0.0)

    def _unlock_if_locked(self):
        frame = self.client.latest
        if frame is not None and ((frame.status_control >> 5) & 0x7) == 1:
            self.client.send(proto.CMD_FRAME_UNLOCK)

    def _activate(self, action):
        self._release_joystick()
        if action == "unlock":
            self.client.send(proto.CMD_FRAME_UNLOCK)
            self.last_cmd = "已发送：解锁"
        elif action == "lock":
            self.client.send(proto.CMD_FRAME_LOCK)
            self.last_cmd = "已发送：电锁"
        elif action == "recenter":
            self._unlock_if_locked()
            self.client.send(proto.CMD_RECENTER)
            self.last_cmd = "已发送：归中"
        elif action == "cancel":
            self.client.send(proto.CMD_CANCEL_LOCK)
            self.last_cmd = "已发送：取消锁定"
        elif action == "auto":
            self.client.send(proto.CMD_AUTO_LOCK_ON)
            self.last_cmd = "已发送：自动锁定"
        elif action == "auto_off":
            self.client.send(proto.CMD_AUTO_LOCK_OFF)
            self.last_cmd = "已发送：关闭自动锁定"
        elif action == "wide":
            self.active = "wide"
            self.last_source_seq = -1
            self.wide.set_active(True)
            self.tele.set_active(False)
            self.client.send(proto.CMD_DETECTOR_SWITCH, p1=1)
            self.last_cmd = "已发送：广角镜头"
        elif action == "tele":
            self.active = "tele"
            self.last_source_seq = -1
            self.wide.set_active(False)
            self.tele.set_active(True)
            self.client.send(proto.CMD_DETECTOR_SWITCH, p1=2)
            self.last_cmd = "已发送：长焦镜头"
        elif action == "zoom_in":
            self.client.send(proto.CMD_ZOOM_IN)
            self.last_cmd = "已发送：变倍+"
        elif action == "zoom_out":
            self.client.send(proto.CMD_ZOOM_OUT)
            self.last_cmd = "已发送：变倍-"
        elif action == "send_angle":
            self._unlock_if_locked()
            yaw = cv2.getTrackbarPos("Yaw x0.1", WINDOW) / 10.0 - 60.0
            pitch = cv2.getTrackbarPos("Pitch x0.1", WINDOW) / 10.0 - 80.0
            self.client.send(proto.CMD_SPECIFIED_ANGLE,
                             p1=int(round(yaw * 100)), p2=int(round(pitch * 100)))
            self.last_cmd = f"指定角：yaw {yaw:.1f}  pitch {pitch:.1f}"
        elif action in ("zero_yaw", "zero_pitch"):
            frame = self.client.latest
            if frame is None:
                self.last_cmd = "无遥测，无法归零单轴"
                return
            self._unlock_if_locked()
            yaw = 0.0 if action == "zero_yaw" else frame.yaw_deg
            pitch = 0.0 if action == "zero_pitch" else frame.pitch_deg
            self.client.send(proto.CMD_SPECIFIED_ANGLE,
                             p1=int(round(yaw * 100)), p2=int(round(pitch * 100)))
            self.last_cmd = (f"Yaw归零：pitch {pitch:+.1f}"
                             if action == "zero_yaw" else
                             f"Pitch归零：yaw {yaw:+.1f}")
        elif action == "stop":
            self.manual.stop()
            self.client.send(proto.CMD_CANCEL_LOCK)
            self.last_cmd = "已停止：取消跟踪并归零速度"

    def _on_mouse(self, event, x, y, flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            hit = hit_button(x, y, self.buttons)
            if hit is not None:
                self.pressed = (hit[0], hit[1])
                return
            if self._joystick_at(x, y):
                self.joystick = True
                self._update_joystick(x, y)
                return
            if y >= DISPLAY_H:
                return
        elif event == cv2.EVENT_MOUSEMOVE:
            hit = hit_button(x, y, self.buttons)
            self.hover = hit[0] if hit is not None else None
            if self.joystick:
                if flags & cv2.EVENT_FLAG_LBUTTON:
                    self._update_joystick(x, y)
                else:
                    self._release_joystick()
            return
        elif event == cv2.EVENT_LBUTTONUP:
            if self.pressed is not None:
                label, action = self.pressed
                self.pressed = None
                hit = hit_button(x, y, self.buttons)
                if hit is not None and hit[0] == label:
                    self._activate(action)
                return
            if self.joystick:
                self._release_joystick()
                return
            return
        if event != cv2.EVENT_LBUTTONDBLCLK:
            return
        if y >= DISPLAY_H:
            return
        frame = self.client.latest
        plugin_lens = "tele" if frame and (frame.status_control2 & 0x10) else "wide"
        if plugin_lens != self.active:
            self.last_cmd = f"镜头切换中：显示 {self.active}，插件 {plugin_lens}"
            return
        x0, y0, _cw, _ch = self.view_crop
        source = (self.wide if self.active == "wide" else self.tele).get()
        source_w = source.shape[1] if source is not None else WIDTH
        source_h = source.shape[0] if source is not None else HEIGHT
        u = x0 + x / self.view_scale_x
        v = y0 + y / self.view_scale_y
        if not (0 <= u < source_w and 0 <= v < source_h):
            return
        self._unlock_if_locked()
        self.client.send(proto.CMD_POINT_LOCK, p1=int(round(u)), p2=int(round(v)))
        self.last_cmd = f"双击锁定：{self.active} ({u:.0f},{v:.0f})"

    def _handle_key(self, key: int):
        if key in (ord("q"), 27):
            self.running = False
            return
        self._release_joystick()
        if key == ord("u"):
            self._activate("unlock")
        elif key == ord("l"):
            self._activate("lock")
        elif key in (ord("r"), ord("h")):
            self._activate("recenter")
        elif key == ord("c"):
            self._activate("cancel")
        elif key == ord("o"):
            self._activate("auto")
        elif key == ord("p"):
            self._activate("auto_off")
        elif key == ord("1"):
            self._activate("wide")
        elif key == ord("2"):
            self._activate("tele")
        elif key == ord("z"):
            self._activate("zoom_in")
        elif key == ord("Z"):
            self._activate("zoom_out")
        elif key == ord(" "):
            self._activate("stop")
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
        lines = []
        colors = []
        if frame is None:
            lines = ["无遥测"] if _HAVE_PIL else ["NO TELEMETRY"]
            colors = [(0, 0, 255)]
        else:
            servo = (frame.status_control >> 5) & 0x7
            tracking = bool(frame.work_state & 1)
            plugin_lens = "tele" if frame.status_control2 & 0x10 else "wide"
            stale = time.monotonic() - self.client.last_mono > 1.0
            servo_text = {1: "电锁", 4: "自由", 6: "跟踪"}.get(
                servo, f"伺服{servo:03b}")
            lens_cn = "广角" if self.active == "wide" else "长焦"
            plugin_cn = "广角" if plugin_lens == "wide" else "长焦"
            zoom_bits = frame.status_control2 & 0x3
            zoom = {1: 1, 2: 2, 3: 4}.get(zoom_bits, 1)
            if _HAVE_PIL:
                miss_text = (f"脱靶量: yaw {frame.yaw_misalignment_deg:+.2f}  "
                             f"pitch {frame.pitch_misalignment_deg:+.2f} °"
                             if tracking else "脱靶量: 无效（无有效跟踪）")
                gate_text = (f"波门: ({frame.gate_x},{frame.gate_y})  "
                             f"{frame.gate_w}x{frame.gate_h}"
                             if tracking else "波门: 无")
                lines = [
                    f"状态: {servo_text}   跟踪: {'是' if tracking else '否'}   "
                    f"模式: {'自动' if frame.autonomous else '手动'}",
                    f"机械角: yaw {frame.yaw_deg:+.2f}  pitch {frame.pitch_deg:+.2f} °",
                    f"陀螺: yaw {frame.gyro_yaw_dps:+.1f}  pitch {frame.gyro_pitch_dps:+.1f} °/s",
                    f"目标LOS: yaw {frame.yaw_los_dps:+.1f}  pitch {frame.pitch_los_dps:+.1f} °/s",
                    miss_text,
                    gate_text,
                    f"镜头: {lens_cn}   变倍: {zoom}x",
                    f"遥测 {self.tlm_rate:.1f} Hz   视频 {self.video_rate:.1f} fps",
                ]
            else:
                miss_text = (f"MISS yaw {frame.yaw_misalignment_deg:+.2f} "
                             f"pitch {frame.pitch_misalignment_deg:+.2f} deg"
                             if tracking else "MISS invalid")
                gate_text = (f"GATE ({frame.gate_x},{frame.gate_y}) "
                             f"{frame.gate_w}x{frame.gate_h}" if tracking else "GATE none")
                lines = [
                    f"STATE {servo_text}  TRACK {tracking}  "
                    f"{'AUTO' if frame.autonomous else 'MANUAL'}",
                    f"ANGLE yaw {frame.yaw_deg:+.2f} pitch {frame.pitch_deg:+.2f} deg",
                    f"GYRO yaw {frame.gyro_yaw_dps:+.1f} pitch {frame.gyro_pitch_dps:+.1f} dps",
                    f"LOS yaw {frame.yaw_los_dps:+.1f} pitch {frame.pitch_los_dps:+.1f} dps",
                    miss_text,
                    gate_text,
                    f"LENS {lens_cn} ZOOM {zoom}x",
                    f"TLM {self.tlm_rate:.1f} Hz VIDEO {self.video_rate:.1f} fps",
                ]
            colors = [(0, 255, 255)] * len(lines)
            colors[0] = (0, 255, 0) if tracking else (0, 255, 255)
            if plugin_lens != self.active:
                lines.append(f"镜头切换中: 显示 {lens_cn} / 插件 {plugin_cn}"
                             if _HAVE_PIL else
                             f"LENS SWITCH display {lens_cn} plugin {plugin_cn}")
                colors.append((0, 165, 255))
            if stale:
                lines.append("遥测超时" if _HAVE_PIL else "TELEMETRY STALE")
                colors.append((0, 0, 255))
            if tracking and frame.gate_w > 0 and plugin_lens == self.active:
                cx = float(frame.gate_x)
                cy = float(frame.gate_y)
                gw = float(frame.gate_w)
                gh = float(frame.gate_h)
                vx, vy = self._source_to_view(cx, cy)
                vw = gw * self.view_scale_x
                vh = gh * self.view_scale_y
                x0 = int(vx - vw / 2.0)
                y0 = int(vy - vh / 2.0)
                x1 = int(vx + vw / 2.0)
                y1 = int(vy + vh / 2.0)
                cv2.rectangle(image, (x0, y0), (x1, y1), (0, 255, 0), 2)
        if self.joystick:
            yaw, pitch = self.joystick_values
            lines.append(f"摇杆: yaw {20 * yaw:+.1f}  pitch {20 * pitch:+.1f} °/s"
                         if _HAVE_PIL else
                         f"JOYSTICK yaw {20 * yaw:+.1f} pitch {20 * pitch:+.1f} dps")
            colors.append((255, 255, 0))
        else:
            if self.trackbar_ready:
                yaw_cmd = cv2.getTrackbarPos("Yaw x0.1", WINDOW) / 10.0 - 60.0
                pitch_cmd = cv2.getTrackbarPos("Pitch x0.1", WINDOW) / 10.0 - 80.0
            else:
                yaw_cmd = pitch_cmd = 0.0
            lines.append(f"指定角: yaw {yaw_cmd:+.1f}  pitch {pitch_cmd:+.1f} °"
                         if _HAVE_PIL else
                         f"CMD yaw {yaw_cmd:+.1f} pitch {pitch_cmd:+.1f} deg")
            colors.append((255, 255, 0))
        if self.last_cmd:
            lines.append(self.last_cmd)
            colors.append((0, 255, 0))
        if _HAVE_PIL:
            now = time.monotonic()
            if (self.telemetry_overlay is None
                    or now - self.telemetry_overlay_time >= 0.2):
                self.telemetry_overlay = self._build_telemetry_overlay(lines, colors)
                self.telemetry_overlay_time = now
            return self._composite_rgba(image, self.telemetry_overlay)
        y = 26
        for text, color in zip(lines, colors):
            self._draw_text(image, text, 12, y, color, 0.55)
            y += 24
        return image

    def _panel(self, image, frame: proto.TelemetryFrame | None):
        panel = np.full((PANEL_H, DISPLAY_W, 3), 28, dtype=np.uint8)
        buttons = self._panel_buttons()
        self.buttons = buttons
        servo = (frame.status_control >> 5) & 0x7 if frame else -1
        auto = bool(frame.autonomous) if frame else False
        for label, action, rect in buttons:
            x, y, width, height = rect
            y -= DISPLAY_H
            active = ((action == "wide" and self.active == "wide")
                      or (action == "tele" and self.active == "tele")
                      or (action == "auto" and auto)
                      or (action == "lock" and servo == 1))
            bg = (0, 110, 0) if active else (60, 60, 60)
            if action == "stop":
                bg = (0, 0, 120)
            if self.hover == label:
                bg = (110, 110, 110) if not active else (0, 150, 0)
            if self.pressed is not None and self.pressed[0] == label:
                bg = (0, 150, 0)
            cv2.rectangle(panel, (x, y), (x + width, y + height), bg, -1)
            cv2.rectangle(panel, (x, y), (x + width, y + height),
                          (200, 200, 200), 1)
        cx, cy = DISPLAY_W // 2, JOYSTICK_Y - DISPLAY_H
        cv2.circle(panel, (cx, cy), JOYSTICK_RADIUS, (150, 150, 150), 1, cv2.LINE_AA)
        cv2.line(panel, (cx - JOYSTICK_RADIUS, cy),
                 (cx + JOYSTICK_RADIUS, cy), (80, 80, 80), 1)
        cv2.line(panel, (cx, cy - JOYSTICK_RADIUS),
                 (cx, cy + JOYSTICK_RADIUS), (80, 80, 80), 1)
        x, y = self.joystick_values if self.joystick else (0.0, 0.0)
        cv2.circle(panel, (int(cx + x * JOYSTICK_RADIUS),
                           int(cy - y * JOYSTICK_RADIUS)),
                   11, (0, 200, 255), -1, cv2.LINE_AA)
        if self.panel_labels is not None:
            self._composite_rgba(panel, self.panel_labels)
        else:
            for label, _action, rect in buttons:
                cv2.putText(panel, label, (rect[0] + 6, rect[1] - DISPLAY_H + 23),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(panel, "YAW / PITCH", (cx - 60, cy + JOYSTICK_RADIUS + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1, cv2.LINE_AA)
            cv2.putText(panel, "set angle on sliders then SEND ANGLE | double-click to lock | space stop | Q quit",
                        (12, PANEL_H - 12), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (230, 230, 230), 1, cv2.LINE_AA)
        return np.vstack((image, panel))

    def run(self):
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        try:
            cv2.setWindowProperty(WINDOW, cv2.WND_PROP_TOPMOST, 1)
        except cv2.error:
            pass
        cv2.createTrackbar("Yaw x0.1", WINDOW, 600, 1200, self._on_trackbar)
        cv2.createTrackbar("Pitch x0.1", WINDOW, 800, 1100, self._on_trackbar)
        cv2.setMouseCallback(WINDOW, self._on_mouse)
        self.trackbar_ready = True
        if not self.args.no_lens_switch:
            self.client.send(proto.CMD_DETECTOR_SWITCH,
                             p1=1 if self.active == "wide" else 2)
        try:
            while self.running:
                receiver = self.wide if self.active == "wide" else self.tele
                source = receiver.get()
                frame = self.client.latest
                new_frame = receiver.seq != self.last_source_seq
                if new_frame:
                    self.last_source_seq = receiver.seq
                    if source is None:
                        display = np.zeros((DISPLAY_H, DISPLAY_W, 3), dtype=np.uint8)
                        self.view_crop = (0, 0, WIDTH, HEIGHT)
                        self.view_scale_x = DISPLAY_W / WIDTH
                        self.view_scale_y = DISPLAY_H / HEIGHT
                        self.source_size = (WIDTH, HEIGHT)
                        self._draw_text(display, f"NO VIDEO ({self.active.upper()})",
                                        12, 60, (0, 0, 255), 0.8)
                    else:
                        display = self._view_geometry(source, frame)
                    display = self._overlay(display, frame)
                    display = self._panel(display, frame)
                    plugin_lens = "tele" if frame and (frame.status_control2 & 0x10) else "wide"
                    if (self.args.snapshot and not self.snapshot_saved
                            and source is not None and frame is not None
                            and plugin_lens == self.active
                            and self.video_rate >= 10 and self.tlm_rate >= 10):
                        Path(self.args.snapshot).parent.mkdir(parents=True, exist_ok=True)
                        cv2.imwrite(self.args.snapshot, display)
                        print(f"已保存上位机截图：{self.args.snapshot}", flush=True)
                        self.snapshot_saved = True
                    cv2.imshow(WINDOW, display)
                    self.display_count += 1
                now = time.monotonic()
                if now - self.last_rate_time >= 1.0:
                    self.tlm_rate = (self.client.count - self.last_tlm_count) / (now - self.last_rate_time)
                    self.video_rate = (receiver.count - self.last_video_count) / (now - self.last_rate_time)
                    self.display_rate = self.display_count / (now - self.last_rate_time)
                    self.last_tlm_count = self.client.count
                    self.last_video_count = receiver.count
                    self.display_count = 0
                    self.last_rate_time = now
                    self.telemetry_overlay = None
                if now - self.last_log_time >= 5.0:
                    print(f"速率：遥测 {self.tlm_rate:.1f} Hz，解码 {self.video_rate:.1f} fps，"
                          f"显示 {self.display_rate:.1f} fps", flush=True)
                    self.last_log_time = now
                key = cv2.waitKey(5) & 0xFF
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


def run_selftest(args) -> int:
    client = TelemetryClient(args.host, args.control_port, args.telemetry_port)
    wide = VideoReceiver(args.wide_port, "wide")
    tele = VideoReceiver(args.tele_port, "tele")
    ok = True
    try:
        deadline = time.monotonic() + args.selftest
        while time.monotonic() < deadline and client.latest is None:
            time.sleep(0.1)
        if client.latest is None:
            print("自检失败：没有遥测")
            return 1
        start = time.monotonic()
        c0, w0, t0 = client.count, wide.count, tele.count
        time.sleep(min(3.0, max(1.0, args.selftest)))
        elapsed = time.monotonic() - start
        rate = (client.count - c0) / elapsed
        video_wide = wide.count - w0
        video_tele = tele.count - t0
        print(f"遥测 {rate:.1f} Hz，广角视频 {video_wide} 帧，长焦视频 {video_tele} 帧")
        if rate < args.selftest_min_rate:
            print(f"自检失败：遥测低于 {args.selftest_min_rate:.0f} Hz")
            ok = False
        elif rate < 80.0:
            print("警告：遥测低于 80 Hz 目标，当前机器渲染负载较高")
        if video_wide <= 0 or video_tele <= 0:
            print("自检失败：双路视频没有解码帧")
            ok = False
        client.send(proto.CMD_FRAME_UNLOCK)
        time.sleep(0.5)
        frame = client.latest
        servo = (frame.status_control >> 5) & 0x7 if frame else -1
        print(f"解锁后伺服状态 {servo:03b}" if servo >= 0 else "解锁后无遥测")
        if servo != 4:
            print("自检失败：解锁后不是自由状态")
            ok = False
        client.send(proto.CMD_SPECIFIED_ANGLE, p1=500, p2=300)
        time.sleep(1.5)
        client.send(proto.CMD_RECENTER)
        time.sleep(0.5)
        print("自检通过" if ok else "自检失败")
        return 0 if ok else 1
    finally:
        wide.close()
        tele.close()
        client.close()


def parse_args() -> argparse.Namespace:
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--config", default=str(ROOT / "ground_station" / "config.ini"))
    pre_args, _ = pre.parse_known_args()
    config = configparser.ConfigParser()
    config.read(pre_args.config, encoding="utf-8")

    def get(section, key, default):
        return config.get(section, key, fallback=default)

    def getint(section, key, default):
        try:
            return config.getint(section, key, fallback=default)
        except ValueError:
            return default

    def getbool(section, key, default):
        try:
            return config.getboolean(section, key, fallback=default)
        except ValueError:
            return default

    parser = argparse.ArgumentParser(description="ASD-2-075-DD0 上位机")
    parser.add_argument("--config", default=pre_args.config)
    parser.add_argument("--host", default=get("network", "host", "127.0.0.1"))
    parser.add_argument("--control-port", type=int,
                        default=getint("network", "control_port", 8790))
    parser.add_argument("--telemetry-port", type=int,
                        default=getint("network", "telemetry_port", 8791))
    parser.add_argument("--wide-port", type=int,
                        default=getint("video", "wide_port", 7001))
    parser.add_argument("--tele-port", type=int,
                        default=getint("video", "tele_port", 7002))
    parser.add_argument("--lens", choices=("wide", "tele"),
                        default=get("session", "lens", "wide"),
                        help="启动时激活的镜头")
    parser.add_argument("--no-lens-switch", action="store_true",
                        default=getbool("session", "no_lens_switch", False),
                        help="启动时不发送镜头切换命令")
    parser.add_argument("--snapshot", default="", help="保存一张上位机显示截图后继续运行")
    parser.add_argument("--selftest", type=float, default=0.0,
                        help="无界面自检秒数；>0 时不打开窗口")
    parser.add_argument("--selftest-min-rate", type=float, default=40.0,
                        help="自检允许的最低遥测频率 Hz")
    parser.add_argument("--save-config", action="store_true",
                        help="把当前连接/视频/镜头配置写回 --config 后退出")
    return parser.parse_args()


def save_config(args: argparse.Namespace) -> None:
    config = configparser.ConfigParser()
    config["network"] = {
        "host": args.host,
        "control_port": str(args.control_port),
        "telemetry_port": str(args.telemetry_port),
    }
    config["video"] = {
        "wide_port": str(args.wide_port),
        "tele_port": str(args.tele_port),
    }
    config["session"] = {
        "lens": args.lens,
        "no_lens_switch": "true" if args.no_lens_switch else "false",
    }
    path = Path(args.config)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        config.write(handle)
    print(f"配置已保存：{path}", flush=True)


if __name__ == "__main__":
    args = parse_args()
    if args.save_config:
        save_config(args)
        sys.exit(0)
    if args.selftest > 0:
        sys.exit(run_selftest(args))
    GCS(args).run()
