#!/usr/bin/env python3
"""A2416A 捷联导引头地面站。"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

try:
    import cv2
    import numpy as np
    CV_IMPORT_ERROR = ""
except ImportError as exc:
    cv2 = np = None
    CV_IMPORT_ERROR = str(exc)

from ground_station.client import TelemetryClient
from ground_station.configuration import DEFAULT_PATH, load, with_forward_override
from ground_station.controls import COMMANDS, box_to_source, key_movement, view_to_source
from ground_station.forward import TelemetryForwarder, endpoint
from video.hardware import HardwareVideoError, require_nvcodec
from video.receiver import VideoReceiver

try:
    from PIL import Image, ImageDraw, ImageFont
    HAVE_PIL = True
except ImportError:
    Image = ImageDraw = ImageFont = None
    HAVE_PIL = False


BUTTONS = (
    ("开启识别", "recognition_on"), ("关闭识别", "recognition_off"),
    ("自动锁定", "auto_lock_on"), ("关闭自动", "auto_lock_off"),
    ("取消跟踪", "cancel"), ("变倍+", "zoom_in"), ("变倍-", "zoom_out"),
)
FORWARD_START_LABEL = "发送艾晟报文"
FORWARD_STOP_LABEL = "停止发送"
PANEL_HEIGHT = 160


def button_layout(width: int, video_height: int):
    columns = 7
    gap, height = 7, 34
    item_width = max(70, (width - gap * (columns + 1)) // columns)
    result = []
    for index, (label, action) in enumerate(BUTTONS):
        row, column = divmod(index, columns)
        result.append((label, action,
                       (gap + column * (item_width + gap), video_height + 7 + row * 41,
                        item_width, height)))
    return result


def _inside(x, y, rect):
    rx, ry, width, height = rect
    return rx <= x < rx + width and ry <= y < ry + height


def window_is_visible(window: str) -> bool:
    """返回OpenCV窗口是否仍存在；用户点击关闭时Qt后端可能直接抛错。"""
    try:
        return cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        return False


class GroundStation:
    def __init__(self, config: dict):
        if cv2 is None or np is None:
            raise RuntimeError(f"地面站图形依赖不可用：{CV_IMPORT_ERROR}")
        self.config = config
        display = config["display"]
        self.width, self.height = display["width"], display["height"]
        self.window_title = display["window_title"]
        host = config["network"]["host"].replace(".", "_")
        self.window = f"A2416A_GCS_{host}_{config['network']['port']}"
        self.buttons = button_layout(self.width, self.height)
        self.forward_input_rects = {
            "ip": (7, self.height + 90, 190, 34),
            "port": (204, self.height + 90, 92, 34),
        }
        self.forward_rect = (303, self.height + 90, 170, 34)
        self.forward_inputs = {
            "ip": config["forward"]["ip"],
            "port": str(config["forward"]["port"]),
        }
        self.forward_focus = None
        self.forward_selected = False
        self.forwarder = TelemetryForwarder()
        self.client = TelemetryClient(config["network"]["host"], config["network"]["port"],
                                      on_telemetry=self.forwarder.offer)
        self.video = VideoReceiver(config["video"]["rtsp_url"],
                                   display_width=self.width, display_height=self.height)
        self.running = True
        self.zoom = 1
        self.recognition_enabled = False
        self.auto_lock_enabled = False
        self.drag_start = None
        self.drag_end = None
        self.last_action = ""
        self.last_rate_time = time.monotonic()
        self.last_rx = self.last_video = 0
        self.telemetry_hz = self.video_hz = 0.0
        self.font = None
        if HAVE_PIL:
            for path in (
                "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
                "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            ):
                try:
                    self.font = ImageFont.truetype(path, 16)
                    break
                except OSError:
                    continue

    def _send_action(self, action: str):
        # 自动锁定依赖检测候选；按钮作为完整操作，先确保识别已开启。
        if action == "auto_lock_on":
            self.client.send(COMMANDS["recognition_on"])
        command = COMMANDS[action]
        self.client.send(command)
        if action == "recognition_on":
            self.recognition_enabled = True
        elif action == "recognition_off":
            self.recognition_enabled = False
            self.auto_lock_enabled = False
        elif action == "auto_lock_on":
            self.recognition_enabled = True
            self.auto_lock_enabled = True
        elif action in ("auto_lock_off", "cancel"):
            self.auto_lock_enabled = False
        elif action == "zoom_in":
            self.zoom = {1: 2, 2: 4, 4: 4}[self.zoom]
        elif action == "zoom_out":
            self.zoom = {1: 1, 2: 1, 4: 2}[self.zoom]
        self.last_action = f"已发送：{next(label for label, name in BUTTONS if name == action)}"

    def _toggle_forward(self):
        state = self.forwarder.snapshot()
        if state["active"]:
            self.forwarder.stop()
            self.last_action = "已停止发送艾晟报文"
        else:
            try:
                target = endpoint(self.forward_inputs["ip"], self.forward_inputs["port"])
            except ValueError as exc:
                self.last_action = str(exc)
                return
            self.forward_inputs = {"ip": target[0], "port": str(target[1])}
            self.config["forward"] = {"ip": target[0], "port": target[1]}
            self.forwarder.start(*target)
            self.forward_focus = None
            self.last_action = f"艾晟报文发送至 {target[0]}:{target[1]}"

    def _forward_at(self, x, y):
        if _inside(x, y, self.forward_rect):
            return "button"
        for name, rect in self.forward_input_rects.items():
            if _inside(x, y, rect):
                return name
        return None

    def _handle_forward_key(self, key: int) -> bool:
        if self.forward_focus is None:
            return False
        if key == 27:
            self.forward_focus = None
            return True
        if key in (10, 13):
            self._toggle_forward()
            self.forward_focus = None
            return True
        if key == 9:
            order = ("ip", "port")
            self.forward_focus = order[(order.index(self.forward_focus) + 1) % len(order)]
            self.forward_selected = True
            return True
        if key in (8, 127):
            text = self.forward_inputs[self.forward_focus]
            self.forward_inputs[self.forward_focus] = text[:-1]
            self.forward_selected = False
            return True
        allowed = ord(".") if self.forward_focus == "ip" else None
        if ord("0") <= key <= ord("9") or key == allowed:
            if self.forward_selected:
                self.forward_inputs[self.forward_focus] = ""
                self.forward_selected = False
            text = self.forward_inputs[self.forward_focus]
            if key == ord(".") and (not text or text.endswith(".")):
                return True
            limit = 45 if self.forward_focus == "ip" else 5
            self.forward_inputs[self.forward_focus] = (text + chr(key))[:limit]
        return True

    def _mouse(self, event, x, y, flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            forward_control = self._forward_at(x, y)
            if forward_control == "button":
                self._toggle_forward()
                return
            if forward_control in self.forward_input_rects:
                if self.forwarder.snapshot()["active"]:
                    self.last_action = "停止发送后可修改艾晟报文目的地址"
                    return
                self.forward_focus = forward_control
                self.forward_selected = True
                return
            self.forward_focus = None
            for _label, action, rect in self.buttons:
                if _inside(x, y, rect):
                    self._send_action(action)
                    return
            if y < self.height:
                self.drag_start = (x, y)
                self.drag_end = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
            if flags & cv2.EVENT_FLAG_LBUTTON:
                self.drag_end = (min(self.width - 1, max(0, x)),
                                 min(self.height - 1, max(0, y)))
        elif event == cv2.EVENT_LBUTTONUP and self.drag_start is not None:
            end = (min(self.width - 1, max(0, x)), min(self.height - 1, max(0, y)))
            distance = math.hypot(end[0] - self.drag_start[0], end[1] - self.drag_start[1])
            if distance < 6:
                u, v = view_to_source(*end, self.width, self.height, self.zoom)
                self.client.send(0xB7, p1=u, p2=v)
                self.auto_lock_enabled = False
                self.last_action = f"点选锁定：({u},{v})"
            else:
                box = box_to_source(self.drag_start, end, self.width, self.height, self.zoom)
                self.client.send(0xB8, p1=box.x, p2=box.y, p3=box.width, p4=box.height)
                self.auto_lock_enabled = False
                self.last_action = f"框选锁定：({box.x},{box.y}) {box.width}x{box.height}"
            self.drag_start = self.drag_end = None

    def _text(self, image, lines):
        if not HAVE_PIL or self.font is None:
            for index, line in enumerate(lines):
                cv2.putText(image, line.encode("ascii", "replace").decode(),
                            (10, 23 + index * 22), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 255), 1, cv2.LINE_AA)
            return image
        pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil)
        for index, line in enumerate(lines):
            draw.text((10, 5 + index * 22), line, font=self.font,
                      fill=(255, 255, 0), stroke_width=1, stroke_fill=(0, 0, 0))
        return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)

    def _frame(self):
        source = self.video.latest
        if source is None:
            image = np.zeros((self.height, self.width, 3), np.uint8)
            cv2.putText(image, "WAITING FOR H264 RTSP", (self.width // 2 - 150, self.height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 255), 1, cv2.LINE_AA)
        elif self.zoom == 1:
            image = source.copy()
        else:
            crop_w, crop_h = source.shape[1] // self.zoom, source.shape[0] // self.zoom
            x0, y0 = (source.shape[1] - crop_w) // 2, (source.shape[0] - crop_h) // 2
            image = cv2.resize(source[y0:y0 + crop_h, x0:x0 + crop_w],
                               (self.width, self.height), interpolation=cv2.INTER_LINEAR)
        panel = np.full((PANEL_HEIGHT, self.width, 3), 28, np.uint8)
        canvas = np.vstack((image, panel))
        labels = []
        for label, action, rect in self.buttons:
            x, y, width, height = rect
            active = (
                (action == "recognition_on" and self.recognition_enabled)
                or (action == "recognition_off" and not self.recognition_enabled)
                or (action == "auto_lock_on" and self.auto_lock_enabled)
                or (action == "auto_lock_off" and not self.auto_lock_enabled)
            )
            cv2.rectangle(canvas, (x, y), (x + width, y + height),
                          (0, 110, 0) if active else (58, 58, 58), -1)
            cv2.rectangle(canvas, (x, y), (x + width, y + height), (180, 180, 180), 1)
            labels.append((label, x + 7, y + 7))
        forward = self.forwarder.snapshot()
        for name, rect in self.forward_input_rects.items():
            ix, iy, iw, ih = rect
            cv2.rectangle(canvas, (ix, iy), (ix + iw, iy + ih), (35, 35, 35), -1)
            cv2.rectangle(canvas, (ix, iy), (ix + iw, iy + ih),
                          (0, 230, 255) if self.forward_focus == name else (180, 180, 180), 1)
            prefix = "IP " if name == "ip" else "Port "
            labels.append((prefix + self.forward_inputs[name], ix + 6, iy + 7))
        x, y, width, height = self.forward_rect
        cv2.rectangle(canvas, (x, y), (x + width, y + height),
                      (0, 110, 0) if forward["active"] else (58, 58, 58), -1)
        cv2.rectangle(canvas, (x, y), (x + width, y + height), (200, 200, 200), 1)
        labels.append((FORWARD_STOP_LABEL if forward["active"] else FORWARD_START_LABEL,
                       x + 10, y + 7))
        status = forward["error"] or f"已发送 {forward['sent']} 包"
        labels.append((f"艾晟 JSON  {status}", 483, self.height + 96))
        canvas = self._texts_at(canvas, labels)
        if self.drag_start is not None and self.drag_end is not None:
            cv2.rectangle(canvas, self.drag_start, self.drag_end, (0, 255, 255), 2)
        cv2.drawMarker(canvas, (self.width // 2, self.height // 2),
                       (0, 255, 255), cv2.MARKER_CROSS, 24, 1)
        sample = self.client.latest
        lines = [
            f"A2416A 捷联  H.264  {self.zoom}x  遥测 {self.telemetry_hz:.1f}Hz  视频 {self.video_hz:.1f}fps",
            "左键点选目标；拖动框选目标；W/A/S/D移动光标；Q退出",
        ]
        if sample is not None:
            frame = sample.frame
            lines.append(
                f"失调角 yaw {getattr(frame, 'yaw_misalignment_deg', 0.0):+.2f}°  "
                f"pitch {getattr(frame, 'pitch_misalignment_deg', 0.0):+.2f}°")
        if self.last_action:
            lines.append(self.last_action)
        return self._text(canvas, lines)

    def _texts_at(self, image, items):
        if not HAVE_PIL or self.font is None:
            for text, x, y in items:
                cv2.putText(image, text.encode("ascii", "replace").decode(), (x, y + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                            cv2.LINE_AA)
            return image
        pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil)
        for text, x, y in items:
            draw.text((x, y), text, font=self.font, fill=(255, 255, 255))
        return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)

    def _update_rates(self):
        now = time.monotonic()
        elapsed = now - self.last_rate_time
        if elapsed >= 1.0:
            self.telemetry_hz = (self.client.received - self.last_rx) / elapsed
            self.video_hz = (self.video.frames - self.last_video) / elapsed
            self.last_rx, self.last_video = self.client.received, self.video.frames
            self.last_rate_time = now

    def run(self):
        cv2.namedWindow(self.window, cv2.WINDOW_AUTOSIZE)
        try:
            # OpenCV 5 的 Qt 后端在首次显示图像时才创建原生窗口句柄。
            cv2.imshow(self.window, self._frame())
            cv2.waitKey(1)
            if not window_is_visible(self.window):
                return
            cv2.setMouseCallback(self.window, self._mouse)
            cv2.setWindowTitle(self.window, self.window_title)
            while self.running:
                self._update_rates()
                cv2.imshow(self.window, self._frame())
                key = cv2.waitKey(1) & 0xFF
                if not window_is_visible(self.window):
                    break
                if key in (ord("q"), 27):
                    break
                if self._handle_forward_key(key):
                    continue
                movement = key_movement(key)
                if movement:
                    self.client.send(0xB1, p1=movement[0], p2=movement[1])
        finally:
            self.close()

    def close(self):
        self.running = False
        self.forwarder.close()
        self.client.close()
        self.video.close()
        cv2.destroyAllWindows()


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_PATH))
    parser.add_argument("--forward-host", default=None,
                        help="发送艾晟报文的默认目的 IP，覆盖 INI [forward].ip")
    parser.add_argument("--forward-port", type=int, default=None,
                        help="发送艾晟报文的默认目的 UDP 端口，覆盖 INI [forward].port")
    parser.add_argument("--preflight", action="store_true",
                        help="只检查NVIDIA H.264编解码能力")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        config = with_forward_override(load(args.config), args.forward_host, args.forward_port)
        elements = require_nvcodec()
        print(f"H.264硬件预检通过：encoder={elements.encoder} decoder={elements.decoder}")
        if args.preflight:
            return 0
        if cv2 is None or np is None:
            raise RuntimeError(f"地面站图形依赖不可用：{CV_IMPORT_ERROR}")
        station = GroundStation(config)
        previous_sigterm = signal.signal(
            signal.SIGTERM, lambda _signum, _frame: setattr(station, "running", False))
        try:
            station.run()
        finally:
            signal.signal(signal.SIGTERM, previous_sigterm)
    except (ValueError, RuntimeError, HardwareVideoError) as exc:
        print(f"A2416A地面站启动失败：{exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
