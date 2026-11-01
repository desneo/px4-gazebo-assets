#!/usr/bin/env python3
"""ASD-2-075-DD0 标准二进制 UDP 网关。

控制帧 45 字节、遥测帧 104 字节；Gazebo 插件仍使用内部 JSON 话题。
可选把内部遥测转换为 PX4 dyt_udp_bridge 的 19 字段 JSON。
"""
from __future__ import annotations

import argparse
import json
import math
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


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class Gateway:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.node = Node()
        self.command_pub = self.node.advertise(
            f"{args.topic_prefix}/command", StringMsg)

        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.control_sock.bind((args.control_host, args.control_port))
        self.control_sock.settimeout(0.2)
        self.telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)

        self.lock = threading.Lock()
        self.running = True
        self.latest: dict = {}
        self.latest_mono = 0.0
        self.last_control: proto.ControlFrame | None = None
        self.last_control_mono = 0.0
        self.locked = True
        self.auto_lock = False
        self.tracking_command = False
        self.goto_target: tuple[float, float] | None = None
        self.lock_after_goto = False
        self.manual_until = 0.0
        self.manual_active = False
        self.zoom = 1
        self.active_lens = "wide"
        self.rx_telemetry = 0
        self.tx_telemetry = 0
        self.rx_control = 0
        self.tx_errors = 0
        self.last_status_log = time.monotonic()
        self.last_result_error = 0.0
        self.last_discrete = None
        self.last_discrete_time = 0.0
        self.px4_tx = 0
        self.px4_errors = 0
        self.px4_sender = None
        if args.px4_json_port:
            if (not math.isfinite(args.px4_json_rate)
                    or args.px4_json_rate <= 0):
                raise ValueError("PX4 JSON 频率必须为有限正数")
            from gateway.px4_forward import Px4JsonSender
            self.px4_sender = Px4JsonSender(
                args.px4_json_host, args.px4_json_port)
            print(f"PX4 JSON：{args.px4_json_host}:{args.px4_json_port}，"
                  f"{args.px4_json_rate:g} Hz", flush=True)
        self.node.subscribe(
            StringMsg, f"{args.topic_prefix}/telemetry", self._on_telemetry)
        self.node.subscribe(
            StringMsg, f"{args.topic_prefix}/command_result", self._on_result)
        self.image_topic = args.image_topic or f"{args.topic_prefix}/camera"
        self.tele_image_topic = (
            args.tele_image_topic or f"{args.topic_prefix}/camera_tele")
        self.video_senders = []
        self.glib_loop = None
        if args.video:
            from gi.repository import GLib
            from gateway.video_tx import VideoSender
            self.glib_loop = GLib.MainLoop()
            threading.Thread(target=self.glib_loop.run, daemon=True).start()
            self.video_senders = [
                VideoSender(self.node, self.image_topic,
                            args.video_host, args.wide_video_port,
                            width=1920, height=1080, fps=30,
                            bitrate_kbps=args.bitrate),
                VideoSender(self.node, self.tele_image_topic,
                            args.video_host, args.tele_video_port,
                            width=1920, height=1080, fps=30,
                            bitrate_kbps=args.bitrate),
            ]
            print(f"RTP 视频：广角 {args.video_host}:{args.wide_video_port}，"
                  f"长焦 {args.video_host}:{args.tele_video_port}", flush=True)

    # ---------------- Gazebo 回调 ----------------
    def _on_telemetry(self, msg: StringMsg) -> None:
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            self.latest = data
            self.latest_mono = time.monotonic()
            self.rx_telemetry += 1
        frame = self._build_telemetry(data)
        self._send_telemetry(frame)
        if self.args.verbose and time.monotonic() - self.last_status_log >= 2.0:
            with self.lock:
                rx = self.rx_telemetry
                tx = self.tx_telemetry
                locked = self.locked
                auto = self.auto_lock
                tracking = self.tracking_command
                px4_tx = self.px4_tx
                px4_errors = self.px4_errors
            print(f"telemetry rx={rx} tx={tx} locked={locked} "
                  f"auto={auto} tracking_cmd={tracking} "
                  f"px4_tx={px4_tx} px4_err={px4_errors}", flush=True)
            self.last_status_log = time.monotonic()

    def _on_result(self, msg: StringMsg) -> None:
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not data.get("ok", False):
            now = time.monotonic()
            if now - self.last_result_error >= 1.0:
                self.last_result_error = now
                print(f"插件拒绝命令 {data.get('cmd')}: {data.get('message')}",
                      flush=True)

    # ---------------- 遥测打包 ----------------
    def _build_telemetry(self, data: dict) -> proto.TelemetryFrame:
        with self.lock:
            locked = self.locked
            auto = self.auto_lock
            last_control = self.last_control
            zoom = self.zoom
            active_lens = self.active_lens
        joints = data.get("joints_deg", {})
        try:
            yaw = float(joints["yaw"]["pos"])
            pitch = float(joints["pitch"]["pos"])
            gyro_yaw = float(joints["yaw"]["vel"])
            gyro_pitch = float(joints["pitch"]["vel"])
        except (KeyError, TypeError, ValueError):
            yaw = pitch = gyro_yaw = gyro_pitch = 0.0
        tracking = data.get("tracking", {})
        target = data.get("target", {})
        miss = target.get("miss", {})
        tracking_valid = (
            data.get("mode") == "TRACK"
            and tracking.get("state") == "TRACK"
            and bool(target.get("valid"))
            and bool(miss.get("valid"))
        )
        box = target.get("box_px")
        if tracking_valid and isinstance(box, list) and len(box) == 4:
            gate_x = int(round(box[0] + box[2] / 2.0))
            gate_y = int(round(box[1] + box[3] / 2.0))
            gate_w = int(round(box[2]))
            gate_h = int(round(box[3]))
        else:
            gate_x = gate_y = gate_w = gate_h = 0
        offset = miss.get("offset_deg") if miss.get("valid") else None
        if isinstance(offset, list) and len(offset) == 2:
            yaw_mis = float(offset[0])
            pitch_mis = -float(offset[1])
        else:
            yaw_mis = pitch_mis = 0.0
        los = data.get("target_los", {})
        if los.get("valid"):
            yaw_los = float(los.get("yaw_rate_dps") or 0.0)
            pitch_los = float(los.get("pitch_rate_dps") or 0.0)
        else:
            yaw_los = pitch_los = 0.0
        servo = 1 if locked else (6 if tracking_valid else 4)
        status_control = (servo & 0x7) << 5
        work_state = 0x08 | 0x20 | (0x01 if tracking_valid else 0)
        if self.args.video:
            work_state |= 0x80
        target_type = 4 if tracking_valid else 0
        target_confidence = 0.8 if tracking_valid else 0.0
        zoom_bits = {1: 1, 2: 2, 4: 3}.get(zoom, 1)
        mode_bits = 3 if auto else (1 if tracking_valid else 0)
        status2 = (zoom_bits & 0x3) | ((mode_bits & 0x3) << 2)
        if active_lens == "tele":
            status2 |= 0x10
        frame = data.get("frame", {})
        timestamp_ms = int(float(data.get("sim_uptime", 0.0)) * 1000.0) & 0xFFFFFFFF
        aircraft_pitch = last_control.aircraft_pitch_deg if last_control else 0.0
        aircraft_roll = last_control.aircraft_roll_deg if last_control else 0.0
        aircraft_heading = last_control.aircraft_heading_deg if last_control else 0.0
        return proto.TelemetryFrame(
            self_test=1 if tracking.get("fault") is None else 2,
            work_state=work_state,
            pitch_deg=pitch, yaw_deg=yaw,
            gate_x=gate_x, gate_y=gate_y,
            gyro_pitch_dps=gyro_pitch, gyro_yaw_dps=gyro_yaw,
            pitch_misalignment_deg=pitch_mis,
            yaw_misalignment_deg=yaw_mis,
            tracking_confidence=25.5 if tracking_valid else 0.0,
            pitch_los_dps=pitch_los, yaw_los_dps=yaw_los,
            status_control=status_control,
            autonomous=1 if auto else 0,
            target_type=target_type,
            target_confidence=target_confidence,
            status_control2=status2,
            gate_w=gate_w, gate_h=gate_h,
            pitch_att_deg=aircraft_pitch,
            yaw_att_deg=aircraft_heading,
            roll_att_deg=aircraft_roll,
            tracking_target_type=target_type,
            aircraft_pitch_deg=aircraft_pitch,
            aircraft_roll_deg=aircraft_roll,
            aircraft_heading_deg=aircraft_heading,
            ip=(8, 2),
            timestamp_ms=timestamp_ms,
        )

    def _send_telemetry(self, frame: proto.TelemetryFrame) -> None:
        try:
            self.telemetry_sock.sendto(
                proto.encode_telemetry(frame),
                (self.args.telemetry_host, self.args.telemetry_port))
            with self.lock:
                self.tx_telemetry += 1
        except OSError:
            with self.lock:
                self.tx_errors += 1

    # ---------------- 控制处理 ----------------
    def _publish(self, payload: dict) -> None:
        self.command_pub.publish(
            StringMsg(data=json.dumps(payload, separators=(",", ":"))))

    def _on_control(self, data: bytes) -> None:
        try:
            frame = proto.decode_control(data)
        except ValueError as exc:
            if self.args.verbose:
                print(f"丢弃控制帧：{exc}", flush=True)
            return
        if frame.command != proto.CMD_FRAME_MOVE:
            key = (frame.command, frame.p1, frame.p2, frame.p3, frame.p4)
            now = time.monotonic()
            if key == self.last_discrete and now - self.last_discrete_time < 0.2:
                return
            self.last_discrete = key
            self.last_discrete_time = now
        with self.lock:
            self.last_control = frame
            self.last_control_mono = time.monotonic()
            self.rx_control += 1
        self._handle(frame)

    def _handle(self, frame: proto.ControlFrame) -> None:
        cmd = frame.command
        if cmd == 0x00:
            return
        if cmd == proto.CMD_FRAME_UNLOCK:
            with self.lock:
                self.locked = False
                self.manual_active = False
            self._publish({"cmd": "stop"})
            self._log("电锁解除")
        elif cmd == proto.CMD_FRAME_LOCK:
            self._publish({"cmd": "stop"})
            with self.lock:
                self.locked = True
                self.auto_lock = False
                self.tracking_command = False
                self.goto_target = None
                self.manual_active = False
            self._log("框架电锁")
        elif cmd in (proto.CMD_SPECIFIED_ANGLE, proto.CMD_SPECIFIED_ANGLE_LOCK):
            self._goto(frame.p1 / 100.0, frame.p2 / 100.0,
                       cmd == proto.CMD_SPECIFIED_ANGLE_LOCK)
        elif cmd == proto.CMD_RECENTER:
            if self.locked:
                self._log("电锁状态下拒绝归中")
                return
            self._publish({"cmd": "home"})
            with self.lock:
                self.goto_target = None
                self.auto_lock = False
                self.tracking_command = False
                self.manual_until = 0.0
                self.manual_active = False
            self._log("框架归中")
        elif cmd == proto.CMD_POINT_LOCK:
            if self.locked:
                self._log("电锁状态下拒绝点选锁定")
                return
            self._point_lock(frame.p1, frame.p2)
        elif cmd == proto.CMD_AUTO_LOCK_ON:
            if self.locked:
                self._log("电锁状态下拒绝自动锁定")
                return
            with self.lock:
                self.auto_lock = True
                self.manual_active = False
            self._try_auto_lock()
            self._log("自动锁定开启")
        elif cmd == proto.CMD_AUTO_LOCK_OFF:
            self._publish({"cmd": "stop"})
            with self.lock:
                self.auto_lock = False
                self.tracking_command = False
                self.manual_active = False
            self._log("自动锁定关闭")
        elif cmd == proto.CMD_CANCEL_LOCK:
            self._publish({"cmd": "stop"})
            with self.lock:
                self.auto_lock = False
                self.tracking_command = False
                self.manual_active = False
            self._log("取消锁定")
        elif cmd == proto.CMD_FRAME_MOVE:
            if self.locked:
                self._log("电锁状态下拒绝框架移动")
                return
            yaw = clamp(frame.p1 / 100.0 * 60.0, -60.0, 60.0)
            pitch = clamp(frame.p2 / 100.0 * 60.0, -60.0, 60.0)
            with self.lock:
                starting = not self.manual_active
                self.manual_active = True
                self.goto_target = None
                self.auto_lock = False
                self.tracking_command = False
                self.manual_until = time.monotonic() + 0.5
            if starting:
                self._publish({"cmd": "stop"})
            self._publish({"cmd": "manual", "yaw": yaw, "pitch": pitch})
        elif cmd == proto.CMD_ZOOM_IN:
            with self.lock:
                self.zoom = min(4, self.zoom * 2)
            self._log(f"广角变倍 {self.zoom}x")
        elif cmd == proto.CMD_ZOOM_OUT:
            with self.lock:
                self.zoom = max(1, self.zoom // 2)
            self._log(f"广角变倍 {self.zoom}x")
        elif cmd == proto.CMD_DETECTOR_SWITCH:
            lens = "wide" if frame.p1 != 2 else "tele"
            self._publish({"cmd": "stop"})
            self._publish({"cmd": "set_camera", "sensor": lens})
            with self.lock:
                self.active_lens = lens
                self.auto_lock = False
                self.tracking_command = False
                self.goto_target = None
                self.manual_until = 0.0
                self.manual_active = False
            self._log(f"激活镜头 {lens}")
        else:
            self._log(f"未实现命令 0x{cmd:02X}")

    def _goto(self, yaw: float, pitch: float, lock_after: bool) -> None:
        if self.locked:
            self._log("电锁状态下拒绝指定角度")
            return
        self._publish({"cmd": "stop"})
        with self.lock:
            self.goto_target = (clamp(yaw, -60.0, 60.0), clamp(pitch, -80.0, 30.0))
            self.lock_after_goto = lock_after
            self.auto_lock = False
            self.tracking_command = False
            self.manual_until = 0.0
            self.manual_active = False
        self._log(f"指定角度 yaw={yaw:.2f} pitch={pitch:.2f}")

    def _image_size(self) -> tuple[int, int]:
        with self.lock:
            data = self.latest
        frame = data.get("frame", {}) if data else {}
        return int(frame.get("width") or 1920), int(frame.get("height") or 1080)

    def _candidates(self) -> list[dict]:
        with self.lock:
            data = self.latest
        candidates = data.get("truth_candidates", []) if data else []
        width, height = self._image_size()
        visible = []
        for candidate in candidates:
            center = candidate.get("center_px", [])
            if len(center) == 2 and 0 <= center[0] < width and 0 <= center[1] < height:
                visible.append(candidate)
        return visible

    def _point_lock(self, x: int, y: int) -> None:
        candidates = self._candidates()
        if not candidates:
            self._log("点选锁定失败：当前没有真值候选")
            return
        def dist2(c: dict) -> float:
            center = c.get("center_px", [0, 0])
            return (float(center[0]) - x) ** 2 + (float(center[1]) - y) ** 2
        best = min(candidates, key=dist2)
        if dist2(best) > self.args.click_radius ** 2:
            self._log("点选锁定失败：点击位置附近没有飞机")
            return
        self._start_track(best, auto=False)

    def _try_auto_lock(self) -> None:
        candidates = self._candidates()
        if not candidates:
            return
        width, height = self._image_size()
        def dist2(c: dict) -> float:
            center = c.get("center_px", [0, 0])
            return (float(center[0]) - width / 2.0) ** 2 + (float(center[1]) - height / 2.0) ** 2
        self._start_track(min(candidates, key=dist2), auto=True)

    def _start_track(self, candidate: dict, auto: bool) -> None:
        with self.lock:
            data = self.latest
            active = self.active_lens
        plugin_active = data.get("camera", {}).get("active") if data else None
        if plugin_active and plugin_active != active:
            self._log(f"镜头切换未完成（插件 {plugin_active} / 请求 {active}）")
            return
        width, height = self._image_size()
        center = candidate.get("center_px", [width / 2.0, height / 2.0])
        box_w = min(self.args.track_box_w, width)
        box_h = min(self.args.track_box_h, height)
        x = clamp(float(center[0]) - box_w / 2.0, 0.0, width - box_w)
        y = clamp(float(center[1]) - box_h / 2.0, 0.0, height - box_h)
        box = [x / width, y / height, box_w / width, box_h / height]
        self._publish({"cmd": "track", "box": box})
        with self.lock:
            self.tracking_command = True
            self.auto_lock = auto
            self.goto_target = None
            self.manual_until = 0.0
            self.manual_active = False
        self._log(f"{'自动' if auto else '点选'}锁定 {candidate.get('name')} "
                  f"center={center}")

    # ---------------- 后台循环 ----------------
    def _goto_loop(self) -> None:
        while self.running:
            time.sleep(0.05)
            with self.lock:
                target = self.goto_target
                locked = self.locked
                tracking = self.tracking_command
                data = self.latest
            if target is None or locked or tracking:
                continue
            joints = data.get("joints_deg", {}) if data else {}
            try:
                yaw = float(joints["yaw"]["pos"])
                pitch = float(joints["pitch"]["pos"])
            except (KeyError, TypeError, ValueError):
                continue
            ey = target[0] - yaw
            ep = target[1] - pitch
            if abs(ey) < 0.2 and abs(ep) < 0.2:
                self._publish({"cmd": "manual", "yaw": 0.0, "pitch": 0.0})
                with self.lock:
                    lock_after = self.lock_after_goto
                    self.goto_target = None
                    self.lock_after_goto = False
                    if lock_after:
                        self.locked = True
                self._log("指定角度到位" + ("，已电锁" if lock_after else ""))
            else:
                self._publish({
                    "cmd": "manual",
                    "yaw": clamp(2.0 * ey, -60.0, 60.0),
                    "pitch": clamp(2.0 * ep, -60.0, 60.0),
                })

    def _manual_watchdog(self) -> None:
        while self.running:
            time.sleep(0.05)
            with self.lock:
                until = self.manual_until
            if until and time.monotonic() > until:
                self._publish({"cmd": "manual", "yaw": 0.0, "pitch": 0.0})
                with self.lock:
                    self.manual_until = 0.0
                    self.manual_active = False

    def _control_timeout_loop(self) -> None:
        while self.running:
            time.sleep(0.2)
            with self.lock:
                last = self.last_control_mono
                tracking = self.tracking_command
                auto = self.auto_lock
            if last and time.monotonic() - last > 2.0 and (tracking or auto):
                self._publish({"cmd": "stop"})
                with self.lock:
                    self.tracking_command = False
                    self.auto_lock = False
                    self.goto_target = None
                    self.manual_until = 0.0
                    self.manual_active = False
                self._log("控制超时，停止跟踪并保持角度")

    def _auto_lock_loop(self) -> None:
        while self.running:
            time.sleep(0.2)
            with self.lock:
                enabled = self.auto_lock
                tracking = self.tracking_command
                data = self.latest
            if not enabled:
                continue
            state = data.get("tracking", {}).get("state") if data else None
            mode = data.get("mode") if data else None
            if tracking and mode == "TRACK" and state in ("INIT", "TRACK"):
                continue
            if tracking and state in ("LOST", "FAULT", "IDLE"):
                with self.lock:
                    self.tracking_command = False
            self._try_auto_lock()

    def _px4_loop(self) -> None:
        period = 1.0 / max(1.0, self.args.px4_json_rate)
        next_send = time.monotonic()
        last_error_log = 0.0
        while self.running:
            now = time.monotonic()
            if now < next_send:
                time.sleep(min(next_send - now, 0.01))
                continue
            next_send = max(next_send + period, now)
            with self.lock:
                data = self.latest
                fresh = data and now - self.latest_mono < 0.5
            if not fresh:
                continue
            try:
                self.px4_sender.send(data)
                with self.lock:
                    self.px4_tx += 1
            except (ValueError, OSError) as exc:
                with self.lock:
                    self.px4_errors += 1
                if self.args.verbose and now - last_error_log >= 2.0:
                    last_error_log = now
                    print(f"PX4 JSON 发送失败：{exc}", flush=True)

    def _log(self, message: str) -> None:
        if self.args.verbose:
            print(message, flush=True)

    def run(self) -> None:
        threads = [
            threading.Thread(target=self._goto_loop, daemon=True),
            threading.Thread(target=self._manual_watchdog, daemon=True),
            threading.Thread(target=self._auto_lock_loop, daemon=True),
            threading.Thread(target=self._control_timeout_loop, daemon=True),
        ]
        if self.px4_sender is not None:
            threads.append(threading.Thread(target=self._px4_loop, daemon=True))
        for thread in threads:
            thread.start()
        print(f"网关启动：控制 UDP {self.args.control_host}:{self.args.control_port}，"
              f"遥测 -> {self.args.telemetry_host}:{self.args.telemetry_port}", flush=True)
        try:
            while self.running:
                try:
                    data, _addr = self.control_sock.recvfrom(2048)
                except socket.timeout:
                    continue
                self._on_control(data)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            for sender in self.video_senders:
                sender.close()
            if self.glib_loop is not None:
                self.glib_loop.quit()
            self.control_sock.close()
            self.telemetry_sock.close()
            if self.px4_sender is not None:
                self.px4_sender.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ASD-2-075-DD0 二进制 UDP 网关")
    parser.add_argument("--control-host", default="0.0.0.0")
    parser.add_argument("--control-port", type=int, default=8790)
    parser.add_argument("--telemetry-host", default="127.0.0.1")
    parser.add_argument("--telemetry-port", type=int, default=8791)
    parser.add_argument("--topic-prefix", default="/aisheng_seeker")
    parser.add_argument("--image-topic", default="",
                        help="广角图像话题；默认 <topic-prefix>/camera")
    parser.add_argument("--tele-image-topic", default="",
                        help="长焦图像话题；默认 <topic-prefix>/camera_tele")
    parser.add_argument("--px4-json-host", default="127.0.0.1")
    parser.add_argument("--px4-json-port", type=int, default=0,
                        help="19 字段 PX4 JSON 目标端口；0 关闭转发")
    parser.add_argument("--px4-json-rate", type=float, default=50.0,
                        help="PX4 JSON 发送频率 Hz（默认：50）")
    parser.add_argument("--click-radius", type=float, default=80.0,
                        help="双击命中真值候选的最大像素距离")
    parser.add_argument("--track-box-w", type=float, default=80.0)
    parser.add_argument("--track-box-h", type=float, default=60.0)
    parser.add_argument("--video", action="store_true", help="启用双路 RTP 视频")
    parser.add_argument("--video-host", default="127.0.0.1")
    parser.add_argument("--wide-video-port", type=int, default=7001)
    parser.add_argument("--tele-video-port", type=int, default=7002)
    parser.add_argument("--bitrate", type=int, default=8000)
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    Gateway(parse_args()).run()
