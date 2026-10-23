#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两轴跟踪吊舱 独立地面站（运营端）。

自包含的独立程序：通过配置的 IP/端口接收 RTSP 视频、显示 UDP 遥测、
下发跟踪框与控制指令。运行不依赖本仓库 `server/`、`client/` 目录。

功能：
- RTSP 强制使用 ``nvh264dec`` 硬解播放，不允许软件解码回退；
- 显式选择关闭视觉/目标检测/自动锁定/手动框选，四种模式互斥；
- 仅在手动框选模式下允许鼠标拖拽，松开提交 ``track``（归一化到整帧 0..1）；
- 显示实时跟踪框、跟踪状态、稳定性、两轴机械姿态/角速度、限位/饱和、
  目标中心角误差、端到端时延（用 ``frame_table``）、显示帧率与丢帧率；
- 按钮与快捷键：复位 reset / 停止 stop / 朝天 home / 清故障 reset_fault，
  二维 yaw/pitch 虚拟摇杆，方向键手动指向；
- 连接状态指示、UDP 登记/续期/重登记与视频断线自动重连。

无显示器/自检模式：
- ``--selftest DURATION``：不依赖 GUI，收 UDP 遥测并打印摘要；
- ``--decode-test SECONDS --snapshot PATH``：不依赖 GUI，验证 RTSP 解码与帧序号。

通信协议遵循 ``models/gimbals/two_axis_pod/docs/control-protocol.md``：一个 UDP 数据报 = 一条 UTF-8 JSON，
客户端以 ``ping`` 登记/续期，有回执指令按 ``seq`` 重传去重，``manual`` 以
≥10 Hz 连续发送。用法示例见 README.md。
"""
from __future__ import annotations

import argparse
import configparser
import ctypes
import ctypes.util
import json
import math
import os
import socket
import statistics
import sys
import threading
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst  # noqa: E402

    Gst.init(None)
    _HAVE_GST = True
    _GST_ERR = ""
except Exception as _exc:  # noqa: BLE001 - 供 selftest 在无 GStreamer 时也能运行
    _HAVE_GST = False
    _GST_ERR = str(_exc)

# 本程序自包含：stamp.py 与主程序同目录，直接导入（不 import server/）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stamp import decode_stamp_from_bgr  # noqa: E402
from telemetry_forward import TelemetryForwarder, ForwardInputs, endpoint, save_destination  # noqa: E402
from packet_rate import PacketRate  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(HERE, "config.ini")

# 地面站只允许 NVIDIA NVDEC。``nvenc`` 是旧配置中误用的名称，仅作为
# 向后兼容别名，二者都严格映射到同一个硬解元件，不构成软件回退。
DECODER_ELEMENTS = {"nvdec": "nvh264dec", "nvenc": "nvh264dec"}

VISION_MODES = ("off", "detect", "auto_lock", "manual_track")
VISION_MODE_CN = {
    "off": "关闭视觉",
    "detect": "目标检测",
    "auto_lock": "自动锁定",
    "manual_track": "手动框选",
}

# models/gimbals/two_axis_pod/docs/control-protocol.md：单条数据报上限 8 KiB；接收缓冲区建议 ≥ 64 KiB。
MAX_DATAGRAM = 8 * 1024
RECV_BUFFER = 65535
SO_RCVBUF_MIN = 1 << 20          # 要求 SO_RCVBUF ≥ 1 MiB

TRACK_COLORS = {
    "IDLE": (150, 150, 150),
    "INIT": (0, 220, 220),
    "TRACK": (0, 255, 0),
    "TEMP_LOST": (0, 200, 255),
    "LOST": (0, 0, 255),
    "FAULT": (0, 0, 255),
}
STABILITY_CN = {"STABLE": "稳定", "UNSTABLE": "不稳定", "UNDETERMINED": "不可判定"}
STATE_CN = {"IDLE": "未跟踪", "INIT": "初始化", "TRACK": "跟踪中",
            "TEMP_LOST": "暂时丢失", "LOST": "已丢失", "FAULT": "故障"}

FONT_CANDIDATES = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
]


# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------
DEFAULT_CONFIG_VALUES = {
    "gimbal_host": "127.0.0.1",
    "udp_port": 8766,
    "rtsp_port": 8555,
    "rtsp_mount": "/two_axis_pod",
    "aircraft_id": 1,
    "display_scale": 0.5,
    "decoder": "nvdec",
    "window_title": "Gimbal Ground Station",
    "client_timeout": 5.0,
    "retry_interval": 0.2,
    "retry_max": 5,
    "reconnect": True,
    "font_path": "",
    "rtsp_transport": "tcp",
    "bridge_manage": False,
    "bridge_world": "two_axis_pod_demo",
    "bridge_model": "two_axis_pod",
    "bridge_topic_prefix": "/two_axis_pod",
    "bridge_image_topic": "",
    "bridge_gz_partition": "",
    "forward_ip": "127.0.0.1",
    "forward_port": 9000,
}

INI_CONFIG_FIELDS = {
    "forward": {
        "ip": ("forward_ip", str),
        "port": ("forward_port", int),
    },
    "bridge": {
        "manage": ("bridge_manage", bool),
        "world": ("bridge_world", str),
        "model": ("bridge_model", str),
        "topic_prefix": ("bridge_topic_prefix", str),
        "image_topic": ("bridge_image_topic", str),
        "gz_partition": ("bridge_gz_partition", str),
    },
    "network": {
        "gimbal_host": ("gimbal_host", str),
        "udp_port": ("udp_port", int),
        "rtsp_port": ("rtsp_port", int),
        "aircraft_id": ("aircraft_id", int),
        "rtsp_mount": ("rtsp_mount", str),
    },
    "video": {
        "display_scale": ("display_scale", float),
        "decoder": ("decoder", str),
        "rtsp_transport": ("rtsp_transport", str),
        "window_title": ("window_title", str),
        "font_path": ("font_path", str),
    },
    "session": {
        "client_timeout": ("client_timeout", float),
        "retry_interval": ("retry_interval", float),
        "retry_max": ("retry_max", int),
        "reconnect": ("reconnect", bool),
    },
}

# 命令行 -> 配置键（后项覆盖前项，例如 --udp-host 优先于 --host）
CLI_TO_CONFIG = [
    ("host", "gimbal_host"),
    ("udp_host", "gimbal_host"),
    ("udp_port", "udp_port"),
    ("rtsp_port", "rtsp_port"),
    ("mount", "rtsp_mount"),
    ("aircraft_id", "aircraft_id"),
    ("display_scale", "display_scale"),
    ("decoder", "decoder"),
    ("window_title", "window_title"),
    ("client_timeout", "client_timeout"),
    ("retry_interval", "retry_interval"),
    ("retry_max", "retry_max"),
]


def load_config(path: str) -> dict:
    cfg = dict(DEFAULT_CONFIG_VALUES)
    if path and os.path.splitext(path)[1].lower() != ".ini":
        raise ValueError(f"Station 配置仅支持 INI 格式: {path}")
    if path and os.path.isfile(path):
        try:
            parser = configparser.ConfigParser(interpolation=None)
            with open(path, "r", encoding="utf-8") as stream:
                parser.read_file(stream)
            data = {}
            unknown_sections = set(parser.sections()) - set(INI_CONFIG_FIELDS)
            if unknown_sections:
                raise ValueError("未知配置节: " + ", ".join(sorted(unknown_sections)))
            for section, fields in INI_CONFIG_FIELDS.items():
                if not parser.has_section(section):
                    continue
                unknown = set(parser.options(section)) - set(fields)
                if unknown:
                    raise ValueError(
                        f"[{section}] 未知配置项: " + ", ".join(sorted(unknown)))
                for option, (key, value_type) in fields.items():
                    if not parser.has_option(section, option):
                        continue
                    if value_type is bool:
                        value = parser.getboolean(section, option)
                    elif value_type is int:
                        value = parser.getint(section, option)
                    elif value_type is float:
                        value = parser.getfloat(section, option)
                    else:
                        value = parser.get(section, option)
                    data[key] = value
            cfg.update({key: value for key, value in data.items() if value is not None})
        except (OSError, ValueError, configparser.Error) as exc:
            print(f"[config] 读取 {path} 失败，使用内置默认值: {exc}", file=sys.stderr)
    elif path:
        print(f"[config] 未找到 {path}，使用内置默认值", file=sys.stderr)
    return cfg


def apply_cli_overrides(cfg: dict, args) -> dict:
    for attr, key in CLI_TO_CONFIG:
        val = getattr(args, attr, None)
        if val is not None:
            cfg[key] = val
    if args.font:
        cfg["font_path"] = args.font
    if getattr(args, "no_manage_bridge", False):
        cfg["bridge_manage"] = False
    return cfg


def udp_endpoint(cfg: dict) -> str:
    return f"udp://{cfg['gimbal_host']}:{cfg['udp_port']}"


def rtsp_url(cfg: dict) -> str:
    mount = "/" + str(cfg.get("rtsp_mount") or "/two_axis_pod").strip("/")
    aircraft_id = cfg.get("aircraft_id", 1)
    if type(aircraft_id) is not int or aircraft_id < 0:
        raise ValueError("aircraft_id 必须为非负整数")
    return f"rtsp://{cfg['gimbal_host']}:{cfg['rtsp_port']}{mount}_{aircraft_id}"


def find_font(cfg: dict):
    candidates = []
    if cfg.get("font_path"):
        candidates.append(cfg["font_path"])
    candidates.extend(FONT_CANDIDATES)
    for p in candidates:
        try:
            if p and os.path.isfile(p):
                return p
        except OSError:
            continue
    return None


# ---------------------------------------------------------------------------
# UDP 客户端：登记/续期 + 遥测接收 + 指令重传（models/gimbals/two_axis_pod/docs/control-protocol.md）
# ---------------------------------------------------------------------------
class UdpClient(threading.Thread):
    """UDP 控制/遥测客户端（一个数据报 = 一条 JSON，models/gimbals/two_axis_pod/docs/control-protocol.md）。

    - 本地 ``bind`` 临时端口，向网关 ``host:port`` 发送；
    - 接收独立线程：``recvfrom(65535)``，解析后按 ``type`` 分发；
    - 每 ``ping_interval``（默认 1 s）发一次 ping：登记 + 续期 + 链路探测；
    - 有回执指令按 ``retry_interval`` 重发**同一 seq**，直到收到
      ``command_result`` 或累计 ``retry_max`` 次后判定失败；
    - ``connected`` = 最近 ``client_timeout`` 秒内收到过服务端消息。
    """

    def __init__(self, host, port, on_log=None, *, client_timeout=5.0,
                 retry_interval=0.2, retry_max=5, ping_interval=1.0,
                 bind_host="0.0.0.0", aircraft_id=1):
        super().__init__(daemon=True)
        if type(aircraft_id) is not int or aircraft_id < 0:
            raise ValueError("aircraft_id 必须为非负整数")
        self.aircraft_id = aircraft_id
        self.host = host
        self.port = int(port)
        self.target = (host, int(port))
        self.clock_domain = self._local_clock_domain()
        self.on_log = on_log or (lambda _s: None)
        self.client_timeout = float(client_timeout)
        self.retry_interval = float(retry_interval)
        self.retry_max = max(1, int(retry_max))
        self.ping_interval = float(ping_interval)

        self.lock = threading.RLock()
        self.telemetry: dict = {}
        self.frame_t0: dict = {}
        self.last_result = None          # 最近一条 command_result
        self.results = []                # 最近若干 command_result（诊断）
        self.pongs = 0
        self.hello = None                # 收到的 hello（视为已登记）
        self.errors = []
        self.messages = 0                # 收到的 telemetry 计数
        self._receive_rate = PacketRate()
        self.last_rx_mono = 0.0          # 最近收到服务端消息的单调时刻
        self.last_telemetry_mono = 0.0
        self.pings_sent = 0              # 已发送 ping 次数（兼作重登记探测）

        self._seq = 0                    # 单调递增序号（models/gimbals/two_axis_pod/docs/control-protocol.md）
        self._pending = {}               # seq -> 待回执指令状态
        self._stop = threading.Event()

        self.sock = None
        self.local_addr = None
        self.bind_error = None
        try:
            self.target = (socket.gethostbyname(host), self.port)
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF,
                                     max(SO_RCVBUF_MIN, RECV_BUFFER))
            except OSError:
                pass
            self.sock.bind((bind_host, 0))       # 临时端口
            self.sock.settimeout(0.2)
            self.local_addr = self.sock.getsockname()
        except OSError as exc:
            self.bind_error = str(exc)

    # --------------------------------------------------------------- 发送
    def next_seq(self) -> int:
        with self.lock:
            self._seq += 1
            return self._seq

    def _raw_send(self, payload: dict) -> bool:
        with self.lock:
            return self._raw_send_locked(payload)

    def _raw_send_locked(self, payload: dict) -> bool:
        if not self._public_command_allowed(payload):
            return False
        if self.sock is None:
            return False
        try:
            data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        except (TypeError, ValueError):
            return False
        if len(data) > MAX_DATAGRAM:
            self.on_log(f"[udp] 丢弃超长数据报 {len(data)}B")
            return False
        try:
            self.sock.sendto(data, self.target)
            return True
        except OSError as exc:
            self.on_log(f"[udp] 发送失败: {exc}")
            return False

    def send_once(self, cmd: str, **extra) -> int:
        """无回执指令（manual / reset_fault / status / bye）；返回 seq。"""
        seq = self.next_seq()
        if not self._raw_send(dict(cmd=cmd, seq=seq, **extra)):
            return -1
        return seq

    def send_reliable(self, cmd: str, **extra) -> int:
        """有回执指令（含 set_vision_mode）：带 seq 发送并按需重传。"""
        if not self._public_command_allowed(dict(cmd=cmd, **extra)):
            self.on_log(f"[udp] 拒绝 {cmd}: {self.identity_error or '机械姿态未就绪或指令参数非法'}")
            return -1
        seq = self.next_seq()
        payload = dict(cmd=cmd, seq=seq, **extra)
        with self.lock:
            self._pending[seq] = {"payload": payload, "t": time.monotonic(),
                                  "sends": 1, "result": None}
        self._raw_send(payload)
        self.on_log(f"[udp] 发送 {cmd} seq={seq}（等待 command_result）")
        return seq

    def send_manual(self, yaw=0.0, pitch=0.0) -> int:
        """manual：无回执，以 ≥10 Hz 连续发送（见 ManualController）。"""
        return self.send_once("manual", yaw=float(yaw), pitch=float(pitch))

    @property
    def identity_error(self):
        """身份来自当前会话 hello；仅收到 pong 或遥测不能解除控制锁定。"""
        with self.lock:
            hello = getattr(self, "hello", None) or {}
            expected = getattr(self, "aircraft_id", 1)
            actual = hello.get("aircraft_id")
            if not hello:
                return f"等待飞机 {expected} 的 hello 身份确认，控制禁用"
            if type(actual) is not int or actual < 0:
                return f"网关未提供有效飞机 ID，期望 {expected}，控制禁用"
            if actual != expected:
                return f"飞机 ID 不一致：期望 {expected}，网关 {actual}，控制禁用"
            if not self.connected:
                return "身份会话已超时，等待重新登记，控制禁用"
            return None

    @property
    def identity_ready(self):
        return self.identity_error is None

    @property
    def public_angles_ready(self):
        """仅在版本、角度标记和新鲜公共姿态均确认后允许角度控制。"""
        with self.lock:
            hello = self.hello or {}
            tele = self.telemetry or {}
            fresh = time.monotonic() - getattr(self, "last_telemetry_mono", 0.0) <= self.client_timeout
            return bool(self.identity_ready and fresh and type(hello.get("version")) is int and hello.get("version") == 7
                        and type(tele.get("protocol_version")) is int and tele.get("protocol_version") == 7
                        and hello.get("angle_convention") == "motion_head_joint_right_positive"
                        and tele.get("angle_convention") == "motion_head_joint_right_positive"
                        and self._valid_attitude(tele.get("attitude_deg")))

    @staticmethod
    def _valid_attitude(attitude):
        if not isinstance(attitude, dict):
            return False
        for axis in ("yaw", "pitch"):
            values = attitude.get(axis)
            if not isinstance(values, dict):
                return False
            for key in ("pos", "vel", "acc"):
                value = values.get(key)
                if type(value) not in (int, float) or not -float("inf") < value < float("inf"):
                    return False
        return True

    @staticmethod
    def _valid_vision(vision):
        if not isinstance(vision, dict):
            return False
        if "mode" in vision and vision["mode"] not in VISION_MODES:
            return False
        if "state" in vision and not isinstance(vision["state"], str):
            return False
        if "error" in vision and vision["error"] is not None and not isinstance(vision["error"], str):
            return False
        if "target_count" in vision and (type(vision["target_count"]) is not int
                                          or vision["target_count"] < 0):
            return False
        if "locked_track_id" in vision and (vision["locked_track_id"] is not None
                                              and type(vision["locked_track_id"]) is not int):
            return False
        for key in ("inference_ms", "input_fps", "output_fps"):
            value = vision.get(key)
            if value is not None and (type(value) not in (int, float)
                                      or not 0 <= value < float("inf")):
                return False
        return True

    def _public_command_allowed(self, payload):
        cmd = payload.get("cmd")
        if cmd not in ("ping", "status", "bye") and not self.identity_ready:
            return False
        if cmd == "set_vision_mode" and payload.get("mode") not in VISION_MODES:
            return False
        if "roll" in payload or (cmd == "reset_axis" and payload.get("axis") not in ("yaw", "pitch")):
            return False
        sensitive = (cmd in ("home", "reset", "reset_axis", "track")
                     or (cmd == "set_vision_mode" and payload.get("mode") == "auto_lock")
                     or (cmd == "manual" and any(payload.get(axis, 0) != 0 for axis in ("yaw", "pitch"))))
        if not sensitive:
            return True
        if not self.public_angles_ready:
            return False
        return True

    def _send_ping(self):
        seq = self.next_seq()
        self.pings_sent += 1
        self._raw_send({"cmd": "ping", "seq": seq, "t": round(time.time(), 3)})

    # --------------------------------------------------------- 后台内务线程
    def _housekeeping_loop(self):
        next_ping = 0.0
        was_connected = False
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_ping:
                self._send_ping()            # 登记 + 续期 + 链路探测
                next_ping = now + self.ping_interval

            conn = self.connected
            if conn != was_connected:
                was_connected = conn
                if conn:
                    self.on_log("[udp] 链路已恢复（重新登记成功）")
                else:
                    self.on_log("[udp] 链路超时，地面站判定未连接；"
                                "持续重发 ping 以重新登记 ...")

            resend = []
            failed = []
            with self.lock:
                for seq, p in list(self._pending.items()):
                    if p["result"] is not None:
                        continue
                    if now - p["t"] < self.retry_interval:
                        continue
                    if p["sends"] >= self.retry_max:
                        failed.append((seq, p["payload"].get("cmd")))
                    else:
                        p["sends"] += 1
                        p["t"] = now
                        resend.append(seq)
            for seq in resend:
                with self.lock:
                    p = self._pending.get(seq)
                if p is not None:
                    self._raw_send(p["payload"])
            for seq, cmd in failed:
                with self.lock:
                    self._pending.pop(seq, None)
                self.on_log(f"[udp] 指令失败（重传 {self.retry_max} 次无回执）: "
                            f"{cmd} seq={seq}")

            self._stop.wait(0.02)

    @property
    def connected(self) -> bool:
        return (self.last_rx_mono > 0
                and (time.monotonic() - self.last_rx_mono) <= self.client_timeout)

    @property
    def registered(self) -> bool:
        with self.lock:
            return self.hello is not None

    # --------------------------------------------------------------- 接收
    def run(self):
        if self.sock is None:
            self.on_log(f"[udp] 初始化失败，无法接收: {self.bind_error}")
            return
        self._receive_rate.reset()
        la = self.local_addr
        try:
            rcvbuf = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        except OSError:
            rcvbuf = -1
        self.on_log(f"[udp] 本地绑定 {la[0]}:{la[1]} -> 网关 "
                    f"{self.host}:{self.port}（SO_RCVBUF={rcvbuf}）")
        threading.Thread(target=self._housekeeping_loop, daemon=True).start()
        while not self._stop.is_set():
            try:
                data, addr = self.sock.recvfrom(RECV_BUFFER)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    break
                continue
            try:
                self._on_datagram(data, addr)
            except Exception as exc:  # noqa: BLE001 - 单条异常不应终止接收
                self.on_log(f"[udp] 处理数据报异常: {exc}")
        try:
            self.sock.close()
        except OSError:
            pass

    @staticmethod
    def _local_clock_domain():
        try:
            with open("/proc/sys/kernel/random/boot_id", encoding="ascii") as stream:
                return stream.read().strip() or None
        except OSError:
            return None

    def _on_datagram(self, data: bytes, addr=None):
        with self.lock:
            self._on_datagram_locked(data, addr)

    def _on_datagram_locked(self, data: bytes, addr=None):
        if addr is not None and addr != self.target:
            return
        if len(data) > MAX_DATAGRAM:
            return
        try:
            text = data.decode("utf-8").strip()      # 忽略首尾空白/结尾换行
        except UnicodeDecodeError:
            return
        if not text:
            return
        try:
            msg = json.loads(text)
        except json.JSONDecodeError:
            return
        if not isinstance(msg, dict):
            return
        mtype = msg.get("type")
        if mtype not in ("telemetry", "hello", "command_result", "pong", "error"):
            return
        if mtype == "command_result":
            if (type(msg.get("seq")) is not int
                    or not isinstance(msg.get("cmd"), str)
                    or type(msg.get("ok")) is not bool):
                return
        if mtype == "hello" and ("clock_domain" in msg
                                  and msg["clock_domain"] is not None
                                  and not isinstance(msg["clock_domain"], str)):
            return
        if mtype == "telemetry":
            for key in ("tracking", "stability", "joints_deg", "attitude_deg", "limits", "cmd_dps", "video", "vision"):
                if key in msg and not isinstance(msg[key], dict):
                    return
            if "attitude_deg" in msg and not self._valid_attitude(msg["attitude_deg"]):
                return
            if "vision" in msg and not self._valid_vision(msg["vision"]):
                return
            table = msg.get("frame_table", [])
            if not isinstance(table, list):
                return
            for entry in table:
                if (not isinstance(entry, list) or len(entry) != 2
                        or type(entry[0]) is not int
                        or type(entry[1]) not in (int, float)
                        or not 0 <= entry[1] < float("inf")):
                    return

        with self.lock:
            if (getattr(self, "hello", None) is not None
                    and not self.connected):
                self.hello = None
                self._pending.clear()
        self.last_rx_mono = time.monotonic()
        if mtype == "telemetry":
            with self.lock:
                self.telemetry = msg
                self.last_telemetry_mono = time.monotonic()
                self.messages += 1
                if self.public_angles_ready:
                    self._receive_rate.record(self.last_telemetry_mono)
                for entry in msg.get("frame_table") or []:
                    if isinstance(entry, (list, tuple)) and len(entry) >= 2:
                        self.frame_t0[entry[0]] = entry[1]
                if len(self.frame_t0) > 3000:
                    for k in sorted(self.frame_t0)[:-1500]:
                        self.frame_t0.pop(k, None)
            callback = getattr(self, "on_telemetry", None)
            if callback is not None and self.identity_ready:
                callback(data)
        elif mtype == "hello":
            with self.lock:
                self.hello = msg
                if not self.public_angles_ready:
                    self._pending.clear()
                    self.telemetry = {}
                    self.last_telemetry_mono = 0.0
            self.on_log(f"[udp] 收到 hello: telemetry_hz={msg.get('telemetry_hz')} "
                        f"video_rtsp_path={msg.get('video_rtsp_path')} "
                        f"aircraft_id={msg.get('aircraft_id')} -> "
                        f"{self.identity_error or '身份已确认'}")
        elif mtype == "command_result":
            seq = msg.get("seq")
            with self.lock:
                self.last_result = msg
                self.results.append(msg)
                if len(self.results) > 50:
                    del self.results[:25]
                p = self._pending.get(seq)
                if p is not None and ("payload" not in p
                                      or p["payload"].get("cmd") == msg["cmd"]):
                    self._pending.pop(seq, None)
            if not msg.get("ok"):
                self.on_log(f"[udp] 指令失败 cmd={msg.get('cmd')} seq={seq}: "
                            f"{msg.get('message')}")
        elif mtype == "pong":
            with self.lock:
                self.pongs += 1
        elif mtype == "error":
            with self.lock:
                self.errors.append(msg)
            self.on_log(f"[udp] 协议错误: {msg.get('message')}")

    # --------------------------------------------------------------- 查询
    def snapshot(self) -> dict:
        with self.lock:
            return dict(self.telemetry)

    def lookup(self, seq):
        with self.lock:
            return self.frame_t0.get(seq)

    @property
    def shared_clock_domain(self):
        with self.lock:
            domain = self.hello.get("clock_domain") if self.hello else None
            return bool(self.clock_domain and domain == self.clock_domain)

    def frame_delay_ms(self, seq, now):
        """仅在同一主机启动时钟域内计算单调时钟时延。"""
        with self.lock:
            if not self.shared_clock_domain:
                return None
            t0 = self.frame_t0.get(seq)
            if t0 is None:
                return None
            delay = now - t0
            return delay * 1000.0 if 0.0 <= delay < 3.0 else None

    def stats(self) -> dict:
        with self.lock:
            rate = self._receive_rate.snapshot()
            target = (self.hello or {}).get("telemetry_hz")
            try:
                target = float(target) if type(target) in (int, float) else None
            except OverflowError:
                target = None
            if target is not None and not 0 < target < float("inf"):
                target = None
            secs = ((time.monotonic() - self.last_rx_mono)
                    if self.last_rx_mono else None)
            return {
                "connected": self.connected,
                "registered": self.registered,
                "identity_ready": self.identity_ready,
                "identity_error": self.identity_error,
                "messages": self.messages,
                "telemetry_target_hz": target,
                "receive_hz": rate["hz"],
                "receive_rate_warming_up": rate["warming_up"],
                "rate_window_s": rate["window_s"],
                "pongs": self.pongs,
                "pings_sent": self.pings_sent,
                "seconds_since_rx": secs,
                "pending": len(self._pending),
                "last_result": self.last_result,
                "results": list(self.results),
                "errors": list(self.errors),
            }

    def find_result(self, cmd=None, seq=None):
        with self.lock:
            for r in reversed(self.results):
                if cmd is not None and r.get("cmd") != cmd:
                    continue
                if seq is not None and r.get("seq") != seq:
                    continue
                return r
            return None

    def stop(self):
        self._stop.set()
        self._receive_rate.reset(active=False)


class ManualController(threading.Thread):
    """把按键脉冲和摇杆输入变成 ≥10 Hz 的 manual 流。

    按轴记录 ``(角速度, 到期时刻)``；每 tick 组合两轴发送；全部到期后再补发
    一条 0 速度指令。持续按住时反复触发 ``hold`` 可续期，从而一直发送。
    """

    def __init__(self, client: UdpClient, rate=10.0):
        super().__init__(daemon=True)
        self.client = client
        self.period = 1.0 / max(1.0, float(rate))
        self.lock = threading.Lock()
        self.axes = {}          # axis -> (value, expire_mono)
        self._was_active = False
        self._stop_event = threading.Event()

    def hold(self, axis, value, duration=0.4):
        if axis not in ("yaw", "pitch"):
            raise ValueError("两轴吊舱只支持 yaw、pitch")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("手动角速度必须为有限数值")
        value = max(-20.0, min(20.0, value))
        with self.lock:
            self.axes[axis] = (value, time.monotonic() + float(duration))

    def cancel(self):
        """取消所有手动输入并立即归零；与发送线程串行，避免归零后残留非零包。"""
        with self.lock:
            active = bool(self.axes) or self._was_active
            self.axes.clear()
            self._was_active = False
            if active:
                self.client.send_manual(0.0, 0.0)

    def run(self):
        while not self._stop_event.is_set():
            now = time.monotonic()
            with self.lock:
                self.axes = {a: v for a, v in self.axes.items() if v[1] > now}
                vals = {a: v[0] for a, v in self.axes.items()}
                active = bool(vals)
                if active or self._was_active:
                    self.client.send_manual(vals.get("yaw", 0.0),
                                            vals.get("pitch", 0.0))
                self._was_active = active
            self._stop_event.wait(self.period)

    def stop(self):
        self._stop_event.set()
        self.cancel()


class PointerGuard:
    """X11 下检查窗口焦点和物理左键，捕获窗口外松手及切换窗口。

    其他窗口系统返回 None，由 GUI 使用短输入租约，避免永久保持角速度。
    """

    def __init__(self, title):
        self.title = title
        self.display = None
        try:
            self.lib = ctypes.CDLL(ctypes.util.find_library("X11"))
            self.lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
            self.lib.XOpenDisplay.restype = ctypes.c_void_p
            self.lib.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
            self.lib.XDefaultRootWindow.restype = ctypes.c_ulong
            self.lib.XGetInputFocus.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_int)]
            self.lib.XFetchName.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_char_p)]
            self.lib.XQueryTree.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)), ctypes.POINTER(ctypes.c_uint)]
            self.lib.XQueryPointer.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_uint)]
            self.lib.XFree.argtypes = [ctypes.c_void_p]
            self.lib.XCloseDisplay.argtypes = [ctypes.c_void_p]
            self.display = self.lib.XOpenDisplay(None)
        except (OSError, TypeError, AttributeError):
            self.display = None

    def held_in_window(self):
        if not self.display:
            return None
        focus, revert = ctypes.c_ulong(), ctypes.c_int()
        self.lib.XGetInputFocus(self.display, ctypes.byref(focus), ctypes.byref(revert))
        window = focus.value
        matched = False
        for _ in range(16):
            if window <= 1:
                break
            name = ctypes.c_char_p()
            if self.lib.XFetchName(self.display, window, ctypes.byref(name)) and name.value:
                matched = name.value.decode("utf-8", errors="replace") == self.title
                self.lib.XFree(name)
            if matched:
                break
            root, parent, children, count = ctypes.c_ulong(), ctypes.c_ulong(), ctypes.POINTER(ctypes.c_ulong)(), ctypes.c_uint()
            if not self.lib.XQueryTree(self.display, window, ctypes.byref(root), ctypes.byref(parent), ctypes.byref(children), ctypes.byref(count)):
                break
            if children:
                self.lib.XFree(children)
            if parent.value == window:
                break
            window = parent.value
        root, child, mask = ctypes.c_ulong(), ctypes.c_ulong(), ctypes.c_uint()
        rx, ry, wx, wy = (ctypes.c_int() for _ in range(4))
        ok = self.lib.XQueryPointer(self.display, self.lib.XDefaultRootWindow(self.display), ctypes.byref(root), ctypes.byref(child), ctypes.byref(rx), ctypes.byref(ry), ctypes.byref(wx), ctypes.byref(wy), ctypes.byref(mask))
        return bool(ok and matched and mask.value & (1 << 8))

    def close(self):
        if self.display:
            self.lib.XCloseDisplay(self.display)
            self.display = None


# ---------------------------------------------------------------------------
# RTSP 解码管线
# ---------------------------------------------------------------------------
def build_pipeline(rtsp: str, decoder: str = "nvdec", verbose: bool = True):
    """构造 RTSP -> BGR appsink 管线；只允许 NVIDIA ``nvh264dec``。"""
    if not _HAVE_GST:
        raise RuntimeError(f"GStreamer/PyGObject 不可用: {_GST_ERR}")
    if decoder not in DECODER_ELEMENTS:
        raise RuntimeError(f"拒绝非硬件解码配置 {decoder!r}；地面站仅允许 nvh264dec")
    elem = DECODER_ELEMENTS[decoder]
    if Gst.ElementFactory.find(elem) is None:
        raise RuntimeError("缺少 NVIDIA 硬解元件 nvh264dec；请安装并启用 GStreamer NVIDIA 插件")
    desc = (
        f"rtspsrc location={rtsp} latency=0 drop-on-latency=true protocols=udp "
        f"! rtph264depay ! h264parse ! {elem} ! videoconvert "
        f"! video/x-raw,format=BGR ! appsink name=sink max-buffers=1 drop=true"
    )
    try:
        pipeline = Gst.parse_launch(desc)
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(f"NVDEC RTSP 管线创建失败: {exc}") from exc
    sink = pipeline.get_by_name("sink")
    if sink is None:
        raise RuntimeError("NVDEC RTSP 管线创建失败：appsink 缺失")
    if verbose:
        print(f"[video] RTSP 管线就绪: {rtsp} (强制硬解 {elem})")
    return pipeline, sink, "nvdec"


# ---------------------------------------------------------------------------
# GUI 地面站
# ---------------------------------------------------------------------------
class GroundStation:
    def __init__(self, cfg: dict, args):
        self.cfg = cfg
        self.args = args
        self.win = cfg.get("window_title") or "Gimbal Ground Station"
        self.scale = float(cfg.get("display_scale", 0.5) or 0.5)
        if not (0.05 <= self.scale <= 2.0):
            self.scale = 0.5

        self.tele = UdpClient(
            cfg.get("gimbal_host", "127.0.0.1"), cfg.get("udp_port", 8766),
            on_log=lambda s: print(s, flush=True),
            client_timeout=float(cfg.get("client_timeout", 5.0)),
            retry_interval=float(cfg.get("retry_interval", 0.2)),
            retry_max=int(cfg.get("retry_max", 5)),
            aircraft_id=cfg.get("aircraft_id", 1))
        self.manual = ManualController(self.tele)
        self.forwarder = TelemetryForwarder()
        self.tele.on_telemetry = self.forwarder.offer
        self.forward_inputs = ForwardInputs(cfg.get("forward_ip", "127.0.0.1"),
                                            cfg.get("forward_port", 9000))
        self.forward_note = "未发送"
        self._forward_pointer = False
        self._forward_pressed = False
        self.pipeline = None
        self.sink = None
        self.decoder = None
        self.lock = threading.Lock()
        self.latest_frame = None
        self.frame_size = (1920, 1080)
        self.display_size = (int(1920 * self.scale), int(1080 * self.scale))
        self.last_frame_mono = time.monotonic()
        self._t_start = time.monotonic()
        self._stop = False
        self.fatal_video_error = ""

        self.mouse_start = None
        self.mouse_end = None
        self.drawing = False
        self.delays = []            # (t, delay_ms)
        self.frame_times = []       # (t, seq)
        self.last_seq = None
        self.drops = 0
        self.expected = 0
        self.last_cmd = ""
        self.last_disp = None
        self.buttons = []
        self.pressed = None
        self.hover = None
        self.joystick = None       # "aim"（yaw/pitch）
        self.joystick_values = (0.0, 0.0)
        self.joystick_event_at = 0.0
        self.pointer_guard = PointerGuard(self.win)

        font_path = find_font(cfg)
        self.font = None
        if font_path:
            try:
                self.font = ImageFont.truetype(font_path, 24)
            except Exception:  # noqa: BLE001
                self.font = None

    # ------------------------------------------------------------- video
    def start_video(self):
        self.pipeline, self.sink, self.decoder = build_pipeline(
            rtsp_url(self.cfg), self.cfg.get("decoder", "nvdec"))
        result = self.pipeline.set_state(Gst.State.PLAYING)
        if result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("NVDEC RTSP 管线启动失败；请检查 NVIDIA 驱动、权限和视频源")
        threading.Thread(target=self.pull_loop, daemon=True).start()
        threading.Thread(target=self.video_watchdog, daemon=True).start()

    def pull_loop(self):
        while not self._stop:
            sink = self.sink
            if sink is None:
                time.sleep(0.05)
                continue
            sample = sink.emit("try-pull-sample", 100_000_000)
            if sample is None:
                time.sleep(0.002)
                continue
            buf = sample.get_buffer()
            caps = sample.get_caps().get_structure(0)
            w = int(caps.get_value("width"))
            h = int(caps.get_value("height"))
            ok, mapinfo = buf.map(Gst.MapFlags.READ)
            now = time.monotonic()
            if ok:
                arr = np.ndarray(shape=(h, w, 3), dtype=np.uint8,
                                 buffer=mapinfo.data, strides=(w * 3, 3, 1))
                seq = decode_stamp_from_bgr(arr, w, h)
                new_disp = (int(round(w * self.scale)), int(round(h * self.scale)))
                frame = cv2.resize(arr, new_disp, interpolation=cv2.INTER_AREA)
                # 遮盖帧序号条（仅影响显示，保留解码结果）
                fh, fw = frame.shape[:2]
                strip_h = max(1, int(16 * fh / h))
                strip_w = min(fw, int(512 * fw / w))
                if strip_h + 1 < fh:
                    frame[fh - strip_h:fh, 0:strip_w] = frame[fh - strip_h - 1, 0:strip_w]
                buf.unmap(mapinfo)
                with self.lock:
                    self.latest_frame = frame
                    self.display_size = new_disp
                    self.last_frame_mono = now
                    if seq is not None:
                        if self.last_seq is not None:
                            gap = (seq - self.last_seq) & 0xFFFFFF
                            if gap == 0:
                                seq = None  # 重复帧
                            elif gap > 100:
                                pass  # 疑似误码，重新对齐
                            else:
                                self.drops += gap - 1
                                self.expected += gap
                        if seq is not None:
                            self.last_seq = seq
                            delay = self.tele.frame_delay_ms(seq, now)
                            if delay is not None:
                                self.delays.append((now, delay))
                            self.frame_times.append((now, seq))
                    # 限制内存
                    if len(self.delays) > 4000:
                        del self.delays[:2000]
                    if len(self.frame_times) > 4000:
                        del self.frame_times[:2000]

    def _recreate_pipeline(self):
        try:
            if self.pipeline is not None:
                self.pipeline.set_state(Gst.State.NULL)
        except Exception:  # noqa: BLE001
            pass
        try:
            pipeline, sink, dec = build_pipeline(
                rtsp_url(self.cfg), self.cfg.get("decoder", "nvdec"), verbose=False)
            self.pipeline = pipeline
            self.sink = sink
            self.decoder = dec
            result = pipeline.set_state(Gst.State.PLAYING)
            if result == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("重建后的 NVDEC 管线无法进入 PLAYING")
            print(f"[video] 已重建 RTSP 管线（解码 {dec}）", flush=True)
        except Exception as exc:  # noqa: BLE001
            self._fail_video(f"重建 NVDEC RTSP 管线失败: {exc}")

    def _fail_video(self, message):
        if not self.fatal_video_error:
            self.fatal_video_error = str(message)
            print(f"[video] 致命错误：{self.fatal_video_error}；地面站退出，不回退软件解码。",
                  file=sys.stderr, flush=True)
        self._stop = True

    def _poll_video_error(self):
        pipeline = self.pipeline
        if pipeline is None:
            return
        bus = pipeline.get_bus()
        message = bus.pop_filtered(Gst.MessageType.ERROR)
        if message is not None:
            error, debug = message.parse_error()
            detail = error.message
            if debug:
                detail += f" ({debug})"
            self._fail_video(f"NVDEC RTSP 管线运行失败: {detail}")

    def video_watchdog(self):
        """视频长时间无新帧时尝试 set NULL/PLAYING，仍无则重建管线。"""
        while not self._stop:
            time.sleep(1.0)
            self._poll_video_error()
            if self._stop:
                return
            if time.monotonic() - self._t_start < 6.0:
                continue
            with self.lock:
                last = self.last_frame_mono
            age = time.monotonic() - last
            if age <= 3.0:
                continue
            print(f"[video] {age:.1f}s 无新帧，尝试重启管线 ...", flush=True)
            try:
                if self.pipeline is not None:
                    self.pipeline.set_state(Gst.State.NULL)
                    time.sleep(0.3)
                    self.pipeline.set_state(Gst.State.PLAYING)
            except Exception:  # noqa: BLE001
                pass
            time.sleep(2.5)
            with self.lock:
                if self.last_frame_mono == last:
                    self._recreate_pipeline()
                    time.sleep(2.5)

    # ------------------------------------------------------------- mouse
    def _button_at(self, x, y):
        for label, rect, cmd, extra in self.buttons:
            if rect[0] <= x <= rect[0] + rect[2] and rect[1] <= y <= rect[1] + rect[3]:
                return (label, rect, cmd, extra)
        return None

    def on_mouse(self, event, x, y, flags, param):
        if self._forward_mouse(event, x, y):
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            hit = self._joystick_at(x, y)
            if hit:
                if not self.tele.public_angles_ready:
                    self.last_cmd = "未确认 两轴机械角度协议，摇杆已禁用"
                    return
                self._exit_tracking_for_manual()
                self.manual.cancel()
                self.joystick = hit
                self._update_joystick(x, y)
                return
            btn = self._button_at(x, y)
            if btn is not None:
                self.pressed = (btn[0], btn[2], x, y, btn[3])
                return
            if y >= self.display_size[1]:
                return
            if self._vision_mode() != "manual_track":
                self.mouse_start = self.mouse_end = None
                self.drawing = False
                self.last_cmd = "当前不是“手动框选”模式，已拒绝画框"
                return
            self.mouse_start = (x, y)
            self.mouse_end = (x, y)
            self.drawing = True
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.joystick:
                if flags & cv2.EVENT_FLAG_LBUTTON:
                    self._update_joystick(x, y)
                else:
                    self._release_joystick()
                return
            hit = self._button_at(x, y)
            self.hover = hit[0] if hit else None
            if self.drawing:
                self.mouse_end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            if self.joystick:
                self._release_joystick()
                return
            if self.pressed is not None:
                label, cmd, px, py, extra = self.pressed
                self.pressed = None
                hit = self._button_at(x, y)
                if hit is not None and hit[0] == label:
                    self._send_ctrl(cmd, f"已下发: {label}", **extra)
                return
            if self.drawing:
                self.mouse_end = (x, y)
                self.drawing = False
                self.submit_box()

    def _joystick_geometry(self):
        dw, dh = self.display_size
        radius = min(56, max(30, dw // 10))
        return ((dw // 2, dh + 105, radius),)

    def _joystick_at(self, x, y):
        cx, cy, radius = self._joystick_geometry()[0]
        return "aim" if math.hypot(x - cx, y - cy) <= radius else None

    def _update_joystick(self, x, y):
        cx, cy, radius = self._joystick_geometry()[0]
        dx, dy = (x - cx) / radius, (cy - y) / radius
        length = math.hypot(dx, dy)
        if length > 1.0:
            dx, dy = dx / length, dy / length
        def response(value):
            return math.copysign(max(0.0, (abs(value) - 0.08) / 0.92), value)
        self.joystick_values = (response(dx), response(dy))
        self.joystick_event_at = time.monotonic()
        self._refresh_joystick()

    def _refresh_joystick(self):
        if self.joystick:
            x, y = self.joystick_values
            self.manual.hold("yaw", 20.0 * x, 0.3)
            self.manual.hold("pitch", 20.0 * y, 0.3)

    def _release_joystick(self):
        self.joystick = None
        self.joystick_values = (0.0, 0.0)
        self.manual.cancel()

    def _check_joystick(self):
        if not self.joystick:
            return
        if not self.tele.public_angles_ready:
            self._release_joystick()
            self.last_cmd = "机械姿态协议失效，已停止手动输入"
            return
        held = self.pointer_guard.held_in_window()
        if held is False or (held is None and time.monotonic() - self.joystick_event_at > 0.3):
            self._release_joystick()
        else:
            self._refresh_joystick()

    def _send_ctrl(self, cmd, label=None, **extra):
        """下发控制指令：有回执指令重传，其余单发。"""
        self._release_joystick()
        if cmd == "reset_axis" and not self.tele.public_angles_ready:
            self.last_cmd = "未确认 两轴机械角度协议，单轴复位已禁用"
            return
        if cmd in ("stop", "home", "reset", "reset_axis", "reset_fault"):
            self._exit_tracking_for_manual()
        if cmd in ("track", "stop", "home", "reset", "reset_axis", "set_vision_mode"):
            self.tele.send_reliable(cmd, **extra)
        else:
            self.tele.send_once(cmd, **extra)
        if label:
            self.last_cmd = label

    def submit_box(self):
        if self._vision_mode() != "manual_track":
            self.mouse_start = self.mouse_end = None
            self.drawing = False
            self.last_cmd = "当前不是“手动框选”模式，已拒绝提交选框"
            return
        if not self.mouse_start or not self.mouse_end:
            return
        x0 = min(self.mouse_start[0], self.mouse_end[0])
        y0 = min(self.mouse_start[1], self.mouse_end[1])
        x1 = max(self.mouse_start[0], self.mouse_end[0])
        y1 = min(self.display_size[1], max(self.mouse_start[1], self.mouse_end[1]))
        if x1 - x0 < 6 or y1 - y0 < 6:
            return
        with self.lock:
            w, h = self.frame_size
            dw, dh = self.display_size
        sx = w / dw
        sy = h / dh
        box = [x0 * sx / w, y0 * sy / h, (x1 - x0) * sx / w, (y1 - y0) * sy / h]
        self.manual.cancel()
        self.tele.send_reliable("track", box=[round(v, 6) for v in box])
        self.last_cmd = "已提交选框，开始跟踪"
        self.mouse_start = None
        self.mouse_end = None

    def _vision_mode(self, telemetry=None):
        tele = self.tele.snapshot() if telemetry is None else telemetry
        vision = (tele or {}).get("vision") or {}
        mode = vision.get("mode", "off")
        return mode if mode in VISION_MODES else "off"

    def _single_target_overlay_active(self, telemetry):
        """手动画框或已选定自动目标时显示插件单目标后端的目标框。"""
        mode = self._vision_mode(telemetry)
        if mode == "manual_track":
            return True
        vision = (telemetry or {}).get("vision") or {}
        return mode == "auto_lock" and vision.get("locked_track_id") is not None

    def _exit_tracking_for_manual(self):
        """人工转动抢占自动/手动框选跟踪；先可靠切至 off。"""
        if self._vision_mode() in {"auto_lock", "manual_track"}:
            self.tele.send_reliable("set_vision_mode", mode="off")
            self.last_cmd = "人工控制：已退出目标跟踪"

    # ------------------------------------------------------------- ui
    def _forward_geometry(self):
        width = min(416, max(300, self.display_size[0] - 16))
        x = max(8, self.display_size[0] - width - 12)
        ip_width = width - 232
        return {"ip": (x, 92, ip_width, 32),
                "port": (x + ip_width + 6, 92, 64, 32),
                "toggle": (x + ip_width + 76, 92, 156, 32)}

    @staticmethod
    def _inside(rect, x, y):
        return rect[0] <= x < rect[0] + rect[2] and rect[1] <= y < rect[1] + rect[3]

    def _forward_mouse(self, event, x, y):
        if not hasattr(self, "forward_inputs"):
            return False
        geometry = self._forward_geometry()
        if event == cv2.EVENT_LBUTTONDOWN:
            hit = next((name for name, rect in geometry.items() if self._inside(rect, x, y)), None)
            self.forward_inputs.focus = None
            if hit is None:
                return False
            self._release_joystick()
            self._forward_pointer = True
            self._forward_pressed = hit == "toggle"
            if hit in ("ip", "port"):
                self._pause_forward_for_edit()
                self.forward_inputs.activate(hit)
            return True
        if event == cv2.EVENT_LBUTTONUP and self._forward_pointer:
            self._forward_pointer = False
            pressed, self._forward_pressed = self._forward_pressed, False
            if pressed and self._inside(geometry["toggle"], x, y):
                self._toggle_forward()
            return True
        return event == cv2.EVENT_MOUSEMOVE and self._forward_pointer

    def _pause_forward_for_edit(self):
        if self.forwarder.snapshot()["active"]:
            self.forwarder.stop()
            self.forward_note = "已暂停，修改后点击发送"

    def _handle_forward_key(self, key):
        inputs = getattr(self, "forward_inputs", None)
        if inputs is not None and inputs.focus is not None and key not in (-1, 0):
            self._pause_forward_for_edit()
        return inputs.key(key) if inputs is not None else False

    def _toggle_forward(self):
        self.forward_inputs.focus = None
        if self.forwarder.snapshot()["active"]:
            self.forwarder.stop()
            self.forward_note = "已停止发送"
            return
        try:
            if not self.tele.public_angles_ready:
                raise ValueError("请等待两轴协议 7 的有效姿态")
            ip, port = endpoint(self.forward_inputs.values["ip"], self.forward_inputs.values["port"])
            if (ip, port) == self.tele.target:
                raise ValueError("接收端不能是吊舱网关控制端口")
            self.forwarder.start(ip, port)
            self.forward_inputs.values.update(ip=ip, port=str(port))
            self.cfg.update(forward_ip=ip, forward_port=port)
            self.forward_note = "等待新遥测"
            try:
                save_destination(self.args.config, ip, port)
            except OSError as exc:
                self.forward_note = "已发送；地址保存失败"
                print(f"[forward] 地址保存失败: {exc}", flush=True)
        except (ValueError, OSError) as exc:
            self.forward_note = str(exc)

    def _draw_forward_panel(self, disp):
        if not hasattr(self, "forward_inputs"):
            return
        geometry = self._forward_geometry()
        status = self.forwarder.snapshot()
        for name, rect in geometry.items():
            x, y, width, height = rect
            active = self.forward_inputs.focus == name
            bg = (0, 110, 0) if name == "toggle" and status["active"] else (35, 35, 35)
            cv2.rectangle(disp, (x, y), (x + width, y + height), bg, -1)
            cv2.rectangle(disp, (x, y), (x + width, y + height),
                          (0, 230, 255) if active else (190, 190, 190), 1)
            if name != "toggle":
                value = self.forward_inputs.values[name]
                visible = value[-max(5, (width - 12) // 10):]
                color = (0, 230, 255) if active and self.forward_inputs.selected else (255, 255, 255)
                cv2.putText(disp, visible + ("|" if active and not self.forward_inputs.selected else ""),
                            (x + 5, y + 22), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)
        x = geometry["ip"][0]
        note = status["error"] or (f"发送中 {status['sent']} 包" if status["active"] and status["sent"] else self.forward_note)
        if status["active"] and self.forward_note == "已发送；地址保存失败":
            note = self.forward_note
        if status["active"] and (not self.tele.identity_ready
                                 or time.monotonic() - self.tele.last_telemetry_mono > 1):
            note = "等待有效新遥测"
        labels = [(geometry["toggle"][0] + 8, 94,
                   "停止发送" if status["active"] else "发送艾晟报文"),
                  (x, 130, note)]
        # 只绘制小区域，避免覆盖左侧姿态和脱靶量文字。
        if self.font is not None:
            pil = Image.fromarray(cv2.cvtColor(disp[64:159, x:], cv2.COLOR_BGR2RGB))
            painter = ImageDraw.Draw(pil)
            small_font = self.font.font_variant(size=18)
            for px, py, label in labels:
                painter.text((px - x, py - 64), label, font=small_font,
                             fill=(255, 150, 60) if status["error"] else (245, 245, 245))
            disp[64:159, x:] = cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)
        else:
            rect = geometry["toggle"]
            cv2.putText(disp, "STOP" if status["active"] else "SEND", (rect[0]+12, 114), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
        self._draw_rate_panel(disp, status)

    @staticmethod
    def _rate_panel_lines(receive, forward, width):
        target = receive.get("telemetry_target_hz")
        target = "未知" if target is None else f"{target:.1f} Hz"
        rx = f"{receive.get('receive_hz', 0.0):.1f} Hz"
        if receive.get("receive_rate_warming_up"):
            rx = "预热中"
        tx = f"{forward.get('send_hz', 0.0):.1f} Hz"
        if not forward["active"]:
            tx = "0.0 Hz（未发送）"
        elif forward.get("send_rate_warming_up"):
            tx = "预热中"
        first = f"目标 {target}   |   实际接收 {rx}"
        second = f"艾晟发送 {tx}"
        if width >= 760:
            return [f"{first}   |   {second}", "最近 2 秒真实时间 · 艾晟发送为本机成功发送速率"]
        return [first, f"{second}  ·  最近 2 秒真实时间"]

    def _draw_rate_panel(self, disp, forward):
        width, height = self.display_size
        top = height + 224
        if disp.shape[0] < top + 56:
            return
        receive = self.tele.stats()
        disp[top:top + 56] = 20
        labels = self._rate_panel_lines(receive, forward, width)
        if self.font is not None:
            pil = Image.fromarray(cv2.cvtColor(disp[top:top + 56], cv2.COLOR_BGR2RGB))
            painter = ImageDraw.Draw(pil)
            size = 18 if width >= 760 else 14
            font = self.font.font_variant(size=size)
            while size > 10 and max(painter.textbbox((0, 0), text, font=font)[2]
                                    for text in labels) > width - 24:
                size -= 1
                font = self.font.font_variant(size=size)
            for index, text in enumerate(labels):
                painter.text((12, 3 + index * 25), text, font=font, fill=(245, 245, 245))
            disp[top:top + 56] = cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)
        else:
            target = receive.get("telemetry_target_hz")
            target = "?" if target is None else f"{target:.1f}"
            rx = "WARMUP" if receive.get("receive_rate_warming_up") else f"{receive.get('receive_hz', 0.0):.1f} Hz"
            tx = ("0.0 Hz (OFF)" if not forward["active"] else
                  "WARMUP" if forward.get("send_rate_warming_up") else f"{forward.get('send_hz', 0.0):.1f} Hz")
            labels = [f"Target {target} Hz | Receive {rx}", f"Aisheng send {tx} | last 2 real seconds"]
            for index, text in enumerate(labels):
                scale = min(.5, (width - 24) / max(1, cv2.getTextSize(
                    text, cv2.FONT_HERSHEY_SIMPLEX, 1., 1)[0][0]))
                cv2.putText(disp, text, (12, top + 20 + index * 25),
                            cv2.FONT_HERSHEY_SIMPLEX, scale, (245, 245, 245), 1, cv2.LINE_AA)

    @staticmethod
    def _miss_lines(tele, fresh):
        target = (tele or {}).get("target") or {}
        miss = target.get("miss") or {}
        if not fresh or not target.get("valid") or not miss.get("valid"):
            return ["脱靶量: 无效（无新鲜有效跟踪目标）"]
        pixel, angle = miss.get("offset_px"), miss.get("offset_deg")
        if (not isinstance(pixel, list) or len(pixel) != 2
                or not isinstance(angle, list) or len(angle) != 2
                or not all(type(v) in (float, int) and math.isfinite(v) for v in pixel + angle)):
            return ["脱靶量: 不可用"]
        return [f"脱靶量 px: X {pixel[0]:+.1f}  Y {pixel[1]:+.1f}",
                f"脱靶量 °:  X {angle[0]:+.3f}  Y {angle[1]:+.3f}（右/下为正）"]

    def draw(self, frame, tele):
        w, h = self.frame_size
        disp = frame
        dw, dh = self.display_size
        st = self.tele.stats()

        # 视野中心十字
        cv2.drawMarker(disp, (dw // 2, dh // 2), (255, 255, 255),
                       cv2.MARKER_CROSS, 24, 1)

        # 拖拽选框
        if self.mouse_start and self.mouse_end:
            cv2.rectangle(disp, self.mouse_start, self.mouse_end, (255, 180, 0), 2)

        # 检测/搜索阶段的多目标框由服务端烧录为红色；手动框选或自动选定
        # 目标后，改画插件单目标后端的目标框。自动目标固定使用绿色。
        target = (tele or {}).get("target") or {}
        track = (tele or {}).get("tracking") or {}
        state = track.get("state", "IDLE")
        auto_single = (self._vision_mode(tele) == "auto_lock"
                       and ((tele or {}).get("vision") or {}).get("locked_track_id") is not None)
        color = (0, 255, 0) if auto_single else TRACK_COLORS.get(state, (200, 200, 200))
        if self._single_target_overlay_active(tele) and target.get("valid") and target.get("box_px"):
            bx, by, bw, bh = target["box_px"]
            sx, sy = dw / w, dh / h
            p0 = (int(bx * sx), int(by * sy))
            p1 = (int((bx + bw) * sx), int((by + bh) * sy))
            cv2.rectangle(disp, p0, p1, color, 2)
            cv2.drawMarker(disp, (int((bx + bw / 2) * sx), int((by + bh / 2) * sy)),
                           color, cv2.MARKER_CROSS, 14, 2)

        # 操作按钮
        dynamic_mount = ((tele or {}).get("mount") or {}).get("static") is False
        button_defs = [("复位", "reset"), ("停止", "stop"),
                       ("回零" if dynamic_mount else "朝天", "home"), ("清故障", "reset_fault")]
        bw, bh = 86, 34
        x0 = dw - len(button_defs) * (bw + 8) - 12
        self.buttons = []
        for i, (label, cmd) in enumerate(button_defs):
            rect = (x0 + i * (bw + 8), 10, bw, bh)
            self.buttons.append((label, rect, cmd, {}))
            bg = (70, 70, 70)
            if self.hover == label:
                bg = (110, 110, 110)
            if self.pressed is not None and self.pressed[0] == label:
                bg = (0, 130, 0)
            cv2.rectangle(disp, (rect[0], rect[1]),
                          (rect[0] + rect[2], rect[1] + rect[3]), bg, -1)
            cv2.rectangle(disp, (rect[0], rect[1]),
                          (rect[0] + rect[2], rect[1] + rect[3]), (200, 200, 200), 1)

        # 文本
        lines = []
        colors = []
        conn = "已连接" if st["connected"] else "未连接"
        conn_color = (0, 255, 0) if st["connected"] else (0, 0, 255)
        if tele:
            t = tele.get("tracking") or {}
            s = tele.get("stability") or {}
            ready = (self.tele.public_angles_ready and tele.get("angle_convention") == "motion_head_joint_right_positive"
                     and UdpClient._valid_attitude(tele.get("attitude_deg")))
            j = tele.get("attitude_deg") or {}
            def angle_line(axis):
                if not ready:
                    return f"{axis}: 机械姿态待确认"
                value = j[axis]
                velocity = f"{value['vel']:+7.2f}°/s" if value['vel'] is not None else "不可用（奇异点）"
                return f"{axis:5s} {value['pos']:+7.2f}°  角速度 {velocity}"
            limits = tele.get("limits") or {}
            lines = [
                f"状态: {STATE_CN.get(t.get('state'), t.get('state'))}   稳定性: {STABILITY_CN.get(s.get('state'), s.get('state'))}   模式: {tele.get('mode')}",
                f"目标中心误差: {s.get('center_err_deg', 0):.2f}°    机械输出: {tele.get('cmd_dps') or {}}",
                angle_line("yaw"), angle_line("pitch"),
            ]
            colors = [(0, 255, 255)] * len(lines)
            colors[0] = (0, 255, 255)
            vision = tele.get("vision") or {}
            vision_mode = self._vision_mode(tele)
            locked = vision.get("locked_track_id")
            lock_text = f"  LOCK #{locked}" if locked is not None else ""
            metric = lambda key: float(vision.get(key) or 0.0)
            lines.append(
                f"视觉: {VISION_MODE_CN[vision_mode]} / {vision.get('state', '未知')}"
                f"{lock_text}   目标 {vision.get('target_count', 0)}   "
                f"推理 {metric('inference_ms'):.1f} ms   "
                f"输入/输出 {metric('input_fps'):.1f}/{metric('output_fps'):.1f} fps"
            )
            colors.append((0, 255, 0) if not vision.get("error") else (0, 0, 255))
            miss_lines = self._miss_lines(tele, st["connected"]
                                          and time.monotonic() - self.tele.last_telemetry_mono <= 1)
            lines.extend(miss_lines)
            colors.extend([(0, 255, 255)] * len(miss_lines))
            if vision.get("error"):
                lines.append(f"视觉错误: {vision['error']}")
                colors.append((0, 0, 255))
            mode = (tele.get("mount") or {}).get("mode")
            mounting = {"upright": "底座向下立装（外轴沿安装 +X）"}.get(mode, "未知")
            if dynamic_mount and mode == "upright":
                mounting = "机载立装（外轴沿安装 +X）"
            lines.append(f"安装: {mounting}   机械角度: {'yaw 左负右正' if ready else '未确认，角度控制禁用'}")
            colors.append((0, 255, 255))
            if ready and tele.get("singular", False):
                lines.append("视线退化：光轴平行外轴时外轴不改变方向，机械控制仍可用")
                colors.append((0, 165, 255))
            if limits.get("saturated") or any(limits.get(k) for k in ("yaw", "pitch")):
                lines.append(f"限位/饱和: {limits}")
                colors.append((0, 165, 255))
            if t.get("fault"):
                lines.append(f"故障: {t.get('fault')}  {t.get('message') or ''}")
                colors.append((0, 0, 255))
            elif t.get("message"):
                lines.append(f"跟踪提示: {t['message']}")
                colors.append((0, 165, 255))
            if self.last_cmd:
                lines.append(self.last_cmd)
                colors.append((0, 255, 0))
            with self.lock:
                now_m = time.monotonic()
                recent = ([d for (t_, d) in self.delays if t_ > now_m - 5.0]
                          if self.tele.shared_clock_domain else [])
                n_frames = len([1 for (t_, _) in self.frame_times if t_ > now_m - 1.0])
                drops, expected = self.drops, self.expected
            if recent:
                mean = statistics.mean(recent)
                p95 = sorted(recent)[int(len(recent) * 0.95) - 1] if len(recent) > 2 else recent[-1]
                drop_rate = drops / expected * 100.0 if expected else 0.0
                lines.append(f"时延 mean {mean:5.1f} ms  p95 {p95:5.1f} ms   "
                             f"显示 {n_frames} fps   丢帧 {drop_rate:.1f}%")
                colors.append((0, 255, 0))
            else:
                lines.append(f"时延不可用   显示 {n_frames} fps")
                colors.append((0, 255, 255))
            lines.append(f"连接: {conn}   已登记: {'是' if st['registered'] else '否'}   "
                         f"({udp_endpoint(self.cfg)})")
            colors.append(conn_color)
        else:
            lines = [f"等待遥测...   连接: {conn}   ({udp_endpoint(self.cfg)})"]
            colors = [(0, 0, 255)]

        lines.insert(0, st["identity_error"] or f"飞机 {self.tele.aircraft_id} 身份已确认")
        colors.insert(0, (0, 0, 255) if st["identity_error"] else (0, 255, 0))

        if self.font is not None:
            top = min(dh, 8 + 30 * (len(lines) + 1) + 10)
            top = max(top, 60)
            crop = disp[0:top, :]
            pil = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil)
            for i, (text, c) in enumerate(zip(lines, colors)):
                draw.text((12, 8 + i * 30), text, font=self.font,
                          fill=(c[2], c[1], c[0]))
            for label, rect, cmd, extra in self.buttons:
                bbox = draw.textbbox((0, 0), label, font=self.font)
                tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
                draw.text((rect[0] + (rect[2] - tw) / 2 - bbox[0],
                           rect[1] + (rect[3] - th) / 2 - bbox[1]),
                          label, font=self.font, fill=(255, 255, 255))
            disp[0:top, :] = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)
        else:
            for i, (text, c) in enumerate(zip(lines, colors)):
                cv2.putText(disp, text, (12, 28 + i * 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2, cv2.LINE_AA)
        # 摇杆区独立于视频，避免遮挡选框；视频尺寸及 ROI 归一化不包含控制区。
        panel = np.full((280, dw, 3), 28, dtype=np.uint8)
        disp = np.vstack((disp, panel))
        mode_buttons = self._vision_mode_buttons()
        self.buttons.extend(mode_buttons)
        current_mode = ((tele or {}).get("vision") or {}).get("mode")
        for label, rect, cmd, extra in mode_buttons:
            active = extra["mode"] == current_mode
            color = ((0, 130, 0) if active else
                     ((110, 110, 110) if self.hover == label else (70, 70, 70)))
            cv2.rectangle(disp, (rect[0], rect[1]),
                          (rect[0] + rect[2], rect[1] + rect[3]), color, -1)
            cv2.rectangle(disp, (rect[0], rect[1]),
                          (rect[0] + rect[2], rect[1] + rect[3]), (200, 200, 200), 1)
        for name, (cx, cy, radius) in zip(("aim",), self._joystick_geometry()):
            cv2.circle(disp, (cx, cy), radius, (150, 150, 150), 1, cv2.LINE_AA)
            cv2.line(disp, (cx - radius, cy), (cx + radius, cy), (80, 80, 80), 1)
            if name == "aim":
                cv2.line(disp, (cx, cy - radius), (cx, cy + radius), (80, 80, 80), 1)
            x, y = self.joystick_values if self.joystick == name else (0.0, 0.0)
            cv2.circle(disp, (int(cx + x * radius), int(cy - y * radius)), 11, (0, 200, 255), -1, cv2.LINE_AA)
            cv2.putText(disp, "YAW / PITCH", (cx - radius, dh + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1, cv2.LINE_AA)
            cv2.putText(disp, "left -yaw / right +yaw", (max(4, cx - 100), dh + 171), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (190, 190, 190), 1, cv2.LINE_AA)
            if name == "aim":
                cv2.putText(disp, "+pitch", (cx + radius + 6, cy - radius + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (190, 190, 190), 1, cv2.LINE_AA)
                cv2.putText(disp, "-pitch", (cx + radius + 6, cy + radius), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (190, 190, 190), 1, cv2.LINE_AA)
        axis_buttons = self._axis_reset_buttons()
        self.buttons.extend(axis_buttons)
        for label, rect, cmd, extra in axis_buttons:
            active = self.pressed is not None and self.pressed[0] == label
            color = (0, 130, 0) if active else ((110, 110, 110) if self.hover == label else (70, 70, 70))
            if not self.tele.public_angles_ready:
                color = (40, 40, 40)
            cv2.rectangle(disp, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), color, -1)
            cv2.rectangle(disp, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (200, 200, 200), 1)
        if self.font is not None:
            pil = Image.fromarray(cv2.cvtColor(disp[dh:, :], cv2.COLOR_BGR2RGB))
            painter = ImageDraw.Draw(pil)
            for label, rect, cmd, extra in mode_buttons + axis_buttons:
                bbox = painter.textbbox((0, 0), label, font=self.font)
                tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
                painter.text((rect[0] + (rect[2] - tw) / 2 - bbox[0],
                              rect[1] - dh + (rect[3] - th) / 2 - bbox[1]),
                             label, font=self.font, fill=(255, 255, 255))
            disp[dh:, :] = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)
        else:
            for label, rect, cmd, extra in mode_buttons:
                cv2.putText(disp, extra["mode"].upper(), (rect[0] + 5, rect[1] + 21),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)
            for label, rect, cmd, extra in axis_buttons:
                cv2.putText(disp, extra["axis"].upper() + " RESET", (rect[0] + 6, rect[1] + 23), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        self._draw_forward_panel(disp)
        return disp

    def _vision_mode_buttons(self):
        dw, dh = self.display_size
        gap = 8
        width = min(130, max(20, (dw - 5 * gap) // 4))
        total = 4 * width + 3 * gap
        x0 = max(4, (dw - total) // 2)
        return [(VISION_MODE_CN[mode], (x0 + i * (width + gap), dh + 2, width, 28),
                 "set_vision_mode", {"mode": mode})
                for i, mode in enumerate(VISION_MODES)]

    def _axis_reset_buttons(self):
        dw, dh = self.display_size
        width, gap = min(140, (dw - 36) // 2), 12
        x0 = (dw - 2 * width - gap) // 2
        return [(axis.capitalize() + "复位", (x0 + i * (width + gap), dh + 182, width, 32), "reset_axis", {"axis": axis})
                for i, axis in enumerate(("yaw", "pitch"))]

    def _manual(self, axis, value, label="", duration=0.4):
        """方向键/按钮手动脉冲：持续 ~duration 秒以 ≥10 Hz 发送后归零。"""
        if not self.tele.public_angles_ready:
            self.manual.cancel()
            self.last_cmd = "未确认 两轴机械角度协议，手动控制已禁用"
            return
        if self.joystick:
            self._release_joystick()
        self._exit_tracking_for_manual()
        self.manual.hold(axis, value, duration)
        if label:
            self.last_cmd = label

    def run(self):
        if not _HAVE_GST:
            print(f"错误：GStreamer/PyGObject 不可用: {_GST_ERR}", file=sys.stderr)
            print("提示：纯遥测自检 --selftest 不需要 GStreamer；"
                  "--decode-test 与 GUI 需要 GStreamer。", file=sys.stderr)
            return 2
        self.tele.start()
        self.manual.start()
        try:
            self.start_video()
        except Exception as exc:  # noqa: BLE001
            self._fail_video(str(exc))
            return 2
        cv2.namedWindow(self.win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.win, self.on_mouse)
        print("地面站已启动：先选择“手动框选”，再拖拽鼠标画框；"
              "按钮[复位/停止/朝天/清故障]或按键 R/S/H/M；"
              "底部虚拟摇杆控制 yaw/pitch，松手停止；"
              "方向键手动 yaw/pitch；ESC 退出")
        self.last_draw_t = 0.0
        while not self._stop:
            if self.args.duration and time.monotonic() - self._t_start > self.args.duration:
                break
            with self.lock:
                new_frame = self.latest_frame is not None
                if new_frame:
                    self.last_frame = self.latest_frame
                    self.latest_frame = None
            tele = self.tele.snapshot()
            now = time.monotonic()
            self._check_joystick()
            if getattr(self, "last_frame", None) is not None and (new_frame or now - self.last_draw_t > 0.1):
                disp = self.draw(self.last_frame.copy(), tele)
                self.last_disp = disp
                cv2.imshow(self.win, disp)
                self.last_draw_t = now
                time.sleep(0.005)
            elif getattr(self, "last_frame", None) is None and now - self.last_draw_t > 0.1:
                blank = np.zeros((self.display_size[1], self.display_size[0], 3), dtype=np.uint8)
                cv2.putText(blank, "Waiting video...", (30, self.display_size[1] // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                self.last_disp = self.draw(blank, tele)
                cv2.imshow(self.win, self.last_disp)
                self.last_draw_t = now
                time.sleep(0.02)
            key = cv2.waitKeyEx(1)
            try:
                visible = cv2.getWindowProperty(self.win, cv2.WND_PROP_VISIBLE)
            except cv2.error:
                visible = 0
            if visible < 1:
                break
            if self._handle_forward_key(key):
                continue
            if key == 27:  # ESC
                break
            elif key in (ord('s'), ord('S')):
                self._send_ctrl("stop", "已停止跟踪")
            elif key in (ord('h'), ord('H')):
                self._send_ctrl("home", "返回朝天")
            elif key in (ord('m'), ord('M')):
                self._send_ctrl("reset_fault", "清除故障")
            elif key in (ord('r'), ord('R')):
                self._send_ctrl("reset", "复位：停止跟踪并朝天")
            elif key == 0 or key == -1:
                pass
            elif key in (2424832, 65361):   # left
                self._manual("yaw", -20, "手动: yaw -20°/s")
            elif key in (2555904, 65363):   # right
                self._manual("yaw", 20, "手动: yaw +20°/s")
            elif key in (2490368, 65362):   # up
                self._manual("pitch", 20, "手动: pitch +20°/s")
            elif key in (2621440, 65364):   # down
                self._manual("pitch", -20, "手动: pitch -20°/s")

        if self.args.gui_snapshot and self.last_disp is not None:
            cv2.imwrite(self.args.gui_snapshot, self.last_disp)
            print("界面截图:", self.args.gui_snapshot)
        return 2 if self.fatal_video_error else 0

    def shutdown(self):
        if getattr(self, "_shutdown_done", False):
            return
        self._shutdown_done = True
        self._stop = True
        forwarder = getattr(self, "forwarder", None)
        if forwarder is not None:
            forwarder.close()
        self._release_joystick()
        self.pointer_guard.close()
        try:
            if self.pipeline is not None:
                self.pipeline.set_state(Gst.State.NULL)
        except Exception:  # noqa: BLE001
            pass
        try:
            self.manual.stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.tele.send_once("bye")       # 可选：立即注销（models/gimbals/two_axis_pod/docs/control-protocol.md）
        except Exception:  # noqa: BLE001
            pass
        self.tele.stop()
        try:
            cv2.destroyAllWindows()
        except Exception:  # noqa: BLE001
            pass


# ---------------------------------------------------------------------------
# 自检：UDP 遥测/控制
# ---------------------------------------------------------------------------
def _selftest_timeout(seconds: float) -> int:
    """超时/重登记子测试：向一个无人监听的本地端口发送。

    验证：连续 ``client_timeout``（此处 1 s）无服务端消息时判定未连接，且客户端
    持续重发 ping（周期性重新登记），同时不会收到任何遥测。
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("127.0.0.1", 0))
        dead_port = probe.getsockname()[1]
    finally:
        probe.close()
    print(f"[selftest] 超时/重登记子测试：目标 127.0.0.1:{dead_port}（无人监听），"
          f"client_timeout=1.0s ...")
    client = UdpClient("127.0.0.1", dead_port, client_timeout=1.0,
                       ping_interval=0.5, retry_interval=0.2, retry_max=2,
                       on_log=lambda s: print("  " + s, flush=True))
    if client.bind_error:
        print(f"[selftest] 超时子测试：本地绑定失败 {client.bind_error}")
        return 7
    client.start()
    try:
        time.sleep(seconds if seconds > 0 else 2.5)
        connected = client.connected
        pings = client.pings_sent
        tele = client.snapshot()
    finally:
        client.stop()
    print(f"[selftest] 超时子测试：connected={connected}（期望 False）  "
          f"重登记 ping 次数={pings}（期望 ≥2）  遥测={'有' if tele else '无'}")
    ok = (not connected) and pings >= 2 and not tele
    print("[selftest] 超时/重登记子测试：" + ("通过" if ok else "失败"))
    return 0 if ok else 7


def run_selftest(cfg: dict, args) -> int:
    duration = float(args.selftest)
    client = UdpClient(
        cfg.get("gimbal_host", "127.0.0.1"), cfg.get("udp_port", 8766),
        on_log=lambda s: print(s, flush=True),
        client_timeout=float(cfg.get("client_timeout", 5.0)),
        retry_interval=float(cfg.get("retry_interval", 0.2)),
        retry_max=int(cfg.get("retry_max", 5)),
        aircraft_id=cfg.get("aircraft_id", 1))
    if client.bind_error:
        print(f"[selftest] 本地 UDP 套接字创建失败: {client.bind_error}")
        return 2
    print(f"[selftest] UDP 客户端 -> {udp_endpoint(cfg)}，采集 {duration:.1f}s ...")
    client.start()
    manual_ctrl = ManualController(client)
    manual_ctrl.start()

    n_tel = 0
    last = None
    hello = None
    status_seq = None
    home_seq = None
    manual_started = False
    manual_max = 0.0
    manual_mode_seen = False
    t0 = time.monotonic()

    while time.monotonic() - t0 < duration:
        elapsed = time.monotonic() - t0
        if status_seq is None and elapsed >= 0.4:
            status_seq = client.send_once("status")
        if args.send_home and home_seq is None and elapsed >= 0.6:
            home_seq = client.send_reliable("home")
        if args.send_manual and not manual_started and elapsed >= 1.0:
            manual_ctrl.hold("pitch", -20.0,
                             duration=min(1.0, max(0.3, duration - 1.2)))
            manual_started = True
        snap = client.snapshot()
        if snap:
            last = snap
            if manual_started:
                cd = (snap.get("attitude_deg") or {}).get("pitch") or {}
                try:
                    val = abs(float(cd.get("vel") or 0.0))
                except (TypeError, ValueError):
                    val = 0.0
                manual_max = max(manual_max, val)
                if snap.get("mode") == "MANUAL":
                    manual_mode_seen = True
        time.sleep(0.05)

    actual = time.monotonic() - t0
    manual_ctrl.stop()
    st = client.stats()
    n_tel = st["messages"]
    results = st["results"]
    pongs = st["pongs"]
    errors = st["errors"]
    hello = client.hello
    rate = n_tel / actual if actual > 0 else 0.0

    print("\n[selftest] ===== 摘要 =====")
    print(f"[selftest] 遥测条数 = {n_tel}  实测频率 ≈ {rate:.1f} Hz (窗口 {actual:.2f}s)")
    print(f"[selftest] 已登记(hello) = {'是' if hello else '否'}  "
          f"已连接 = {st['connected']}  pong = {pongs}  "
          f"ping 发送 = {st['pings_sent']}")
    print(f"[selftest] command_result = {len(results)}   待回执 = {st['pending']}   "
          f"error = {len(errors)}")
    if hello is not None:
        print(f"[selftest] hello: proto={hello.get('proto')} version={hello.get('version')} "
              f"telemetry_hz={hello.get('telemetry_hz')} "
              f"video_rtsp_path={hello.get('video_rtsp_path')}")
    if last is not None:
        t = last.get("tracking") or {}
        s = last.get("stability") or {}
        table = last.get("frame_table") or []
        video = last.get("video") or {}
        print(f"[selftest] mode = {last.get('mode')}")
        print(f"[selftest] tracking.state = {t.get('state')}  "
              f"fault = {t.get('fault')}  message = {t.get('message')}")
        print(f"[selftest] stability.state = {s.get('state')}  "
              f"center_err_deg = {s.get('center_err_deg')}")
        print(f"[selftest] attitude_deg（两轴机械姿态） = {last.get('attitude_deg')}")
        print(f"[selftest] joints_deg（机械诊断） = {last.get('joints_deg')}")
        print(f"[selftest] frame_table 长度 = {len(table)}  "
              f"末条 = {table[-1] if table else None}")
        print(f"[selftest] video = {video}")
        print(f"[selftest] server_uptime = {last.get('server_uptime')}  "
              f"ts_wall = {last.get('ts_wall')}")
    if results:
        for r in results:
            print(f"[selftest] command_result: cmd={r.get('cmd')} ok={r.get('ok')} "
                  f"seq={r.get('seq')} message={r.get('message')}")

    rc = 0
    if not st["identity_ready"]:
        print(f"[selftest] 失败：{st['identity_error']}")
        rc = 8
    elif not client.public_angles_ready:
        print("[selftest] 失败：hello 与新鲜机械姿态必须同时符合协议 7 / motion_head_joint_right_positive")
        rc = 9
    if last is None or n_tel == 0:
        print("[selftest] 失败：未收到任何遥测")
        rc = 1
    elif not (last.get("frame_table")):
        print("[selftest] 警告：frame_table 为空（时延将无法计算）")

    if args.send_home:
        r = client.find_result(cmd="home", seq=home_seq)
        if r is not None:
            print(f"[selftest] home 指令：已收到 command_result seq={home_seq} "
                  f"ok={r.get('ok')}")
        else:
            print(f"[selftest] 失败：发送 home(seq={home_seq}) 后未收到对应 command_result")
            rc = 5

    if args.send_manual:
        r = client.find_result(cmd="manual")
        if r is not None:
            print("[selftest] manual 指令：已收到 command_result")
        elif manual_mode_seen and manual_max > 0.1:
            print("[selftest] manual 指令：已生效（遥测证据 mode=MANUAL, "
                  f"机械 pitch 角速度峰值={manual_max:.1f}°/s；"
                  "UDP 网关不转发 manual 成功回执）")
        else:
            print("[selftest] 失败：发送 manual 后既无 command_result 也无遥测生效证据")
            rc = 6

    client.stop()
    if args.test_timeout is not None:
        rtc = _selftest_timeout(args.test_timeout)
        if rtc:
            rc = rc or rtc

    print(f"[selftest] 退出码 = {rc}")
    return rc


# ---------------------------------------------------------------------------
# 自检：RTSP 解码 + 帧序号
# ---------------------------------------------------------------------------
def run_decode_test(cfg: dict, args) -> int:
    if not _HAVE_GST:
        print(f"[decode-test] 失败：GStreamer 不可用: {_GST_ERR}", file=sys.stderr)
        return 2
    url = rtsp_url(cfg)
    duration = float(args.decode_test)
    print(f"[decode-test] 连接 {url}，解码 {duration:.1f}s ...")
    try:
        pipeline, sink, dec = build_pipeline(url, cfg.get("decoder", "nvdec"))
    except Exception as exc:  # noqa: BLE001
        print(f"[decode-test] 管线创建失败: {exc}", file=sys.stderr)
        return 2
    result = pipeline.set_state(Gst.State.PLAYING)
    if result == Gst.StateChangeReturn.FAILURE:
        print("[decode-test] 失败：NVDEC 管线无法进入 PLAYING；请检查 NVIDIA 驱动、权限和视频源",
              file=sys.stderr)
        pipeline.set_state(Gst.State.NULL)
        return 2

    t0 = time.monotonic()
    frames = 0
    decoded = 0
    seqs = []
    res = None
    latest = None
    first_frame_at = None
    while time.monotonic() - t0 < duration:
        message = pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
        if message is not None:
            error, debug = message.parse_error()
            print(f"[decode-test] 失败：NVDEC 管线运行错误: {error.message}"
                  f"{f' ({debug})' if debug else ''}", file=sys.stderr)
            pipeline.set_state(Gst.State.NULL)
            return 2
        sample = sink.emit("try-pull-sample", 100_000_000)
        if sample is None:
            time.sleep(0.005)
            continue
        buf = sample.get_buffer()
        caps = sample.get_caps().get_structure(0)
        w = int(caps.get_value("width"))
        h = int(caps.get_value("height"))
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            continue
        arr = np.ndarray(shape=(h, w, 3), dtype=np.uint8,
                         buffer=mapinfo.data, strides=(w * 3, 3, 1))
        seq = decode_stamp_from_bgr(arr, w, h)
        latest = arr.copy()
        res = (w, h)
        buf.unmap(mapinfo)
        frames += 1
        if first_frame_at is None:
            first_frame_at = time.monotonic()
            print(f"[decode-test] 首帧到达: {w}x{h}  解码器 {dec}")
        if seq is not None:
            decoded += 1
            seqs.append(seq)
        time.sleep(0.002)
    pipeline.set_state(Gst.State.NULL)

    span = (time.monotonic() - t0)
    print("\n[decode-test] ===== 摘要 =====")
    print(f"[decode-test] 分辨率 = {res}  解码器 = {dec}")
    print(f"[decode-test] 取帧总数 = {frames}  时序成功率 = "
          f"{(decoded / frames * 100.0) if frames else 0.0:.1f}%  ({decoded}/{frames})")
    if frames and first_frame_at is not None:
        run = span - (first_frame_at - t0)
        print(f"[decode-test] 有效取帧时长 ≈ {run:.2f}s  平均取帧率 ≈ "
              f"{(frames / run) if run > 0 else 0:.1f} fps")
    if seqs:
        print(f"[decode-test] 帧序号范围 = {seqs[0]} .. {seqs[-1]}（mod 2^24）")

    rc = 0
    if frames == 0:
        print("[decode-test] 失败：未取到任何视频帧")
        rc = 1
    elif decoded == 0:
        print("[decode-test] 失败：未能解出帧序号（stamp）")
        rc = 3
    if args.snapshot:
        if latest is not None:
            h, w = latest.shape[:2]
            cv2.drawMarker(latest, (w // 2, h // 2), (0, 255, 0),
                           cv2.MARKER_CROSS, 60, 2)
            out_dir = os.path.dirname(os.path.abspath(args.snapshot))
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
            if cv2.imwrite(args.snapshot, latest):
                print(f"[decode-test] 已保存截图 = {args.snapshot} ({w}x{h})")
            else:
                print(f"[decode-test] 截图保存失败 = {args.snapshot}")
                rc = rc or 4
        else:
            print("[decode-test] 无帧可保存截图")
            rc = rc or 4
    print(f"[decode-test] 退出码 = {rc}")
    return rc


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="两轴跟踪吊舱独立地面站（RTSP 视频 + UDP 遥测/控制，models/gimbals/two_axis_pod/docs/control-protocol.md）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help="INI 配置文件路径（不支持 JSON）")
    ap.add_argument("--no-manage-bridge", action="store_true",
                    help="只连接网关，不由 Station 启动/停止（覆盖 [bridge] manage）")
    ap.add_argument("--udp-host", default=None,
                    help="云台网关主机（UDP 控制/遥测，覆盖配置）")
    ap.add_argument("--udp-port", type=int, default=None,
                    help="UDP 控制/遥测端口（网关）")
    ap.add_argument("--host", default=None,
                    help="--udp-host 的别名（兼容旧参数）")
    ap.add_argument("--ws-port", type=int, default=None,
                    help="已弃用：映射为 --udp-port（兼容旧脚本）")
    ap.add_argument("--rtsp-port", type=int, default=None, help="RTSP 端口")
    ap.add_argument("--aircraft-id", type=int, default=None, help="飞机 ID，默认 1")
    ap.add_argument("--mount", default=None, help="RTSP 路径前缀（末尾自动追加 _飞机ID）")
    ap.add_argument("--display-scale", type=float, default=None,
                    help="显示缩放（0.05..2.0）")
    ap.add_argument("--decoder", choices=["nvdec", "nvenc"], default=None,
                    help="视频解码器（仅 NVIDIA nvh264dec；nvenc 为旧配置兼容别名）")
    ap.add_argument("--window-title", default=None, help="窗口标题")
    ap.add_argument("--font", default=None, help="中文渲染字体路径")
    ap.add_argument("--client-timeout", type=float, default=None,
                    help="判定未连接的服务端消息超时（秒）")
    ap.add_argument("--retry-interval", type=float, default=None,
                    help="有回执指令重传间隔（秒）")
    ap.add_argument("--retry-max", type=int, default=None,
                    help="有回执指令最大发送次数（含首次）")

    ap.add_argument("--selftest", type=float, metavar="DURATION", default=None,
                    help="无 GUI 自检：UDP 登记并采集 DURATION 秒遥测后打印摘要退出")
    ap.add_argument("--decode-test", type=float, metavar="SECONDS", default=None,
                    help="无 GUI 自检：解码 RTSP SECONDS 秒、验证帧序号后退出")
    ap.add_argument("--snapshot", default=None,
                    help="--decode-test 保存一张 PNG 的路径（叠加中心十字）")
    ap.add_argument("--send-home", action="store_true",
                    help="selftest 中额外发送 home，校验带 seq 的 command_result")
    ap.add_argument("--send-manual", action="store_true",
                    help="selftest 中额外以 10Hz 发送 manual(pitch=-20) 再归零，校验生效")
    ap.add_argument("--test-timeout", type=float, nargs="?", const=2.5,
                    default=None, metavar="SECONDS",
                    help="selftest 中追加超时/重登记子测试（默认 2.5s）")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="GUI 运行 N 秒后自动退出（0 = 一直运行）")
    ap.add_argument("--gui-snapshot", default=None,
                    help="GUI 退出前保存界面截图路径")
    return ap


def main(argv=None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    try:
        cfg = load_config(args.config)
    except ValueError as exc:
        ap.error(str(exc))
    cfg = apply_cli_overrides(cfg, args)

    # 兼容旧参数 --ws-port：映射为 UDP 端口
    if args.ws_port is not None and args.udp_port is None:
        print("[ground_station] --ws-port 已弃用，按 --udp-port 处理",
              file=sys.stderr)
        cfg["udp_port"] = args.ws_port

    if args.selftest is not None:
        return run_selftest(cfg, args)
    if args.decode_test is not None:
        return run_decode_test(cfg, args)

    print(f"[ground_station] UDP  {udp_endpoint(cfg)}")
    print(f"[ground_station] RTSP {rtsp_url(cfg)}  decoder={cfg.get('decoder')}  "
          f"display_scale={cfg.get('display_scale')}")
    gs = GroundStation(cfg, args)
    try:
        return gs.run()
    finally:
        gs.shutdown()


if __name__ == "__main__":
    sys.exit(main())
