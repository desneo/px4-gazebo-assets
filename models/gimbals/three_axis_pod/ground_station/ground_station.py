#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三轴跟踪吊舱 独立地面站（运营端）。

自包含的独立程序：通过配置的 IP/端口接收 RTSP 视频、显示 UDP 遥测、
下发跟踪框与控制指令。运行不依赖本仓库 `server/`、`client/` 目录。

功能：
- RTSP 硬解播放（默认 ``nvh264dec``，可配置/自动回退 ``avdec_h264``）；
- 鼠标拖拽画框，松开自动提交 ``track``（归一化到整帧 0..1）；
- 显示实时跟踪框、跟踪状态、稳定性、三轴关节角/角速度、限位/饱和、
  目标中心角误差、端到端时延（用 ``frame_table``）、显示帧率与丢帧率；
- 按钮与快捷键：复位 reset / 停止 stop / 朝天 home / 清故障 reset_fault，
  方向键与 ``[`` ``]`` 手动指向；
- 连接状态指示、UDP 登记/续期/重登记与视频断线自动重连。

无显示器/自检模式：
- ``--selftest DURATION``：不依赖 GUI，收 UDP 遥测并打印摘要；
- ``--decode-test SECONDS --snapshot PATH``：不依赖 GUI，验证 RTSP 解码与帧序号。

通信协议遵循 ``models/gimbals/three_axis_pod/docs/system.md``：一个 UDP 数据报 = 一条 UTF-8 JSON，
客户端以 ``ping`` 登记/续期，有回执指令按 ``seq`` 重传去重，``manual`` 以
≥10 Hz 连续发送。用法示例见 README.md。
"""
from __future__ import annotations

import argparse
import json
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

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(HERE, "config.json")

# 解码器配置项 -> GStreamer 元件
DECODER_ELEMENTS = {"nvenc": "nvh264dec", "avdec": "avdec_h264"}

# models/gimbals/three_axis_pod/docs/system.md：单条数据报上限 8 KiB；接收缓冲区建议 ≥ 64 KiB。
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
    "udp_port": 8765,
    "rtsp_port": 8554,
    "rtsp_mount": "/gimbal",
    "display_scale": 0.5,
    "decoder": "nvenc",
    "window_title": "Gimbal Ground Station",
    "client_timeout": 5.0,
    "retry_interval": 0.2,
    "retry_max": 5,
    "reconnect": True,
    "font_path": "",
}

# 命令行 -> 配置键（后项覆盖前项，例如 --udp-host 优先于 --host）
CLI_TO_CONFIG = [
    ("host", "gimbal_host"),
    ("udp_host", "gimbal_host"),
    ("udp_port", "udp_port"),
    ("rtsp_port", "rtsp_port"),
    ("mount", "rtsp_mount"),
    ("display_scale", "display_scale"),
    ("decoder", "decoder"),
    ("window_title", "window_title"),
    ("client_timeout", "client_timeout"),
    ("retry_interval", "retry_interval"),
    ("retry_max", "retry_max"),
]


def load_config(path: str) -> dict:
    cfg = dict(DEFAULT_CONFIG_VALUES)
    if path and os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict):
                # 兼容旧配置：ws_port -> udp_port
                if "udp_port" not in data and "ws_port" in data:
                    data["udp_port"] = data["ws_port"]
                cfg.update({k: v for k, v in data.items() if v is not None})
        except (OSError, json.JSONDecodeError) as exc:
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
    return cfg


def udp_endpoint(cfg: dict) -> str:
    return f"udp://{cfg['gimbal_host']}:{cfg['udp_port']}"


def rtsp_url(cfg: dict) -> str:
    mount = str(cfg.get("rtsp_mount") or "/gimbal")
    if not mount.startswith("/"):
        mount = "/" + mount
    return f"rtsp://{cfg['gimbal_host']}:{cfg['rtsp_port']}{mount}"


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
# UDP 客户端：登记/续期 + 遥测接收 + 指令重传（models/gimbals/three_axis_pod/docs/system.md）
# ---------------------------------------------------------------------------
class UdpClient(threading.Thread):
    """UDP 控制/遥测客户端（一个数据报 = 一条 JSON，models/gimbals/three_axis_pod/docs/system.md）。

    - 本地 ``bind`` 临时端口，向网关 ``host:port`` 发送；
    - 接收独立线程：``recvfrom(65535)``，解析后按 ``type`` 分发；
    - 每 ``ping_interval``（默认 1 s）发一次 ping：登记 + 续期 + 链路探测；
    - 有回执指令按 ``retry_interval`` 重发**同一 seq**，直到收到
      ``command_result`` 或累计 ``retry_max`` 次后判定失败；
    - ``connected`` = 最近 ``client_timeout`` 秒内收到过服务端消息。
    """

    def __init__(self, host, port, on_log=None, *, client_timeout=5.0,
                 retry_interval=0.2, retry_max=5, ping_interval=1.0,
                 bind_host="0.0.0.0"):
        super().__init__(daemon=True)
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
        self.last_rx_mono = 0.0          # 最近收到服务端消息的单调时刻
        self.pings_sent = 0              # 已发送 ping 次数（兼作重登记探测）

        self._seq = 0                    # 单调递增序号（models/gimbals/three_axis_pod/docs/system.md）
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
        self._raw_send(dict(cmd=cmd, seq=seq, **extra))
        return seq

    def send_reliable(self, cmd: str, **extra) -> int:
        """有回执指令（track / stop / home / reset）：带 seq 发送并按需重传。"""
        seq = self.next_seq()
        payload = dict(cmd=cmd, seq=seq, **extra)
        with self.lock:
            self._pending[seq] = {"payload": payload, "t": time.monotonic(),
                                  "sends": 1, "result": None}
        self._raw_send(payload)
        self.on_log(f"[udp] 发送 {cmd} seq={seq}（等待 command_result）")
        return seq

    def send_manual(self, yaw=0.0, pitch=0.0, roll=0.0) -> int:
        """manual：无回执，以 ≥10 Hz 连续发送（见 ManualController）。"""
        return self.send_once("manual", yaw=float(yaw), pitch=float(pitch),
                              roll=float(roll))

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
            for key in ("tracking", "stability", "joints_deg", "limits", "cmd_dps", "video"):
                if key in msg and not isinstance(msg[key], dict):
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
        self.last_rx_mono = time.monotonic()
        if mtype == "telemetry":
            with self.lock:
                self.telemetry = msg
                self.messages += 1
                for entry in msg.get("frame_table") or []:
                    if isinstance(entry, (list, tuple)) and len(entry) >= 2:
                        self.frame_t0[entry[0]] = entry[1]
                if len(self.frame_t0) > 3000:
                    for k in sorted(self.frame_t0)[:-1500]:
                        self.frame_t0.pop(k, None)
        elif mtype == "hello":
            with self.lock:
                self.hello = msg
            self.on_log(f"[udp] 收到 hello: telemetry_hz={msg.get('telemetry_hz')} "
                        f"video_rtsp_path={msg.get('video_rtsp_path')} -> 已登记")
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
            secs = ((time.monotonic() - self.last_rx_mono)
                    if self.last_rx_mono else None)
            return {
                "connected": self.connected,
                "registered": self.hello is not None,
                "messages": self.messages,
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


class ManualController(threading.Thread):
    """把按键/按钮的“手动脉冲”变成 ≥10 Hz 的 manual 流（models/gimbals/three_axis_pod/docs/system.md）。

    按轴记录 ``(角速度, 到期时刻)``；每 tick 组合三轴发送；全部到期后再补发
    一条 0 速度指令。持续按住时反复触发 ``hold`` 可续期，从而一直发送。
    """

    def __init__(self, client: UdpClient, rate=10.0):
        super().__init__(daemon=True)
        self.client = client
        self.period = 1.0 / max(1.0, float(rate))
        self.lock = threading.Lock()
        self.axes = {}          # axis -> (value, expire_mono)
        self._was_active = False
        self._stop = threading.Event()

    def hold(self, axis, value, duration=0.4):
        with self.lock:
            self.axes[axis] = (float(value), time.monotonic() + float(duration))

    def run(self):
        while not self._stop.is_set():
            now = time.monotonic()
            with self.lock:
                self.axes = {a: v for a, v in self.axes.items() if v[1] > now}
                vals = {a: v[0] for a, v in self.axes.items()}
            active = bool(vals)
            if active or self._was_active:
                self.client.send_manual(vals.get("yaw", 0.0),
                                        vals.get("pitch", 0.0),
                                        vals.get("roll", 0.0))
            self._was_active = active
            self._stop.wait(self.period)

    def stop(self):
        self._stop.set()


# ---------------------------------------------------------------------------
# RTSP 解码管线
# ---------------------------------------------------------------------------
def build_pipeline(rtsp: str, decoder: str = "nvenc", verbose: bool = True):
    """构造 RTSP -> BGR appsink 管线；优先 decoder，失败自动回退另一种。"""
    if not _HAVE_GST:
        raise RuntimeError(f"GStreamer/PyGObject 不可用: {_GST_ERR}")
    order = [decoder] + [d for d in ("nvenc", "avdec") if d != decoder]
    last_err = "无可用解码器"
    for dec in order:
        elem = DECODER_ELEMENTS.get(dec, "nvh264dec")
        if Gst.ElementFactory.find(elem) is None:
            last_err = f"缺少解码元件 {elem}"
            continue
        desc = (
            f"rtspsrc location={rtsp} latency=0 drop-on-latency=true protocols=udp "
            f"! rtph264depay ! h264parse ! {elem} ! videoconvert "
            f"! video/x-raw,format=BGR ! appsink name=sink max-buffers=1 drop=true"
        )
        try:
            pipeline = Gst.parse_launch(desc)
        except Exception as exc:  # noqa: BLE001
            last_err = str(exc)
            continue
        sink = pipeline.get_by_name("sink")
        if sink is None:
            last_err = "appsink 缺失"
            continue
        if verbose and dec != decoder:
            print(f"[video] 解码器 {DECODER_ELEMENTS[decoder]} 不可用，"
                  f"已回退 {elem}")
        if verbose:
            print(f"[video] RTSP 管线就绪: {rtsp} (解码 {elem})")
        return pipeline, sink, dec
    raise RuntimeError(f"RTSP 解码管线创建失败: {last_err}")


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
            cfg.get("gimbal_host", "127.0.0.1"), cfg.get("udp_port", 8765),
            on_log=lambda s: print(s, flush=True),
            client_timeout=float(cfg.get("client_timeout", 5.0)),
            retry_interval=float(cfg.get("retry_interval", 0.2)),
            retry_max=int(cfg.get("retry_max", 5)))
        self.manual = ManualController(self.tele)
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
            rtsp_url(self.cfg), self.cfg.get("decoder", "nvenc"))
        self.pipeline.set_state(Gst.State.PLAYING)
        threading.Thread(target=self.pull_loop, daemon=True).start()
        threading.Thread(target=self.video_watchdog, daemon=True).start()

    def pull_loop(self):
        while not self._stop:
            sink = self.sink
            if sink is None:
                time.sleep(0.05)
                continue
            sample = sink.emit("pull-sample")
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
                rtsp_url(self.cfg), self.cfg.get("decoder", "nvenc"), verbose=False)
            self.pipeline = pipeline
            self.sink = sink
            self.decoder = dec
            pipeline.set_state(Gst.State.PLAYING)
            print(f"[video] 已重建 RTSP 管线（解码 {dec}）", flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"[video] 重建 RTSP 管线失败: {exc}", flush=True)

    def video_watchdog(self):
        """视频长时间无新帧时尝试 set NULL/PLAYING，仍无则重建管线。"""
        while not self._stop:
            time.sleep(1.0)
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
        for label, rect, cmd in self.buttons:
            if rect[0] <= x <= rect[0] + rect[2] and rect[1] <= y <= rect[1] + rect[3]:
                return (label, rect, cmd)
        return None

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            btn = self._button_at(x, y)
            if btn is not None:
                self.pressed = (btn[0], btn[2], x, y)
                return
            self.mouse_start = (x, y)
            self.mouse_end = (x, y)
            self.drawing = True
        elif event == cv2.EVENT_MOUSEMOVE:
            hit = self._button_at(x, y)
            self.hover = hit[0] if hit else None
            if self.drawing:
                self.mouse_end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            if self.pressed is not None:
                label, cmd, px, py = self.pressed
                self.pressed = None
                hit = self._button_at(x, y)
                if hit is not None and hit[0] == label:
                    self._send_ctrl(cmd, f"已执行: {label}")
                return
            if self.drawing:
                self.mouse_end = (x, y)
                self.drawing = False
                self.submit_box()

    def _send_ctrl(self, cmd, label=None):
        """下发控制指令：有回执指令重传，其余单发。"""
        if cmd in ("track", "stop", "home", "reset"):
            self.tele.send_reliable(cmd)
        else:
            self.tele.send_once(cmd)
        if label:
            self.last_cmd = label

    def submit_box(self):
        if not self.mouse_start or not self.mouse_end:
            return
        x0 = min(self.mouse_start[0], self.mouse_end[0])
        y0 = min(self.mouse_start[1], self.mouse_end[1])
        x1 = max(self.mouse_start[0], self.mouse_end[0])
        y1 = max(self.mouse_start[1], self.mouse_end[1])
        if x1 - x0 < 6 or y1 - y0 < 6:
            return
        with self.lock:
            w, h = self.frame_size
            dw, dh = self.display_size
        sx = w / dw
        sy = h / dh
        box = [x0 * sx / w, y0 * sy / h, (x1 - x0) * sx / w, (y1 - y0) * sy / h]
        self.tele.send_reliable("track", box=[round(v, 6) for v in box])
        self.last_cmd = "已提交选框，开始跟踪"
        self.mouse_start = None
        self.mouse_end = None

    # ------------------------------------------------------------- ui
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

        # 实时跟踪框（遥测 box_px 为 1080p 系 -> 缩放到显示尺寸）
        target = (tele or {}).get("target") or {}
        track = (tele or {}).get("tracking") or {}
        state = track.get("state", "IDLE")
        color = TRACK_COLORS.get(state, (200, 200, 200))
        if target.get("valid") and target.get("box_px"):
            bx, by, bw, bh = target["box_px"]
            sx, sy = dw / w, dh / h
            p0 = (int(bx * sx), int(by * sy))
            p1 = (int((bx + bw) * sx), int((by + bh) * sy))
            cv2.rectangle(disp, p0, p1, color, 2)
            cv2.drawMarker(disp, (int((bx + bw / 2) * sx), int((by + bh / 2) * sy)),
                           color, cv2.MARKER_CROSS, 14, 2)

        # 操作按钮
        button_defs = [("复位", "reset"), ("停止", "stop"),
                       ("朝天", "home"), ("清故障", "reset_fault")]
        bw, bh = 86, 34
        x0 = dw - len(button_defs) * (bw + 8) - 12
        self.buttons = []
        for i, (label, cmd) in enumerate(button_defs):
            rect = (x0 + i * (bw + 8), 10, bw, bh)
            self.buttons.append((label, rect, cmd))
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
            j = tele.get("joints_deg") or {}
            limits = tele.get("limits") or {}
            lines = [
                f"状态: {STATE_CN.get(t.get('state'), t.get('state'))}   稳定性: {STABILITY_CN.get(s.get('state'), s.get('state'))}   模式: {tele.get('mode')}",
                f"目标中心误差: {s.get('center_err_deg', 0):.2f}°    伺服: {tele.get('cmd_dps') or {}}",
                f"yaw   {j.get('yaw', {}).get('pos', 0):+7.2f}°  角速度 {j.get('yaw', {}).get('vel', 0):+7.2f}°/s",
                f"pitch {j.get('pitch', {}).get('pos', 0):+7.2f}°  角速度 {j.get('pitch', {}).get('vel', 0):+7.2f}°/s",
                f"roll  {j.get('roll', {}).get('pos', 0):+7.2f}°  角速度 {j.get('roll', {}).get('vel', 0):+7.2f}°/s",
            ]
            colors = [(0, 255, 255)] * len(lines)
            colors[0] = (0, 255, 255)
            if limits.get("saturated") or any(limits.get(k) for k in ("yaw", "pitch", "roll")):
                lines.append(f"限位/饱和: {limits}")
                colors.append((0, 165, 255))
            if t.get("fault"):
                lines.append(f"故障: {t.get('fault')}  {t.get('message') or ''}")
                colors.append((0, 0, 255))
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

        if self.font is not None:
            top = min(dh, 8 + 30 * (len(lines) + 1) + 10)
            top = max(top, 60)
            crop = disp[0:top, :]
            pil = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil)
            for i, (text, c) in enumerate(zip(lines, colors)):
                draw.text((12, 8 + i * 30), text, font=self.font,
                          fill=(c[2], c[1], c[0]))
            for label, rect, cmd in self.buttons:
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
        return disp

    def _manual(self, axis, value, label="", duration=0.4):
        """方向键/按钮手动脉冲：持续 ~duration 秒以 ≥10 Hz 发送后归零。"""
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
        self.start_video()
        cv2.namedWindow(self.win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.win, self.on_mouse)
        print("地面站已启动：拖拽鼠标画框并松开自动跟踪；"
              "按钮[复位/停止/朝天/清故障]或按键 R/S/H/M；"
              "方向键手动 yaw/pitch；[ ] 手动 roll；ESC 退出")
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
            if getattr(self, "last_frame", None) is not None and (new_frame or now - self.last_draw_t > 0.1):
                disp = self.draw(self.last_frame.copy(), tele)
                self.last_disp = disp
                cv2.imshow(self.win, disp)
                self.last_draw_t = now
                time.sleep(0.005)
            elif getattr(self, "last_frame", None) is None and self.last_disp is None:
                blank = np.zeros((self.display_size[1], self.display_size[0], 3), dtype=np.uint8)
                cv2.putText(blank, "Waiting video...", (30, self.display_size[1] // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                cv2.imshow(self.win, blank)
                time.sleep(0.02)
            key = cv2.waitKeyEx(1)
            if key == 27:  # ESC
                break
            elif key in (ord('s'), ord('S')):
                self._send_ctrl("stop", "已停止跟踪")
            elif key in (ord('h'), ord('H')):
                self._send_ctrl("home", "返回朝天")
            elif key in (ord('m'), ord('M')):
                self._send_ctrl("reset_fault", "清除故障")
            elif key in (ord('r'), ord('R')):
                self._send_ctrl("reset", "复位：停止跟踪并水平朝前")
            elif key == 0 or key == -1:
                pass
            elif key in (2424832, 65361):   # left
                self._manual("yaw", 30, "手动: yaw +30°/s")
            elif key in (2555904, 65363):   # right
                self._manual("yaw", -30, "手动: yaw -30°/s")
            elif key in (2490368, 65362):   # up
                self._manual("pitch", 20, "手动: pitch +20°/s")
            elif key in (2621440, 65364):   # down
                self._manual("pitch", -20, "手动: pitch -20°/s")
            elif key == ord('['):
                self._manual("roll", 20, "手动: roll +20°/s")
            elif key == ord(']'):
                self._manual("roll", -20, "手动: roll -20°/s")

        if self.args.gui_snapshot and self.last_disp is not None:
            cv2.imwrite(self.args.gui_snapshot, self.last_disp)
            print("界面截图:", self.args.gui_snapshot)
        self.shutdown()
        return 0

    def shutdown(self):
        self._stop = True
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
            self.tele.send_once("bye")       # 可选：立即注销（models/gimbals/three_axis_pod/docs/system.md）
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
        cfg.get("gimbal_host", "127.0.0.1"), cfg.get("udp_port", 8765),
        on_log=lambda s: print(s, flush=True),
        client_timeout=float(cfg.get("client_timeout", 5.0)),
        retry_interval=float(cfg.get("retry_interval", 0.2)),
        retry_max=int(cfg.get("retry_max", 5)))
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
            manual_ctrl.hold("yaw", 20.0,
                             duration=min(1.0, max(0.3, duration - 1.2)))
            manual_started = True
        snap = client.snapshot()
        if snap:
            last = snap
            if manual_started:
                cd = snap.get("cmd_dps") or {}
                try:
                    val = abs(float(cd.get("yaw") or 0.0))
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
        print(f"[selftest] joints_deg = yaw {((last.get('joints_deg') or {}).get('yaw') or {}).get('pos')} / "
              f"pitch {((last.get('joints_deg') or {}).get('pitch') or {}).get('pos')} / "
              f"roll {((last.get('joints_deg') or {}).get('roll') or {}).get('pos')} (°)")
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
                  f"cmd_dps.yaw 峰值={manual_max:.1f}°/s；"
                  "该插件对 manual 不返回 command_result，属对端设计）")
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
        pipeline, sink, dec = build_pipeline(url, cfg.get("decoder", "nvenc"))
    except Exception as exc:  # noqa: BLE001
        print(f"[decode-test] 管线创建失败: {exc}", file=sys.stderr)
        return 2
    pipeline.set_state(Gst.State.PLAYING)

    t0 = time.monotonic()
    frames = 0
    decoded = 0
    seqs = []
    res = None
    latest = None
    first_frame_at = None
    while time.monotonic() - t0 < duration:
        sample = sink.emit("pull-sample")
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
        description="三轴跟踪吊舱独立地面站（RTSP 视频 + UDP 遥测/控制，models/gimbals/three_axis_pod/docs/system.md）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help="JSON 配置文件路径")
    ap.add_argument("--udp-host", default=None,
                    help="云台网关主机（UDP 控制/遥测，覆盖配置）")
    ap.add_argument("--udp-port", type=int, default=None,
                    help="UDP 控制/遥测端口（网关）")
    ap.add_argument("--host", default=None,
                    help="--udp-host 的别名（兼容旧参数）")
    ap.add_argument("--ws-port", type=int, default=None,
                    help="已弃用：映射为 --udp-port（兼容旧脚本）")
    ap.add_argument("--rtsp-port", type=int, default=None, help="RTSP 端口")
    ap.add_argument("--mount", default=None, help="RTSP 挂载点（如 /gimbal）")
    ap.add_argument("--display-scale", type=float, default=None,
                    help="显示缩放（0.05..2.0）")
    ap.add_argument("--decoder", choices=["nvenc", "avdec"], default=None,
                    help="视频解码器：nvenc=硬解 / avdec=软解（不可用时自动回退）")
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
                    help="selftest 中额外以 10Hz 发送 manual(yaw) 再归零，校验生效")
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
    cfg = load_config(args.config)
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
    return gs.run()


if __name__ == "__main__":
    sys.exit(main())
