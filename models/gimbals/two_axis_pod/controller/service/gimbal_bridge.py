#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云台侧网络网关（gz-transport -> IP:端口）。

把 Gazebo 内 `gz-two-axis-pod-controller` 插件暴露的 gz-transport 话题，桥接成地面站可用的
UDP + RTSP 接口，对外协议与 `models/gimbals/two_axis_pod/docs/system.md`（控制/遥测）、
`models/gimbals/two_axis_pod/docs/system.md`（遥测字段）一致。视频仍为 RTSP/RTP over UDP。

gz-transport 侧（与插件对接）：
- 订阅 ``/two_axis_pod/telemetry``（StringMsg，JSON 遥测）
- 订阅 ``/two_axis_pod/command_result``（StringMsg，JSON 指令结果）
- 发布 ``/two_axis_pod/command``（StringMsg，JSON 指令）-> 插件
- 复用 ``gz_source.py`` 的 ``GzSource``：相机图像 + 仿真时钟 -> 单调时钟映射

网络侧（面向地面站，见 models/gimbals/two_axis_pod/docs/system.md）：
- UDP ``udp://<host>:8766``：一条数据报 = 一条 UTF-8 JSON；服务端按客户端地址登记并
  以 ``--telemetry-hz`` 广播 ``telemetry``；指令经 ``seq`` 幂等去重后转插件。
- RTSP ``rtsp://<host>:8555/two_axis_pod``：H.264（复用 ``video.py``）

线程模型：``video_thread`` 只取最新帧投递；
``telemetry_thread`` 负责 UDP 遥测广播；主线程阻塞在 UDP 接收循环；gz-transport
回调线程只更新共享状态（``command_result`` 按唯一请求 ID 关联 ``seq`` 后转发）。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import sys
import threading
import time
import uuid
from pathlib import Path
from collections import deque

from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.msgs10.stringmsg_pb2 import StringMsg  # noqa: E402

from gz_source import GzSource  # noqa: E402
import video as video_mod  # noqa: E402
from video import VideoStreamer  # noqa: E402

STAMP_MASK = 0xFFFFFF

TELEMETRY_TOPIC = "/two_axis_pod/telemetry"
COMMAND_TOPIC = "/two_axis_pod/command"
RESULT_TOPIC = "/two_axis_pod/command_result"

# 单条数据报上限（models/gimbals/two_axis_pod/docs/system.md：8 KiB），超过不发送/丢弃
MAX_DATAGRAM = 8 * 1024
# 接收缓冲区（models/gimbals/two_axis_pod/docs/system.md：建议 >= 64 KiB）
RECV_BUFFER = 65535
SEQ_WINDOW = 1024
MAX_PENDING = 4096
RESULT_TIMEOUT = 30.0
SESSION_RETENTION = 60.0

# 直接透传给插件的指令（插件负责执行并回 command_result）
PASSTHROUGH_CMDS = {"track", "stop", "home", "reset", "reset_axis", "manual", "reset_fault"}

# x264 软编降级管线（NVENC 不可用时使用，使用本地 video 模块）
X264_ENCODE_LAUNCH = (
    "appsrc name=vsrc is-live=true format=time do-timestamp=false "
    "caps=video/x-raw,format=RGB,width={w},height={h},framerate={fps}/1 "
    "max-buffers=3 leaky-type=downstream block=false "
    "! queue leaky=downstream max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
    "! videoconvert "
    "! x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate} "
    "key-int-max={fps} bframes=0 sliced-threads=true "
    "! rtph264pay pt=96 config-interval=1 "
    "! udpsink host=127.0.0.1 port={udp_port} sync=false async=false"
)


def _as_int(value):
    """把候选 seq 规整为 int；布尔/非整数/缺失返回 None。"""
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float) and value.is_integer():
        return int(value)
    return None


class _Client:
    """一个已登记客户端地址的会话状态（models/gimbals/two_axis_pod/docs/system.md§6）。"""

    __slots__ = ("addr", "last_seen", "highest_seq", "last_control_seq", "seen", "results")

    def __init__(self, addr):
        self.addr = addr
        self.last_seen = time.monotonic()
        self.highest_seq = -1
        self.last_control_seq = -1
        self.seen: dict[int, str] = {}          # seq -> 请求内容
        self.results: dict[int, list[str]] = {}  # seq -> 已生成的回执载荷


class GimbalBridge:
    """gz-transport <-> UDP/RTSP 网关。"""

    def __init__(self, args):
        self.args = args
        self._running = True
        self._t_start = time.time()

        # GzSource：提供 gz-transport Node、相机帧与仿真->单调时钟映射
        self.source = GzSource(args.world, args.gimbal_model)

        self.lock = threading.RLock()
        self._latest_tel = None                 # 插件遥测 JSON -> dict
        self._frame = None                      # (seq,w,h,sim_stamp,t_arrive,t0,data)
        self._img_counter = 0
        self.frame_registry = deque(maxlen=600)  # (seq24, t0_mono)
        self._last_reg_seq = None

        self.clients_lock = threading.RLock()
        self.clients: dict[tuple, _Client] = {}
        self.retired_clients: dict[tuple, _Client] = {}
        self.pending_requests: dict[str, tuple] = {}
        try:
            self.clock_domain = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        except OSError:
            self.clock_domain = None

        self.video: VideoStreamer | None = None
        self._video_error: str | None = None

        # UDP 控制/遥测服务端（models/gimbals/two_axis_pod/docs/system.md）
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF,
                                 max(1 << 20, RECV_BUFFER))
        except OSError:
            pass
        self.sock.bind((args.udp_host, args.udp_port))
        self.sock.settimeout(0.5)               # 便于 _running 翻转后及时退出

        # gz-transport：订阅插件遥测/结果，发布指令
        self._cmd_pub = self.source.node.advertise(COMMAND_TOPIC, StringMsg)
        self.source.node.subscribe(StringMsg, TELEMETRY_TOPIC, self._on_telemetry)
        self.source.node.subscribe(StringMsg, RESULT_TOPIC, self._on_command_result)
        # 相机图像：话题名可配（GzSource 内部固定订阅默认 link/sensor，
        # 这里按参数重新订阅，保证 --camera-link/--camera-sensor 生效）
        img_topic = (f"/world/{args.world}/model/{args.gimbal_model}/link/"
                     f"{args.camera_link}/sensor/{args.camera_sensor}/image")
        self.source.node.subscribe(Image, img_topic, self._on_image)
        print(f"[bridge] 订阅图像 {img_topic}")

    # ---------------- gz-transport 回调（非主线程） ----------------
    def _on_telemetry(self, msg):
        try:
            tel = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(tel, dict):
            return
        with self.lock:
            self._latest_tel = tel

    def _on_command_result(self, msg):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(data, dict):
            return
        data.setdefault("type", "command_result")
        self._forward_command_result(data)

    def _on_image(self, msg):
        t_arrive = time.monotonic()
        seq = None
        for entry in msg.header.data:
            if entry.key == "seq" and len(entry.value) > 0:
                try:
                    seq = int(entry.value[0])
                except ValueError:
                    pass
        if seq is None:
            with self.lock:
                self._img_counter += 1
                seq = self._img_counter
        sim_stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        off = self.source.clock_offset
        t0 = (sim_stamp + off) if off > 0 else t_arrive
        with self.lock:
            self._frame = (seq, int(msg.width), int(msg.height), sim_stamp,
                           t_arrive, t0, bytes(msg.data))

    # ---------------- 视频线程 ----------------
    def video_thread(self):
        while self._running:
            with self.lock:
                frame = self._frame
            if frame is not None:
                seq, w, h, _sim, _arr, t0, data = frame
                if self.video is None and self._video_error is None:
                    self._start_video(w, h)
                if self.video is not None:
                    seq24 = seq & STAMP_MASK
                    if self.video.push(seq24, data, t0):
                        with self.lock:
                            if seq24 != self._last_reg_seq:
                                self.frame_registry.append((seq24, t0))
                                self._last_reg_seq = seq24
            time.sleep(1.0 / 60.0)

    def _start_video(self, width, height):
        try:
            if self.args.encoder == "x264":
                video_mod.ENCODE_LAUNCH = X264_ENCODE_LAUNCH
            self.video = VideoStreamer(width, height,
                                       rtsp_port=self.args.rtsp_port,
                                       mount=self.args.mount,
                                       bitrate_kbps=self.args.video_bitrate,
                                       udp_port=self.args.rtp_port)
            print(f"[video] RTSP 服务已启动 "
                  f"rtsp://0.0.0.0:{self.args.rtsp_port}{self.args.mount} "
                  f"({width}x{height}, encoder={self.args.encoder})",
                  flush=True)
        except Exception as exc:  # noqa: BLE001 - 编码器不可用不应拖垮网关
            self._video_error = str(exc)
            print(f"[video] 编码管线初始化失败（网关继续提供遥测/指令）: {exc}",
                  flush=True)

    # ---------------- 遥测组装 ----------------
    def snapshot(self) -> dict | None:
        with self.lock:
            tel = self._latest_tel
            table = list(self.frame_registry)[-60:]
        if tel is None:
            return None
        out = dict(tel)  # 插件遥测所有字段
        out.setdefault("type", "telemetry")
        out["frame_table"] = [[int(s), round(t0, 6)] for (s, t0) in table]
        out["video"] = {
            "pushed": self.video.frames_pushed if self.video else 0,
            "dropped": self.video.frames_dropped if self.video else 0,
            "clients": self.video.clients if self.video else 0,
        }
        # 便于地面站做时钟/存活判断（models/gimbals/two_axis_pod/docs/system.md；插件未提供）
        out["ts_wall"] = round(time.time(), 6)
        out["server_uptime"] = round(time.time() - self._t_start, 1)
        return out

    # ---------------- UDP 发送 ----------------
    def _send_json(self, addr, payload: dict | str) -> bool:
        text = payload if isinstance(payload, str) else json.dumps(payload)
        data = text.encode("utf-8")
        if len(data) > MAX_DATAGRAM:
            print(f"[udp] 丢弃超长数据报 {len(data)}B -> {addr}", flush=True)
            return False
        try:
            self.sock.sendto(data, addr)
            return True
        except OSError as exc:  # noqa: BLE001
            print(f"[udp] 发送到 {addr} 失败: {exc}", flush=True)
            return False

    # ---------------- 客户端登记 ----------------
    def _on_datagram(self, data: bytes, addr):
        if len(data) > MAX_DATAGRAM:
            print(f"[udp] 丢弃超长数据报 {len(data)}B <- {addr}", flush=True)
            return
        try:
            text = data.decode("utf-8").strip()
        except UnicodeDecodeError:
            self._send_json(addr, {"type": "error", "message": "非 UTF-8 数据报",
                                   "seq": None})
            return
        if not text:
            return
        try:
            msg = json.loads(text)
        except json.JSONDecodeError:
            self._send_json(addr, {"type": "error", "message": "无效 JSON",
                                   "seq": None})
            return
        if not isinstance(msg, dict):
            self._send_json(addr, {"type": "error", "message": "无效 JSON",
                                   "seq": None})
            return
        cmd = msg.get("cmd")
        if not isinstance(cmd, str) or cmd not in PASSTHROUGH_CMDS | {"ping", "bye", "status"}:
            self._send_json(addr, {"type": "command_result", "cmd": cmd if isinstance(cmd, str) else None,
                                   "seq": _as_int(msg.get("seq")), "ok": False,
                                   "message": "无效指令"})
            return
        now = time.monotonic()
        with self.clients_lock:
            self._expire_requests(now)
            cl = self.clients.get(addr)
            if cl is not None and now - cl.last_seen > self.args.client_timeout:
                self._retire_client(cl)
                cl = None
            new = cl is None
            if new:
                cl = self.retired_clients.pop(addr, None) or _Client(addr)
                self.clients[addr] = cl
            cl.last_seen = now
        if new:
            hello = {
                "type": "hello",
                "proto": "gimbal-udp",
                "version": 3,
                "angle_convention": "side_mount_joint",
                "clock_domain": self.clock_domain,
                "telemetry_hz": self.args.telemetry_hz,
                "video_rtsp_path": self.args.mount,
            }
            self._send_json(addr, hello)
            print(f"[udp] 客户端登记 {addr}，当前 {len(self.clients)} 个", flush=True)
        self._handle_command(cl, msg)

    # ---------------- 指令处理（models/gimbals/two_axis_pod/docs/system.md§6） ----------------
    def _handle_command(self, cl: _Client, msg: dict):
        # 与插件回调、会话过期共用锁，先登记关联再发布。
        with self.clients_lock:
            self._handle_command_locked(cl, msg)

    def _handle_command_locked(self, cl: _Client, msg: dict):
        addr = cl.addr
        cmd = msg.get("cmd")
        seq = _as_int(msg.get("seq"))
        if "seq" in msg and (seq is None or seq < 0):
            self._send_json(addr, {"type": "command_result", "cmd": cmd,
                                   "seq": msg.get("seq"), "ok": False,
                                   "message": "seq 必须是非负整数"})
            return
        if not isinstance(cmd, str):
            self._reply_and_cache(cl, seq, {"type": "command_result", "cmd": None,
                                            "ok": False, "message": "cmd 必须是字符串"})
            return
        if cmd == "bye":
            self._retire_client(cl)
            return
        if cmd == "ping":
            self._send_json(addr, {"type": "pong", "t": msg.get("t"), "seq": seq})
            return

        if seq is not None:
            fingerprint = json.dumps(msg, sort_keys=True, separators=(",", ":"))
            if seq in cl.seen:
                if cl.seen[seq] != fingerprint:
                    self._send_json(addr, {"type": "command_result", "cmd": cmd,
                                           "seq": seq, "ok": False,
                                           "message": "seq 已用于其他请求"})
                else:
                    for payload in cl.results.get(seq, []):
                        self._send_json(addr, payload)
                return
            if seq <= cl.highest_seq - SEQ_WINDOW:
                self._send_json(addr, {"type": "command_result", "cmd": cmd,
                                       "seq": seq, "ok": False,
                                       "message": "seq 已超出去重窗口，未执行"})
                return
            cl.highest_seq = max(cl.highest_seq, seq)
            cl.seen[seq] = fingerprint
            for old_seq in list(cl.seen):
                if old_seq <= cl.highest_seq - SEQ_WINDOW:
                    cl.seen.pop(old_seq, None)
                    cl.results.pop(old_seq, None)

        if cmd == "status":
            snap = self.snapshot()
            self._reply_and_cache(cl, seq, snap if snap is not None else
                                  {"type": "error", "message": "暂无遥测"})
            return

        if seq is not None and cmd in PASSTHROUGH_CMDS:
            if seq < cl.last_control_seq:
                self._reply_and_cache(cl, seq, {"type": "command_result", "cmd": cmd,
                                                "ok": False, "message": "过时指令，未执行"})
                return
            cl.last_control_seq = seq

        if cmd == "reset_axis":
            axis = msg.get("axis")
            if not isinstance(axis, str) or axis not in {"yaw", "pitch"}:
                self._reply_and_cache(cl, seq, {"type": "command_result", "cmd": cmd,
                    "ok": False, "message": "axis 必须为 yaw 或 pitch"})
                return
            self._record_and_publish(cl, seq, {"cmd": cmd, "axis": axis})
            return

        if cmd in {"track", "manual"}:
            try:
                if cmd == "track":
                    box = msg.get("box")
                    if not isinstance(box, (list, tuple)) or len(box) != 4:
                        raise ValueError("选框必须包含四个数值")
                    values = box
                    keys = None
                else:
                    keys = ("yaw", "pitch")
                    if "roll" in msg:
                        raise ValueError("两轴吊舱不支持 roll 输入")
                    values = [msg.get(key, 0.0) for key in keys]
                if any(isinstance(value, bool) or not isinstance(value, (int, float))
                       for value in values):
                    raise ValueError("参数必须是数值")
                values = [float(value) for value in values]
                if not all(math.isfinite(value) for value in values):
                    raise ValueError("参数必须是有限数值")
                if cmd == "track" and (values[2] <= 0 or values[3] <= 0):
                    raise ValueError("选框宽高必须大于零")
                out = {"cmd": cmd}
                if keys:
                    out.update(zip(keys, values))
                else:
                    out["box"] = values
            except (TypeError, ValueError, OverflowError) as exc:
                self._reply_and_cache(cl, seq, {"type": "command_result", "cmd": cmd,
                                                "ok": False, "message": str(exc)})
                return
            self._record_and_publish(cl, seq, out)
            return

        if cmd in PASSTHROUGH_CMDS:
            self._record_and_publish(cl, seq, {"cmd": cmd})
            return
        self._reply_and_cache(cl, seq, {"type": "command_result", "cmd": cmd,
                                        "ok": False, "message": f"未知指令 {cmd}"})

    def _reply_and_cache(self, cl: _Client, seq, payload: dict):
        """立即回执并缓存；所有带 seq 请求的本地结果均保留关联。"""
        payload = dict(payload, seq=seq)
        text = json.dumps(payload)
        if seq is not None and seq in cl.seen:
            cl.results[seq] = [text]
        self._send_json(cl.addr, text)

    def _record_and_publish(self, cl: _Client, seq, out: dict):
        """使用网关唯一 ID 绑定会话、客户端 seq 和命令。"""
        self._expire_requests(time.monotonic())
        needs_result = out["cmd"] in {"track", "stop", "home", "reset", "reset_axis"}
        if needs_result and len(self.pending_requests) >= MAX_PENDING:
            self._reply_and_cache(cl, seq, {"type": "command_result", "cmd": out["cmd"],
                                            "ok": False, "message": "等待回执队列已满，未执行"})
            return
        request_id = uuid.uuid4().hex
        if needs_result:
            self.pending_requests[request_id] = (cl, seq, out["cmd"], time.monotonic())
        m = StringMsg()
        m.data = json.dumps(dict(out, request_id=request_id))
        try:
            published = self._cmd_pub.publish(m)
        except Exception:
            published = False
        if published is False:
            self.pending_requests.pop(request_id, None)
            self._reply_and_cache(cl, seq, {"type": "command_result", "cmd": out["cmd"],
                                            "ok": False, "message": "发布指令失败"})

    def _forward_command_result(self, data: dict):
        """仅将回执交付给 ID 绑定且仍有效的原始会话。"""
        if not isinstance(data, dict) or not isinstance(data.get("request_id"), str):
            return
        with self.clients_lock:
            self._expire_requests(time.monotonic())
            pending = self.pending_requests.get(data["request_id"])
            if pending is None:
                return
            cl, seq, cmd, _started = pending
            if data.get("cmd") != cmd:
                return
            if self.clients.get(cl.addr) is not cl:
                self.pending_requests.pop(data["request_id"], None)
                return
            if time.monotonic() - cl.last_seen > self.args.client_timeout:
                self._retire_client(cl)
                return
            self.pending_requests.pop(data["request_id"], None)
            payload = dict(data)
            payload.pop("request_id", None)
            self._reply_and_cache(cl, seq, payload)

    def _retire_client(self, cl):
        """取消失效会话的在途请求，短期保留去重结果以防重登记重复执行。"""
        self.clients.pop(cl.addr, None)
        for request_id, (owner, seq, cmd, _started) in list(self.pending_requests.items()):
            if owner is cl:
                self.pending_requests.pop(request_id, None)
                if seq is not None and seq in cl.seen:
                    cl.results[seq] = [json.dumps({"type": "command_result", "cmd": cmd,
                                                  "seq": seq, "ok": False,
                                                  "message": "会话已过期，执行结果未知"})]
        self.retired_clients[cl.addr] = cl

    def _expire_requests(self, now):
        for addr, cl in list(self.retired_clients.items()):
            if now - cl.last_seen > self.args.client_timeout + SESSION_RETENTION:
                self.retired_clients.pop(addr, None)
        for request_id, (cl, seq, cmd, started) in list(self.pending_requests.items()):
            if now - started > RESULT_TIMEOUT:
                self.pending_requests.pop(request_id, None)
                payload = {"type": "command_result", "cmd": cmd, "seq": seq,
                           "ok": False, "message": "插件回执超时，执行结果未知"}
                if self.clients.get(cl.addr) is cl:
                    self._reply_and_cache(cl, seq, payload)

    # ---------------- 广播循环 ----------------
    def _broadcast_telemetry(self):
        now = time.monotonic()
        with self.clients_lock:
            self._expire_requests(now)
            for cl in list(self.clients.values()):
                if now - cl.last_seen > self.args.client_timeout:
                    self._retire_client(cl)
            items = list(self.clients.items())
        snap = self.snapshot()
        if snap is None:
            return
        text = json.dumps(snap)
        for addr, _cl in items:
            self._send_json(addr, text)

    def telemetry_thread(self):
        period = 1.0 / max(1, self.args.telemetry_hz)
        while self._running:
            t0 = time.monotonic()
            try:
                self._broadcast_telemetry()
            except Exception as exc:  # noqa: BLE001 - 广播异常不应终止网关
                print(f"[udp] 遥测广播异常: {exc}", flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    # ---------------- UDP 接收循环 ----------------
    def udp_rx_loop(self):
        print(f"[udp] 遥测/指令服务 udp://{self.args.udp_host}:"
              f"{self.args.udp_port} (telemetry_hz={self.args.telemetry_hz}, "
              f"client_timeout={self.args.client_timeout}s)", flush=True)
        while self._running:
            try:
                data, addr = self.sock.recvfrom(RECV_BUFFER)
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    continue
                break
            try:
                self._on_datagram(data, addr)
            except Exception as exc:  # noqa: BLE001 - 单条异常不应终止服务
                print(f"[udp] 处理 {addr} 数据报异常: {exc}", flush=True)

    def close(self):
        self._running = False
        try:
            self.sock.close()
        except OSError:
            pass


def main():
    parser = argparse.ArgumentParser(
        description="云台侧网络网关（gz-transport -> UDP/RTSP）")
    parser.add_argument("--world", default="two_axis_pod_demo")
    parser.add_argument("--gimbal-model", default="two_axis_pod")
    parser.add_argument("--camera-link", default="camera_link")
    parser.add_argument("--camera-sensor", default="camera")
    parser.add_argument("--udp-host", default="0.0.0.0")
    parser.add_argument("--udp-port", type=int, default=8766)
    parser.add_argument("--client-timeout", type=float, default=5.0,
                        help="客户端登记超时（秒，models/gimbals/two_axis_pod/docs/system.md）")
    parser.add_argument("--rtsp-port", type=int, default=8555)
    parser.add_argument("--rtp-port", type=int, default=5602,
                        help="本机编码器与 RTSP 中继之间的 RTP 端口；并行实例须使用不同端口")
    parser.add_argument("--mount", default="/two_axis_pod")
    parser.add_argument("--video-bitrate", type=int, default=8000,
                        help="H.264 码率（kbps）")
    parser.add_argument("--telemetry-hz", type=int, default=50)
    parser.add_argument("--encoder", choices=["nvenc", "x264"], default="nvenc",
                        help="视频编码器：nvenc(nvcudah264enc，默认) / x264 软编降级")
    args = parser.parse_args()

    try:
        import faulthandler
        faulthandler.register(signal.SIGUSR1, all_threads=True, chain=False)
    except Exception:  # noqa: BLE001
        pass

    bridge = GimbalBridge(args)
    threading.Thread(target=bridge.video_thread, daemon=True).start()
    threading.Thread(target=bridge.telemetry_thread, daemon=True).start()
    try:
        bridge.udp_rx_loop()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
        if bridge.video is not None:
            try:
                bridge.video.close()
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    main()
