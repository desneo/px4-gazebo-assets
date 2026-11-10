"""将每份有效104字节遥测转发为艾晟桥兼容JSON。"""
from __future__ import annotations

from collections import deque
import ipaddress
import json
import socket
import threading
import time

from common import protocol as proto
from ground_station.client import TelemetrySample


# 必须与 PX4 aisheng_udp_bridge 的严格输入契约保持一致。标准帧无法提供
# los_omega_n/e/d，因此桥接器按其既有约定将三个字段发布为 NaN。
AISHENG_TELEMETRY_FIELDS = (
    "timestamp", "self_check", "work_status", "track_status",
    "pitch_angle", "yaw_angle", "gate_x", "gate_y",
    "gyro_pitch_rate", "gyro_yaw_rate", "pitch_misalignment",
    "yaw_misalignment", "tracking_confidence", "pitch_los_rate",
    "yaw_los_rate", "status", "autonomous", "target_type",
    "target_confidence", "target_lon", "target_lat", "target_alt",
    "status2", "tracking_gate_w", "tracking_gate_h", "pitch_attitude",
    "yaw_attitude", "roll_attitude", "tracking_target_type", "laser_range",
    "laser_update", "aircraft_pitch", "aircraft_roll", "aircraft_yaw",
    "ip_segment", "ip_node", "device_timestamp",
)


def endpoint(ip: str, port: int) -> tuple[str, int]:
    try:
        address = str(ipaddress.ip_address(str(ip).strip()))
        if isinstance(port, bool) or not str(port).strip().isdigit():
            raise ValueError
        number = int(port)
        if not 1 <= number <= 65535:
            raise ValueError
        return address, number
    except ValueError as exc:
        raise ValueError("接收地址须为合法 IP，端口须在 1～65535 内") from exc


def aisheng_telemetry_message(sample: TelemetrySample) -> dict:
    """返回 ``aisheng_udp_bridge`` 可直接接收的顶层37字段报文。"""
    if not isinstance(sample, TelemetrySample) or len(sample.raw) != 104:
        raise ValueError("只能转发实际收到的104字节遥测样本")
    # 再解码一次，防止调用者替换 frame 造成原始帧与发送内容不一致，
    # 同时在发送前再次验证帧头、长度和校验和。
    decoded = proto.decode_telemetry(sample.raw)
    decoded_fields = proto.telemetry_message(decoded, sample.received_monotonic_us)
    return {name: decoded_fields[name] for name in AISHENG_TELEMETRY_FIELDS}


class TelemetryForwarder:
    def __init__(self, *, max_age_s: float = 1.0, queue_size: int = 64,
                 socket_factory=socket.socket):
        if max_age_s <= 0 or queue_size <= 0:
            raise ValueError("转发有效期和队列长度必须为正")
        self.max_age_s = float(max_age_s)
        self.socket_factory = socket_factory
        self.queue = deque(maxlen=queue_size)
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.socket = None
        self.target = None
        self.session = 0
        self.last_sequence = 0
        self.sent = 0
        self.dropped = 0
        self.error = ""
        self.closed = False
        self.thread = threading.Thread(target=self._run, name="a2416a-forward", daemon=True)
        self.thread.start()

    def start(self, ip: str, port: int):
        target = endpoint(ip, port)
        family = socket.AF_INET6 if ":" in target[0] else socket.AF_INET
        sock = self.socket_factory(family, socket.SOCK_DGRAM)
        sock.setblocking(False)
        with self.lock:
            if self.closed:
                sock.close()
                raise RuntimeError("遥测转发器已经关闭")
            if self.socket is not None:
                self.socket.close()
            self.socket = sock
            self.target = target
            self.session += 1
            self.queue.clear()
            self.last_sequence = 0
            self.sent = self.dropped = 0
            self.error = ""

    def offer(self, sample: TelemetrySample):
        with self.lock:
            if self.socket is None or self.closed:
                return
            if not isinstance(sample, TelemetrySample):
                self.dropped += 1
                self.error = "转发来源不是有效遥测样本"
                return
            if sample.sequence <= self.last_sequence:
                return
            self.last_sequence = sample.sequence
            if len(self.queue) == self.queue.maxlen:
                self.dropped += 1
            self.queue.append((self.session, sample))
            self.wake.set()

    def stop(self):
        with self.lock:
            self.session += 1
            if self.socket is not None:
                self.socket.close()
            self.socket = None
            self.target = None
            self.queue.clear()
            self.error = ""

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "active": self.socket is not None,
                "target": self.target,
                "sent": self.sent,
                "dropped": self.dropped,
                "queued": len(self.queue),
                "error": self.error,
            }

    def _run(self):
        while True:
            self.wake.wait()
            with self.lock:
                if self.closed:
                    return
                if not self.queue:
                    self.wake.clear()
                    continue
                session, sample = self.queue.popleft()
                sock, target = self.socket, self.target
            if sock is None or session != self.session:
                continue
            if not 0 <= time.monotonic() - sample.received_monotonic <= self.max_age_s:
                with self.lock:
                    self.dropped += 1
                    self.error = "遥测样本已过期"
                continue
            try:
                raw = json.dumps(aisheng_telemetry_message(sample), separators=(",", ":"),
                                 ensure_ascii=False, allow_nan=False).encode("utf-8")
                sock.sendto(raw, target)
                with self.lock:
                    self.sent += 1
                    self.error = ""
            except (OSError, TypeError, ValueError) as exc:
                with self.lock:
                    self.dropped += 1
                    self.error = f"艾晟报文发送失败：{exc}"

    def close(self):
        self.stop()
        with self.lock:
            self.closed = True
            self.wake.set()
        self.thread.join(timeout=1.0)
