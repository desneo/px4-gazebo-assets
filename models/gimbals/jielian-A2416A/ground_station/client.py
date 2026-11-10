"""A2416A 单 socket UDP 控制与100 Hz遥测客户端。"""
from __future__ import annotations

from dataclasses import dataclass
import ipaddress
import socket
import threading
import time
from typing import Callable

from common import protocol as proto


@dataclass(frozen=True)
class TelemetrySample:
    sequence: int
    received_monotonic: float
    received_monotonic_us: int
    raw: bytes
    frame: object


class TelemetryClient:
    """同一 UDP socket 发控制并接收网关回传遥测。

    离散命令连续发送三份。心跳和连续光标命令各发送一份，心跳线程固定
    10 Hz，不额外引入握手或确认报文。
    """

    HEARTBEAT_PERIOD = 0.1
    REPEAT_COUNT = 3
    CONTINUOUS_COMMANDS = frozenset((0x00, 0xB1))

    def __init__(self, host: str, port: int, *, on_telemetry: Callable | None = None,
                 socket_factory=socket.socket, start: bool = True):
        address = ipaddress.ip_address(host)
        if isinstance(port, bool) or not 1 <= int(port) <= 65535:
            raise ValueError("导引头 UDP 端口须在 1～65535 内")
        family = socket.AF_INET6 if address.version == 6 else socket.AF_INET
        self.peer = (str(address), int(port))
        self.on_telemetry = on_telemetry
        self.socket = socket_factory(family, socket.SOCK_DGRAM)
        self.socket.bind(("::" if address.version == 6 else "0.0.0.0", 0))
        self.socket.connect(self.peer)
        self.socket.settimeout(0.05)
        self.latest: TelemetrySample | None = None
        self.received = 0
        self.rejected = 0
        self.sent = 0
        self.last_error = ""
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        if start:
            self.start()

    def start(self):
        if self._threads:
            return
        self._threads = [
            threading.Thread(target=self._receive_loop, name="a2416a-telemetry", daemon=True),
            threading.Thread(target=self._heartbeat_loop, name="a2416a-heartbeat", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def send(self, command: int, **fields) -> int:
        raw = proto.make_control(command, **fields)
        count = 1 if command in self.CONTINUOUS_COMMANDS else self.REPEAT_COUNT
        sent = 0
        with self._lock:
            if self._stop.is_set():
                return 0
            for _ in range(count):
                try:
                    self.socket.send(raw)
                    sent += 1
                    self.sent += 1
                except (ConnectionRefusedError, OSError) as exc:
                    self.last_error = f"控制发送失败：{exc}"
                    break
        return sent

    def _heartbeat_loop(self):
        deadline = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= deadline:
                self.send(0x00)
                deadline += self.HEARTBEAT_PERIOD
                if deadline <= now:
                    deadline = now + self.HEARTBEAT_PERIOD
            self._stop.wait(max(0.0, deadline - time.monotonic()))

    def _receive_loop(self):
        while not self._stop.is_set():
            try:
                raw = self.socket.recv(65535)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                continue
            try:
                frame = proto.decode_telemetry(raw)
            except (TypeError, ValueError):
                with self._lock:
                    self.rejected += 1
                continue
            received_ns = time.monotonic_ns()
            with self._lock:
                self.received += 1
                sample = TelemetrySample(
                    sequence=self.received,
                    received_monotonic=received_ns / 1_000_000_000,
                    received_monotonic_us=received_ns // 1000,
                    raw=bytes(raw),
                    frame=frame,
                )
                self.latest = sample
                self.last_error = ""
            if self.on_telemetry is not None:
                try:
                    self.on_telemetry(sample)
                except Exception as exc:  # noqa: BLE001 - 回调不能终止收包线程
                    self.last_error = f"遥测回调失败：{exc}"

    def close(self):
        self._stop.set()
        try:
            self.socket.close()
        except OSError:
            pass
        for thread in self._threads:
            if thread is not threading.current_thread():
                thread.join(timeout=0.5)
