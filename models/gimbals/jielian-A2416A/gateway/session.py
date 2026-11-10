"""A2416A 单客户端回传租约、命令白名单和重复帧抑制。"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable

from protocol import codec as proto


@dataclass(frozen=True)
class SessionResult:
    accepted: bool
    frame: proto.ControlFrame | None = None
    duplicate: bool = False
    error: str = ""


_ZERO_PARAMETER_COMMANDS = proto.SUPPORTED_COMMANDS - frozenset((
    proto.CMD_STREAM_SET,
    proto.CMD_CROSSHAIR_MOVE,
    proto.CMD_AIM_TRIM,
    proto.CMD_POINT_LOCK,
    proto.CMD_BOX_LOCK,
    proto.CMD_IP_SET,
))


def _validate_parameters(frame: proto.ControlFrame) -> None:
    cmd = frame.command
    values = (frame.p1, frame.p2, frame.p3, frame.p4)
    if cmd in _ZERO_PARAMETER_COMMANDS and any(values):
        raise ValueError(f"{frame.name} 的参数必须为零")
    if cmd in (proto.CMD_CROSSHAIR_MOVE, proto.CMD_AIM_TRIM):
        if not (-100 <= frame.p1 <= 100 and -100 <= frame.p2 <= 100):
            raise ValueError(f"{frame.name} 的 p1/p2 必须为 -100～100")
        if frame.p3 or frame.p4:
            raise ValueError(f"{frame.name} 的 p3/p4 必须为零")
    elif cmd == proto.CMD_POINT_LOCK:
        if not (0 <= frame.p1 <= 1919 and 0 <= frame.p2 <= 1079):
            raise ValueError("point_lock 坐标超出 1920×1080 像素范围")
        if frame.p3 or frame.p4:
            raise ValueError("point_lock 的 p3/p4 必须为零")
    elif cmd == proto.CMD_BOX_LOCK:
        if not (0 <= frame.p1 <= 1919 and 0 <= frame.p2 <= 1079):
            raise ValueError("box_lock 左上角超出 1920×1080 像素范围")
        if not (1 <= frame.p3 <= 1920 and 1 <= frame.p4 <= 1080):
            raise ValueError("box_lock 宽高必须为 1～1920 / 1～1080")
        if frame.p1 + frame.p3 > 1920 or frame.p2 + frame.p4 > 1080:
            raise ValueError("box_lock 区域必须完全位于 1920×1080 图像内")
    elif cmd == proto.CMD_STREAM_SET:
        if not 0 <= frame.p1 <= 800 or any((frame.p2, frame.p3, frame.p4)):
            raise ValueError("stream_set 仅允许 p1=0～800（单位0.01 Mbps）")
    elif cmd == proto.CMD_IP_SET:
        if not (0 <= frame.p1 <= 255 and 0 <= frame.p2 <= 255):
            raise ValueError("ip_set 的 X1/X2 必须为 0～255")
        if frame.p3 or frame.p4:
            raise ValueError("ip_set 的 p3/p4 必须为零")


class ControlSession:
    """合法白名单帧续租，回传地址始终使用最近合法源地址。

    三次发送的离散命令在 ``dedup_s`` 内只执行一次，但每个合法重复帧仍
    续租。心跳和光标连续移动不去重。无有效控制帧超过 ``lease_s`` 后只
    产生一次 ``peer_timeout`` 事件。
    """
    def __init__(self, lease_s: float = 2.0, dedup_s: float = 0.2, *,
                 clock: Callable[[], float] = time.monotonic):
        for value, name in ((lease_s, "lease_s"), (dedup_s, "dedup_s")):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} 必须为正有限数值")
        self.lease_s = float(lease_s)
        self.dedup_s = float(dedup_s)
        self.clock = clock
        self.peer: tuple[str, int] | None = None
        self.last_seen: float | None = None
        self.last_frame: proto.ControlFrame | None = None
        self.last_error = ""
        self._last_discrete_key: tuple[int, int, int, int, int] | None = None
        self._last_discrete_time: float | None = None

    def receive(self, raw: bytes, peer: tuple[str, int], now: float | None = None) -> SessionResult:
        now = self.clock() if now is None else now
        self.poll(now)
        try:
            frame = proto.decode_control(raw)
            if frame.command not in proto.SUPPORTED_COMMANDS:
                raise ValueError(f"不支持的捷联导引头命令 0x{frame.command:02X}")
            _validate_parameters(frame)
        except ValueError as exc:
            self.last_error = str(exc)
            return SessionResult(False, error=self.last_error)

        self.peer = peer
        self.last_seen = now
        self.last_frame = frame
        self.last_error = ""
        duplicate = False
        if frame.command not in proto.CONTINUOUS_COMMANDS:
            key = (frame.command, frame.p1, frame.p2, frame.p3, frame.p4)
            duplicate = (key == self._last_discrete_key
                         and self._last_discrete_time is not None
                         and now - self._last_discrete_time < self.dedup_s)
            if not duplicate:
                self._last_discrete_key = key
                self._last_discrete_time = now
        return SessionResult(True, frame=frame, duplicate=duplicate)

    def poll(self, now: float | None = None) -> str | None:
        now = self.clock() if now is None else now
        if (self.peer is not None and self.last_seen is not None
                and now - self.last_seen >= self.lease_s):
            self.peer = None
            self.last_seen = None
            self.last_frame = None
            self._last_discrete_key = None
            self._last_discrete_time = None
            return "peer_timeout"
        return None

    def telemetry_peer(self, now: float | None = None) -> tuple[str, int] | None:
        self.poll(now)
        return self.peer
