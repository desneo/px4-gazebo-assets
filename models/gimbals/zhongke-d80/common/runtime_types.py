"""二进制接收测量及本地会话元信息，不扩展厂商帧。"""
from __future__ import annotations
from dataclasses import dataclass
import math
from types import MappingProxyType
from collections.abc import Mapping


@dataclass(frozen=True)
class ReceivedTelemetry:
    rx_seq: int
    received_monotonic: float
    fields: Mapping[str, int | float]
    source: tuple[str, int]
    raw: bytes

    def __post_init__(self):
        if type(self.rx_seq) is not int or self.rx_seq < 0:
            raise ValueError('本地接收序号必须为非负整数')
        if not math.isfinite(self.received_monotonic) or self.received_monotonic < 0:
            raise ValueError('接收单调时间无效')
        if not isinstance(self.raw, bytes) or len(self.raw) != 32:
            raise ValueError('接收测量必须保留32字节原始二进制帧')
        object.__setattr__(self, 'fields', MappingProxyType(dict(self.fields)))


@dataclass(frozen=True)
class JointSample:
    sim_time: float
    step_seq: int
    yaw: float
    pitch: float
    yaw_vel: float
    pitch_vel: float

    @property
    def valid(self):
        return self.step_seq >= 0 and all(math.isfinite(x) for x in
            (self.sim_time, self.yaw, self.pitch, self.yaw_vel, self.pitch_vel))
