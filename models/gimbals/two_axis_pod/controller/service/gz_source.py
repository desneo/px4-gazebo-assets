"""Gazebo transport 节点与仿真时间到单调时间的映射。

视频由网关按配置订阅，姿态和跟踪完全由模型插件处理。
"""
import threading
import time
from collections import deque
from gz.msgs10.clock_pb2 import Clock
from gz.transport13 import Node


class GzSource:
    def __init__(self, world, gimbal_model):
        self.node = Node()
        self._lock = threading.Lock()
        self._offsets = deque(maxlen=201)
        self._clock_offset = 0.0
        self.node.subscribe(Clock, f"/world/{world}/clock", self._on_clock)

    def _on_clock(self, msg):
        sim_time = msg.sim.sec + msg.sim.nsec * 1e-9
        if sim_time <= 0:
            return
        with self._lock:
            self._offsets.append(time.monotonic() - sim_time)
            if len(self._offsets) >= 5:
                values = sorted(self._offsets)
                self._clock_offset = values[len(values) // 2]

    @property
    def clock_offset(self):
        with self._lock:
            return self._clock_offset
