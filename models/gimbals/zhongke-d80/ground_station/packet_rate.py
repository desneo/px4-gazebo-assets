"""按真实单调时钟统计最近一段时间内的数据包速率。"""
from collections import deque
import threading
import time


class PacketRate:
    """2 秒滑动窗口；首 0.25 秒预热，不依赖界面读取或刷新次数。"""

    def __init__(self, window_s=2.0, warmup_s=0.25, *, active=True):
        if not 0 < warmup_s <= window_s < float("inf"):
            raise ValueError("频率窗口须为正的有限秒数，且不短于预热时间")
        self.window_s = float(window_s)
        self.warmup_s = float(warmup_s)
        self._lock = threading.Lock()
        self._times = deque()
        self._first_at = None
        self._active = active

    def _expire(self, now):
        cutoff = now - self.window_s
        while self._times and self._times[0] <= cutoff:
            self._times.popleft()

    def record(self, now=None):
        with self._lock:
            now = time.monotonic() if now is None else now
            if not self._active:
                return
            if self._first_at is None:
                self._first_at = now
            self._times.append(now)
            self._expire(now)

    def reset(self, *, active=True):
        with self._lock:
            self._times.clear()
            self._first_at = None
            self._active = active

    def snapshot(self, now=None):
        with self._lock:
            now = time.monotonic() if now is None else now
            self._expire(now)
            age = 0.0 if self._first_at is None else max(0.0, now - self._first_at)
            warming = bool(self._active and self._times and age < self.warmup_s)
            count = len(self._times)
            if not self._active or not count or warming:
                hz = 0.0
            elif age < self.window_s:
                # 首包是启动期观察区间的起点，N 包之间有 N-1 个间隔。
                hz = max(0, count - 1) / age
            else:
                hz = count / self.window_s
            return {"hz": hz, "warming_up": warming, "samples": count,
                    "window_s": self.window_s}
