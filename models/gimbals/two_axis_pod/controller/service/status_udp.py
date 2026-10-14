"""独立于地面站会话的 UDP JSON 状态副本上报，只使用 Python 标准库。"""
from dataclasses import dataclass, replace
import ipaddress
import json
import math
from pathlib import Path
import socket
import threading
import time


MAX_DATAGRAM = 8 * 1024
DEFAULT_CONFIG = Path(__file__).with_name("status_udp.json")


@dataclass(frozen=True)
class StatusUdpConfig:
    enabled: bool = False
    ip: str = "127.0.0.1"
    port: int = 9000
    hz: float = 20.0
    stale_timeout_s: float = 1.0

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError("enabled 必须为 true 或 false")
        if not isinstance(self.ip, str):
            raise ValueError("ip 必须为 IPv4 或 IPv6 地址字符串")
        try:
            ipaddress.ip_address(self.ip)
        except ValueError as exc:
            raise ValueError("ip 必须为有效的 IPv4 或 IPv6 地址") from exc
        if type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError("port 必须为 1～65535 的整数")
        for name, value in (("hz", self.hz), ("stale_timeout_s", self.stale_timeout_s)):
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(f"{name} 必须为正的有限数值")
        if self.hz > 100:
            raise ValueError("hz 不得超过 100")


def load_status_config(path=DEFAULT_CONFIG, *, ip=None, port=None, hz=None):
    """读取配置；命令行 IP 和端口必须同时给出，并启用额外上报。"""
    if (ip is None) != (port is None):
        raise ValueError("--status-ip 和 --status-port 必须同时指定")
    try:
        values = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取状态上报配置 {path}: {exc}") from exc
    if not isinstance(values, dict):
        raise ValueError("状态上报配置必须为 JSON 对象")
    try:
        config = StatusUdpConfig(**values)
    except TypeError as exc:
        raise ValueError(f"状态上报配置包含未知字段: {exc}") from exc
    overrides = {}
    if ip is not None:
        overrides.update(enabled=True, ip=ip, port=port)
    if hz is not None:
        overrides["hz"] = hz
    return replace(config, **overrides)


class StatusUdpPublisher:
    """独立 socket、独立发送线程；异常丢弃当前包，不阻塞 station 控制。"""

    def __init__(self, config, *, log=print):
        self.config = config
        self.log = log
        self._stop = threading.Event()
        self._thread = None
        self._last_error = -math.inf
        self._socket = None
        if config.enabled:
            family = socket.AF_INET6 if ipaddress.ip_address(config.ip).version == 6 else socket.AF_INET
            self._socket = socket.socket(family, socket.SOCK_DGRAM)
            self._socket.setblocking(False)

    def _error(self, message):
        now = time.monotonic()
        if now - self._last_error >= 5.0:
            self._last_error = now
            self.log(f"[status-udp] {message}")

    def publish(self, snapshot, received_at):
        """一个数据报是一份原有遥测 JSON；尚无遥测或超过时效时不发送旧状态。"""
        if self._socket is None or self._stop.is_set() or snapshot is None or received_at is None:
            return False
        if time.monotonic() - received_at > self.config.stale_timeout_s:
            return False
        try:
            data = json.dumps(snapshot, ensure_ascii=False, allow_nan=False,
                              separators=(",", ":")).encode("utf-8")
            if len(data) > MAX_DATAGRAM:
                self._error(f"丢弃超长状态报文 {len(data)}B")
                return False
            self._socket.sendto(data, (self.config.ip, self.config.port))
            return True
        except (OSError, TypeError, ValueError) as exc:
            self._error(f"状态发送失败: {exc}")
            return False

    def start(self, snapshot_provider):
        if self._socket is None:
            return
        if self._thread is not None:
            raise RuntimeError("状态上报线程已启动")
        self.log(f"[status-udp] 额外上报 -> {self.config.ip}:{self.config.port}，"
                 f"{self.config.hz:g} Hz（独立于 station）")
        self._thread = threading.Thread(target=self._run, args=(snapshot_provider,),
                                        name="status-udp", daemon=True)
        self._thread.start()

    def _run(self, snapshot_provider):
        period = 1.0 / self.config.hz
        deadline = time.monotonic()
        while not self._stop.is_set():
            try:
                self.publish(*snapshot_provider())
            except Exception as exc:  # 单次取样失败不终止独立发送循环。
                self._error(f"读取状态失败: {exc}")
            deadline += period
            now = time.monotonic()
            if deadline <= now:
                deadline = now + period  # 耗时操作后不补发积压报文。
            self._stop.wait(max(0.0, deadline - now))

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._socket is not None:
            self._socket.close()
