"""逐份接收的中科二进制遥测转换为有限数值 JSON，独立线程发送。"""
from __future__ import annotations

from collections import deque
import ipaddress
import json
import math
from pathlib import Path
import re
import socket
import threading
import time

from common.runtime_types import ReceivedTelemetry
from common.protocol import decode_telemetry
from ground_station.packet_rate import PacketRate

FIELDS = ('status1', 'status2', 'status3', 'zoom', 'offset_x', 'offset_y',
          'frame_roll', 'frame_pitch', 'frame_yaw', 'roll_rate', 'pitch_rate',
          'yaw_rate', 'laser_range', 'self_check')
BYTE_FIELDS = {'status1', 'status2', 'status3', 'self_check'}


def endpoint(ip, port):
    try:
        address = str(ipaddress.ip_address(str(ip).strip()))
        if isinstance(port, bool) or not str(port).strip().isdigit():
            raise ValueError
        number = int(port)
        if not 1 <= number <= 65535:
            raise ValueError
        return address, number
    except ValueError as exc:
        raise ValueError('接收地址须为合法 IP，端口须在 1～65535 内') from exc


def save_destination(path, ip, port):
    ip, port = endpoint(ip, port)
    path = Path(path)
    lines = path.read_text(encoding='utf-8').splitlines(keepends=True)
    section = None
    found = set()
    for index, line in enumerate(lines):
        match = re.match(r'\s*\[([^]]+)\]', line)
        if match:
            section = match.group(1).strip()
        if section == 'forward':
            match = re.match(r'(\s*)(ip|port)\s*=.*', line)
            if match:
                key = match.group(2)
                lines[index] = f'{match.group(1)}{key} = {ip if key == "ip" else port}\n'
                found.add(key)
    if found != {'ip', 'port'}:
        raise ValueError('地面站配置须包含 [forward] ip 和 port')
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(''.join(lines), encoding='utf-8')
    temporary.replace(path)


def json_measurement(sample):
    if not isinstance(sample, ReceivedTelemetry) or len(sample.raw) != 32:
        raise ValueError('只接受实际中科二进制接收样本')
    values = dict(sample.fields)
    if set(values) != set(FIELDS):
        raise ValueError('中科遥测字段集合不匹配')
    for name, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'中科字段 {name} 必须为有限数值')
        if name in BYTE_FIELDS and (type(value) is not int or not 0 <= value <= 255):
            raise ValueError(f'中科字段 {name} 必须为原始字节')
    decoded = decode_telemetry(sample.raw)
    if values != decoded:
        raise ValueError('中科接收值与原始二进制帧不一致')
    return decoded


class TelemetryForwarder:
    def __init__(self, *, max_age_s=1.0):
        self.max_age_s = float(max_age_s)
        if not math.isfinite(self.max_age_s) or self.max_age_s <= 0:
            raise ValueError('中科转发有效期须为正有限秒数')
        self._lock = threading.RLock()
        self._wake = threading.Event()
        self._queue = deque(maxlen=16)
        self._socket = None
        self._target = None
        self._closed = False
        self._session = 0
        self._last_rx_seq = -1
        self.sent = self.dropped = 0
        self.error = ''
        self._rate = PacketRate(active=False)
        self._thread = threading.Thread(target=self._run, name='zhongke-d80-forward', daemon=True)
        self._thread.start()

    def start(self, ip, port):
        target = endpoint(ip, port)
        family = socket.AF_INET6 if ':' in target[0] else socket.AF_INET
        sock = socket.socket(family, socket.SOCK_DGRAM)
        sock.setblocking(False)
        with self._lock:
            if self._closed:
                sock.close()
                raise RuntimeError('中科转发器已关闭')
            if self._socket:
                self._socket.close()
            self._socket, self._target = sock, target
            self._session += 1
            self._queue.clear()
            self.sent = self.dropped = 0
            self.error = ''
            self._rate.reset()

    def offer(self, sample):
        with self._lock:
            if self._closed or self._socket is None:
                return
            if not isinstance(sample, ReceivedTelemetry):
                self.dropped += 1
                self.error = '转发来源不是有效二进制接收样本'
                return
            if sample.rx_seq <= self._last_rx_seq:
                return
            self._last_rx_seq = sample.rx_seq
            if len(self._queue) == self._queue.maxlen:
                self.dropped += 1
            self._queue.append((self._session, sample))
            self._wake.set()

    def stop(self):
        with self._lock:
            self._session += 1
            if self._socket:
                self._socket.close()
            self._socket = None
            self._queue.clear()
            self.error = ''
            self._rate.reset(active=False)

    def snapshot(self):
        with self._lock:
            rate = self._rate.snapshot()
            return dict(active=self._socket is not None, target=self._target,
                        sent=self.sent, dropped=self.dropped, queued=len(self._queue),
                        error=self.error, send_hz=rate['hz'], warming_up=rate['warming_up'])

    def _run(self):
        while True:
            self._wake.wait()
            with self._lock:
                if self._closed:
                    return
                if not self._queue:
                    self._wake.clear()
                    continue
                session, sample = self._queue.popleft()
                if self._socket is None or session != self._session:
                    continue
                age = time.monotonic() - sample.received_monotonic
                if not 0 <= age <= self.max_age_s:
                    self.dropped += 1
                    self.error = '中科遥测已过期，已丢弃'
                    continue
                try:
                    payload = json_measurement(sample)
                    payload['timestamp'] = time.time_ns() // 1_000_000
                    raw = json.dumps(payload, separators=(',', ':'), allow_nan=False).encode('utf-8')
                    self._socket.sendto(raw, self._target)
                    self.sent += 1
                    self._rate.record()
                    self.error = ''
                except (ValueError, TypeError, OverflowError, OSError) as exc:
                    self.dropped += 1
                    self.error = f'中科报文发送失败：{exc}'

    def close(self):
        self.stop()
        with self._lock:
            self._closed = True
            self._wake.set()
        self._thread.join(timeout=1)


class ForwardInputs:
    def __init__(self, ip, port):
        self.values = dict(ip=str(ip), port=str(port))
        self.focus = None
        self.selected = False

    def activate(self, field):
        self.focus = field
        self.selected = True

    def key(self, key):
        if self.focus is None:
            return False
        if key in (13, 10, 27):
            self.focus = None
            return True
        if key == 9:
            self.activate('port' if self.focus == 'ip' else 'ip')
            return True
        value = self.values[self.focus]
        if key in (8, 127):
            self.values[self.focus] = '' if self.selected else value[:-1]
            self.selected = False
        elif 0 <= key < 128 and chr(key) in ('0123456789.:abcdefABCDEF' if self.focus == 'ip' else '0123456789'):
            self.values[self.focus] = ('' if self.selected else value) + chr(key)
            self.selected = False
        return True
