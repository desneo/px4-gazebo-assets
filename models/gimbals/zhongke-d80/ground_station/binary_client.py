"""中科标准 UDP 客户端；单一 socket 收发，不添加握手或重传。"""
from __future__ import annotations

import ipaddress
import math
import socket
import threading
import time

from common.protocol import encode_command, decode_telemetry
from common.runtime_types import ReceivedTelemetry
from ground_station.packet_rate import PacketRate


class BinaryClient(threading.Thread):
    def __init__(self, host, port, *, local_bind=None, local_port=0,
                 max_age_s=1.0, on_telemetry=None, on_log=None):
        super().__init__(name='zhongke-d80-udp', daemon=True)
        try:
            address = ipaddress.ip_address(host)
            bind_address=ipaddress.ip_address(local_bind or ('::' if address.version==6 else '0.0.0.0'))
        except ValueError as exc:
            raise ValueError('中科远端及本地绑定地址必须为合法IP') from exc
        if (isinstance(port,bool) or not str(port).isdigit() or not 1<=int(port)<=65535):
            raise ValueError('中科远端UDP端口须在1～65535内')
        if (isinstance(local_port,bool) or not str(local_port).isdigit() or not 0<=int(local_port)<=65535):
            raise ValueError('中科本地接收端口须在0～65535内')
        if address.version!=bind_address.version:
            raise ValueError('中科本地绑定和远端UDP的IP版本必须一致')
        self.peer = (str(address), int(port))
        self.max_age_s = float(max_age_s)
        if not math.isfinite(self.max_age_s) or self.max_age_s <= 0:
            raise ValueError('中科遥测有效期须为正有限秒数')
        self.on_telemetry = on_telemetry
        self.on_log = on_log
        self._socket = socket.socket(socket.AF_INET6 if address.version == 6 else socket.AF_INET,
                                     socket.SOCK_DGRAM)
        try:
            self._socket.bind((str(bind_address),int(local_port)))
        except OSError as exc:
            self._socket.close()
            raise ValueError(f'中科本地接收端口绑定失败：{bind_address}:{local_port}：{exc}') from exc
        self.local_endpoint=tuple(self._socket.getsockname()[:2])
        self._socket.settimeout(.05)
        self._stop_event = threading.Event()
        self._lock = threading.RLock()
        self._latest = None
        self._received = 0
        self._rejected = 0
        self._error = ''
        self._closed = False
        self._rate = PacketRate()

    def send(self, command, *, x=0, y=0, p3=0, zoom_rate=0):
        packet = encode_command(command, x=x, y=y, p3=p3, zoom_rate=zoom_rate)
        with self._lock:
            if self._closed:
                return False
            try:
                self._socket.sendto(packet, self.peer)
                return True
            except OSError as exc:
                self._error = f'中科控制发送失败：{exc}'
                if self.on_log:
                    self.on_log(self._error)
                return False

    def run(self):
        heartbeat_at = 0.0
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now >= heartbeat_at:
                self.send(0x00)
                heartbeat_at = now + .2
            try:
                raw, source = self._socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if (source[0], source[1]) != self.peer:
                with self._lock:
                    self._rejected += 1
                continue
            try:
                fields = decode_telemetry(raw)
            except (ValueError, TypeError):
                with self._lock:
                    self._rejected += 1
                continue
            arrived = time.monotonic()
            with self._lock:
                if self._closed:
                    break
                self._received += 1
                sample = ReceivedTelemetry(rx_seq=self._received,
                                           received_monotonic=arrived,
                                           fields=fields, source=self.peer, raw=bytes(raw))
                self._latest = sample
                self._rate.record(arrived)
                self._error = ''
            if self.on_telemetry:
                try:
                    self.on_telemetry(sample)
                except Exception as exc:
                    if self.on_log:
                        self.on_log(f'中科遥测处理失败：{exc}')

    def snapshot(self):
        with self._lock:
            return self._latest

    def stats(self):
        with self._lock:
            age = (time.monotonic() - self._latest.received_monotonic
                   if self._latest is not None else float('inf'))
            rate = self._rate.snapshot()
            return dict(connected=not self._closed and age <= self.max_age_s,
                        received=self._received, rejected=self._rejected, age_s=age,
                        receive_hz=rate['hz'], warming_up=rate['warming_up'], error=self._error)

    def stop(self):
        with self._lock:
            if self._closed:
                return
            if self.ident is not None:
                self.send(0x0E)
                self.send(0x24)
            self._closed = True
            self._stop_event.set()
            self._socket.close()
            self._rate.reset(active=False)
        if self.is_alive() and threading.current_thread() is not self:
            self.join(timeout=1)
