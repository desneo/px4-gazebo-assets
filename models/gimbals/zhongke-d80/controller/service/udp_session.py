"""中科仿真 UDP 单客户端租约与手动停车约定；不扩展厂商帧。"""
from __future__ import annotations

import math
import time

from common import protocol


class UdpSession:
    """验证帧后续租；空指令保活与非零手动速度watchdog相互独立。

    on_stop(reason) 在租约或人工速度超时时调用，供网关停止实际控制。
    本类不创建socket，不生成ACK，也不改写标准报文。
    """
    def __init__(self, lease_s=2.0, manual_timeout_s=0.5, *,
                 clock=time.monotonic, on_stop=None, accepted_commands=None):
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0
               for v in (lease_s, manual_timeout_s)):
            raise ValueError("租约和手动超时必须为正有限秒数")
        self.lease_s = float(lease_s)
        self.manual_timeout_s = float(manual_timeout_s)
        self.clock = clock
        self.on_stop = on_stop
        self.accepted_commands = (protocol.SUPPORTED_COMMANDS if accepted_commands is None
                                  else frozenset(accepted_commands))
        self.peer = None
        self.last_seen = None
        self.last_manual = None
        self.mode = "IDLE"
        self.manual_nonzero = False
        self.last_error = ""

    def _stop(self, reason):
        self.manual_nonzero = False
        self.last_manual = None
        self.mode = "IDLE"
        if self.on_stop is not None:
            self.on_stop(reason)

    def poll(self, now=None):
        """轮询超时并最多报告一次事件；即使没有UDP流量也必须定期调用。"""
        now = self.clock() if now is None else now
        if self.peer is not None and now - self.last_seen >= self.lease_s:
            self.peer = None
            self.last_seen = None
            self._stop("peer_timeout")
            return "peer_timeout"
        if (self.mode == "MANUAL" and self.manual_nonzero
                and now - self.last_manual >= self.manual_timeout_s):
            self._stop("manual_timeout")
            return "manual_timeout"
        return None

    def receive(self, raw, peer, now=None):
        """返回可执行控制dict；拒绝帧返回None，错误不能续租或劫持对端。"""
        now = self.clock() if now is None else now
        # 真实时间过期独立于收到的帧是否有效。
        self.poll(now)
        try:
            command = protocol.decode_command(raw)
            if command["cmd"] not in self.accepted_commands:
                raise ValueError("该中科控制能力未实现")
        except ValueError as exc:
            self.last_error = str(exc)
            return None
        if self.peer is not None and self.peer != peer:
            self.last_error = "当前中科UDP租约属于其他客户端"
            return None
        self.peer = peer
        self.last_seen = now
        self.last_error = ""
        cmd = command["cmd"]
        if cmd == protocol.CMD_MANUAL:
            self.manual_nonzero = bool(command["x"] or command["y"])
            self.last_manual = now if self.manual_nonzero else None
            self.mode = "MANUAL" if self.manual_nonzero else "IDLE"
        elif cmd != protocol.CMD_NOP:
            # 接受模式切换时撤销旧手动计时，不能在HOME/TRACK中复活。
            self.manual_nonzero = False
            self.last_manual = None
            self.mode = {
                protocol.CMD_HOME: "HOME", protocol.CMD_FRAME_ANGLE: "GOTO",
                protocol.CMD_POINT_TRACK: "TRACK", protocol.CMD_AUTO_LOCK: "AUTO",
                protocol.CMD_STOP_TRACK: "IDLE",
            }[cmd]
        return command

    def telemetry_peer(self, now=None):
        self.poll(now)
        return self.peer
