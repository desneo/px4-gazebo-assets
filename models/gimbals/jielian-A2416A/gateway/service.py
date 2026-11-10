"""A2416A UDP网关核心和100 Hz运行循环。"""
from __future__ import annotations

from dataclasses import dataclass, replace
import logging
import socket
import threading
import time
from typing import Any, Callable, Mapping

from protocol import codec as proto
from .session import ControlSession, SessionResult


LOGGER = logging.getLogger(__name__)


@dataclass
class GatewayState:
    recognition_enabled: bool = False
    auto_lock_enabled: bool = False
    zoom: int = 1
    enhancement_enabled: bool = False
    video_enabled: bool = True
    osd_mode: str = "full"
    bitrate_mbps: float = 2.0
    logical_ip: tuple[int, int] = (8, 2)


class GatewayCore:
    """不依赖Gazebo和socket的可测试网关状态机。"""
    def __init__(self, *, clock: Callable[[], float] = time.monotonic,
                 command_sink: Callable[[dict[str, Any]], None] | None = None,
                 state_sink: Callable[[GatewayState], None] | None = None):
        self.clock = clock
        self.session = ControlSession(clock=clock)
        self.command_sink = command_sink
        self.state_sink = state_sink
        self.state = GatewayState()
        self.latest_measurement: dict[str, Any] = {}
        self.last_control: proto.ControlFrame | None = None
        self.start_mono = clock()
        self.tx_sequence = 0
        self.last_rejection = ""
        self.lock = threading.RLock()

    def update_measurement(self, measurement: Mapping[str, Any]) -> None:
        if not isinstance(measurement, Mapping):
            raise ValueError("遥测测量必须为映射")
        with self.lock:
            self.latest_measurement = dict(measurement)

    def receive_control(self, raw: bytes, peer: tuple[str, int],
                        now: float | None = None) -> SessionResult:
        with self.lock:
            effective_now = self.clock() if now is None else now
            # 先由Core消费超时事件，避免过期后的首个坏包在Session内部清理
            # 租约却没有向插件发布session_timeout。
            self.poll(effective_now)
            result = self.session.receive(raw, peer, effective_now)
            if not result.accepted:
                self.last_rejection = result.error
                return result
            assert result.frame is not None
            self.last_control = result.frame
            if not result.duplicate:
                self._execute(result.frame)
            return result

    def _publish(self, payload: dict[str, Any]) -> None:
        if self.command_sink is not None:
            self.command_sink(payload)

    def _state_changed(self) -> None:
        if self.state_sink is not None:
            self.state_sink(replace(self.state))

    def _execute(self, frame: proto.ControlFrame) -> None:
        cmd = frame.command
        if cmd == proto.CMD_HEARTBEAT:
            self._publish({"cmd": "heartbeat", "p1": 0, "p2": 0, "p3": 0, "p4": 0})
            return
        if cmd == proto.CMD_RECOGNITION_ON:
            self.state.recognition_enabled = True
        elif cmd == proto.CMD_RECOGNITION_OFF:
            self.state.recognition_enabled = False
            self.state.auto_lock_enabled = False
        elif cmd == proto.CMD_AUTO_LOCK_ON:
            self.state.auto_lock_enabled = True
        elif cmd == proto.CMD_AUTO_LOCK_OFF:
            self.state.auto_lock_enabled = False
        elif cmd in (proto.CMD_POINT_LOCK, proto.CMD_BOX_LOCK,
                     proto.CMD_CROSSHAIR_LOCK, proto.CMD_CANCEL_TRACK):
            # 人工锁定或显式取消不能被仍开启的自动锁定立即覆盖。
            self.state.auto_lock_enabled = False
        elif cmd == proto.CMD_ZOOM_IN:
            self.state.zoom = min(4, self.state.zoom * 2)
        elif cmd == proto.CMD_ZOOM_OUT:
            self.state.zoom = max(1, self.state.zoom // 2)
        elif cmd == proto.CMD_IMAGE_ENHANCE_ON:
            self.state.enhancement_enabled = True
        elif cmd == proto.CMD_IMAGE_ENHANCE_OFF:
            self.state.enhancement_enabled = False
        elif cmd == proto.CMD_STREAM_SET:
            self.state.bitrate_mbps = 2.0 if frame.p1 == 0 else frame.p1 / 100.0
        elif cmd == proto.CMD_IP_SET:
            self.state.logical_ip = (frame.p1, frame.p2)
        elif cmd == proto.CMD_VIDEO_ON:
            self.state.video_enabled = True
        elif cmd == proto.CMD_VIDEO_OFF:
            self.state.video_enabled = False
        elif cmd == proto.CMD_OSD_FULL:
            self.state.osd_mode = "full"
        elif cmd == proto.CMD_OSD_HALF:
            self.state.osd_mode = "half"
        elif cmd == proto.CMD_OSD_OFF:
            self.state.osd_mode = "off"

        name = proto.command_name(cmd)
        payload = {
            "cmd": name, "command": cmd,
            "p1": frame.p1, "p2": frame.p2, "p3": frame.p3, "p4": frame.p4,
        }
        if cmd == proto.CMD_CROSSHAIR_MOVE:
            payload["max_speed_px_s"] = 400
        self._publish(payload)
        self._state_changed()

    def poll(self, now: float | None = None) -> str | None:
        with self.lock:
            event = self.session.poll(now)
            if event == "peer_timeout":
                self.state.recognition_enabled = False
                self.state.auto_lock_enabled = False
                # 使用插件已经支持的命令保证超时真正停止识别、自动锁定和
                # 人工跟踪，而不是发送只能用于日志的私有伪命令。
                for command in ("recognition_off", "auto_lock_off", "cancel_track"):
                    self._publish({"cmd": command, "reason": "control_timeout",
                                   "p1": 0, "p2": 0, "p3": 0, "p4": 0})
                self._state_changed()
            return event

    @staticmethod
    def _gate(measurement: Mapping[str, Any]) -> tuple[int, int, int, int]:
        gate = measurement.get("gate_px", (0, 0, 0, 0))
        if isinstance(gate, Mapping):
            x, y = gate.get("x", 0), gate.get("y", 0)
            w, h = gate.get("w", 0), gate.get("h", 0)
        elif isinstance(gate, (list, tuple)) and len(gate) == 4:
            x, y, w, h = gate
        else:
            return 0, 0, 0, 0
        try:
            return (round(float(x) + float(w) / 2.0),
                    round(float(y) + float(h) / 2.0),
                    max(0, round(float(w))), max(0, round(float(h))))
        except (TypeError, ValueError, OverflowError):
            return 0, 0, 0, 0

    def build_telemetry(self, now: float | None = None) -> proto.TelemetryFrame:
        now = self.clock() if now is None else now
        with self.lock:
            measurement = dict(self.latest_measurement)
            state = replace(self.state)
            control = self.last_control
            self.tx_sequence += 1
            tx_sequence = self.tx_sequence

        tracking_state = str(measurement.get("tracking_state", "IDLE")).upper()
        tracking_valid = tracking_state in ("TRACK", "TRACKING", "TEMP_LOST") and bool(
            measurement.get("locked_target_name"))
        gate_x, gate_y, gate_w, gate_h = self._gate(measurement)
        if not tracking_valid:
            gate_x = gate_y = gate_w = gate_h = 0
        miss = measurement.get("misalignment_deg", {})
        if not isinstance(miss, Mapping):
            miss = {}
        pitch_mis = float(miss.get("pitch", 0.0)) if tracking_valid else 0.0
        yaw_mis = float(miss.get("yaw", 0.0)) if tracking_valid else 0.0
        targets_data = measurement.get("targets", [])
        if not isinstance(targets_data, (list, tuple)):
            targets_data = []
        targets = proto.targets_from_mappings(
            [item for item in targets_data if isinstance(item, Mapping)])
        first_type = targets[0].type if tracking_valid else 0
        # 协议位图：bit4..3=单可见光源(01)，bit6..5=真值跟踪算法(01)。
        # 插件的 work_state 是IDLE/DETECTING/...枚举，不能直接并入线协议。
        work_state = 0x08 | 0x20
        if tracking_valid:
            work_state |= 0x01
        if state.video_enabled:
            work_state |= 0x80
        zoom_bits = {1: 1, 2: 2, 4: 3}[state.zoom]
        mode_bits = 3 if state.auto_lock_enabled else (1 if tracking_valid else 0)
        status2 = zoom_bits | (mode_bits << 2)
        attitude = measurement.get("aircraft_attitude_deg", {})
        if isinstance(attitude, Mapping):
            aircraft_pitch = float(attitude.get(
                "pitch", control.aircraft_pitch_deg if control else 0.0))
            aircraft_roll = float(attitude.get(
                "roll", control.aircraft_roll_deg if control else 0.0))
            aircraft_heading = float(attitude.get(
                "yaw", control.aircraft_heading_deg if control else 0.0))
        else:
            aircraft_pitch = control.aircraft_pitch_deg if control else 0.0
            aircraft_roll = control.aircraft_roll_deg if control else 0.0
            aircraft_heading = control.aircraft_heading_deg if control else 0.0
        aircraft_heading %= 360.0
        uptime_ms = max(0, round((now - self.start_mono) * 1000.0)) & 0xFFFFFFFF
        measurement_seq = int(measurement.get("measurement_seq", 0))
        return proto.TelemetryFrame(
            self_test=int(measurement.get("self_test", 1)),
            work_state=work_state,
            # 捷联式不输出框架角、平台角速率或LOS角速率。
            pitch_deg=0.0, yaw_deg=0.0,
            gate_x=gate_x, gate_y=gate_y,
            gyro_pitch_dps=0.0, gyro_yaw_dps=0.0,
            pitch_misalignment_deg=pitch_mis,
            yaw_misalignment_deg=yaw_mis,
            tracking_confidence=(float(measurement.get("tracking_confidence", 25.5))
                                 if tracking_valid else 0.0),
            pitch_los_dps=0.0, yaw_los_dps=0.0,
            status_control=0,
            autonomous=1 if state.auto_lock_enabled else 0,
            target_type=first_type,
            target_confidence=targets[0].confidence if tracking_valid else 0.0,
            targets=targets,
            status_control2=status2,
            gate_w=gate_w, gate_h=gate_h,
            pitch_att_deg=0.0, yaw_att_deg=0.0, roll_att_deg=0.0,
            tracking_target_type=first_type,
            laser_distance_m=0.0, laser_flags=0,
            aircraft_pitch_deg=aircraft_pitch,
            aircraft_roll_deg=aircraft_roll,
            aircraft_heading_deg=aircraft_heading,
            ip=state.logical_ip,
            timestamp_ms=uptime_ms,
            measurement_seq=measurement_seq,
        )

    def telemetry_packet(self, now: float | None = None) -> bytes:
        return proto.encode_telemetry(self.build_telemetry(now))


class UdpGateway:
    """单socket控制/遥测网关；遥测按100 Hz调度而不是随50 Hz测量触发。"""
    def __init__(self, core: GatewayCore, host: str = "0.0.0.0", port: int = 8001,
                 *, clock: Callable[[], float] = time.monotonic):
        self.core = core
        self.clock = clock
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.settimeout(0.002)
        self.running = False
        self.rx_control = 0
        self.rejected_control = 0
        self.tx_telemetry = 0
        self.tx_errors = 0

    @property
    def address(self) -> tuple[str, int]:
        return self.sock.getsockname()

    def close(self) -> None:
        self.running = False
        self.sock.close()

    def step(self, next_tx: float) -> float:
        try:
            data, peer = self.sock.recvfrom(2048)
        except socket.timeout:
            pass
        else:
            result = self.core.receive_control(data, peer)
            if result.accepted:
                self.rx_control += 1
            else:
                self.rejected_control += 1
                LOGGER.warning("拒绝来自 %s:%s 的控制帧：%s",
                               peer[0], peer[1], result.error)
        now = self.clock()
        self.core.poll(now)
        if now >= next_tx:
            peer = self.core.session.telemetry_peer(now)
            if peer is not None:
                try:
                    self.sock.sendto(self.core.telemetry_packet(now), peer)
                    self.tx_telemetry += 1
                except OSError:
                    self.tx_errors += 1
            # 跳过积压时隙，不能突发补发陈旧遥测。
            next_tx += (int(max(0.0, now - next_tx) / 0.01) + 1) * 0.01
        return next_tx

    def run(self) -> None:
        self.running = True
        next_tx = self.clock()
        try:
            while self.running:
                next_tx = self.step(next_tx)
        finally:
            self.running = False
