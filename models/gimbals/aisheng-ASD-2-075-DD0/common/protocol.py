"""艾晟 ASD-2-075-DD0 标准通信协议 V3.0 的二进制编解码。

控制帧 45 字节，遥测帧 104 字节，小端；校验和为字节 4 起求和取低 8 位。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import struct

CONTROL_HEADER = b"\xaa\x55"
CONTROL_LENGTH = 0x2D
CONTROL_SIZE = 45
TELEMETRY_HEADER = b"\x55\xaa"
TELEMETRY_LENGTH = 0x68
TELEMETRY_SIZE = 104

# 开关控制量
CMD_STREAM_SET = 0x11
CMD_IR_CORRECTION = 0x22
CMD_PIP = 0x44
CMD_FRAME_MOVE = 0xB1
CMD_TRACK_POINT_TRIM = 0xB2
CMD_GATE_GROW = 0xB3
CMD_GATE_SHRINK = 0xB4
CMD_POINT_LOCK = 0xB7
CMD_BOX_LOCK = 0xB8
CMD_ZOOM_IN = 0xB9
CMD_ZOOM_OUT = 0x9B
CMD_CANCEL_LOCK = 0x8E
CMD_FRAME_LOCK = 0xE9
CMD_FRAME_UNLOCK = 0x9E
CMD_SPECIFIED_ANGLE = 0x90
CMD_SCAN_ON = 0x7F
CMD_SCAN_OFF = 0xF7
CMD_DETECTOR_SWITCH = 0x53
CMD_GEO_TRACK = 0x3A
CMD_AUTO_RECOGNITION_ON = 0x3F
CMD_AUTO_RECOGNITION_OFF = 0xF3
CMD_AUTO_LOCK_ON = 0x5F
CMD_AUTO_LOCK_OFF = 0xF5
CMD_LASER_ON = 0x4F
CMD_LASER_OFF = 0xF4
CMD_IMAGE_ENHANCE_ON = 0x6F
CMD_IMAGE_ENHANCE_OFF = 0xF6
CMD_STORAGE_ON = 0x6E
CMD_STORAGE_OFF = 0xE6
CMD_IP_SET = 0x8F
CMD_CONTRAST_UP = 0x5C
CMD_CONTRAST_DOWN = 0xA3
CMD_BRIGHTNESS_UP = 0x5B
CMD_BRIGHTNESS_DOWN = 0xA4
CMD_RECENTER = 0x91
CMD_GYRO_DRIFT = 0x92
CMD_RECENTER_LOCK = 0xEA
CMD_STOW_LOCK = 0xEB
CMD_GYRO_CALIBRATE = 0x93
CMD_RTP_OFF = 0x10
CMD_RTP_ON = 0x01
CMD_PHOTO = 0x13
CMD_SPECIFIED_ANGLE_LOCK = 0x14
CMD_OSD_ON = 0x15
CMD_OSD_HALF = 0x16
CMD_OSD_OFF = 0x17
CMD_DEFOG_ON = 0x18
CMD_DEFOG_OFF = 0x19
CMD_BACKGROUND_LOCK = 0x20
CMD_BACKGROUND_UNLOCK = 0x21
CMD_TRACK_BOX_ON = 0x23
CMD_TRACK_BOX_OFF = 0x24
CMD_SPECIFIED_DOWN_ANGLE = 0x94
CMD_SERVO_TRACK_ON = 0x25
CMD_SERVO_TRACK_OFF = 0x26

COMMAND_NAMES = {
    CMD_FRAME_LOCK: "frame_lock",
    CMD_FRAME_UNLOCK: "frame_unlock",
    CMD_SPECIFIED_ANGLE: "specified_angle",
    CMD_SPECIFIED_ANGLE_LOCK: "specified_angle_lock",
    CMD_RECENTER: "recenter",
    CMD_POINT_LOCK: "point_lock",
    CMD_BOX_LOCK: "box_lock",
    CMD_CANCEL_LOCK: "cancel_lock",
    CMD_AUTO_LOCK_ON: "auto_lock_on",
    CMD_AUTO_LOCK_OFF: "auto_lock_off",
    CMD_FRAME_MOVE: "frame_move",
    CMD_ZOOM_IN: "zoom_in",
    CMD_ZOOM_OUT: "zoom_out",
    CMD_DETECTOR_SWITCH: "detector_switch",
}


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _scaled(value: float, scale: float, low: int, high: int) -> int:
    if not math.isfinite(value):
        value = 0.0
    return int(_clamp(round(value * scale), low, high))


def _checksum(frame: bytearray, start: int, end: int) -> int:
    return sum(frame[start:end]) & 0xFF


@dataclass
class ControlFrame:
    command: int
    p1: int = 0
    p2: int = 0
    p3: int = 0
    p4: int = 0
    aircraft_pitch_deg: float = 0.0
    aircraft_roll_deg: float = 0.0
    aircraft_heading_deg: float = 0.0
    status: int = 0
    aircraft_lat_deg: float = 0.0
    aircraft_lon_deg: float = 0.0
    aircraft_alt_m: float = 0.0
    timestamp_ms: int = 0

    @property
    def name(self) -> str:
        return COMMAND_NAMES.get(self.command, f"0x{self.command:02X}")


@dataclass
class TelemetryTarget:
    """标准帧中的单个目标；经纬度保留 1e-7 deg 原始整数。"""

    type: int = 0
    confidence: float = 0.0
    lon: int = 0
    lat: int = 0
    alt_m: float = 0.0


@dataclass
class TelemetryFrame:
    self_test: int = 0x01
    work_state: int = 0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    gate_x: int = 0
    gate_y: int = 0
    gyro_pitch_dps: float = 0.0
    gyro_yaw_dps: float = 0.0
    pitch_misalignment_deg: float = 0.0
    yaw_misalignment_deg: float = 0.0
    tracking_confidence: float = 0.0
    pitch_los_dps: float = 0.0
    yaw_los_dps: float = 0.0
    status_control: int = 0
    autonomous: int = 0
    target_type: int = 0
    target_confidence: float = 0.0
    target_lon_deg: float = 0.0
    target_lat_deg: float = 0.0
    target_alt_m: float = 0.0
    status_control2: int = 0
    gate_w: int = 0
    gate_h: int = 0
    pitch_att_deg: float = 0.0
    yaw_att_deg: float = 0.0
    roll_att_deg: float = 0.0
    tracking_target_type: int = 0
    laser_distance_m: float = 0.0
    laser_flags: int = 0
    aircraft_pitch_deg: float = 0.0
    aircraft_roll_deg: float = 0.0
    aircraft_heading_deg: float = 0.0
    ip: tuple[int, int] = (8, 2)
    timestamp_ms: int = 0
    # 未提供时使用上方首目标属性，其余三个目标为零，兼容现有网关。
    targets: tuple[TelemetryTarget, ...] = ()


def telemetry_targets(frame: TelemetryFrame) -> tuple[TelemetryTarget, ...]:
    if frame.targets:
        if len(frame.targets) != 4:
            raise ValueError("标准遥测必须包含四个目标槽位")
        return frame.targets
    return (TelemetryTarget(
        type=frame.target_type, confidence=frame.target_confidence,
        lon=_scaled(frame.target_lon_deg, 1e7, -2147483648, 2147483647),
        lat=_scaled(frame.target_lat_deg, 1e7, -2147483648, 2147483647),
        alt_m=frame.target_alt_m), TelemetryTarget(), TelemetryTarget(), TelemetryTarget())


def telemetry_message(frame: TelemetryFrame, timestamp_us: int) -> dict:
    """按 DytAishengTelemetry.msg 导出可由标准帧获得的全部字段。

    timestamp 为地面站接收时的 monotonic 微秒；device_timestamp 为设备上电毫秒。
    标准帧没有 los_omega_n/e/d，因此不输出这三个字段。
    """
    targets = telemetry_targets(frame)
    return {
        "timestamp": timestamp_us,
        "self_check": frame.self_test,
        "work_status": frame.work_state,
        "track_status": frame.work_state & 1,
        "pitch_angle": frame.pitch_deg,
        "yaw_angle": frame.yaw_deg,
        "gate_x": frame.gate_x,
        "gate_y": frame.gate_y,
        "gyro_pitch_rate": frame.gyro_pitch_dps,
        "gyro_yaw_rate": frame.gyro_yaw_dps,
        "pitch_misalignment": frame.pitch_misalignment_deg,
        "yaw_misalignment": frame.yaw_misalignment_deg,
        "tracking_confidence": frame.tracking_confidence,
        "pitch_los_rate": frame.pitch_los_dps,
        "yaw_los_rate": frame.yaw_los_dps,
        "status": frame.status_control,
        "autonomous": frame.autonomous,
        "target_type": [target.type for target in targets],
        "target_confidence": [target.confidence for target in targets],
        "target_lon": [target.lon for target in targets],
        "target_lat": [target.lat for target in targets],
        "target_alt": [target.alt_m for target in targets],
        "status2": frame.status_control2,
        "tracking_gate_w": frame.gate_w,
        "tracking_gate_h": frame.gate_h,
        "pitch_attitude": frame.pitch_att_deg,
        "yaw_attitude": frame.yaw_att_deg,
        "roll_attitude": frame.roll_att_deg,
        "tracking_target_type": frame.tracking_target_type,
        "laser_range": frame.laser_distance_m,
        "laser_update": frame.laser_flags,
        "aircraft_pitch": frame.aircraft_pitch_deg,
        "aircraft_roll": frame.aircraft_roll_deg,
        "aircraft_yaw": frame.aircraft_heading_deg,
        "ip_segment": frame.ip[0],
        "ip_node": frame.ip[1],
        "device_timestamp": frame.timestamp_ms,
    }


def encode_control(frame: ControlFrame) -> bytes:
    data = bytearray(CONTROL_SIZE)
    data[0:2] = CONTROL_HEADER
    data[2] = CONTROL_LENGTH
    data[3] = frame.command & 0xFF
    struct.pack_into("<h", data, 4, _scaled(frame.p1, 1, -32768, 32767))
    struct.pack_into("<h", data, 6, _scaled(frame.p2, 1, -32768, 32767))
    struct.pack_into("<H", data, 8, _scaled(frame.p3, 1, 0, 65535))
    struct.pack_into("<H", data, 10, _scaled(frame.p4, 1, 0, 65535))
    data[12] = 0
    struct.pack_into("<h", data, 13, _scaled(frame.aircraft_pitch_deg, 100, -32768, 32767))
    struct.pack_into("<h", data, 15, _scaled(frame.aircraft_roll_deg, 100, -32768, 32767))
    struct.pack_into("<H", data, 17, _scaled(frame.aircraft_heading_deg, 100, 0, 65535))
    data[26] = frame.status & 0xFF
    struct.pack_into("<i", data, 27, _scaled(frame.aircraft_lat_deg, 1e7, -2147483648, 2147483647))
    struct.pack_into("<i", data, 31, _scaled(frame.aircraft_lon_deg, 1e7, -2147483648, 2147483647))
    struct.pack_into("<I", data, 35, _scaled(frame.aircraft_alt_m, 1000, 0, 4294967295))
    data[39] = 0
    struct.pack_into("<I", data, 40, frame.timestamp_ms & 0xFFFFFFFF)
    data[44] = _checksum(data, 3, 44)
    return bytes(data)


def decode_control(data: bytes) -> ControlFrame:
    if len(data) != CONTROL_SIZE:
        raise ValueError(f"控制帧长度必须为 {CONTROL_SIZE}，实际 {len(data)}")
    if data[0:2] != CONTROL_HEADER:
        raise ValueError("控制帧头错误")
    if data[2] != CONTROL_LENGTH:
        raise ValueError(f"控制帧长度字段错误：0x{data[2]:02X}")
    if data[44] != _checksum(bytearray(data), 3, 44):
        raise ValueError("控制帧校验和错误")
    p1, p2, p3, p4 = struct.unpack_from("<hhHH", data, 4)
    pitch, roll, heading = struct.unpack_from("<hhH", data, 13)
    lat, lon, alt = struct.unpack_from("<iiI", data, 27)
    timestamp = struct.unpack_from("<I", data, 40)[0]
    return ControlFrame(
        command=data[3], p1=p1, p2=p2, p3=p3, p4=p4,
        aircraft_pitch_deg=pitch / 100.0,
        aircraft_roll_deg=roll / 100.0,
        aircraft_heading_deg=heading / 100.0,
        status=data[26],
        aircraft_lat_deg=lat / 1e7,
        aircraft_lon_deg=lon / 1e7,
        aircraft_alt_m=alt / 1000.0,
        timestamp_ms=timestamp,
    )


def encode_telemetry(frame: TelemetryFrame) -> bytes:
    data = bytearray(TELEMETRY_SIZE)
    data[0:2] = TELEMETRY_HEADER
    data[2] = TELEMETRY_LENGTH
    data[3] = frame.self_test & 0xFF
    data[4] = frame.work_state & 0xFF
    struct.pack_into("<h", data, 5, _scaled(frame.pitch_deg, 100, -32768, 32767))
    struct.pack_into("<h", data, 7, _scaled(frame.yaw_deg, 100, -32768, 32767))
    struct.pack_into("<h", data, 9, _scaled(frame.gate_x, 1, -32768, 32767))
    struct.pack_into("<h", data, 11, _scaled(frame.gate_y, 1, -32768, 32767))
    struct.pack_into("<h", data, 13, _scaled(frame.gyro_pitch_dps, 100, -2000, 2000))
    struct.pack_into("<h", data, 15, _scaled(frame.gyro_yaw_dps, 100, -2000, 2000))
    struct.pack_into("<h", data, 17, _scaled(frame.pitch_misalignment_deg, 100, -2000, 2000))
    struct.pack_into("<h", data, 19, _scaled(frame.yaw_misalignment_deg, 100, -2000, 2000))
    data[21] = _scaled(frame.tracking_confidence, 10, 0, 255)
    struct.pack_into("<h", data, 22, _scaled(frame.pitch_los_dps, 100, -2000, 2000))
    struct.pack_into("<h", data, 24, _scaled(frame.yaw_los_dps, 100, -2000, 2000))
    data[26] = frame.status_control & 0xFF
    data[27] = frame.autonomous & 0xFF
    for index, target in enumerate(telemetry_targets(frame)):
        offset = 28 + index * 12
        data[offset] = target.type & 0xFF
        data[offset + 1] = _scaled(target.confidence, 100, 0, 255)
        struct.pack_into("<iiH", data, offset + 2,
                         _scaled(target.lon, 1, -2147483648, 2147483647),
                         _scaled(target.lat, 1, -2147483648, 2147483647),
                         _scaled(target.alt_m, 10, 0, 65535))
    data[76] = frame.status_control2 & 0xFF
    struct.pack_into("<H", data, 77, _scaled(frame.gate_w, 1, 0, 65535))
    struct.pack_into("<H", data, 79, _scaled(frame.gate_h, 1, 0, 65535))
    struct.pack_into("<h", data, 81, _scaled(frame.pitch_att_deg, 100, -32768, 32767))
    struct.pack_into("<h", data, 83, _scaled(frame.yaw_att_deg, 100, -32768, 32767))
    struct.pack_into("<h", data, 85, _scaled(frame.roll_att_deg, 100, -32768, 32767))
    data[87] = frame.tracking_target_type & 0xFF
    struct.pack_into("<H", data, 88, _scaled(frame.laser_distance_m, 10, 0, 65535))
    data[90] = frame.laser_flags & 0xFF
    struct.pack_into("<h", data, 91, _scaled(frame.aircraft_pitch_deg, 100, -32768, 32767))
    struct.pack_into("<h", data, 93, _scaled(frame.aircraft_roll_deg, 100, -32768, 32767))
    struct.pack_into("<H", data, 95, _scaled(frame.aircraft_heading_deg, 100, 0, 65535))
    data[97] = frame.ip[0] & 0xFF
    data[98] = frame.ip[1] & 0xFF
    struct.pack_into("<I", data, 99, frame.timestamp_ms & 0xFFFFFFFF)
    data[103] = _checksum(data, 3, 103)
    return bytes(data)


def decode_telemetry(data: bytes) -> TelemetryFrame:
    if len(data) != TELEMETRY_SIZE:
        raise ValueError(f"遥测帧长度必须为 {TELEMETRY_SIZE}，实际 {len(data)}")
    if data[0:2] != TELEMETRY_HEADER:
        raise ValueError("遥测帧头错误")
    if data[2] != TELEMETRY_LENGTH:
        raise ValueError(f"遥测帧长度字段错误：0x{data[2]:02X}")
    if data[103] != _checksum(bytearray(data), 3, 103):
        raise ValueError("遥测帧校验和错误")
    pitch, yaw = struct.unpack_from("<hh", data, 5)
    gate_x, gate_y = struct.unpack_from("<hh", data, 9)
    gyro_pitch, gyro_yaw = struct.unpack_from("<hh", data, 13)
    pitch_mis, yaw_mis = struct.unpack_from("<hh", data, 17)
    pitch_los, yaw_los = struct.unpack_from("<hh", data, 22)
    targets = []
    for index in range(4):
        offset = 28 + index * 12
        lon, lat, altitude = struct.unpack_from("<iiH", data, offset + 2)
        targets.append(TelemetryTarget(
            type=data[offset], confidence=data[offset + 1] / 100.0,
            lon=lon, lat=lat, alt_m=altitude / 10.0))
    first = targets[0]
    gate_w, gate_h = struct.unpack_from("<HH", data, 77)
    pitch_att, yaw_att, roll_att = struct.unpack_from("<hhh", data, 81)
    laser_distance = struct.unpack_from("<H", data, 88)[0]
    aircraft_pitch, aircraft_roll, aircraft_heading = struct.unpack_from("<hhH", data, 91)
    timestamp = struct.unpack_from("<I", data, 99)[0]
    return TelemetryFrame(
        self_test=data[3], work_state=data[4],
        pitch_deg=pitch / 100.0, yaw_deg=yaw / 100.0,
        gate_x=gate_x, gate_y=gate_y,
        gyro_pitch_dps=gyro_pitch / 100.0, gyro_yaw_dps=gyro_yaw / 100.0,
        pitch_misalignment_deg=pitch_mis / 100.0,
        yaw_misalignment_deg=yaw_mis / 100.0,
        tracking_confidence=data[21] / 10.0,
        pitch_los_dps=pitch_los / 100.0, yaw_los_dps=yaw_los / 100.0,
        status_control=data[26], autonomous=data[27],
        target_type=first.type, target_confidence=first.confidence,
        target_lon_deg=first.lon / 1e7, target_lat_deg=first.lat / 1e7,
        target_alt_m=first.alt_m, targets=tuple(targets),
        status_control2=data[76], gate_w=gate_w, gate_h=gate_h,
        pitch_att_deg=pitch_att / 100.0, yaw_att_deg=yaw_att / 100.0,
        roll_att_deg=roll_att / 100.0, tracking_target_type=data[87],
        laser_distance_m=laser_distance / 10.0, laser_flags=data[90],
        aircraft_pitch_deg=aircraft_pitch / 100.0,
        aircraft_roll_deg=aircraft_roll / 100.0,
        aircraft_heading_deg=aircraft_heading / 100.0,
        ip=(data[97], data[98]), timestamp_ms=timestamp,
    )


def make_control(command: int, **kwargs) -> bytes:
    return encode_control(ControlFrame(command=command, **kwargs))
