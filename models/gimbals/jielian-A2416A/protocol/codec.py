"""A2416A 45/104 字节标准协议的严格二进制编解码。

帧头按协议文档中的线序字节处理，不将 ``AA 55`` / ``55 AA`` 当作
主机端整数。多字节字段为小端，校验和为命令字（偏移3）至校验字节
之前所有字节之和的低8位。
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import struct
from typing import Any, Mapping, Sequence

CONTROL_HEADER = b"\xaa\x55"
CONTROL_LENGTH = 0x2D
CONTROL_SIZE = 45
TELEMETRY_HEADER = b"\x55\xaa"
TELEMETRY_LENGTH = 0x68
TELEMETRY_SIZE = 104

CMD_HEARTBEAT = 0x00
CMD_STREAM_SET = 0x11
CMD_CROSSHAIR_MOVE = 0xB1
CMD_AIM_TRIM = 0xB2
CMD_GATE_GROW = 0xB3
CMD_GATE_SHRINK = 0xB4
CMD_POINT_LOCK = 0xB7
CMD_BOX_LOCK = 0xB8
CMD_ZOOM_IN = 0xB9
CMD_ZOOM_OUT = 0x9B
CMD_CANCEL_TRACK = 0x8E
CMD_CROSSHAIR_LOCK = 0xE8
CMD_RECOGNITION_ON = 0x3F
CMD_RECOGNITION_OFF = 0xF3
CMD_AUTO_LOCK_ON = 0x5F
CMD_AUTO_LOCK_OFF = 0xF5
CMD_IMAGE_ENHANCE_ON = 0x6F
CMD_IMAGE_ENHANCE_OFF = 0xF6
CMD_IP_SET = 0x8F
CMD_VIDEO_OFF = 0x10
CMD_VIDEO_ON = 0x01
CMD_OSD_FULL = 0x15
CMD_OSD_HALF = 0x16
CMD_OSD_OFF = 0x17

# 兼容已有地面站使用的名称。
CMD_FRAME_MOVE = CMD_CROSSHAIR_MOVE
CMD_TRACK_POINT_TRIM = CMD_AIM_TRIM
CMD_CANCEL_LOCK = CMD_CANCEL_TRACK
CMD_AUTO_RECOGNITION_ON = CMD_RECOGNITION_ON
CMD_AUTO_RECOGNITION_OFF = CMD_RECOGNITION_OFF
CMD_RTP_OFF = CMD_VIDEO_OFF
CMD_RTP_ON = CMD_VIDEO_ON
CMD_OSD_ON = CMD_OSD_FULL

COMMAND_NAMES = {
    CMD_HEARTBEAT: "heartbeat",
    CMD_STREAM_SET: "stream_set",
    CMD_CROSSHAIR_MOVE: "crosshair_move",
    CMD_AIM_TRIM: "aim_trim",
    CMD_GATE_GROW: "gate_grow",
    CMD_GATE_SHRINK: "gate_shrink",
    CMD_POINT_LOCK: "point_lock",
    CMD_BOX_LOCK: "box_lock",
    CMD_ZOOM_IN: "zoom_in",
    CMD_ZOOM_OUT: "zoom_out",
    CMD_CANCEL_TRACK: "cancel_track",
    CMD_CROSSHAIR_LOCK: "crosshair_lock",
    CMD_RECOGNITION_ON: "recognition_on",
    CMD_RECOGNITION_OFF: "recognition_off",
    CMD_AUTO_LOCK_ON: "auto_lock_on",
    CMD_AUTO_LOCK_OFF: "auto_lock_off",
    CMD_IMAGE_ENHANCE_ON: "image_enhance_on",
    CMD_IMAGE_ENHANCE_OFF: "image_enhance_off",
    CMD_IP_SET: "ip_set",
    CMD_VIDEO_OFF: "video_off",
    CMD_VIDEO_ON: "video_on",
    CMD_OSD_FULL: "osd_full",
    CMD_OSD_HALF: "osd_half",
    CMD_OSD_OFF: "osd_off",
}
SUPPORTED_COMMANDS = frozenset(COMMAND_NAMES)
CONTINUOUS_COMMANDS = frozenset((CMD_HEARTBEAT, CMD_CROSSHAIR_MOVE))


def command_name(command: int) -> str:
    return COMMAND_NAMES.get(command, f"0x{command:02X}")


def _finite(value: Any, name: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} 必须为有限数值")
    return float(value)


def _integer(value: Any, name: str, low: int, high: int) -> int:
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} 必须为 {low}～{high} 的整数")
    return value


def _scaled(value: Any, scale: float, low: int, high: int, name: str) -> int:
    raw = round(_finite(value, name) * scale)
    return max(low, min(high, raw))


def _checksum(data: bytes | bytearray, end: int) -> int:
    return sum(data[3:end]) & 0xFF


def _verify(data: bytes, *, size: int, header: bytes, length: int, kind: str) -> bytes:
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise ValueError(f"{kind}必须为字节序列")
    raw = bytes(data)
    if len(raw) != size:
        raise ValueError(f"{kind}长度必须为 {size} 字节，实际 {len(raw)}")
    if raw[:2] != header:
        raise ValueError(f"{kind}帧头错误")
    if raw[2] != length:
        raise ValueError(f"{kind}长度字段错误：0x{raw[2]:02X}")
    if raw[-1] != _checksum(raw, size - 1):
        raise ValueError(f"{kind}校验和错误")
    return raw


@dataclass(frozen=True)
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
    reserved_12: int = 0
    reserved_19_25: bytes = b"\0" * 7
    reserved_39: int = 0
    checksum: int | None = field(default=None, compare=False)

    @property
    def name(self) -> str:
        return command_name(self.command)


@dataclass(frozen=True)
class TelemetryTarget:
    type: int = 0
    confidence: float = 0.0
    lon: int = 0
    lat: int = 0
    alt_m: float = 0.0

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "TelemetryTarget":
        lon = data.get("lon", data.get("lon_e7"))
        lat = data.get("lat", data.get("lat_e7"))
        if lon is None:
            lon = _scaled(data.get("lon_deg", 0.0), 1e7, -2147483648, 2147483647,
                          "target.lon_deg")
        if lat is None:
            lat = _scaled(data.get("lat_deg", 0.0), 1e7, -2147483648, 2147483647,
                          "target.lat_deg")
        return cls(
            type=int(data.get("type", 0)),
            confidence=float(data.get("confidence", 0.0)),
            lon=int(lon), lat=int(lat),
            alt_m=float(data.get("alt_m", 0.0)),
        )


@dataclass(frozen=True)
class TelemetryFrame:
    self_test: int = 1
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
    targets: tuple[TelemetryTarget, ...] = ()
    measurement_seq: int = field(default=0, compare=False)
    checksum: int | None = field(default=None, compare=False)


def telemetry_targets(frame: TelemetryFrame) -> tuple[TelemetryTarget, ...]:
    if frame.targets:
        if len(frame.targets) != 4:
            raise ValueError("标准遥测必须恰好包含四个目标槽位")
        return tuple(frame.targets)
    first = TelemetryTarget(
        type=frame.target_type,
        confidence=frame.target_confidence,
        lon=_scaled(frame.target_lon_deg, 1e7, -2147483648, 2147483647,
                    "target_lon_deg"),
        lat=_scaled(frame.target_lat_deg, 1e7, -2147483648, 2147483647,
                    "target_lat_deg"),
        alt_m=frame.target_alt_m,
    )
    return (first, TelemetryTarget(), TelemetryTarget(), TelemetryTarget())


def encode_control(frame: ControlFrame) -> bytes:
    command = _integer(frame.command, "command", 0, 255)
    p1 = _integer(frame.p1, "p1", -32768, 32767)
    p2 = _integer(frame.p2, "p2", -32768, 32767)
    p3 = _integer(frame.p3, "p3", 0, 65535)
    p4 = _integer(frame.p4, "p4", 0, 65535)
    status = _integer(frame.status, "status", 0, 255)
    timestamp = _integer(frame.timestamp_ms, "timestamp_ms", 0, 0xFFFFFFFF)
    if not isinstance(frame.reserved_19_25, bytes) or len(frame.reserved_19_25) != 7:
        raise ValueError("reserved_19_25 必须为7字节")
    data = bytearray(CONTROL_SIZE)
    data[:2] = CONTROL_HEADER
    data[2] = CONTROL_LENGTH
    data[3] = command
    struct.pack_into("<hhHH", data, 4, p1, p2, p3, p4)
    data[12] = _integer(frame.reserved_12, "reserved_12", 0, 255)
    struct.pack_into(
        "<hhH", data, 13,
        _scaled(frame.aircraft_pitch_deg, 100, -32768, 32767, "aircraft_pitch_deg"),
        _scaled(frame.aircraft_roll_deg, 100, -32768, 32767, "aircraft_roll_deg"),
        _scaled(frame.aircraft_heading_deg, 100, 0, 65535, "aircraft_heading_deg"),
    )
    data[19:26] = frame.reserved_19_25
    data[26] = status
    struct.pack_into(
        "<iiI", data, 27,
        _scaled(frame.aircraft_lat_deg, 1e7, -2147483648, 2147483647,
                "aircraft_lat_deg"),
        _scaled(frame.aircraft_lon_deg, 1e7, -2147483648, 2147483647,
                "aircraft_lon_deg"),
        _scaled(frame.aircraft_alt_m, 1000, 0, 0xFFFFFFFF, "aircraft_alt_m"),
    )
    data[39] = _integer(frame.reserved_39, "reserved_39", 0, 255)
    struct.pack_into("<I", data, 40, timestamp)
    data[44] = _checksum(data, 44)
    return bytes(data)


def decode_control(data: bytes) -> ControlFrame:
    raw = _verify(data, size=CONTROL_SIZE, header=CONTROL_HEADER,
                  length=CONTROL_LENGTH, kind="控制帧")
    p1, p2, p3, p4 = struct.unpack_from("<hhHH", raw, 4)
    pitch, roll, heading = struct.unpack_from("<hhH", raw, 13)
    lat, lon, alt = struct.unpack_from("<iiI", raw, 27)
    return ControlFrame(
        command=raw[3], p1=p1, p2=p2, p3=p3, p4=p4,
        aircraft_pitch_deg=pitch / 100.0,
        aircraft_roll_deg=roll / 100.0,
        aircraft_heading_deg=heading / 100.0,
        status=raw[26], aircraft_lat_deg=lat / 1e7,
        aircraft_lon_deg=lon / 1e7, aircraft_alt_m=alt / 1000.0,
        timestamp_ms=struct.unpack_from("<I", raw, 40)[0],
        reserved_12=raw[12], reserved_19_25=raw[19:26],
        reserved_39=raw[39], checksum=raw[44],
    )


def make_control(command: int, p1: int = 0, p2: int = 0,
                 p3: int = 0, p4: int = 0, **kwargs: Any) -> bytes:
    return encode_control(ControlFrame(
        command=command, p1=p1, p2=p2, p3=p3, p4=p4, **kwargs))


def encode_telemetry(frame: TelemetryFrame) -> bytes:
    data = bytearray(TELEMETRY_SIZE)
    data[:2] = TELEMETRY_HEADER
    data[2] = TELEMETRY_LENGTH
    data[3] = _integer(frame.self_test, "self_test", 0, 255)
    data[4] = _integer(frame.work_state, "work_state", 0, 255)
    struct.pack_into(
        "<hhhhhhhh", data, 5,
        _scaled(frame.pitch_deg, 100, -32768, 32767, "pitch_deg"),
        _scaled(frame.yaw_deg, 100, -32768, 32767, "yaw_deg"),
        _scaled(frame.gate_x, 1, -32768, 32767, "gate_x"),
        _scaled(frame.gate_y, 1, -32768, 32767, "gate_y"),
        _scaled(frame.gyro_pitch_dps, 100, -2000, 2000, "gyro_pitch_dps"),
        _scaled(frame.gyro_yaw_dps, 100, -2000, 2000, "gyro_yaw_dps"),
        _scaled(frame.pitch_misalignment_deg, 100, -2000, 2000,
                "pitch_misalignment_deg"),
        _scaled(frame.yaw_misalignment_deg, 100, -2000, 2000,
                "yaw_misalignment_deg"),
    )
    data[21] = _scaled(frame.tracking_confidence, 10, 0, 255,
                       "tracking_confidence")
    struct.pack_into(
        "<hh", data, 22,
        _scaled(frame.pitch_los_dps, 100, -2000, 2000, "pitch_los_dps"),
        _scaled(frame.yaw_los_dps, 100, -2000, 2000, "yaw_los_dps"),
    )
    data[26] = _integer(frame.status_control, "status_control", 0, 255)
    data[27] = _integer(frame.autonomous, "autonomous", 0, 255)
    for index, target in enumerate(telemetry_targets(frame)):
        offset = 28 + 12 * index
        data[offset] = _integer(target.type, f"targets[{index}].type", 0, 255)
        data[offset + 1] = _scaled(target.confidence, 100, 0, 255,
                                   f"targets[{index}].confidence")
        struct.pack_into(
            "<iiH", data, offset + 2,
            _integer(target.lon, f"targets[{index}].lon", -2147483648, 2147483647),
            _integer(target.lat, f"targets[{index}].lat", -2147483648, 2147483647),
            _scaled(target.alt_m, 10, 0, 65535, f"targets[{index}].alt_m"),
        )
    data[76] = _integer(frame.status_control2, "status_control2", 0, 255)
    struct.pack_into(
        "<HHhhh", data, 77,
        _integer(frame.gate_w, "gate_w", 0, 65535),
        _integer(frame.gate_h, "gate_h", 0, 65535),
        _scaled(frame.pitch_att_deg, 100, -32768, 32767, "pitch_att_deg"),
        _scaled(frame.yaw_att_deg, 100, -32768, 32767, "yaw_att_deg"),
        _scaled(frame.roll_att_deg, 100, -32768, 32767, "roll_att_deg"),
    )
    data[87] = _integer(frame.tracking_target_type, "tracking_target_type", 0, 255)
    struct.pack_into("<H", data, 88,
                     _scaled(frame.laser_distance_m, 10, 0, 65535,
                             "laser_distance_m"))
    data[90] = _integer(frame.laser_flags, "laser_flags", 0, 255)
    struct.pack_into(
        "<hhH", data, 91,
        _scaled(frame.aircraft_pitch_deg, 100, -32768, 32767,
                "aircraft_pitch_deg"),
        _scaled(frame.aircraft_roll_deg, 100, -32768, 32767,
                "aircraft_roll_deg"),
        _scaled(frame.aircraft_heading_deg, 100, 0, 65535,
                "aircraft_heading_deg"),
    )
    if len(frame.ip) != 2:
        raise ValueError("ip 必须为两个末段整数")
    data[97] = _integer(frame.ip[0], "ip[0]", 0, 255)
    data[98] = _integer(frame.ip[1], "ip[1]", 0, 255)
    struct.pack_into("<I", data, 99,
                     _integer(frame.timestamp_ms, "timestamp_ms", 0, 0xFFFFFFFF))
    data[103] = _checksum(data, 103)
    return bytes(data)


def decode_telemetry(data: bytes) -> TelemetryFrame:
    raw = _verify(data, size=TELEMETRY_SIZE, header=TELEMETRY_HEADER,
                  length=TELEMETRY_LENGTH, kind="遥测帧")
    pitch, yaw, gate_x, gate_y, gyro_pitch, gyro_yaw, pitch_mis, yaw_mis = \
        struct.unpack_from("<hhhhhhhh", raw, 5)
    pitch_los, yaw_los = struct.unpack_from("<hh", raw, 22)
    targets = []
    for index in range(4):
        offset = 28 + index * 12
        lon, lat, altitude = struct.unpack_from("<iiH", raw, offset + 2)
        targets.append(TelemetryTarget(
            type=raw[offset], confidence=raw[offset + 1] / 100.0,
            lon=lon, lat=lat, alt_m=altitude / 10.0))
    gate_w, gate_h, pitch_att, yaw_att, roll_att = \
        struct.unpack_from("<HHhhh", raw, 77)
    aircraft_pitch, aircraft_roll, aircraft_heading = \
        struct.unpack_from("<hhH", raw, 91)
    first = targets[0]
    return TelemetryFrame(
        self_test=raw[3], work_state=raw[4],
        pitch_deg=pitch / 100.0, yaw_deg=yaw / 100.0,
        gate_x=gate_x, gate_y=gate_y,
        gyro_pitch_dps=gyro_pitch / 100.0,
        gyro_yaw_dps=gyro_yaw / 100.0,
        pitch_misalignment_deg=pitch_mis / 100.0,
        yaw_misalignment_deg=yaw_mis / 100.0,
        tracking_confidence=raw[21] / 10.0,
        pitch_los_dps=pitch_los / 100.0,
        yaw_los_dps=yaw_los / 100.0,
        status_control=raw[26], autonomous=raw[27],
        target_type=first.type, target_confidence=first.confidence,
        target_lon_deg=first.lon / 1e7, target_lat_deg=first.lat / 1e7,
        target_alt_m=first.alt_m, targets=tuple(targets),
        status_control2=raw[76], gate_w=gate_w, gate_h=gate_h,
        pitch_att_deg=pitch_att / 100.0,
        yaw_att_deg=yaw_att / 100.0,
        roll_att_deg=roll_att / 100.0,
        tracking_target_type=raw[87],
        laser_distance_m=struct.unpack_from("<H", raw, 88)[0] / 10.0,
        laser_flags=raw[90],
        aircraft_pitch_deg=aircraft_pitch / 100.0,
        aircraft_roll_deg=aircraft_roll / 100.0,
        aircraft_heading_deg=aircraft_heading / 100.0,
        ip=(raw[97], raw[98]),
        timestamp_ms=struct.unpack_from("<I", raw, 99)[0],
        checksum=raw[103],
    )


def telemetry_message(frame: TelemetryFrame, timestamp_us: int) -> dict[str, Any]:
    """导出地面站可直接转发的完整、稳定JSON结构。"""
    timestamp_us = _integer(timestamp_us, "timestamp_us", 0, 0x7FFFFFFFFFFFFFFF)
    targets = telemetry_targets(frame)
    return {
        "frame_header": TELEMETRY_HEADER.hex().upper(),
        "frame_length": TELEMETRY_SIZE,
        "raw_frame_hex": encode_telemetry(frame).hex().upper(),
        "checksum": frame.checksum,
        "checksum_valid": True,
        "reserved_bytes": [],
        "timestamp": timestamp_us,
        "measurement_seq": frame.measurement_seq,
        "self_check": frame.self_test,
        "work_status": frame.work_state,
        "track_status": frame.work_state & 0x01,
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


def targets_from_mappings(items: Sequence[Mapping[str, Any]]) -> tuple[TelemetryTarget, ...]:
    """将最多四个真值目标转换为协议槽位，并补零到四个。"""
    targets = [TelemetryTarget.from_mapping(item) for item in items[:4]]
    targets.extend(TelemetryTarget() for _ in range(4 - len(targets)))
    return tuple(targets)
