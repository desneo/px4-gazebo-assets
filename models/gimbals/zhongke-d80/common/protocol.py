"""中科 DYT 固定二进制帧；数值使用厂商协议规定的物理单位。

控制帧为 16 字节，遥测帧为 32 字节，多字节数值均为小端。
此模块只负责线格式，不实现 UDP 会话或伺服限速。
"""
from __future__ import annotations

from collections.abc import Mapping
from decimal import Decimal, ROUND_HALF_UP
import math
import struct

COMMAND_HEADER = b"\xeb\x90"
COMMAND_SIZE = 16
TELEMETRY_HEADER = b"\xee\x16"
TELEMETRY_SIZE = 32

CMD_NOP = 0x00
CMD_POINT_TRACK = 0x0D
CMD_STOP_TRACK = 0x0E
CMD_AUTO_LOCK = 0x0F
CMD_MANUAL = 0x24
CMD_FRAME_ANGLE = 0x26
CMD_HOME = 0x2B

SUPPORTED_COMMANDS = frozenset((
    CMD_NOP, CMD_POINT_TRACK, CMD_STOP_TRACK, CMD_AUTO_LOCK,
    CMD_MANUAL, CMD_FRAME_ANGLE, CMD_HOME,
))
COMMAND_NAMES = {
    CMD_NOP: "空指令", CMD_POINT_TRACK: "指点跟踪",
    CMD_STOP_TRACK: "停止跟踪", CMD_AUTO_LOCK: "自动锁定",
    CMD_MANUAL: "手动速度", CMD_FRAME_ANGLE: "指定框架角", CMD_HOME: "归中",
}
TELEMETRY_FIELDS = (
    "status1", "status2", "status3", "zoom", "offset_x", "offset_y",
    "frame_roll", "frame_pitch", "frame_yaw", "roll_rate", "pitch_rate",
    "yaw_rate", "laser_range", "self_check",
)
_ZERO_PARAMETER_COMMANDS = frozenset((CMD_NOP, CMD_STOP_TRACK, CMD_AUTO_LOCK, CMD_HOME))
_SIGNED_FIELDS = (
    ("offset_x", 6, "0.05"), ("offset_y", 8, "0.05"),
    ("frame_roll", 10, "0.01"), ("frame_pitch", 12, "0.01"),
    ("frame_yaw", 14, "0.01"), ("roll_rate", 20, "0.01"),
    ("pitch_rate", 22, "0.01"), ("yaw_rate", 24, "0.01"),
)


def _integer(value, name: str, low: int, high: int) -> int:
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} 必须为 {low}～{high} 的整数")
    return value


def _number(value, name: str) -> Decimal:
    try:
        finite = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError(f"{name} 必须为有限数值")
    return Decimal(str(value))


def quantize(value, unit: str | float, low: int, high: int, name: str = "数值") -> int:
    """物理量转协议整数，最近整数、半单位远离零；拒绝编码溢出。"""
    raw = int((_number(value, name) / Decimal(str(unit))).to_integral_value(
        rounding=ROUND_HALF_UP))
    if not low <= raw <= high:
        raise ValueError(f"{name} 超出协议表达范围")
    return raw


def _verify(data: bytes, size: int, header: bytes, kind: str) -> None:
    if not isinstance(data, (bytes, bytearray, memoryview)) or len(data) != size:
        raise ValueError(f"{kind}长度必须为 {size} 字节")
    if data[:2] != header:
        raise ValueError(f"{kind}帧头错误")
    if data[-1] != sum(data[:-1]) & 0xFF:
        raise ValueError(f"{kind}校验和错误")


def encode_command(cmd: int, x: int = 0, y: int = 0, p3: int = 0,
                   zoom_rate: int = 0) -> bytes:
    """编码原始整数控制参数；速度单位 0.1°/s、角度单位 0.01°。"""
    _integer(cmd, "控制信息", 0, 255)
    if cmd not in SUPPORTED_COMMANDS:
        raise ValueError(f"不支持的中科指令 0x{cmd:02X}")
    _integer(x, "参数 X", -32768, 32767)
    _integer(y, "参数 Y", -32768, 32767)
    _integer(p3, "参数3", 0, 255)
    _integer(zoom_rate, "变焦速率", -128, 127)
    if p3 or zoom_rate:
        raise ValueError("当前支持指令的参数3和变焦速率必须为零")
    if cmd in _ZERO_PARAMETER_COMMANDS and (x or y):
        raise ValueError(f"{COMMAND_NAMES[cmd]}的 X/Y 参数必须为零")
    data = bytearray(COMMAND_SIZE)
    data[:2] = COMMAND_HEADER
    struct.pack_into("<BhhBb", data, 2, cmd, x, y, p3, zoom_rate)
    data[15] = sum(data[:15]) & 0xFF
    return bytes(data)


def decode_command(data: bytes) -> dict:
    _verify(data, COMMAND_SIZE, COMMAND_HEADER, "控制帧")
    cmd, x, y, p3, zoom_rate = struct.unpack_from("<BhhBb", data, 2)
    # 复用输入约束，但不把坏包重新编码为可执行指令。
    encode_command(cmd, x, y, p3, zoom_rate)
    if any(data[9:15]):
        raise ValueError("当前控制帧预留字节必须为零")
    return {"cmd": cmd, "x": x, "y": y, "p3": p3, "zoom_rate": zoom_rate}


def encode_telemetry(frame: Mapping) -> bytes:
    """编码14个物理量字段；status3低4位由zoom生成，上4位原样保存。

    调用方必须提供完整测量。没有值时不得以默认零伪造测量；横滚等
    明确不具备的自由度由能力映射层显式填零。
    """
    if not isinstance(frame, Mapping) or set(frame) != set(TELEMETRY_FIELDS):
        raise ValueError("遥测必须恰好包含14个标准字段，不含 timestamp")
    data = bytearray(TELEMETRY_SIZE)
    data[:2] = TELEMETRY_HEADER
    data[2] = _integer(frame["status1"], "status1", 0, 255)
    data[3] = _integer(frame["status2"], "status2", 0, 255)
    status3 = _integer(frame["status3"], "status3", 0, 255)
    if _number(frame["zoom"], "zoom") < 0:
        raise ValueError("zoom 不能为负数")
    zoom = quantize(frame["zoom"], "0.1", 0, 4095, "zoom")
    data[4] = zoom & 0xFF
    data[5] = (status3 & 0xF0) | (zoom >> 8)
    for field, offset, unit in _SIGNED_FIELDS:
        struct.pack_into("<h", data, offset, quantize(frame[field], unit, -32768, 32767, field))
    if _number(frame["laser_range"], "laser_range") < 0:
        raise ValueError("laser_range 不能为负数")
    struct.pack_into("<H", data, 26, quantize(frame["laser_range"], "0.1", 0, 65535, "laser_range"))
    data[28] = _integer(frame["self_check"], "self_check", 0, 255)
    data[31] = sum(data[:31]) & 0xFF
    return bytes(data)


def decode_telemetry(data: bytes) -> dict:
    """严格校验后解码，返回14个标准字段，不添加时间戳或内部元信息。"""
    _verify(data, TELEMETRY_SIZE, TELEMETRY_HEADER, "遥测帧")
    frame = {
        "status1": data[2], "status2": data[3], "status3": data[5],
        "zoom": (data[4] | ((data[5] & 0x0F) << 8)) / 10.0,
    }
    for field, offset, unit in _SIGNED_FIELDS:
        raw = struct.unpack_from("<h", data, offset)[0]
        # 整数除法比例避免最大值乘0.05产生浮点尾差越过schema边界。
        frame[field] = raw / (20.0 if unit == "0.05" else 100.0)
    frame["laser_range"] = struct.unpack_from("<H", data, 26)[0] / 10.0
    frame["self_check"] = data[28]
    return frame
