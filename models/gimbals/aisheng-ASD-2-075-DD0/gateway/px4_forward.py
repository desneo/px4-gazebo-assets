"""新导引头内部 JSON -> PX4 现有 19 字段艾晟 JSON。"""
from __future__ import annotations

import json
import math
import socket
import time

FLOAT32_MAX = 3.4028234663852886e38


def _finite(value, name):
    """校验有限数值并限制在 float32 可表示范围内。"""
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} 不是有限数值")
    if abs(value) > FLOAT32_MAX:
        raise ValueError(f"{name} 超出 float32 范围")
    return float(value)


def _scaled(value, name):
    value = _finite(value, name)
    return round(value, 6) if value else 0.0


def _rate(value, name):
    return max(-20.0, min(20.0, _scaled(value, name)))


def _require_dict(value, name):
    if not isinstance(value, dict):
        raise ValueError(f"缺少{name}")
    return value


def _los(data):
    """返回 (pitch_rate, yaw_rate, omega_ned)；明确无效时均为 0。"""
    los = _require_dict(data.get("target_los"), "目标惯性视线角速度")
    if (los.get("frame") != "NED"
            or los.get("observer_reference") != "aircraft_nominal_cg"):
        raise ValueError("目标惯性视线坐标系或观测参考点不匹配")
    if type(los.get("valid")) is not bool or not isinstance(los.get("reason"), str):
        raise ValueError("目标惯性视线有效标志或原因字段非法")
    if not los["valid"]:
        if ("pitch_rate_dps" not in los or "yaw_rate_dps" not in los
                or los["pitch_rate_dps"] is not None or los["yaw_rate_dps"] is not None):
            raise ValueError("无效目标惯性视线速率必须明确为 null")
        rates = (0.0, 0.0)
    else:
        if not isinstance(los.get("target_name"), str) or not los["target_name"]:
            raise ValueError("目标惯性视线缺少目标实体名称")
        rates = (_finite(los.get("pitch_rate_dps"), "pitch_rate_dps"),
                 _finite(los.get("yaw_rate_dps"), "yaw_rate_dps"))
    if type(los.get("omega_valid")) is not bool or not isinstance(los.get("omega_reason"), str):
        raise ValueError("目标视线角速度向量有效标志或原因字段非法")
    if "omega_ned_dps" not in los:
        raise ValueError("缺少目标视线NED角速度向量")
    omega = los["omega_ned_dps"]
    if not los["omega_valid"]:
        if omega is not None:
            raise ValueError("无效目标视线角速度向量必须明确为 null")
        omega = (0.0, 0.0, 0.0)
    else:
        if not isinstance(los.get("target_name"), str) or not los["target_name"]:
            raise ValueError("目标视线角速度向量缺少目标实体名称")
        if not isinstance(omega, (list, tuple)) or len(omega) != 3:
            raise ValueError("目标视线角速度向量必须有N/E/D三个分量")
        omega = tuple(_finite(value, "omega_ned_dps") for value in omega)
    return rates[0], rates[1], omega


def _target(data):
    """按艾晟有效跟踪判定返回最终目标，失效时返回 None。"""
    tracking = _require_dict(data.get("tracking"), "跟踪状态")
    target = _require_dict(data.get("target"), "跟踪目标")
    miss = target.get("miss") or {}
    if not isinstance(miss, dict):
        raise ValueError("目标脱靶量必须是对象")
    valid = (data.get("mode") == "TRACK" and tracking.get("state") == "TRACK"
             and target.get("valid") is True and miss.get("valid") is True)
    return target if valid else None


def px4_message(data: dict, timestamp_ms: int | None = None) -> dict:
    """把导引头内部遥测转换成 dyt_udp_bridge 期望的 19 字段 JSON。"""
    if (not isinstance(data, dict) or data.get("type") != "telemetry"
            or type(data.get("protocol_version")) is not int
            or data["protocol_version"] != 8
            or data.get("angle_convention") != "motion_head_joint_right_positive"):
        raise ValueError("艾晟转换需要两轴协议 8 遥测")
    attitude = _require_dict(data.get("attitude_deg"), "两轴机械姿态")
    for axis in ("pitch", "yaw"):
        _require_dict(attitude.get(axis), f"{axis}机械姿态")
    target = _target(data)
    valid = target is not None
    miss = target.get("miss") if valid else {}
    offsets = miss.get("offset_deg") if valid else None
    if valid and (not isinstance(offsets, (list, tuple)) or len(offsets) != 2):
        raise ValueError("有效跟踪缺少二维角脱靶量")
    box = target.get("box_px") if valid else None
    if valid and (not isinstance(box, (list, tuple)) or len(box) != 4):
        raise ValueError("有效跟踪缺少像素跟踪框")
    gate_width = _scaled(box[2], "tracking_gate_w") if valid else 0.0
    gate_height = _scaled(box[3], "tracking_gate_h") if valid else 0.0
    if valid and (gate_width <= 0 or gate_height <= 0):
        raise ValueError("有效跟踪框宽高必须大于0")
    if valid:
        values = tuple(_finite(value, "box_px") for value in box)
        if values[0] < 0 or values[1] < 0 or values[2] <= 0 or values[3] <= 0:
            raise ValueError("跟踪框坐标必须为非负有限数值，宽高必须大于0")
        center = (values[0] + values[2] / 2, values[1] + values[3] / 2)
        if not all(math.isfinite(value) for value in center):
            raise ValueError("跟踪框中心超出有限范围")
        gate_x, gate_y = (math.floor(value + 0.5) for value in center)
    else:
        gate_x = gate_y = 0
    los_pitch, los_yaw, omega = _los(data)
    if timestamp_ms is None:
        timestamp_ms = time.time_ns() // 1_000_000
    if type(timestamp_ms) is not int or timestamp_ms < 0:
        raise ValueError("timestamp 必须为非负整数毫秒")
    return {
        "timestamp": timestamp_ms,
        "track_status": int(valid),
        "pitch_angle": _scaled(attitude["pitch"].get("pos"), "pitch_angle"),
        "yaw_angle": _scaled(attitude["yaw"].get("pos"), "yaw_angle"),
        "pitch_misalignment": _scaled(offsets[1], "pitch_misalignment") if valid else 0.0,
        "yaw_misalignment": _scaled(offsets[0], "yaw_misalignment") if valid else 0.0,
        "tracking_confidence": 25.5 if valid else 0.0,
        "target_confidence": 0.80,
        "gyro_pitch_rate": _rate(attitude["pitch"].get("vel"), "gyro_pitch_rate"),
        "gyro_yaw_rate": _rate(attitude["yaw"].get("vel"), "gyro_yaw_rate"),
        "pitch_los_rate": _rate(los_pitch, "pitch_los_rate"),
        "yaw_los_rate": _rate(los_yaw, "yaw_los_rate"),
        "los_omega_n": omega[0],
        "los_omega_e": omega[1],
        "los_omega_d": omega[2],
        "tracking_gate_w": gate_width,
        "tracking_gate_h": gate_height,
        "gate_x": gate_x,
        "gate_y": gate_y,
    }


class Px4JsonSender:
    """按发送时刻生成时间戳，把 19 字段 JSON 发往本机 dyt_udp_bridge。"""

    def __init__(self, host: str, port: int):
        if not 0 < int(port) <= 65535:
            raise ValueError("PX4 JSON 端口必须在 1～65535 范围内")
        self.address = (host, int(port))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sent = 0
        self.errors = 0

    def send(self, data: dict) -> bytes:
        payload = json.dumps(
            px4_message(data, time.time_ns() // 1_000_000),
            separators=(",", ":"), allow_nan=False).encode("utf-8")
        try:
            self.socket.sendto(payload, self.address)
        except OSError:
            self.errors += 1
            raise
        self.sent += 1
        return payload

    def close(self) -> None:
        self.socket.close()
