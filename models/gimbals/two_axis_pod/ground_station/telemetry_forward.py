"""Station 的艾晟 JSON 输出与地址编辑；发送不阻塞视频和控制。"""
from collections import deque
import ipaddress
import json
import math
from pathlib import Path
import re
import socket
import threading
import time

from packet_rate import PacketRate


def target_los_rates(message):
    """返回有效 NED 惯性视线速率（原始 °/s）；明确无效时返回 None。"""
    los = message.get("target_los") if isinstance(message, dict) else None
    if not isinstance(los, dict):
        raise ValueError("缺少目标惯性视线角速度")
    if (los.get("frame") != "NED"
            or los.get("observer_reference") != "aircraft_nominal_cg"):
        raise ValueError("目标惯性视线坐标系或观测参考点不匹配")
    if type(los.get("valid")) is not bool or not isinstance(los.get("reason"), str):
        raise ValueError("目标惯性视线有效标志或原因字段非法")
    if not los["valid"]:
        if ("pitch_rate_dps" not in los or "yaw_rate_dps" not in los
                or los["pitch_rate_dps"] is not None or los["yaw_rate_dps"] is not None):
            raise ValueError("无效目标惯性视线速率必须明确为 null")
        return None
    if not isinstance(los.get("target_name"), str) or not los["target_name"]:
        raise ValueError("目标惯性视线缺少目标实体名称")
    values = tuple(los.get(f"{axis}_rate_dps") for axis in ("pitch", "yaw"))
    try:
        finite = all(type(value) in (int, float) and math.isfinite(value) for value in values)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError("目标惯性视线角速度不是有限数值")
    return values


def target_los_omega(message):
    """读取独立NED视线角速度向量，°/s；不受方位/仰角奇异或机械限速影响。"""
    los = message.get("target_los") if isinstance(message, dict) else None
    if not isinstance(los, dict):
        raise ValueError("缺少目标惯性视线角速度")
    if (los.get("frame") != "NED"
            or los.get("observer_reference") != "aircraft_nominal_cg"):
        raise ValueError("目标惯性视线坐标系或观测参考点不匹配")
    if type(los.get("omega_valid")) is not bool or not isinstance(los.get("omega_reason"), str):
        raise ValueError("目标视线角速度向量有效标志或原因字段非法")
    if "omega_ned_dps" not in los:
        raise ValueError("缺少目标视线NED角速度向量")
    values = los["omega_ned_dps"]
    if not los["omega_valid"]:
        if values is not None:
            raise ValueError("无效目标视线角速度向量必须明确为 null")
        return None
    if not isinstance(los.get("target_name"), str) or not los["target_name"]:
        raise ValueError("目标视线角速度向量缺少目标实体名称")
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError("目标视线角速度向量必须有N/E/D三个分量")
    try:
        finite = all(type(value) in (int, float) and math.isfinite(value) for value in values)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError("目标视线角速度向量不是有限数值")
    if any(abs(value) > 3.4028234663852886e38 for value in values):
        raise ValueError("目标视线角速度向量超出 float32 范围")
    return tuple(float(value) for value in values)


def valid_tracking_target(message):
    """沿用艾晟有效跟踪判定；返回最终跟踪目标，失效时返回None。"""
    target = message.get("target") or {}
    tracking = message.get("tracking") or {}
    vision = message.get("vision") or {}
    tracking_enabled = (isinstance(vision, dict) and not vision.get("error")
                        and (vision.get("mode") == "manual_track"
                             or (vision.get("mode") == "auto_lock"
                                 and vision.get("state") == "AUTO_LOCKED"
                                 and type(vision.get("locked_track_id")) is int)))
    miss = (target.get("miss") or {}) if isinstance(target, dict) else {}
    valid = (tracking_enabled and message.get("mode") == "TRACK" and isinstance(tracking, dict)
             and tracking.get("state") == "TRACK" and isinstance(target, dict)
             and target.get("valid") is True and isinstance(miss, dict)
             and miss.get("valid") is True)
    return target if valid else None


def tracking_gate_center(message, fresh=True):
    """原始视频最终框中心像素，正数half-up；无效0/0，过期返回None。"""
    if not fresh:
        return None
    if not isinstance(message, dict):
        raise ValueError("缺少跟踪遥测")
    target = valid_tracking_target(message)
    if target is None:
        return (0, 0)
    box = target.get("box_px")
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        raise ValueError("有效跟踪缺少像素跟踪框")
    try:
        finite = all(type(value) in (int, float) and math.isfinite(value) for value in box)
    except OverflowError:
        finite = False
    if not finite or box[0] < 0 or box[1] < 0 or box[2] <= 0 or box[3] <= 0:
        raise ValueError("跟踪框坐标必须为非负有限数值，宽高必须大于0")
    # 不先对box做小数位舍入，边界亚像素会改变half-up结果。
    center = (box[0] + box[2] / 2, box[1] + box[3] / 2)
    if not all(math.isfinite(value) for value in center):
        raise ValueError("跟踪框中心超出有限范围")
    return tuple(math.floor(value + 0.5) for value in center)


def aisheng_message(message):
    """两轴 v8 遥测 -> 艾晟测量字段；发送时间戳由发送线程追加。"""
    if (not isinstance(message, dict) or message.get("type") != "telemetry"
            or type(message.get("protocol_version")) is not int
            or message["protocol_version"] != 8
            or message.get("angle_convention") != "motion_head_joint_right_positive"):
        raise ValueError("艾晟转换需要两轴协议 8 遥测")

    def scaled(value):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError("艾晟数值字段不是有限数值")
        raw = value
        if not math.isfinite(raw) or abs(raw) > 3.4028234663852886e38:
            raise ValueError("艾晟数值超出 float32 范围")
        return float(round(raw, 6)) if raw else 0.0

    def limited_rate(value):
        # 先校验有限数值与 float32 范围，非法遥测不能通过限幅变成有效包。
        rate = scaled(value)
        return max(-20.0, min(20.0, rate))

    attitude = message.get("attitude_deg")
    if not isinstance(attitude, dict) or any(not isinstance(attitude.get(axis), dict)
                                             for axis in ("pitch", "yaw")):
        raise ValueError("缺少两轴机械姿态")
    los_rates = target_los_rates(message)
    los_pitch_rate, los_yaw_rate = los_rates if los_rates is not None else (0.0, 0.0)
    omega = target_los_omega(message)
    omega_n, omega_e, omega_d = omega if omega is not None else (0.0, 0.0, 0.0)
    target = valid_tracking_target(message)
    valid = target is not None
    miss = target.get("miss") if valid else {}
    offsets = miss.get("offset_deg") if valid else None
    if valid and (not isinstance(offsets, (list, tuple)) or len(offsets) != 2):
        raise ValueError("有效跟踪缺少二维角脱靶量")
    box = target.get("box_px") if valid else None
    if valid and (not isinstance(box, (list, tuple)) or len(box) != 4):
        raise ValueError("有效跟踪缺少像素跟踪框")
    gate_width = scaled(box[2]) if valid else 0.0
    gate_height = scaled(box[3]) if valid else 0.0
    if valid and (gate_width <= 0 or gate_height <= 0):
        raise ValueError("有效跟踪框宽高必须大于0")
    gate_x, gate_y = tracking_gate_center(message)
    return {
        "track_status": int(valid),
        "pitch_angle": scaled(attitude["pitch"].get("pos")),
        "yaw_angle": scaled(attitude["yaw"].get("pos")),
        "pitch_misalignment": scaled(offsets[1]) if valid else 0.0,
        "yaw_misalignment": scaled(offsets[0]) if valid else 0.0,
        "tracking_confidence": 25.5 if valid else 0.0,
        "target_confidence": 0.80,
        "gyro_pitch_rate": limited_rate(attitude["pitch"].get("vel")),
        "gyro_yaw_rate": limited_rate(attitude["yaw"].get("vel")),
        "pitch_los_rate": limited_rate(los_pitch_rate),
        "yaw_los_rate": limited_rate(los_yaw_rate),
        "los_omega_n": omega_n,
        "los_omega_e": omega_e,
        "los_omega_d": omega_d,
        "tracking_gate_w": gate_width,
        "tracking_gate_h": gate_height,
        "gate_x": gate_x,
        "gate_y": gate_y,
    }


def endpoint(ip, port):
    address = ipaddress.ip_address(ip.strip())
    if not str(port).isascii() or not str(port).isdigit() or not 1 <= int(port) <= 65535:
        raise ValueError("UDP 端口必须为 1～65535 的整数")
    return str(address), int(port)


def save_destination(path, ip, port):
    """只改 [forward] 地址，保留 INI 中已有的中文注释。"""
    ip, port = endpoint(ip, port)
    path = Path(path)
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    start = next((i for i, line in enumerate(lines) if line.strip() == "[forward]"), None)
    if start is None:
        lines += ["\n[forward]\n", "# 艾晟 UDP 接收地址；每次启动仍需点击发送艾晟报文。\n",
                  f"ip = {ip}\n", f"port = {port}\n"]
    else:
        end = next((i for i in range(start + 1, len(lines))
                    if lines[i].lstrip().startswith("[")), len(lines))
        values = {"ip": ip, "port": str(port)}
        for i in range(start + 1, end):
            match = re.match(r"^(\s*(ip|port)\s*=\s*)", lines[i])
            if match:
                lines[i] = match[1] + values.pop(match[2], ip if match[2] == "ip" else str(port)) + "\n"
        lines[end:end] = [f"{key} = {value}\n" for key, value in values.items()]
    temporary = path.with_name(path.name + ".forward.tmp")
    temporary.write_text("".join(lines), encoding="utf-8")
    temporary.replace(path)


class TelemetryForwarder:
    """独立非阻塞转换/发送线程，有界队列；每份新遥测最多发送一次。"""
    def __init__(self):
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._queue = deque(maxlen=16)
        self._socket = None
        self._target = None
        self._closed = False
        self.sent = self.dropped = 0
        self.error = ""
        self._send_rate = PacketRate(active=False)
        self._thread = threading.Thread(target=self._run, name="station-forward", daemon=True)
        self._thread.start()

    def start(self, ip, port):
        target = endpoint(ip, port)
        family = socket.AF_INET6 if ":" in target[0] else socket.AF_INET
        sock = socket.socket(family, socket.SOCK_DGRAM)
        sock.setblocking(False)
        with self._lock:
            if self._closed:
                sock.close()
                raise RuntimeError("转发器已关闭")
            if self._socket:
                self._socket.close()
            self._socket, self._target = sock, target
            self._queue.clear()
            self.sent = self.dropped = 0
            self.error = ""
            self._send_rate.reset()

    def offer(self, raw):
        with self._lock:
            if self._socket is None or self._closed:
                return
            if len(self._queue) == self._queue.maxlen:
                self.dropped += 1
            self._queue.append(raw)
            self._wake.set()

    def stop(self):
        with self._lock:
            if self._socket:
                self._socket.close()
            self._socket = None
            self._queue.clear()
            self.error = ""
            self._send_rate.reset(active=False)

    def snapshot(self):
        with self._lock:
            rate = self._send_rate.snapshot()
            return dict(active=self._socket is not None, target=self._target,
                        sent=self.sent, dropped=self.dropped, error=self.error,
                        send_hz=rate["hz"], send_rate_warming_up=rate["warming_up"],
                        rate_window_s=rate["window_s"])

    def _run(self):
        while True:
            self._wake.wait()
            with self._lock:
                if self._closed:
                    return
                if not self._queue:
                    self._wake.clear()
                    continue
                raw = self._queue.popleft()
                try:
                    payload = aisheng_message(json.loads(raw))
                    # 记录发送时的Unix整数毫秒，不沿用源遥测或入队时刻。
                    payload["timestamp"] = time.time_ns() // 1_000_000
                    data = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
                    self._socket.sendto(data, self._target)
                    self.sent += 1
                    self._send_rate.record()
                    self.error = ""
                except (ValueError, TypeError, OverflowError) as exc:
                    self.dropped += 1
                    self.error = f"艾晟转换失败: {exc}"
                except OSError as exc:
                    self.dropped += 1
                    self.error = f"发送失败: {exc}"

    def close(self):
        self.stop()
        with self._lock:
            self._closed = True
            self._wake.set()
        self._thread.join(timeout=1)


class ForwardInputs:
    """IP/端口输入状态，焦点内消费按键，不触发云台快捷键。"""
    def __init__(self, ip, port):
        self.values = {"ip": str(ip), "port": str(port)}
        self.focus = None
        self.selected = False

    def activate(self, field):
        self.focus = field
        self.selected = True

    def key(self, key):
        if self.focus is None or key in (-1, 0):
            return False
        field = self.focus
        if key == 27:
            self.focus = None
        elif key in (10, 13):
            self.focus = None
        elif key == 9:
            self.activate("port" if field == "ip" else "ip")
        elif key == 1:  # Ctrl+A
            self.selected = True
        elif key in (8, 127, 3014656, 65535):
            self.values[field] = "" if self.selected else self.values[field][:-1]
            self.selected = False
        elif 32 <= key < 127:
            char = chr(key)
            if char in ("0123456789abcdefABCDEF.:" if field == "ip" else "0123456789"):
                text = "" if self.selected else self.values[field]
                self.values[field] = (text + char)[:45 if field == "ip" else 5]
                self.selected = False
        return True
