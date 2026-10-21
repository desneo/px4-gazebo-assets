"""Station 的艾晟 JSON 输出与地址编辑；发送不阻塞视频和控制。"""
from collections import deque
import ipaddress
import json
import math
from pathlib import Path
import re
import socket
import threading


def aisheng_message(message):
    """两轴 v5 遥测 -> 九字段艾晟原始量；目标置信度固定为 80.0。"""
    if (not isinstance(message, dict) or message.get("type") != "telemetry"
            or type(message.get("protocol_version")) is not int
            or message["protocol_version"] != 5
            or message.get("angle_convention") != "upright_joint"):
        raise ValueError("艾晟转换需要两轴协议 5 遥测")

    def scaled(value):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError("角度、角速度或脱靶量不是有限数值")
        raw = value * 100.
        if not math.isfinite(raw) or abs(raw) > 3.4028234663852886e38:
            raise ValueError("艾晟数值超出 float32 范围")
        return float(round(raw, 6)) if raw else 0.0

    attitude = message.get("attitude_deg")
    if not isinstance(attitude, dict) or any(not isinstance(attitude.get(axis), dict)
                                             for axis in ("pitch", "yaw")):
        raise ValueError("缺少两轴机械姿态")
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
    offsets = miss.get("offset_deg") if valid else None
    if valid and (not isinstance(offsets, (list, tuple)) or len(offsets) != 2):
        raise ValueError("有效跟踪缺少二维角脱靶量")
    return {
        "track_status": int(valid),
        "pitch_angle": scaled(attitude["pitch"].get("pos")),
        "yaw_angle": scaled(attitude["yaw"].get("pos")),
        "pitch_misalignment": scaled(offsets[1]) if valid else 0.0,
        "yaw_misalignment": scaled(offsets[0]) if valid else 0.0,
        "tracking_confidence": 255.0 if valid else 0.0,
        "target_confidence": 80.0,
        "pitch_los_rate": scaled(attitude["pitch"].get("vel")),
        "yaw_los_rate": scaled(attitude["yaw"].get("vel")),
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

    def snapshot(self):
        with self._lock:
            return dict(active=self._socket is not None, target=self._target,
                        sent=self.sent, dropped=self.dropped, error=self.error)

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
                    data = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
                    self._socket.sendto(data, self._target)
                    self.sent += 1
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
