#!/usr/bin/env python3
"""PX4 SITL 飞行验收；参数来自 ../parameters.json，结果与遥测写入 JSON。"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

from pymavlink import mavutil

# XY 速度 + Z 位置；忽略 XY 位置、Z 速度、加速度和偏航。
TYPE_MASK = 0b0000110111100011
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "parameters.json"


def load_settings(path: Path) -> dict:
    settings = json.loads(path.read_text())["flight_test"]
    positive = (
        "setpoint_rate_hz", "telemetry_timeout_s", "heartbeat_timeout_s",
        "command_timeout_s", "phase_wall_timeout_s", "takeoff_altitude_m",
        "climb_speed_m_s", "altitude_tolerance_m", "hover_seconds",
        "hover_drift_limit_m", "cruise_speed_km_h", "cruise_seconds",
        "sprint_speed_km_h", "sprint_seconds", "ramp_seconds",
        "speed_acquire_timeout_s", "deceleration_m_s2", "landing_timeout_s",
    )
    for key in positive:
        value = settings[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"flight_test.{key} 必须为有限正数")
    if settings["setpoint_rate_hz"] < 10:
        raise ValueError("setpoint_rate_hz 必须至少为 10 Hz")
    if not math.isfinite(settings["speed_setpoint_margin_km_h"]) or settings["speed_setpoint_margin_km_h"] < 0:
        raise ValueError("speed_setpoint_margin_km_h 必须为有限非负数")
    return settings


class ContinuousHold:
    """只对连续、相邻的新鲜仿真样本累计时间。"""
    def __init__(self, max_gap: float):
        self.start = None
        self.previous = None
        self.longest = 0.0
        self.max_gap = max_gap

    def update(self, stamp: float, valid: bool) -> float:
        if self.previous is not None and stamp <= self.previous:
            raise RuntimeError("遥测仿真时钟倒退或重复")
        if not valid or (self.previous is not None and stamp - self.previous > self.max_gap):
            self.start = None
        if valid and self.start is None:
            self.start = stamp
        self.previous = stamp
        elapsed = stamp - self.start if self.start is not None else 0.0
        self.longest = max(self.longest, elapsed)
        return elapsed


class FlightCheck:
    def __init__(self, settings: dict, report: dict, ground_truth=False):
        self.cfg = settings
        self.report = report
        self.master = mavutil.mavlink_connection(settings["endpoint"], source_system=255)
        self.position = None
        self.velocity = None
        self.sim_time = None
        self.last_position_wall = 0.0
        self.last_heartbeat_wall = 0.0
        self.heartbeat = None
        self.landed = None
        self.last_landed_wall = 0.0
        self.last_send = 0.0
        self.last_gcs = 0.0
        self.phase = "connect"
        self.speed_command = 0.0
        self.z_command = 0.0
        self.require_offboard = False
        self.armed_by_script = False
        self.truth = None
        if ground_truth:
            from gazebo_truth import GazeboTruth
            self.truth = GazeboTruth()
        self.truth_samples = []

    def measurement_samples(self):
        if self.truth is not None:
            return self.truth_samples
        return getattr(self, "latest_samples", None) or [(self.sim_time, self.position, self.velocity)]

    def armed(self) -> bool:
        return bool(self.heartbeat and self.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def mode(self) -> str:
        return mavutil.interpret_px4_mode(self.heartbeat.base_mode, self.heartbeat.custom_mode) if self.heartbeat else "UNKNOWN"

    def service(self, check_health=True) -> bool:
        """所有等待共用此循环，保持 OFFBOARD 与地面站心跳不中断。"""
        now = time.monotonic()
        if now - self.last_gcs >= 1:
            self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_gcs = now
        if self.master.target_system and now - self.last_send >= 1 / self.cfg["setpoint_rate_hz"]:
            self.master.mav.set_position_target_local_ned_send(
                int((self.sim_time or 0) * 1000) & 0xFFFFFFFF,
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, TYPE_MASK,
                0, 0, self.z_command, self.speed_command, 0, 0, 0, 0, 0, 0, 0)
            self.last_send = now
        fresh = False
        self.latest_samples = []
        # 有上限，避免消息洪泛饿死设定值发送。
        for _ in range(300):
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            kind = msg.get_type()
            # pymavlink 自动选定 system，但 target_component 初始仍为广播值 0。
            # 先从飞控心跳确定组件，才能按来源过滤后续遥测。
            if (self.heartbeat is None and kind == "HEARTBEAT"
                    and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
                    and msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1):
                self.master.target_system = msg.get_srcSystem()
                self.master.target_component = msg.get_srcComponent()
            if msg.get_srcSystem() != self.master.target_system or msg.get_srcComponent() != self.master.target_component:
                continue
            if kind == "HEARTBEAT":
                self.heartbeat = msg
                self.last_heartbeat_wall = now
            elif kind == "EXTENDED_SYS_STATE":
                self.landed = msg.landed_state
                self.last_landed_wall = now
            elif kind == "LOCAL_POSITION_NED":
                stamp = msg.time_boot_ms / 1000.0
                if self.sim_time is not None and stamp < self.sim_time:
                    raise RuntimeError("PX4 时钟重置；本次验收无效")
                if stamp == self.sim_time:
                    continue
                values = [msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz]
                if not all(math.isfinite(value) for value in values):
                    raise RuntimeError("位置或速度遥测含非有限数")
                self.sim_time = stamp
                self.position = values[:3]
                self.velocity = values[3:]
                self.last_position_wall = now
                fresh = True
                self.latest_samples.append((stamp, self.position, self.velocity))
                self.report["samples"].append({"phase": self.phase, "sim_time_s": stamp,
                    "position_ned_m": self.position, "velocity_ned_m_s": self.velocity,
                    "command_vx_m_s": self.speed_command, "command_z_m": self.z_command})
            elif kind == "STATUSTEXT":
                self.report["status_text"].append({"wall_time": time.time(), "text": msg.text})
        if check_health:
            if now - self.last_position_wall > self.cfg["telemetry_timeout_s"]:
                raise RuntimeError("位置遥测超时")
            if now - self.last_heartbeat_wall > self.cfg["heartbeat_timeout_s"]:
                raise RuntimeError("飞控心跳超时")
            if self.require_offboard and (self.mode() != "OFFBOARD" or not self.armed()):
                raise RuntimeError(f"飞行中退出 OFFBOARD 或解除解锁：{self.mode()}")
        if self.truth is not None and self.truth.origin is not None:
            self.truth_samples = self.truth.drain(self.home_z)
            for stamp, position, velocity in self.truth_samples:
                self.report.setdefault("ground_truth_samples", []).append({"phase": self.phase,
                    "sim_time_s": stamp, "position_ned_m": position, "velocity_ned_m_s": velocity})
            fresh = fresh or bool(self.truth_samples)
            if check_health and now - self.truth.last_wall > self.cfg["telemetry_timeout_s"]:
                raise RuntimeError("Gazebo 真值位姿超时")
        time.sleep(min(0.01, 0.25 / self.cfg["setpoint_rate_hz"]))
        return fresh

    def connect(self):
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            self.service(check_health=False)
            if self.heartbeat and self.position is not None:
                if self.heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
                    raise RuntimeError("目标不是 PX4 飞控")
                break
            if self.heartbeat and not hasattr(self, "requested_streams"):
                for message_id, hz in ((32, 50), (245, 5)):
                    self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, message_id, 1e6 / hz, 0, 0, 0, 0, 0)
                self.requested_streams = True
        else:
            raise RuntimeError("60 秒内未收到 PX4 心跳与位置遥测")
        if self.armed():
            raise RuntimeError("飞控已解锁；拒绝接管正在进行的飞行")
        self.z_command = self.position[2]
        self.home_z = self.position[2]
        if self.truth is not None:
            deadline = time.monotonic() + self.cfg["command_timeout_s"]
            while not self.truth.calibrate():
                if time.monotonic() > deadline:
                    raise RuntimeError("未收到 Gazebo 真值，请检查 GZ_PARTITION")
                self.service()
        print(f"已连接 PX4 system={self.master.target_system}")

    def command_until(self, action, predicate, timeout):
        deadline = time.monotonic() + timeout
        next_command = 0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_command:
                action()
                next_command = time.monotonic() + 1
            self.service()
            if predicate():
                return
        raise RuntimeError("模式或解锁命令未在期限内确认")

    def set_mode(self, mode):
        self.command_until(lambda: self.master.set_mode_px4(mode, 0, 0),
            lambda: self.mode() == mode, self.cfg["command_timeout_s"])

    def phase_loop(self, name, callback, timeout=None):
        self.phase = name
        print(f"航段：{name}")
        deadline = time.monotonic() + (timeout or self.cfg["phase_wall_timeout_s"])
        while time.monotonic() < deadline:
            if self.service() and callback():
                return
        raise RuntimeError(f"{name} 超过墙钟安全时限")

    def hold(self, name, duration, callback=None):
        start = self.sim_time
        def step():
            if callback:
                callback()
            return self.sim_time - start >= duration
        self.phase_loop(name, step)

    def altitude_ok(self):
        return abs(self.position[2] - self.flight_z) <= self.cfg["altitude_tolerance_m"]

    def ramp(self, target, duration, name):
        start_time, start_speed = self.sim_time, self.speed_command
        def step():
            fraction = min(1, (self.sim_time - start_time) / duration)
            self.speed_command = start_speed + (target - start_speed) * fraction
            return fraction >= 1
        self.phase_loop(name, step)

    def speed_test(self, name, speed_kmh, duration):
        target = speed_kmh / 3.6
        command = (speed_kmh + self.cfg["speed_setpoint_margin_km_h"]) / 3.6
        self.ramp(command, self.cfg["ramp_seconds"], f"{name}_ramp")
        tracker = ContinuousHold(self.cfg["telemetry_timeout_s"])
        start = self.sim_time
        peak = 0
        max_altitude_error = 0
        def step():
            nonlocal peak, max_altitude_error
            # 一次接收可能包含多条遥测，逐条判定，避免漏掉短暂掉速或高度越界。
            samples = self.measurement_samples()
            for stamp, position, velocity in samples:
                speed = math.hypot(*velocity[:2])
                peak = max(peak, speed)
                error = abs(position[2] - self.flight_z)
                max_altitude_error = max(max_altitude_error, error)
                tracker.update(stamp, speed >= target and error <= self.cfg["altitude_tolerance_m"])
            if tracker.longest >= duration:
                return True
            if self.sim_time - start > self.cfg["speed_acquire_timeout_s"] + duration:
                raise RuntimeError(f"{name} 未连续达到速度和高度要求")
            return False
        try:
            self.phase_loop(name, step)
        finally:
            self.report["results"][name] = {"required_speed_km_h": speed_kmh,
                "required_continuous_s": duration, "longest_continuous_s": tracker.longest,
                "max_altitude_error_m": max_altitude_error,
                "peak_speed_km_h": peak * 3.6, "passed": tracker.longest >= duration}

    def land(self):
        self.require_offboard = False
        self.set_mode("LAND")
        def complete():
            return (not self.armed() and self.landed == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                and time.monotonic() - self.last_landed_wall < self.cfg["telemetry_timeout_s"])
        self.phase_loop("landing", complete, self.cfg["landing_timeout_s"])
        self.report["landing_confirmed"] = True

    def run(self):
        self.connect()
        self.hold("offboard_preload", 2)
        self.set_mode("OFFBOARD")
        self.armed_by_script = True
        self.command_until(lambda: self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0),
            self.armed, self.cfg["command_timeout_s"])
        self.require_offboard = True
        self.flight_z = self.home_z - self.cfg["takeoff_altitude_m"]
        start = self.sim_time
        def climb():
            self.z_command = max(self.flight_z, self.home_z - self.cfg["climb_speed_m_s"] * (self.sim_time - start))
            return self.z_command == self.flight_z and abs(self.position[2] - self.flight_z) < 0.5 and abs(self.velocity[2]) < 0.3
        self.phase_loop("takeoff", climb)
        origin = self.truth.position(self.home_z) if self.truth else self.position[:]
        drift, altitude_error = 0, 0
        def hover():
            nonlocal drift, altitude_error
            for _, position, _ in self.measurement_samples():
                drift = max(drift, math.hypot(position[0] - origin[0], position[1] - origin[1]))
                altitude_error = max(altitude_error, abs(position[2] - self.flight_z))
        self.hold("hover", self.cfg["hover_seconds"], hover)
        passed = drift <= self.cfg["hover_drift_limit_m"] and altitude_error <= self.cfg["altitude_tolerance_m"]
        self.report["results"]["hover"] = {"max_drift_m": drift, "max_altitude_error_m": altitude_error, "passed": passed}
        if not passed:
            raise RuntimeError("悬停漂移或高度误差超限")
        self.speed_test("cruise", self.cfg["cruise_speed_km_h"], self.cfg["cruise_seconds"])
        self.speed_test("sprint", self.cfg["sprint_speed_km_h"], self.cfg["sprint_seconds"])
        self.ramp(0, max(1, abs(self.speed_command) / self.cfg["deceleration_m_s2"]), "deceleration")
        self.phase_loop("stop", lambda: math.hypot(*self.velocity[:2]) < 1 and self.altitude_ok())
        self.land()

    def emergency_land(self):
        # 不发送 disarm；即使遥测已失联也尝试把 LAND 命令交给飞控。
        self.require_offboard = False
        self.speed_command = 0
        if self.armed_by_script and self.last_heartbeat_wall:
            for _ in range(3):
                self.master.set_mode_px4("LAND", 0, 0)
                self.service(check_health=False)
            try:
                self.land()
            except Exception as exc:
                self.report["recovery_error"] = str(exc)
        else:
            self.report["recovery_error"] = "未确认本脚本解锁的飞控状态，未发送解除解锁命令"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--endpoint")
    parser.add_argument("--cruise-speed", type=float, help="巡航门槛 km/h")
    parser.add_argument("--sprint-speed", type=float, help="冲刺门槛 km/h")
    parser.add_argument("--target-speed", type=float, help="兼容参数：冲刺门槛 m/s")
    parser.add_argument("--hover-seconds", type=float)
    parser.add_argument("--cruise-seconds", type=float)
    parser.add_argument("--sprint-seconds", type=float)
    parser.add_argument("--altitude", type=float, help="起飞相对高度 m")
    parser.add_argument("--log", type=Path, default=Path(f"/tmp/rocket-flight-{time.time_ns()}.json"))
    parser.add_argument("--ground-truth", action="store_true", help="使用同一GZ_PARTITION下的Gazebo真值判定速度、漂移与高度")
    args = parser.parse_args()
    settings = load_settings(args.config)
    for arg, key in (("endpoint", "endpoint"), ("cruise_speed", "cruise_speed_km_h"),
        ("sprint_speed", "sprint_speed_km_h"), ("hover_seconds", "hover_seconds"),
        ("cruise_seconds", "cruise_seconds"), ("sprint_seconds", "sprint_seconds"), ("altitude", "takeoff_altitude_m")):
        if getattr(args, arg) is not None:
            settings[key] = getattr(args, arg)
    if args.target_speed is not None:
        settings["sprint_speed_km_h"] = args.target_speed * 3.6
    for key, value in settings.items():
        if isinstance(value, (int, float)) and (not math.isfinite(value) or value < 0 or (value == 0 and key != "speed_setpoint_margin_km_h")):
            parser.error(f"{key} 必须为有限正数")
    px4_limit = json.loads(args.config.read_text())["px4"]["parameters"]["MPC_XY_VEL_MAX"]
    requested = (max(settings["cruise_speed_km_h"], settings["sprint_speed_km_h"])
        + settings["speed_setpoint_margin_km_h"]) / 3.6
    if requested > px4_limit:
        parser.error(f"速度设定值 {requested:.2f} m/s 超过配置 MPC_XY_VEL_MAX={px4_limit}")
    if settings["setpoint_rate_hz"] < 10:
        parser.error("setpoint_rate_hz 必须至少为 10 Hz")
    report = {"settings": settings, "started_at_unix_s": time.time(), "results": {}, "samples": [], "status_text": [], "landing_confirmed": False, "passed": False}
    report["model_parameters"] = json.loads(args.config.read_text())
    report["config_sha256"] = hashlib.sha256(args.config.read_bytes()).hexdigest()
    report["measurement_source"] = "gazebo_pose" if args.ground_truth else "px4_local_position"
    check = None
    code = 1
    try:
        check = FlightCheck(settings, report, ground_truth=args.ground_truth)
        check.run()
        report["passed"] = True
        code = 0
    except (Exception, KeyboardInterrupt) as exc:
        report["error"] = str(exc) or "用户中断"
        print(f"验收未通过：{report['error']}")
        if check is not None:
            try:
                check.emergency_land()
            except Exception as recovery:
                report["recovery_error"] = str(recovery)
    finally:
        report["finished_at_unix_s"] = time.time()
        args.log.parent.mkdir(parents=True, exist_ok=True)
        args.log.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(f"结果日志：{args.log}")
        if check is not None:
            if check.truth is not None:
                check.truth.close()
            check.master.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
