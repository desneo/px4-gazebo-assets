#!/usr/bin/env python3
"""单一速度设定值试飞：独立真值统计、受控减速与正常降落。"""
from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / 'models/vehicles/rocket_racing_quad/tests'))
from flight_check import FlightCheck, mavutil  # noqa: E402
from speed_metrics import StableSpeed  # noqa: E402

DEFAULTS = dict(altitude_m=80., target_speed_m_s=95., hold_seconds=10., ramp_seconds=25.,
    acquire_seconds=15., speed_cap_m_s=95., deceleration_m_s2=4., boundary_m=4500.,
    braking_margin_m=500., stable_std_m_s=.5,
    stable_accel_m_s2=.2, stable_range_m_s=2., stable_max_gap_s=.15, altitude_tolerance_m=2.,
    speed_tolerance_m_s=1., hover_seconds=3., climb_speed_m_s=3.,
    endpoint='udpin:0.0.0.0:14540', setpoint_rate_hz=30., telemetry_timeout_s=5.,
    heartbeat_timeout_s=5., command_timeout_s=20., phase_wall_timeout_s=300.,
    landing_timeout_s=180.)


def load_config(path):
    config = DEFAULTS.copy()
    supplied = json.loads(path.read_text()) if path else {}
    if not isinstance(supplied, dict):
        raise ValueError('场景配置必须为 JSON 对象')
    config.update(supplied)
    for key in DEFAULTS:
        value = config[key]
        if key == 'endpoint':
            if not isinstance(value, str) or not value.startswith('udpin:'):
                raise ValueError('endpoint 必须为 MAVLink udpin 地址')
        elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} 必须为有限正数')
    if config['setpoint_rate_hz'] < 10:
        raise ValueError('setpoint_rate_hz 至少为 10 Hz')
    if config['boundary_m'] > 4500:
        raise ValueError('boundary_m 不得超过 4500 m 地形边界余量')
    if config['braking_margin_m'] >= config['boundary_m']:
        raise ValueError('braking_margin_m 必须小于 boundary_m')
    return config


class SampleSink(list):
    def __init__(self, stream, kind):
        self.stream, self.kind, self.count = stream, kind, 0

    def append(self, value):
        self.stream.write(json.dumps({'kind': self.kind, **value}, ensure_ascii=False)+'\n')
        self.count += 1
        if self.count % 100 == 0:
            self.stream.flush()


class TrialTruth:
    """世界 ENU 位姿差分提供速度；PX4 估计量仅用于控制与健康检查。"""
    def __init__(self, model):
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.lock = threading.Lock()
        self.rows = deque()
        self.latest = self.previous = self.origin = None
        self.last_wall = 0.
        self.model = model
        self.topic = '/world/rocket_speed_comparison/dynamic_pose/info'
        self.node = Node()
        if not self.node.subscribe(Pose_V, self.topic, self.receive):
            raise RuntimeError(f'无法订阅 {self.topic}')

    def receive(self, message):
        stamp = message.header.stamp.sec+message.header.stamp.nsec*1e-9
        for pose in message.pose:
            if pose.name != self.model:
                continue
            p, q = pose.position, pose.orientation
            row = (stamp, p.x, p.y, p.z, (q.w, q.x, q.y, q.z))
            with self.lock:
                if self.latest and stamp <= self.latest[0]:
                    return
                self.latest, self.last_wall = row, time.monotonic()
                if self.origin is not None:
                    self.rows.append(row)
            return

    def calibrate(self):
        with self.lock:
            if self.latest is None:
                return False
            self.origin = self.latest[1:4]
            self.previous = self.latest
            self.rows.clear()
        return True

    def to_ned(self, row, home_z):
        ox, oy, oz = self.origin
        return [row[2]-oy, row[1]-ox, home_z+oz-row[3]]

    def drain(self, home_z):
        with self.lock:
            rows = list(self.rows)
            self.rows.clear()
        samples = []
        self.pose_rows = []
        for row in rows:
            old, self.previous = self.previous, row
            dt = row[0]-old[0]
            if dt <= 0:
                raise RuntimeError('Gazebo 时钟倒退')
            position = self.to_ned(row, home_z)
            velocity = [(row[2]-old[2])/dt, (row[1]-old[1])/dt, (old[3]-row[3])/dt]
            if not all(math.isfinite(v) for v in position+velocity+list(row[4])):
                raise RuntimeError('Gazebo 真值包含非法数值')
            samples.append((row[0], position, velocity))
            self.pose_rows.append({'sim_time_s': row[0], 'position_enu_m': row[1:4],
                                   'quaternion_wxyz': row[4]})
        return samples

    def position(self, home_z):
        with self.lock:
            row = self.latest
        return self.to_ned(row, home_z)

    def close(self):
        self.node.unsubscribe(self.topic)


class SpeedTrial(FlightCheck):
    def __init__(self, config, report, stream, model_name):
        super().__init__(config, report)
        try:
            self.truth = TrialTruth(model_name)
        except Exception:
            self.master.close()
            raise
        self.pose_sink = SampleSink(stream, 'gazebo_pose')
        self.metrics = StableSpeed(config['hold_seconds'], config['stable_max_gap_s'],
            config['altitude_tolerance_m'], config['stable_std_m_s'], config['stable_accel_m_s2'],
            config['target_speed_m_s'], max(config['speed_tolerance_m_s'], config['target_speed_m_s']*.01),
            config['stable_range_m_s'])
        self.flight_peak = None
        self.altitude_error = 0.
        self.latest_window = None
        self.geofence_active = False
        self.max_distance = 0.
        self.brake_requested = False

    def service(self, check_health=True):
        fresh = super().service(check_health)
        if self.truth.origin is not None:
            for row in getattr(self.truth, 'pose_rows', []):
                self.pose_sink.append({'phase': self.phase, **row})
            for stamp, position, velocity in self.truth_samples:
                distance = math.hypot(*position[:2])
                self.max_distance = max(self.max_distance, distance)
                if self.geofence_active and distance > self.cfg['boundary_m']:
                    self.report['limit_reason'] = 'geofence'
                    raise RuntimeError('Gazebo 真值位置超过场景航程边界')
                if self.phase in ('speed_ramp', 'speed_plateau'):
                    speed = math.hypot(*velocity[:2])
                    stopping_distance = speed*speed/(2*self.cfg['deceleration_m_s2'])
                    if distance+stopping_distance+self.cfg['braking_margin_m'] >= self.cfg['boundary_m']:
                        self.brake_requested = True
                        self.report['limit_reason'] = 'braking_boundary'
                        self.report.setdefault('braking_trigger', {'sim_time_s': stamp,
                            'distance_m': distance, 'actual_speed_m_s': speed,
                            'estimated_stopping_distance_m': stopping_distance})
                    self.flight_peak = max(self.flight_peak or 0., speed)
                    error = abs(position[2]-self.flight_z)
                    self.altitude_error = max(self.altitude_error, error)
                    if self.phase == 'speed_plateau':
                        window = self.metrics.update(stamp, speed, error)
                        if window is not None:
                            self.latest_window = window
        return fresh

    def run(self):
        self.connect()
        self.report['home_z_m'] = self.home_z
        self.hold('offboard_preload', 2.)
        self.set_mode('OFFBOARD')
        self.armed_by_script = True
        self.command_until(lambda: self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0),
            self.armed, self.cfg['command_timeout_s'])
        self.require_offboard = True
        self.geofence_active = True
        self.flight_z = self.home_z-self.cfg['altitude_m']
        start = self.sim_time

        def climb():
            self.z_command = max(self.flight_z, self.home_z-self.cfg['climb_speed_m_s']*(self.sim_time-start))
            return self.z_command == self.flight_z and abs(self.position[2]-self.flight_z) < .5 and abs(self.velocity[2]) < .3

        self.phase_loop('takeoff', climb)
        self.hold('hover', self.cfg['hover_seconds'])
        ramp_start = self.sim_time
        def accelerate():
            fraction = min(1., (self.sim_time-ramp_start)/self.cfg['ramp_seconds'])
            self.speed_command = self.cfg['target_speed_m_s']*fraction
            return fraction >= 1. or self.brake_requested
        self.phase_loop('speed_ramp', accelerate)
        if not self.brake_requested:
            start = self.truth.latest[0]
            def plateau():
                elapsed = self.truth.latest[0]-start
                return (self.brake_requested or
                        elapsed >= self.cfg['acquire_seconds']+self.cfg['hold_seconds']+2*self.cfg['stable_max_gap_s'])
            self.phase_loop('speed_plateau', plateau)
        actual_speed = math.hypot(*self.velocity[:2])
        self.ramp(0., max(1., max(self.speed_command, actual_speed)/self.cfg['deceleration_m_s2']), 'deceleration')
        self.phase_loop('stop', lambda: math.hypot(*self.velocity[:2]) < 1. and self.altitude_ok())
        self.geofence_active = False
        self.land()
        self.report['flight_succeeded'] = True

    def emergency_land(self):
        self.geofence_active = False
        super().emergency_land()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=HERE/'config.json')
    parser.add_argument('--model-config', type=Path, required=True)
    parser.add_argument('--model-name', choices=('rocket_racing_quad', 'rocket_two_axis_pod'), required=True)
    parser.add_argument('--target-speed', type=float)
    parser.add_argument('--hold-seconds', type=float)
    parser.add_argument('--ramp-seconds', type=float)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--validate-only', action='store_true', help='仅校验配置，不连接 PX4 或 Gazebo')
    args = parser.parse_args()
    config = load_config(args.config)
    for name, key in (('target_speed', 'target_speed_m_s'), ('hold_seconds', 'hold_seconds'), ('ramp_seconds', 'ramp_seconds')):
        if getattr(args, name) is not None:
            config[key] = getattr(args, name)
    for key in ('target_speed_m_s', 'hold_seconds', 'ramp_seconds'):
        if not math.isfinite(config[key]) or config[key] <= 0:
            parser.error(f'{key} 必须为有限正数')
    source = json.loads(args.model_config.read_text())
    model_cap = source['px4']['parameters']['MPC_XY_VEL_MAX']
    if isinstance(model_cap, bool) or not isinstance(model_cap, (int, float)) or not math.isfinite(model_cap) or model_cap <= 0:
        parser.error('模型 MPC_XY_VEL_MAX 必须为有限正数')
    cap = min(config['speed_cap_m_s'], model_cap)
    if config['target_speed_m_s'] > cap:
        parser.error(f'目标速度超过当前控制上限 {cap} m/s')
    if args.validate_only:
        print(f'配置有效：{args.model_name}，目标 {config["target_speed_m_s"]} m/s，控制上限 {cap} m/s')
        return 0
    args.log.parent.mkdir(parents=True, exist_ok=True)
    samples_path = args.log.with_suffix('.samples.jsonl')
    report = {'settings': config, 'model_name': args.model_name,
              'model_parameters': source, 'config_sha256': hashlib.sha256(args.model_config.read_bytes()).hexdigest(),
              'control_speed_cap_m_s': cap, 'measurement_source': 'gazebo_pose_difference',
              'started_at_unix_s': time.time(), 'flight_succeeded': False, 'landing_confirmed': False,
              'valid_stable_window': False, 'limit_reason': None, 'samples_path': str(samples_path), 'results': {}}
    trial = None
    def interrupt(signum, frame):
        raise KeyboardInterrupt(f'信号 {signum}')
    signal.signal(signal.SIGTERM, interrupt)
    code = 1
    with samples_path.open('w') as stream:
        for key in ('samples', 'status_text', 'ground_truth_samples'):
            report[key] = SampleSink(stream, key)
        try:
            trial = SpeedTrial(config, report, stream, args.model_name)
            trial.run()
            code = 0
        except (Exception, KeyboardInterrupt) as exc:
            report['error'] = str(exc) or '用户中断'
            report['limit_reason'] = report['limit_reason'] or 'flight_failure'
            if trial is not None:
                try:
                    trial.emergency_land()
                except Exception as recovery:
                    report['recovery_error'] = str(recovery)
        finally:
            if trial is not None:
                report.update(trial.metrics.summary())
                report['peak_speed_m_s'] = trial.flight_peak
                report['max_altitude_error_m'] = trial.altitude_error
                report['max_distance_m'] = trial.max_distance
                if report['flight_succeeded'] and report['limit_reason'] in (None, 'braking_boundary'):
                    if report['limit_reason'] == 'braking_boundary':
                        report['test_stop_reason'] = 'braking_boundary'
                    if report['target_reached']:
                        report['limit_reason'] = ('control_limited' if abs(config['target_speed_m_s']-cap) < 1e-6 else 'requested_speed_reached')
                    else:
                        report['limit_reason'] = 'target_not_reached' if report['valid_stable_window'] else 'no_stable_window'
                trial.truth.close()
                trial.master.close()
            for key in ('samples', 'status_text', 'ground_truth_samples'):
                value = report[key]
                if isinstance(value, SampleSink):
                    report[key+'_count'] = value.count
                    del report[key]
            report['passed'] = bool(report['flight_succeeded'] and report['landing_confirmed'] and report.get('target_reached'))
            report['finished_at_unix_s'] = time.time()
            args.log.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(f'试验结果：{args.log}')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
