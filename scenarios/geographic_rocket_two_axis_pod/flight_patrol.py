#!/usr/bin/env python3
"""PX4 实际动力学往返飞行；用 MAVLink 控制，用 Gazebo 位姿独立验收。"""

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
from patrol_math import at_endpoint, desired_velocity, slew_velocity  # noqa: E402


def load_config(path):
    overrides = json.loads(path.read_text())
    config = json.loads((HERE / 'config.json').read_text())
    if not isinstance(overrides, dict):
        raise ValueError('场景配置必须为 JSON 对象')
    unknown = overrides.keys() - config.keys()
    if unknown:
        raise ValueError(f'未知场景配置字段：{", ".join(sorted(unknown))}')
    config.update(overrides)
    for key, value in config.items():
        if key == 'endpoint':
            if not isinstance(value, str) or not value.startswith('udpin:') or not value.endswith(':14540'):
                raise ValueError('endpoint 必须是 udpin 监听地址，端口与本场景 PX4 instance0 一致为 14540')
        elif isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} 必须是有限正数')
    if config['setpoint_rate_hz'] < 10:
        raise ValueError('setpoint_rate_hz 必须至少 10 Hz')
    if config['leg_length_m'] > 4000:
        raise ValueError('航段长度必须保留地形边界余量，不得超过 4000 m')
    return config


class SampleSink(list):
    """连续巡航流式保存遥测，报告只保存计数，不累积无限样本。"""
    def __init__(self, stream, kind):
        self.stream, self.kind, self.count = stream, kind, 0

    def append(self, value):
        self.stream.write(json.dumps({'kind': self.kind, **value}, ensure_ascii=False) + '\n')
        self.count += 1
        if self.count % 100 == 0:
            self.stream.flush()


class PatrolTruth:
    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.lock = threading.Lock()
        self.rows = deque(maxlen=20000)
        self.latest = self.previous = self.origin = None
        self.last_wall = 0.
        self.node = Node()
        self.topic = '/world/geographic_rocket_two_axis_pod/dynamic_pose/info'
        if not self.node.subscribe(Pose_V, self.topic, self.receive):
            raise RuntimeError(f'无法订阅 Gazebo 真值：{self.topic}')

    def receive(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
        for pose in message.pose:
            if pose.name != 'rocket_two_axis_pod':
                continue
            row = (stamp, pose.position.x, pose.position.y, pose.position.z)
            with self.lock:
                if self.latest and stamp <= self.latest[0]:
                    return
                self.latest = row
                self.last_wall = time.monotonic()
                if self.origin is not None:
                    self.rows.append(row)
            return

    def calibrate(self):
        with self.lock:
            if self.latest is None:
                return False
            self.origin = self.latest[1:]
            self.previous = self.latest
            self.rows.clear()
            return True

    def drain(self, home_z):
        with self.lock:
            rows = list(self.rows)
            self.rows.clear()
        samples = []
        for row in rows:
            old, self.previous = self.previous, row
            dt = row[0] - old[0]
            if dt <= 0:
                raise RuntimeError('Gazebo 真值时钟倒退')
            position = self.to_ned(row, home_z)
            velocity = [(row[2]-old[2])/dt, (row[1]-old[1])/dt, (old[3]-row[3])/dt]
            if not all(math.isfinite(v) for v in position + velocity):
                raise RuntimeError('Gazebo 真值含非有限值')
            samples.append((row[0], position, velocity))
        return samples

    def to_ned(self, row, home_z):
        ox, oy, oz = self.origin
        return [row[2]-oy, row[1]-ox, home_z+oz-row[3]]

    def position(self, home_z):
        with self.lock:
            row = self.latest
        return self.to_ned(row, home_z)

    def close(self):
        self.node.unsubscribe(self.topic)


class Patrol(FlightCheck):
    def __init__(self, config, report, cycles, stream):
        super().__init__(config, report)
        try:
            self.truth = PatrolTruth()
        except Exception:
            self.master.close()
            raise
        self.cycles = cycles
        self.stream = stream
        self.completed_legs = 0
        self.metrics = None

    def service(self, check_health=True):
        fresh = super().service(check_health)
        if self.metrics is not None:
            for stamp, position, velocity in self.truth_samples:
                metrics = self.metrics
                metrics['samples'] += 1
                metrics['max_altitude_error_m'] = max(metrics['max_altitude_error_m'], abs(position[2]-self.flight_z))
                metrics['max_cross_track_error_m'] = max(metrics['max_cross_track_error_m'], abs(position[1]))
                # 去除加速和减速区，单独记录航段中部实际速度。
                margin = self.cfg['speed_m_s'] ** 2 / (2*self.cfg['acceleration_m_s2']) + 20
                if margin < position[0] < self.cfg['leg_length_m'] - margin:
                    speed = math.hypot(*velocity[:2])
                    metrics['cruise_samples'] += 1
                    metrics['cruise_speed_sum_m_s'] += speed
                    if abs(speed-self.cfg['speed_m_s']) <= self.cfg['speed_tolerance_m_s']:
                        metrics['cruise_samples_within_tolerance'] += 1
        return fresh

    def run(self):
        self.connect()
        self.home_north = self.position[0]
        self.hold('offboard_preload', 2)
        self.set_mode('OFFBOARD')
        self.armed_by_script = True
        self.command_until(lambda: self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0),
            self.armed, self.cfg['command_timeout_s'])
        self.require_offboard = True
        self.flight_z = self.home_z - self.cfg['altitude_m']
        start = self.sim_time

        def climb():
            self.z_command = max(self.flight_z, self.home_z-self.cfg['climb_speed_m_s']*(self.sim_time-start))
            return self.z_command == self.flight_z and abs(self.position[2]-self.flight_z) < .5 and abs(self.velocity[2]) < .3

        self.phase_loop('takeoff', climb)
        self.hold('hover', self.cfg['hover_seconds'])
        while self.cycles == 0 or self.completed_legs < self.cycles * 2:
            relative_target = self.cfg['leg_length_m'] if self.completed_legs % 2 == 0 else 0.
            self.fly_leg(relative_target)
            self.completed_legs += 1
            self.report['completed_legs'] = self.completed_legs
        self.speed_command = 0.
        self.land()

    def fly_leg(self, relative_target):
        target = self.home_north + relative_target
        previous_time = self.sim_time
        self.metrics = {'samples': 0, 'max_altitude_error_m': 0.,
                        'max_cross_track_error_m': 0., 'cruise_samples': 0,
                        'cruise_speed_sum_m_s': 0., 'cruise_samples_within_tolerance': 0}

        def update():
            nonlocal previous_time
            dt, previous_time = self.sim_time-previous_time, self.sim_time
            error = target-self.position[0]
            desired = desired_velocity(error, self.cfg['speed_m_s'],
                                       self.cfg['acceleration_m_s2'], self.cfg['position_gain_s_inv'])
            self.speed_command = slew_velocity(self.speed_command, desired,
                                              self.cfg['acceleration_m_s2'], dt)
            return at_endpoint(error, self.velocity[0], self.cfg['endpoint_tolerance_m'],
                               self.cfg['endpoint_speed_m_s']) and self.altitude_ok()

        name = f'leg_{self.completed_legs+1}_to_{"B" if relative_target else "A"}'
        self.phase_loop(name, update)
        metrics, self.metrics = self.metrics, None
        truth_position = self.truth.position(self.home_z)
        endpoint_error = abs(truth_position[0]-relative_target)
        cruise_count = metrics['cruise_samples']
        metrics.update({'phase': name, 'target_north_from_start_m': relative_target,
                        'truth_endpoint_ned_m': truth_position,
                        'truth_endpoint_error_m': endpoint_error,
                        'cruise_mean_speed_m_s': metrics['cruise_speed_sum_m_s']/cruise_count if cruise_count else None,
                        'cruise_speed_within_tolerance_fraction': metrics['cruise_samples_within_tolerance']/cruise_count if cruise_count else None})
        metrics['passed'] = (metrics['samples'] > 0 and endpoint_error <= self.cfg['truth_endpoint_tolerance_m']
                             and metrics['max_altitude_error_m'] <= self.cfg['altitude_tolerance_m']
                             and metrics['max_cross_track_error_m'] <= self.cfg['cross_track_tolerance_m']
                             and cruise_count > 0 and metrics['cruise_speed_within_tolerance_fraction'] >= .95)
        # 连续模式不在内存中保存无限航段；完整证据流式保存在 JSONL。
        self.report['last_leg'] = metrics
        self.stream.write(json.dumps({'kind': 'leg_result', **metrics}, ensure_ascii=False) + '\n')
        self.stream.flush()
        print(json.dumps(metrics, ensure_ascii=False), flush=True)
        if not metrics['passed']:
            raise RuntimeError(f'{name} Gazebo 真值验收未通过')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=HERE / 'config.json')
    parser.add_argument('--model-config', type=Path, required=True,
                        help='实际动态装配的 parameters.json，用于核对PX4速度限制及保存指纹')
    parser.add_argument('--cycles', type=int, default=0, help='完整往返次数；0持续巡航')
    parser.add_argument('--log', type=Path, default=ROOT / 'runs/geographic_rocket_two_axis_pod/flight_result.json')
    args = parser.parse_args()
    try:
        config = load_config(args.config)
        model_config = json.loads(args.model_config.read_text())
        if args.cycles < 0:
            raise ValueError('cycles 必须为非负整数')
        limit = model_config['px4']['parameters']['MPC_XY_VEL_MAX']
        if isinstance(limit, bool) or not isinstance(limit, (int, float)) or not math.isfinite(limit) or limit <= 0:
            raise ValueError('实际模型 MPC_XY_VEL_MAX 必须是有限正数')
        if config['speed_m_s'] > limit:
            raise ValueError(f'巡航设定速度超过实际PX4 MPC_XY_VEL_MAX={limit}')
    except (ValueError, OSError, KeyError) as error:
        parser.error(str(error))
    args.log.parent.mkdir(parents=True, exist_ok=True)
    sample_path = args.log.with_suffix('.jsonl')
    report = {'settings': config, 'cycles_requested': args.cycles,
              'started_at_unix_s': time.time(), 'measurement_source': 'gazebo_pose',
              'model_config_sha256': hashlib.sha256(args.model_config.read_bytes()).hexdigest(),
              'model_parameters': model_config, 'results': {}, 'completed_legs': 0,
              'landing_confirmed': False, 'passed': False, 'samples_file': str(sample_path)}
    check, code = None, 1

    def interrupt(signum, frame):
        raise KeyboardInterrupt('收到停止请求')

    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGINT, interrupt)
    with sample_path.open('w') as stream:
        for key, kind in (('samples', 'px4'), ('ground_truth_samples', 'gazebo'), ('status_text', 'px4_status')):
            report[key] = SampleSink(stream, kind)
        try:
            check = Patrol(config, report, args.cycles, stream)
            check.run()
            report['passed'], code = True, 0
        except (Exception, KeyboardInterrupt) as error:
            report['error'] = str(error) or '用户中断'
            print(f'巡航停止：{report["error"]}', flush=True)
            # 一次停止请求进入正常降落，后续停止请求不打断有界降落恢复。
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            if check is not None:
                try:
                    check.emergency_land()
                except Exception as recovery:
                    report['recovery_error'] = str(recovery)
            if isinstance(error, KeyboardInterrupt):
                report['stopped_by_user'] = True
                code = 0 if report['landing_confirmed'] else 1
        finally:
            report['finished_at_unix_s'] = time.time()
            for key in ('samples', 'ground_truth_samples', 'status_text'):
                report[key + '_count'] = report.pop(key).count
            if check is not None:
                if check.metrics is not None:
                    report['incomplete_leg'] = check.metrics
                try:
                    check.truth.close()
                finally:
                    check.master.close()
            args.log.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(f'飞行报告：{args.log}；原始样本：{sample_path}', flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
