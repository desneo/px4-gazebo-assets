#!/usr/bin/env python3
"""用完整飞行 ULog 与实际吊舱姿态遥测验收悬停稳定性。"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np


AXES = ('roll', 'pitch')
RATE_FIELDS = {'roll': 'xyz[0]', 'pitch': 'xyz[1]'}
SETPOINT_FIELDS = {'roll': 'roll', 'pitch': 'pitch'}
TORQUE_FIELDS = {'roll': 'unallocated_torque[0]', 'pitch': 'unallocated_torque[1]'}


def finite_number(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


def validate_settings(settings):
    required = {
        'hover_settle_s', 'spectral_window_s', 'attitude_trend_window_s',
        'rate_error_rms_limit_dps', 'rate_error_p95_limit_dps',
        'rate_error_max_limit_dps', 'rate_hf_rms_limit_dps',
        'attitude_p95_limit_deg', 'attitude_peak_to_peak_limit_deg',
        'allocator_unachieved_ratio_limit',
        'allocator_unachieved_continuous_limit_s',
        'unallocated_torque_p99_limit', 'motor_floor_threshold',
        'motor_floor_ratio_limit', 'camera_los_p95_limit_px',
        'camera_los_max_limit_px',
    }
    if (not isinstance(settings, dict) or
            set(settings) != required | {'spectral_band_hz', 'camera_los_required'}):
        raise ValueError('stability_validation 字段不完整或含未知字段')
    if not isinstance(settings['camera_los_required'], bool):
        raise ValueError('camera_los_required 必须为布尔值')
    for key in required:
        value = settings[key]
        if not finite_number(value) or value < 0:
            raise ValueError(f'stability_validation.{key} 必须为有限非负数')
    band = settings['spectral_band_hz']
    if (not isinstance(band, list) or len(band) != 2 or
            not all(finite_number(value) for value in band) or
            not 0 < band[0] < band[1]):
        raise ValueError('spectral_band_hz 必须为两个递增有限正数')
    for key in ('allocator_unachieved_ratio_limit', 'motor_floor_ratio_limit'):
        if settings[key] > 1:
            raise ValueError(f'{key} 必须位于0～1')
    if settings['spectral_window_s'] <= 0 or settings['attitude_trend_window_s'] <= 0:
        raise ValueError('频谱窗与姿态趋势窗必须为有限正数')
    return settings


def phase_window(report, key, settle):
    rows = report.get(key)
    if not isinstance(rows, list):
        raise ValueError(f'飞行报告缺少 {key}')
    stamps = [row.get('sim_time_s') for row in rows if row.get('phase') == 'hover']
    if len(stamps) < 2 or not all(finite_number(value) for value in stamps):
        raise ValueError(f'{key} 缺少有效hover时间窗')
    start, end = min(stamps) + settle, max(stamps)
    if end <= start:
        raise ValueError(f'{key} 的hover时间不足以跳过前{settle:g}s')
    return start, end


def dataset_data(ulog, name):
    try:
        return ulog.get_dataset(name).data
    except (KeyError, IndexError, ValueError) as error:
        raise ValueError(f'ULog缺少必需主题 {name}') from error


def array(data, field, label=None):
    if field not in data:
        raise ValueError(f'ULog主题缺少字段 {label or field}')
    values = np.asarray(data[field])
    if values.ndim != 1 or not np.all(np.isfinite(values)):
        raise ValueError(f'ULog字段 {label or field} 含无效值')
    return values.astype(float, copy=False)


def time_seconds(data, label):
    values = array(data, 'timestamp', f'{label}.timestamp') * 1e-6
    if len(values) < 2 or np.any(np.diff(values) <= 0):
        raise ValueError(f'{label}时间必须严格递增')
    return values


def window_values(data, field, start, end, label):
    stamps = time_seconds(data, label)
    mask = (stamps >= start) & (stamps <= end)
    values = array(data, field, f'{label}.{field}')
    if len(values) != len(stamps):
        raise ValueError(f'{label}.{field}长度与时间戳不一致')
    if np.count_nonzero(mask) < 4:
        raise ValueError(f'{label}在正式hover窗内样本不足')
    return stamps[mask], values[mask]


def interpolate(source_t, source_v, target_t, label):
    if target_t[0] < source_t[0] or target_t[-1] > source_t[-1]:
        raise ValueError(f'{label}没有覆盖完整hover正式窗口')
    return np.interp(target_t, source_t, source_v)


def percentile_abs(values, fraction):
    return float(np.percentile(np.abs(values), fraction))


def uniform_samples(stamps, values, high_hz):
    gaps = np.diff(stamps)
    median = float(np.median(gaps))
    if not math.isfinite(median) or median <= 0:
        raise ValueError('角速度采样间隔无效')
    sample_hz = 1.0 / median
    if sample_hz < 2.2 * high_hz:
        raise ValueError(f'角速度采样率{sample_hz:.1f}Hz不足以验收{high_hz:g}Hz频带')
    count = int(math.floor((stamps[-1] - stamps[0]) * sample_hz)) + 1
    if count < 16:
        raise ValueError('正式hover窗口不足以计算频谱')
    target = stamps[0] + np.arange(count) / sample_hz
    return target, np.interp(target, stamps, values), sample_hz


def welch_metrics(stamps, values, low_hz, high_hz, window_s):
    _, samples, sample_hz = uniform_samples(stamps, values, high_hz)
    length = min(len(samples), max(16, int(round(window_s * sample_hz))))
    if length % 2:
        length -= 1
    step = max(1, length // 2)
    window = np.hanning(length)
    scale = sample_hz * float(np.sum(window * window))
    spectra = []
    for offset in range(0, len(samples) - length + 1, step):
        segment = samples[offset:offset + length]
        segment = segment - np.mean(segment)
        power = np.abs(np.fft.rfft(segment * window)) ** 2 / scale
        if length > 2:
            power[1:-1] *= 2
        spectra.append(power)
    if not spectra:
        raise ValueError('正式hover窗口不足一个频谱窗')
    psd = np.mean(spectra, axis=0)
    frequencies = np.fft.rfftfreq(length, 1.0 / sample_hz)
    mask = (frequencies >= low_hz) & (frequencies <= high_hz)
    if not np.any(mask):
        raise ValueError('频谱分辨率无法覆盖目标频带')
    df = sample_hz / length
    band_power = float(np.sum(psd[mask]) * df)
    peak_index = np.flatnonzero(mask)[int(np.argmax(psd[mask]))]
    return dict(sample_hz=sample_hz, rms_dps=math.sqrt(max(0., band_power)),
                peak_hz=float(frequencies[peak_index]),
                peak_psd_dps2_hz=float(psd[peak_index]))


def quaternion_euler(q0, q1, q2, q3):
    norm = np.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    if np.any(~np.isfinite(norm)) or np.any(np.abs(norm - 1) > .01):
        raise ValueError('vehicle_attitude.q不是有效单位四元数')
    w, x, y, z = (value / norm for value in (q0, q1, q2, q3))
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    return np.unwrap(roll), np.unwrap(pitch)


def centered_trend(stamps, values, width_s):
    result = np.empty_like(values)
    prefix = np.concatenate(([0.], np.cumsum(values)))
    left = right = 0
    half = width_s / 2
    for index, stamp in enumerate(stamps):
        while left < len(stamps) and stamps[left] < stamp - half:
            left += 1
        while right < len(stamps) and stamps[right] <= stamp + half:
            right += 1
        result[index] = (prefix[right] - prefix[left]) / (right - left)
    return result


def longest_true(stamps, flags):
    longest = 0.
    start = None
    previous = None
    median_gap = float(np.median(np.diff(stamps))) if len(stamps) > 1 else 0.
    for stamp, flag in zip(stamps, flags):
        if flag and start is None:
            start = stamp
        if not flag and start is not None:
            longest = max(longest, previous + median_gap - start)
            start = None
        previous = stamp
    if start is not None:
        longest = max(longest, stamps[-1] + median_gap - start)
    return float(longest)


def quaternion_rotate(q, vector):
    w, x, y, z = q
    vx, vy, vz = vector
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return np.array((vx+w*tx+y*tz-z*ty,
                     vy+w*ty+z*tx-x*tz,
                     vz+w*tz+x*ty-y*tx), dtype=float)


def camera_intrinsics(model_sdf):
    if model_sdf is None:
        raise ValueError('没有提供运行模型SDF')
    model = ET.parse(model_sdf).getroot().find('model')
    cameras = model.findall(".//sensor[@type='camera']/camera")
    if len(cameras) != 1:
        raise ValueError('运行模型必须包含唯一视频相机')
    camera = cameras[0]
    width = int(camera.findtext('image/width'))
    height = int(camera.findtext('image/height'))
    hfov = float(camera.findtext('horizontal_fov'))
    if width <= 0 or height <= 0 or not 0 < hfov < math.pi:
        raise ValueError('运行相机内参无效')
    return width, height, hfov


def camera_los(camera_path, model_sdf, window, settings):
    required = settings['camera_los_required']
    unavailable = dict(available=False, passed=False if required else None, required=required)
    if camera_path is None or not camera_path.is_file() or camera_path.stat().st_size == 0:
        return dict(unavailable, reason='未收到实际camera.quat_world遥测')
    rows, invalid = [], 0
    for line in camera_path.read_text().splitlines():
        try:
            item = json.loads(line)
            stamp = float(item['ts_mono'])
            quat = tuple(float(value) for value in item['camera']['quat_world'])
            if len(quat) != 4 or not math.isfinite(stamp) or not all(map(math.isfinite, quat)):
                raise ValueError
            rows.append((stamp, quat, item.get('mode')))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            invalid += 1
    start, end = window
    rows = [row for row in rows if start <= row[0] <= end]
    if len(rows) < 10:
        return dict(unavailable, reason='实际camera.quat_world未覆盖hover正式窗口',
                    total_valid_rows=len(rows), invalid_rows=invalid)
    modes = sorted({row[2] for row in rows})
    if modes != ['IDLE']:
        return dict(unavailable, available=True, passed=False, required=True,
                    reason=f'hover正式窗口吊舱模式不是IDLE: {modes}',
                    total_valid_rows=len(rows), invalid_rows=invalid, modes=modes)
    try:
        width, height, hfov = camera_intrinsics(model_sdf)
        stamps = np.array([row[0] for row in rows])
        if np.any(np.diff(stamps) <= 0):
            raise ValueError('camera.quat_world时间不是严格递增')
        los, right, down = [], [], []
        for _, quat, _ in rows:
            norm = math.sqrt(sum(value*value for value in quat))
            if abs(norm - 1) > .01:
                raise ValueError('camera.quat_world不是单位四元数')
            q = tuple(value / norm for value in quat)
            los.append(quaternion_rotate(q, (1., 0., 0.)))
            right.append(quaternion_rotate(q, (0., -1., 0.)))
            down.append(quaternion_rotate(q, (0., 0., -1.)))
        los, right, down = map(np.asarray, (los, right, down))
        baseline = np.column_stack([centered_trend(stamps, los[:, index],
                                                   settings['attitude_trend_window_s'])
                                    for index in range(3)])
        baseline /= np.linalg.norm(baseline, axis=1)[:, None]
        forward = np.sum(baseline * los, axis=1)
        horizontal = np.arctan2(np.sum(baseline * right, axis=1), forward)
        vertical = np.arctan2(np.sum(baseline * down, axis=1), forward)
        focal = width / (2 * math.tan(hfov / 2))
        x = focal * np.tan(horizontal)
        y = focal * np.tan(vertical)
        radial = np.hypot(x, y)
    except (ValueError, OSError, ET.ParseError) as error:
        return dict(unavailable, reason=str(error), total_valid_rows=len(rows), invalid_rows=invalid)
    p95, maximum = float(np.percentile(radial, 95)), float(np.max(radial))
    passed = (p95 <= settings['camera_los_p95_limit_px'] and
              maximum <= settings['camera_los_max_limit_px'])
    return dict(available=True, required=True, passed=passed,
                source='two_axis telemetry camera.quat_world', modes=modes,
                total_valid_rows=len(rows), invalid_rows=invalid,
                image=dict(width=width, height=height, horizontal_fov_deg=math.degrees(hfov)),
                p95_radial_px=p95, max_radial_px=maximum,
                horizontal_peak_to_peak_px=float(np.ptp(x)),
                vertical_peak_to_peak_px=float(np.ptp(y)),
                limits=dict(p95_radial_px=settings['camera_los_p95_limit_px'],
                            max_radial_px=settings['camera_los_max_limit_px']))


def analyze(ulog, report, snapshot, camera_path=None, model_sdf=None):
    settings = validate_settings(snapshot.get('stability_validation'))
    ulog_window = phase_window(report, 'samples', settings['hover_settle_s'])
    truth_window = None
    try:
        truth_window = phase_window(report, 'ground_truth_samples', settings['hover_settle_s'])
    except ValueError:
        pass
    rates = dataset_data(ulog, 'vehicle_angular_velocity')
    setpoints = dataset_data(ulog, 'vehicle_rates_setpoint')
    attitudes = dataset_data(ulog, 'vehicle_attitude')
    allocator = dataset_data(ulog, 'control_allocator_status')
    motors = dataset_data(ulog, 'actuator_motors')
    low_hz, high_hz = settings['spectral_band_hz']
    axis_results = {}
    rate_t = time_seconds(rates, 'vehicle_angular_velocity')
    rate_mask = (rate_t >= ulog_window[0]) & (rate_t <= ulog_window[1])
    rate_t = rate_t[rate_mask]
    if len(rate_t) < 4:
        raise ValueError('vehicle_angular_velocity在hover正式窗口样本不足')
    sp_t = time_seconds(setpoints, 'vehicle_rates_setpoint')
    att_t = time_seconds(attitudes, 'vehicle_attitude')
    q = [array(attitudes, f'q[{index}]', f'vehicle_attitude.q[{index}]') for index in range(4)]
    roll, pitch = quaternion_euler(*q)
    attitude_by_axis = {'roll': roll, 'pitch': pitch}
    for axis in AXES:
        actual_all = array(rates, RATE_FIELDS[axis], f'vehicle_angular_velocity.{RATE_FIELDS[axis]}')
        actual = actual_all[rate_mask]
        setpoint = interpolate(sp_t, array(setpoints, SETPOINT_FIELDS[axis],
                                          f'vehicle_rates_setpoint.{SETPOINT_FIELDS[axis]}'),
                               rate_t, 'vehicle_rates_setpoint')
        error = np.degrees(actual - setpoint)
        spectrum = welch_metrics(rate_t, error, low_hz, high_hz,
                                 settings['spectral_window_s'])
        att_mask = (att_t >= ulog_window[0]) & (att_t <= ulog_window[1])
        selected_t, selected = att_t[att_mask], np.degrees(attitude_by_axis[axis][att_mask])
        if len(selected) < 4:
            raise ValueError('vehicle_attitude在hover正式窗口样本不足')
        residual = selected - centered_trend(selected_t, selected,
                                             settings['attitude_trend_window_s'])
        metrics = dict(rate_error_rms_dps=float(np.sqrt(np.mean(error*error))),
                       rate_error_p95_dps=percentile_abs(error, 95),
                       rate_error_max_dps=float(np.max(np.abs(error))),
                       spectral_band=spectrum,
                       attitude_detrended_p95_deg=percentile_abs(residual, 95),
                       attitude_detrended_peak_to_peak_deg=float(np.ptp(residual)))
        checks = dict(
            rate_error_rms=metrics['rate_error_rms_dps'] <= settings['rate_error_rms_limit_dps'],
            rate_error_p95=metrics['rate_error_p95_dps'] <= settings['rate_error_p95_limit_dps'],
            rate_error_max=metrics['rate_error_max_dps'] <= settings['rate_error_max_limit_dps'],
            rate_hf_rms=spectrum['rms_dps'] <= settings['rate_hf_rms_limit_dps'],
            attitude_p95=metrics['attitude_detrended_p95_deg'] <= settings['attitude_p95_limit_deg'],
            attitude_peak_to_peak=(metrics['attitude_detrended_peak_to_peak_deg'] <=
                                   settings['attitude_peak_to_peak_limit_deg']))
        axis_results[axis] = dict(**metrics, checks=checks, passed=all(checks.values()))

    allocator_t = time_seconds(allocator, 'control_allocator_status')
    allocator_mask = (allocator_t >= ulog_window[0]) & (allocator_t <= ulog_window[1])
    allocator_t = allocator_t[allocator_mask]
    achieved = array(allocator, 'torque_setpoint_achieved',
                     'control_allocator_status.torque_setpoint_achieved')[allocator_mask].astype(bool)
    if len(achieved) < 4:
        raise ValueError('control_allocator_status在hover正式窗口样本不足')
    unachieved = ~achieved
    allocation = dict(unachieved_ratio=float(np.mean(unachieved)),
                      longest_unachieved_s=longest_true(allocator_t, unachieved), axes={})
    for axis in AXES:
        values = array(allocator, TORQUE_FIELDS[axis],
                       f'control_allocator_status.{TORQUE_FIELDS[axis]}')[allocator_mask]
        allocation['axes'][axis] = dict(unallocated_torque_p99=float(np.percentile(np.abs(values), 99)))
    allocation['checks'] = dict(
        unachieved_ratio=(allocation['unachieved_ratio'] <=
                          settings['allocator_unachieved_ratio_limit']),
        longest_unachieved=(allocation['longest_unachieved_s'] <=
                            settings['allocator_unachieved_continuous_limit_s']),
        unallocated_roll=(allocation['axes']['roll']['unallocated_torque_p99'] <=
                          settings['unallocated_torque_p99_limit']),
        unallocated_pitch=(allocation['axes']['pitch']['unallocated_torque_p99'] <=
                           settings['unallocated_torque_p99_limit']))
    allocation['passed'] = all(allocation['checks'].values())

    motor_t = time_seconds(motors, 'actuator_motors')
    motor_mask = (motor_t >= ulog_window[0]) & (motor_t <= ulog_window[1])
    motor_results = []
    for index in range(4):
        values = array(motors, f'control[{index}]', f'actuator_motors.control[{index}]')[motor_mask]
        if len(values) < 4:
            raise ValueError('actuator_motors在hover正式窗口样本不足')
        ratio = float(np.mean(values <= settings['motor_floor_threshold']))
        motor_results.append(dict(index=index, floor_ratio=ratio,
                                  passed=ratio <= settings['motor_floor_ratio_limit']))
    motor_summary = dict(floor_threshold=settings['motor_floor_threshold'], motors=motor_results,
                         passed=all(item['passed'] for item in motor_results))
    camera = (camera_los(camera_path, model_sdf, truth_window, settings) if truth_window else
              dict(available=False, passed=False if settings['camera_los_required'] else None,
                   required=settings['camera_los_required'],
                   reason='飞行报告没有Gazebo真值hover时间窗，无法同步camera.quat_world'))
    passed = (all(item['passed'] for item in axis_results.values()) and allocation['passed'] and
              motor_summary['passed'] and (camera['passed'] is not False))
    return dict(schema_version=1, passed=passed, settings=settings,
                hover=dict(ulog_time_s=list(ulog_window), gazebo_time_s=list(truth_window) if truth_window else None,
                           skipped_initial_s=settings['hover_settle_s']),
                axes=axis_results, allocator=allocation, motors=motor_summary, camera_los=camera)


def load_ulog(path):
    try:
        from pyulog import ULog
    except ImportError as error:
        raise RuntimeError('缺少pyulog，无法执行稳定性门禁') from error
    return ULog(str(path), message_name_filter_list=[
        'vehicle_angular_velocity', 'vehicle_rates_setpoint', 'vehicle_attitude',
        'control_allocator_status', 'actuator_motors'])


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ulog', type=Path, required=True)
    parser.add_argument('--flight-report', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--camera-jsonl', type=Path)
    parser.add_argument('--model-sdf', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--update-flight-report', action='store_true')
    args = parser.parse_args()
    required_inputs = (('ulog', args.ulog), ('flight_report', args.flight_report),
                       ('config', args.config))
    provenance = {name: dict(path=str(path.resolve())) for name, path in required_inputs}
    try:
        for name, path in required_inputs:
            provenance[name]['sha256'] = file_sha256(path)
        report = json.loads(args.flight_report.read_text())
        snapshot = json.loads(args.config.read_text())
        result = analyze(load_ulog(args.ulog), report, snapshot, args.camera_jsonl, args.model_sdf)
        result['inputs'] = provenance
        code = 0 if result['passed'] else 1
    except Exception as error:  # 保留结构化失败证据，命令行由非零退出码门禁。
        result = dict(schema_version=1, passed=False, error=str(error), inputs=provenance)
        code = 2
    args.output.parent.mkdir(parents=True, exist_ok=True)
    atomic_json(args.output, result)
    if args.update_flight_report:
        try:
            report = json.loads(args.flight_report.read_text())
            report['stability_validation'] = result
            atomic_json(args.flight_report, report)
        except Exception as error:
            print(f'无法写回飞行报告: {error}', file=sys.stderr)
            return 2
    if result.get('error'):
        print(result['error'], file=sys.stderr)
    else:
        print(f"稳定性验收：{'通过' if result['passed'] else '失败'}；{args.output}")
    return code


if __name__ == '__main__':
    raise SystemExit(main())
