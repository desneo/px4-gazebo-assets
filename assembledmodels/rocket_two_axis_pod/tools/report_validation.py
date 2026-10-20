#!/usr/bin/env python3
"""归档当前组合模型完整飞行验收的原始报告、摘要及速度/高度曲线。"""
from __future__ import annotations

import argparse
from datetime import datetime
import gzip
import hashlib
import io
import json
import math
import os
from pathlib import Path
from zoneinfo import ZoneInfo

import generate_dynamic_model as dynamic

ROOT = Path(__file__).resolve().parents[1]
PHASES = ('hover', 'cruise', 'sprint')
truth = dynamic.load_module('rocket_pod_archive_truth', dynamic.ROCKET / 'tests/gazebo_truth.py')


def finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def report_snapshot(report):
    snapshot = report.get('model_parameters')
    if not isinstance(snapshot, dict):
        raise ValueError('报告缺少完整运行快照')
    encoded = (json.dumps(snapshot, indent=2, ensure_ascii=False) + '\n').encode('utf-8')
    if report.get('config_sha256') != hashlib.sha256(encoded).hexdigest():
        raise ValueError('报告配置摘要与运行快照不一致')
    runtime = snapshot.get('runtime')
    if not isinstance(runtime, dict) or not isinstance(runtime.get('source_sha256'), dict) or not runtime['source_sha256']:
        raise ValueError('报告缺少原生来源指纹，不能用于当前组合模型验收')
    return snapshot


def validate_samples(rows, *, estimated=False):
    if not isinstance(rows, list) or not rows:
        raise ValueError('报告缺少飞行原始采样')
    previous = None
    for row in rows:
        values = [row.get('sim_time_s')]
        for key in ('position_ned_m', 'velocity_ned_m_s'):
            vector = row.get(key)
            if not isinstance(vector, list) or len(vector) != 3:
                raise ValueError('飞行采样缺少三维位置或速度')
            values += vector
        if estimated:
            values.append(row.get('command_z_m'))
        if not all(finite(value) for value in values):
            raise ValueError('飞行采样含无效或非有限数值')
        if previous is not None and row['sim_time_s'] <= previous:
            raise ValueError('飞行采样时间必须严格递增')
        previous = row['sim_time_s']


def continuous_duration(rows, phase, settings, target_z):
    start = previous = None
    longest = 0.
    for row in rows:
        if row.get('phase') != phase:
            continue
        stamp = row['sim_time_s']
        valid = (math.hypot(*row['velocity_ned_m_s'][:2]) >= settings[phase + '_speed_km_h'] / 3.6
                 and abs(row['position_ned_m'][2] - target_z) <= settings['altitude_tolerance_m'])
        if not valid or (previous is not None and stamp - previous > settings['telemetry_timeout_s']):
            start = None
        if valid and start is None:
            start = stamp
        if start is not None:
            longest = max(longest, stamp - start)
        previous = stamp
    return longest


def validate_report(report, current_sources, defaults):
    snapshot = report_snapshot(report)
    if snapshot['runtime']['source_sha256'] != current_sources or not current_sources:
        raise ValueError('报告来源与当前组合生成代码、原生资产或验收入口不同')
    target = {'world_name': dynamic.NAME + '_test', 'model_name': dynamic.NAME}
    if (report.get('measurement_source') != 'gazebo_pose' or report.get('measurement_target') != target
            or snapshot.get('model', {}).get('name') != target['model_name']
            or snapshot.get('simulation', {}).get('world_name') != target['world_name']):
        raise ValueError('报告必须使用当前组合测试世界与组合实体的 Gazebo 真值')
    truth.validate_measurement_reference(snapshot, report.get('measurement_reference'))
    if report.get('passed') is not True or report.get('landing_confirmed') is not True:
        raise ValueError('完整飞行必须通过且确认降落停机，不能归档为通过')
    if report.get('error') or report.get('recovery_error'):
        raise ValueError('报告包含飞行或恢复错误')
    settings = report.get('settings')
    if not isinstance(settings, dict) or snapshot.get('flight_test') != defaults:
        raise ValueError('运行快照的完整验收门槛与当前默认配置不一致')
    # PX4 instance 只改变连接端口，不改变完整飞行的任何验收门槛。
    if {key: value for key, value in settings.items() if key != 'endpoint'} != {
            key: value for key, value in defaults.items() if key != 'endpoint'}:
        raise ValueError('报告设置必须采用当前默认完整验收门槛')
    if not isinstance(settings.get('endpoint'), str) or not settings['endpoint']:
        raise ValueError('报告缺少实际 MAVLink 连接地址')
    started, finished = report.get('started_at_unix_s'), report.get('finished_at_unix_s')
    if not finite(started) or not finite(finished) or finished <= started:
        raise ValueError('报告必须包含有效的起止时间')
    results = report.get('results', {})
    if not isinstance(results, dict) or any(not isinstance(results.get(phase), dict) or
                                          results[phase].get('passed') is not True for phase in PHASES):
        raise ValueError('悬停、巡航和冲刺结果必须完整且全部通过')
    hover = results['hover']
    for key, limit in (('max_drift_m', settings['hover_drift_limit_m']),
                       ('max_altitude_error_m', settings['altitude_tolerance_m'])):
        if not finite(hover.get(key)) or not 0 <= hover[key] <= limit:
            raise ValueError('悬停实测漂移或高度误差超限')
    validate_samples(report.get('samples'), estimated=True)
    validate_samples(report.get('ground_truth_samples'))
    phases = {row.get('phase') for row in report['ground_truth_samples']}
    if not set((*PHASES, 'landing')) <= phases:
        raise ValueError('原始 Gazebo 真值采样缺少完整飞行阶段')
    target_z = min(row['command_z_m'] for row in report['samples'])
    windows = {}
    for phase in ('cruise', 'sprint'):
        result = results[phase]
        duration, speed = settings[phase + '_seconds'], settings[phase + '_speed_km_h']
        if result.get('required_speed_km_h') != speed or result.get('required_continuous_s') != duration:
            raise ValueError('速度阶段要求与默认验收门槛不一致')
        if not finite(result.get('longest_continuous_s')) or result['longest_continuous_s'] < duration:
            raise ValueError('速度阶段没有完成规定的连续保持时间')
        windows[phase] = continuous_duration(report['ground_truth_samples'], phase, settings, target_z)
        if windows[phase] < duration:
            raise ValueError('原始 Gazebo 真值没有满足连续速度和高度要求')
    return dict(current_sources), windows


def curve_png(report):
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/rocket-pod-matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    estimated, truth = report['samples'], report['ground_truth_samples']
    origin = estimated[0]['sim_time_s']
    target_z = min(row['command_z_m'] for row in estimated)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for rows, label, color, style in ((estimated, 'PX4 estimate', '#777777', '--'),
                                      (truth, 'Gazebo ground truth', '#16818d', '-')):
        times = [row['sim_time_s'] - origin for row in rows]
        axes[0].plot(times, [math.hypot(*row['velocity_ned_m_s'][:2]) * 3.6 for row in rows],
                     style, color=color, label=label, linewidth=1.2)
        axes[1].plot(times, [target_z - row['position_ned_m'][2] for row in rows],
                     style, color=color, linewidth=1.2)
    for phase, color in (('cruise', '#b17e25'), ('sprint', '#b44b40')):
        axes[0].axhline(report['settings'][phase + '_speed_km_h'], color=color,
                        linewidth=.8, label=phase.capitalize() + ' threshold')
    tolerance = report['settings']['altitude_tolerance_m']
    axes[1].axhspan(-tolerance, tolerance, color='#16818d', alpha=.1)
    axes[0].set(ylabel='Horizontal speed (km/h)', title='Rocket with two-axis pod — full flight validation')
    axes[1].set(ylabel='Height error from target (m)', xlabel='Simulation time from connection (s)')
    axes[1].set_ylim(-max(4, 2*tolerance), max(4, 2*tolerance))
    for ax in axes:
        ax.grid(alpha=.2)
    axes[0].legend(loc='upper left', bbox_to_anchor=(.01, .87), ncol=2, fontsize=9)
    for phase in (*PHASES, 'landing'):
        rows = [row for row in estimated if row.get('phase') == phase]
        if rows:
            left, right = rows[0]['sim_time_s'] - origin, rows[-1]['sim_time_s'] - origin
            for ax in axes:
                ax.axvspan(left, right, color='#777777', alpha=.05)
            axes[0].text((left + right)/2, axes[0].get_ylim()[1]*.96, phase,
                         ha='center', va='top', fontsize=9)
    fig.tight_layout()
    data = io.BytesIO()
    fig.savefig(data, format='png', dpi=160)
    plt.close(fig)
    return data.getvalue()


def archive_report(raw, *, output_root=ROOT, current_sources=None, defaults=None, renderer=curve_png):
    report = json.loads(raw)
    # 先验证快照自身，避免将缺少原生来源的旧报告冒充本次配置。
    report_snapshot(report)
    if current_sources is None:
        current_sources = dynamic.source_fingerprints()
    if defaults is None:
        defaults = json.loads((dynamic.ROCKET / 'parameters.json').read_text())['flight_test']
    sources, windows = validate_report(report, current_sources, defaults)
    summary = {key: report[key] for key in ('passed', 'landing_confirmed', 'measurement_source',
               'measurement_target', 'measurement_reference', 'config_sha256', 'results', 'settings', 'model_parameters')}
    summary.update(validated_at=datetime.fromtimestamp(report['finished_at_unix_s'], ZoneInfo('Asia/Shanghai')).isoformat(),
                   elapsed_wall_s=report['finished_at_unix_s'] - report['started_at_unix_s'],
                   raw_report_sha256=hashlib.sha256(raw).hexdigest(),
                   raw_report='../tests/results/flight_result.json.gz', asset_sha256=sources,
                   sample_counts={key: len(report[key]) for key in ('samples', 'ground_truth_samples')},
                   verified_continuous_s=windows)
    curve = renderer(report)
    output_root = Path(output_root)
    docs, results = output_root / 'docs', output_root / 'tests/results'
    docs.mkdir(parents=True, exist_ok=True)
    results.mkdir(parents=True, exist_ok=True)
    (results / 'flight_result.json.gz').write_bytes(gzip.compress(raw, mtime=0))
    (docs / 'validation.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    (docs / 'flight_validation.png').write_bytes(curve)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    args = parser.parse_args()
    raw = args.input.read_bytes()
    if args.input.suffix == '.gz':
        raw = gzip.decompress(raw)
    archive_report(raw)
    print(ROOT / 'docs/validation.json')
    print(ROOT / 'docs/flight_validation.png')
    print(ROOT / 'tests/results/flight_result.json.gz')


if __name__ == '__main__':
    main()
