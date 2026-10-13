#!/usr/bin/env python3
"""将飞行JSON整理为压缩原始证据、验证摘要和科学曲线。"""
import argparse
from datetime import datetime
import gzip
import hashlib
import json
import os
from pathlib import Path
from zoneinfo import ZoneInfo
os.environ.setdefault('MPLCONFIGDIR', '/tmp/rocket-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    args = parser.parse_args()
    raw = gzip.decompress(args.input.read_bytes()) if args.input.suffix == '.gz' else args.input.read_bytes()
    report = json.loads(raw)
    current_hash = hashlib.sha256((ROOT / 'parameters.json').read_bytes()).hexdigest()
    if report['config_sha256'] != current_hash:
        raise ValueError('报告配置与当前模型配置不同，请使用对应配置生成资产后再归档')
    docs = ROOT / 'docs'; docs.mkdir(exist_ok=True)
    results = ROOT / 'tests/results'; results.mkdir(exist_ok=True)
    archive = results / 'flight_result.json.gz'
    archive.write_bytes(gzip.compress(raw, mtime=0))
    summary = {key: report[key] for key in ('passed', 'landing_confirmed', 'measurement_source', 'config_sha256', 'results', 'settings', 'model_parameters')}
    summary['validated_at'] = datetime.fromtimestamp(report['finished_at_unix_s'], ZoneInfo('Asia/Shanghai')).isoformat()
    summary['elapsed_wall_s'] = report['finished_at_unix_s'] - report['started_at_unix_s']
    summary['raw_report_sha256'] = hashlib.sha256(raw).hexdigest()
    summary['raw_report'] = '../tests/results/flight_result.json.gz'
    summary['sample_counts'] = {key: len(report.get(key, [])) for key in ('samples', 'ground_truth_samples')}
    assets = ['parameters.json', 'model.sdf', 'tests/world.sdf', 'tools/generate_model.py', 'tools/generate_meshes.py', 'tests/flight_check.py', 'tests/gazebo_truth.py', 'tests/run.sh', 'tests/run_check.sh']
    assets += [str(p.relative_to(ROOT)) for folder in ('meshes', 'px4') for p in sorted((ROOT / folder).iterdir()) if p.is_file()]
    summary['asset_sha256'] = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in assets}
    (docs / 'validation.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    truth = report.get('ground_truth_samples', [])
    estimated = report['samples']
    t0 = estimated[0]['sim_time_s']
    target_z = min(row['command_z_m'] for row in estimated)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for rows, label, color, style in ((estimated, 'PX4 estimate', '#777777', '--'), (truth, 'Gazebo ground truth', '#16818d', '-')):
        if not rows:
            continue
        time_values = [row['sim_time_s'] - t0 for row in rows]
        speed = [np.hypot(*row['velocity_ned_m_s'][:2])*3.6 for row in rows]
        height_error = [target_z - row['position_ned_m'][2] for row in rows]
        axes[0].plot(time_values, speed, style, color=color, label=label, linewidth=1.2)
        axes[1].plot(time_values, height_error, style, color=color, linewidth=1.2)
    axes[0].axhline(report['settings']['cruise_speed_km_h'], color='#b17e25', linewidth=.8, label='Cruise threshold')
    axes[0].axhline(report['settings']['sprint_speed_km_h'], color='#b44b40', linewidth=.8, label='Sprint threshold')
    tol = report['settings']['altitude_tolerance_m']
    axes[1].axhspan(-tol, tol, color='#16818d', alpha=.1)
    axes[0].set(ylabel='Horizontal speed (km/h)', title='Rocket racing quad — flight validation')
    axes[1].set(ylabel='Height error from target (m)', xlabel='Simulation time from connection (s)')
    # 只展开飞行中的高度误差；起飞和降落超出范围属预期，不用于巡航/冲刺高度判定。
    axes[1].set_ylim(-4, 4)
    for ax in axes:
        ax.grid(alpha=.2)
    axes[0].legend(loc='upper left', bbox_to_anchor=(.01, .87), ncol=2, fontsize=9)
    for phase in ('hover', 'cruise', 'sprint', 'landing'):
        rows = [row for row in estimated if row['phase'] == phase]
        if rows:
            a, b = rows[0]['sim_time_s']-t0, rows[-1]['sim_time_s']-t0
            for ax in axes:
                ax.axvspan(a, b, color='#777777', alpha=.05)
            axes[0].text((a+b)/2, axes[0].get_ylim()[1]*.96, phase, ha='center', va='top', fontsize=9)
    fig.tight_layout()
    fig.savefig(docs / 'flight_validation.png', dpi=160)
    print(docs / 'validation.json')
    print(archive, archive.stat().st_size, 'bytes')


if __name__ == '__main__':
    main()
