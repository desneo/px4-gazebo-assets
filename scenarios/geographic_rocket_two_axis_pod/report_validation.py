#!/usr/bin/env python3
"""归档实际飞行与图像跟踪记录，生成当前配置的独立验收摘要和曲线。"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil

os.environ.setdefault('MPLCONFIGDIR', '/tmp/rocket-pod-validation-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generate(run, output, binding_dir=None):
    flight = json.loads((run / 'flight_result.json').read_text())
    tracking = json.loads((run / 'tracking/summary.json').read_text())
    if not flight.get('passed') or not flight.get('landing_confirmed') or tracking.get('result') != 'passed':
        raise ValueError('只有实际飞行、降落和独立图像跟踪检查均通过，才能生成通过证据')
    binding = None
    if binding_dir is not None:
        binding_dir = binding_dir.resolve()
        binding = json.loads((binding_dir / 'binding_summary.json').read_text())
        if binding.get('result') != 'passed' or not binding.get('source_fingerprints_current'):
            raise ValueError('网关身份与实际RTSP视频检查未通过')
        for name, expected in binding['source_sha256'].items():
            if sha(ROOT / name) != expected:
                raise ValueError(f'RTSP验证后源文件变化：{name}')
    rows = [json.loads(line) for line in (run / 'flight_result.jsonl').read_text().splitlines()]
    legs = [r for r in rows if r['kind'] == 'leg_result']
    if len(legs) != flight['completed_legs'] or not all(leg['passed'] for leg in legs):
        raise ValueError('逐航段原始证据与飞行摘要不一致')
    target_rows = [json.loads(line) for line in (run / 'tracking/tracking.jsonl').read_text().splitlines()]
    target_rows = [r for r in target_rows if r['kind'] == 'sample']
    output.mkdir(parents=True, exist_ok=True)
    evidence = HERE / 'tests/results'
    evidence.mkdir(parents=True, exist_ok=True)
    archives = {}
    for relative, name in [('flight_result.json', 'combined_flight.json.gz'),
                           ('flight_result.jsonl', 'combined_samples.jsonl.gz'),
                           ('tracking/tracking.jsonl', 'tracking_samples.jsonl.gz')]:
        data = (run / relative).read_bytes()
        path = evidence / name
        path.write_bytes(gzip.compress(data, mtime=0))
        archives[str(path.relative_to(HERE))] = sha(path)
    if binding is not None:
        for source_name, archive_name in [('binding_samples.jsonl', 'rtsp_binding_samples.jsonl.gz'),
                                           ('binding_summary.json', 'rtsp_binding_summary.json.gz')]:
            path = evidence / archive_name
            path.write_bytes(gzip.compress((binding_dir / source_name).read_bytes(), mtime=0))
            archives[str(path.relative_to(HERE))] = sha(path)
    manifest_path = run / 'models/rocket_two_axis_pod/manifest.json'
    summary = dict(passed=True, landing_confirmed=True,
                   scope='当前450mm组合体：PX4实际往返飞行与真实视频空中目标跟踪',
                   settings=flight['settings'], completed_legs=flight['completed_legs'], legs=legs,
                   tracking=tracking, assembly_manifest=json.loads(manifest_path.read_text()),
                   controller_library_sha256=sha(ROOT / 'models/gimbals/two_axis_pod/controller/build/lib/libgz-two-axis-pod-controller.so'),
                   evidence_sha256=archives,
                   limits='仅覆盖当前无风地理世界及可达的静止空中靶标；人工交互验收独立，未验证任意目标或所有载机姿态。')
    if binding is not None:
        summary['rtsp_binding'] = binding
    (output / 'validation.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    shutil.copyfile(binding_dir / 'rtsp_snapshot.png' if binding_dir is not None
                    else run / 'rtsp_snapshot.png', output / 'pod_video.png')
    snapshots = sorted((run / 'tracking').glob('frame_*.png'))
    if snapshots:
        shutil.copyfile(snapshots[len(snapshots) // 2], output / 'tracking_snapshot.png')

    plt.rcParams.update({'font.family': ['Noto Sans CJK SC', 'DejaVu Sans'], 'axes.unicode_minus': False})
    moving = [r for r in rows if r['kind'] == 'gazebo' and r['phase'].startswith('leg_')]
    commands = [r for r in rows if r['kind'] == 'px4' and r['phase'].startswith('leg_')]
    if not moving or not commands or not target_rows:
        raise ValueError('缺少实际运动或图像跟踪样本')
    t0 = moving[0]['sim_time_s']
    times = [r['sim_time_s'] - t0 for r in moving]
    altitude = flight['settings']['altitude_m']
    home_z = commands[0]['command_z_m'] + altitude
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), layout='constrained')
    axes[0].plot(times, [r['position_ned_m'][0] for r in moving], color='#237cad')
    for target in (0, flight['settings']['leg_length_m']):
        axes[0].axhline(target, color='#777777', linestyle='--', linewidth=.7)
    axes[0].set_ylabel('北向位移 / m')
    axes[1].plot(times, [home_z - r['position_ned_m'][2] for r in moving], color='#288f61')
    axes[1].axhline(altitude, color='#777777', linestyle='--', linewidth=.7)
    axes[1].set_ylabel('起点相对高度 / m')
    axes[2].plot(times, [r['velocity_ned_m_s'][0] for r in moving], color='#d48024')
    axes[2].set_ylabel('北向速度 / m/s')
    tracks = [r['stamp'] - t0 for r in target_rows]
    axes[3].plot(tracks, [r['target_ray_error_deg'] for r in target_rows], label='跟踪框与独立目标方向误差', color='#784fbd')
    axes[3].plot(tracks, [r['optical_axis_error_deg'] for r in target_rows], label='光轴与目标方向误差', alpha=.7, color='#237cad')
    axes[3].set_ylabel('角误差 / °')
    axes[3].set_xlabel('巡航开始后的仿真时间 / s')
    axes[3].legend(loc='upper right')
    for ax in axes:
        ax.grid(alpha=.2)
    fig.suptitle(f"450 mm 组合体 | {altitude:g} m / {flight['settings']['speed_m_s']:g} m/s / {flight['settings']['leg_length_m']:g} m\n"
                 f"{flight['completed_legs']} 个真实航段及降落通过；图像跟踪 {tracking['synchronized_sim_seconds']:.1f} 仿真秒，独立确认 {tracking['independently_confirmed_ratio']:.0%}")
    fig.savefig(output / 'flight_tracking.png', dpi=160)
    plt.close(fig)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, default=HERE / 'docs')
    parser.add_argument('--binding-dir', type=Path, help='要求网关ID、RTSP路径及帧表关联均通过并归档')
    args = parser.parse_args()
    generate(args.run_dir.resolve(), args.output_dir.resolve(), args.binding_dir)
    print(f'组合体验收摘要与实际曲线：{args.output_dir.resolve()}')
