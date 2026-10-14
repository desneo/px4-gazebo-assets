#!/usr/bin/env python3
"""从已保存的实际试飞证据生成速度对比；没有有效窗口时不推定最高速度。"""
import argparse
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_trial(directory):
    errors = []
    values = []
    for name in ('trial.json', 'asset_manifest.json'):
        try:
            value = json.loads((directory / name).read_text())
            if not isinstance(value, dict):
                raise ValueError('证据必须为 JSON 对象')
        except (OSError, ValueError) as error:
            errors.append(f'{name}: {error}')
            value = {}
        values.append(value)
    trial, manifest = values
    if not (directory / 'trial.samples.jsonl').is_file():
        errors.append('trial.samples.jsonl: 缺少原始采样')
    trial['evidence_errors'] = errors
    current = {name: (ROOT / name).is_file() and digest(ROOT / name) == expected
               for name, expected in manifest.get('source_sha256', {}).items()}
    return trial, manifest, current


def compare(baseline, assembled, tolerance=1., manifests=None, launcher_results=None):
    manifests = manifests or ({}, {})
    roles = (('standalone', 'rocket_racing_quad'), ('assembled', 'rocket_two_axis_pod'))
    correct_roles = all(t.get('model_name') == model and m.get('model_name') == model
                        and m.get('variant') == variant
                        for t, m, (variant, model) in zip((baseline, assembled), manifests, roles))
    same_profile = bool(baseline.get('settings')) and baseline.get('settings') == assembled.get('settings')
    same_cap = (baseline.get('control_speed_cap_m_s') is not None and
                baseline.get('control_speed_cap_m_s') == assembled.get('control_speed_cap_m_s'))
    environment = [m.get('environment_sha256') for m in manifests]
    same_environment = bool(environment[0]) and environment[0] == environment[1]
    scene_prefix = 'scenarios/rocket_speed_comparison/'
    scene_sources = [{name: value for name, value in m.get('source_sha256', {}).items()
                      if name.startswith(scene_prefix)} for m in manifests]
    same_scene_sources = bool(scene_sources[0]) and scene_sources[0] == scene_sources[1]
    evidence_complete = not any(t.get('evidence_errors') for t in (baseline, assembled))
    launcher_succeeded = all(result.get('returncode') == 0
                             for result in (launcher_results or {}).values())
    usable = all(t.get('flight_succeeded') and t.get('landing_confirmed') and
                 t.get('valid_stable_window') for t in (baseline, assembled))
    usable = bool(usable and same_profile and same_cap and correct_roles and same_environment
                  and same_scene_sources and evidence_complete and launcher_succeeded)
    speeds = [t['stable_window']['mean_m_s'] if t.get('stable_window') else None
              for t in (baseline, assembled)]
    difference = speeds[1] - speeds[0] if usable else None
    reached = difference >= -tolerance if usable else None
    both_cap = usable and all(t.get('limit_reason') == 'control_limited'
                             for t in (baseline, assembled))
    if not usable:
        conclusion = 'inconclusive'
    elif both_cap:
        conclusion = 'both_reach_current_control_cap' if reached else 'assembled_below_baseline'
    elif not all(t.get('target_reached') for t in (baseline, assembled)):
        # 未达设定值时只保存观测稳态，不能将一次短试飞直接称为动力极限。
        conclusion = 'observed_stable_speed_only'
    else:
        conclusion = 'assembled_matches_baseline' if reached else 'assembled_below_baseline'
    return dict(conclusion=conclusion, comparable=usable, same_profile=same_profile,
                correct_model_roles=correct_roles, same_environment=same_environment,
                same_scene_sources=same_scene_sources, evidence_complete=evidence_complete,
                launcher_succeeded=launcher_succeeded,
                same_control_cap=same_cap, standalone_stable_m_s=speeds[0],
                assembled_stable_m_s=speeds[1], assembled_minus_standalone_m_s=difference,
                comparison_tolerance_m_s=tolerance, assembled_reaches_baseline=reached,
                both_control_limited=both_cap,
                control_speed_cap_m_s=baseline.get('control_speed_cap_m_s'),
                aerodynamics_scope='current_simulation_same_drag_parameters_no_separate_pod_drag')


def plot_trials(directories, trials, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, constrained_layout=True)
    for directory, trial, color, label in zip(directories, trials,
            ('#2563eb', '#dc2626'), ('Standalone', 'Assembled')):
        times, speeds, heights = [], [], []
        for line in (directory / 'trial.samples.jsonl').open():
            row = json.loads(line)
            if row['kind'] != 'ground_truth_samples' or row['phase'] not in (
                    'speed_ramp', 'speed_plateau', 'deceleration'):
                continue
            times.append(row['sim_time_s'])
            v = row['velocity_ned_m_s']
            speeds.append((v[0]**2+v[1]**2)**.5)
            # Ground-truth NED uses the PX4 home-z offset. Relative height is
            # referenced to calibrated takeoff, recorded explicitly in report.
            heights.append(trial.get('home_z_m', 0.)-row['position_ned_m'][2])
        if not times:
            continue
        origin = times[0]
        axes[0].plot([t-origin for t in times], speeds, color=color, label=label, linewidth=.9)
        axes[1].plot([t-origin for t in times], heights, color=color, linewidth=.9)
        window = trial.get('stable_window')
        if window:
            axes[0].axvspan(window['start_sim_time_s']-origin,
                           window['end_sim_time_s']-origin, color=color, alpha=.12)
    axes[0].axhline(trials[0]['control_speed_cap_m_s'], color='#64748b', linestyle='--', label='Current cap')
    axes[0].set_ylabel('Ground-truth speed (m/s)')
    axes[1].set_ylabel('Height above takeoff (m)')
    axes[1].set_xlabel('Simulation seconds from acceleration start')
    axes[0].legend()
    for axis in axes:
        axis.grid(alpha=.25)
    figure.savefig(output, dpi=160)
    plt.close(figure)


def publish(directories, trials, manifests, report):
    docs, evidence = HERE / 'docs', HERE / 'tests/results'
    docs.mkdir(exist_ok=True)
    evidence.mkdir(parents=True, exist_ok=True)
    files = {}
    for variant, directory, trial, manifest in zip(('standalone', 'assembled'), directories, trials, manifests):
        for name in ('trial.json', 'trial.samples.jsonl', 'asset_manifest.json',
                     'flight.log', 'px4.log', 'gazebo.log', 'config.json'):
            if not (directory / name).is_file():
                continue
            destination = evidence / f'{variant}_{name}.gz'
            with (directory / name).open('rb') as source, destination.open('wb') as raw:
                with gzip.GzipFile(filename='', mode='wb', fileobj=raw, mtime=0) as compressed:
                    shutil.copyfileobj(source, compressed)
            files[str(destination.relative_to(HERE))] = digest(destination)
    if all((d / 'trial.samples.jsonl').is_file() for d in directories) and all(
            'control_speed_cap_m_s' in trial for trial in trials):
        plot_trials(directories, trials, docs / 'speed_comparison.png')
    else:
        # 缺少采样的失败报告不能沿用上一组成功曲线。
        (docs / 'speed_comparison.png').unlink(missing_ok=True)
    report['evidence_sha256'] = files
    (docs / 'validation.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    return docs / 'validation.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--standalone', type=Path, required=True)
    parser.add_argument('--assembled', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--tolerance', type=float, default=1.)
    parser.add_argument('--publish', action='store_true')
    parser.add_argument('--launcher-results', type=Path)
    args = parser.parse_args()
    if not 0 < args.tolerance <= 2:
        parser.error('比较容差必须在 (0,2] m/s 内')
    directories = (args.standalone.resolve(), args.assembled.resolve())
    data = [load_trial(d) for d in directories]
    trials, manifests, current = zip(*data)
    launcher_results = {}
    if args.launcher_results:
        launcher_results = json.loads(args.launcher_results.read_text())
    report = compare(*trials, tolerance=args.tolerance, manifests=manifests,
                     launcher_results=launcher_results)
    report.update(generated_at_utc=datetime.now(timezone.utc).isoformat(),
                  source_fingerprints_current=all(bool(c) and all(c.values()) for c in current),
                  launcher_results=launcher_results,
                  source_fingerprint_checks=dict(zip(('standalone', 'assembled'), current)),
                  trials=dict(zip(('standalone', 'assembled'), trials)),
                  manifests=dict(zip(('standalone', 'assembled'), manifests)))
    if args.publish:
        if not report['source_fingerprints_current']:
            parser.error('当前源资产与试飞指纹不一致，不能发布为当前配置验证')
        print(publish(directories, trials, manifests, report))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in (
        'trials', 'manifests', 'source_fingerprint_checks', 'evidence_sha256')}, ensure_ascii=False, indent=2))
    return 0 if report['comparable'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
