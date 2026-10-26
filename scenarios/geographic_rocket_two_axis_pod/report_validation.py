#!/usr/bin/env python3
"""归档实际飞行与图像跟踪记录，生成当前配置的独立验收摘要和曲线。"""
import argparse
import gzip
import hashlib
import importlib.util
import io
import math
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

os.environ.setdefault('MPLCONFIGDIR', '/tmp/rocket-pod-validation-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()



def read_json(path):
    return json.loads(Path(path).read_text())


def read_rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def close(a, b):
    return finite(a) and finite(b) and math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)


def validate_source_hashes(values, label):
    require(isinstance(values, dict) and values, f'{label}缺少源文件指纹')
    for name, expected in values.items():
        path = (ROOT / name).resolve()
        require(not Path(name).is_absolute() and path.is_relative_to(ROOT.resolve()), f'{label}源文件路径越界')
        require(path.is_file() and sha(path) == expected, f'{label}源文件变化：{name}')


def assembly_tools():
    """复用装配的正式继承检查，不另设模型、惯量或PX4阈值。"""
    directory = ROOT / 'assembledmodels/rocket_two_axis_pod/tools'
    spec = importlib.util.spec_from_file_location('geo_archive_dynamic_check', directory / 'check_dynamic_model.py')
    module = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(directory))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(directory))
    return module


def validate_flight(run, flight, rows):
    require(flight.get('passed') is True and flight.get('landing_confirmed') is True
            and not flight.get('error') and not flight.get('recovery_error'), '实际飞行或降落未通过')
    require(flight.get('measurement_source') == 'gazebo_pose', '飞行必须使用Gazebo独立真值')
    start, finish = flight.get('started_at_unix_s'), flight.get('finished_at_unix_s')
    require(finite(start) and finite(finish) and 0 < start < finish, '飞行起止时间无效')
    directory = run / 'models/rocket_two_axis_pod'
    config_path = directory / 'parameters.json'
    snapshot = read_json(config_path)
    require(flight.get('model_config_sha256') == sha(config_path)
            and flight.get('model_parameters') == snapshot, '飞行模型快照或SHA-256与运行资产不一致')
    manifest = read_json(directory / 'manifest.json')
    check = assembly_tools()
    try:
        check.check_output(directory, manifest)
    except (AssertionError, KeyError) as error:
        raise ValueError('动态模型、PX4配置或当前源指纹与运行快照不一致') from error
    settings = flight['settings']
    configured = read_json(HERE / 'config.json')
    configured.update(read_json(run / 'config.json'))
    require(settings == configured, '飞行设置与运行配置不一致')
    kinds = {'px4': 'samples_count', 'gazebo': 'ground_truth_samples_count', 'px4_status': 'status_text_count'}
    for kind, count_key in kinds.items():
        require(sum(row.get('kind') == kind for row in rows) == flight.get(count_key),
                f'原始{kind}采样数量与飞行摘要不一致')
    legs = [row for row in rows if row.get('kind') == 'leg_result']
    require(len(legs) == flight.get('completed_legs') and len(legs) >= 2,
            '逐航段原始证据与飞行摘要不一致或未完成往返')
    cycles = flight.get('cycles_requested')
    require(isinstance(cycles, int) and not isinstance(cycles, bool) and cycles > 0 and len(legs) == cycles*2,
            '完整往返次数未完成')
    require(flight.get('last_leg') == {key: value for key, value in legs[-1].items() if key != 'kind'},
            '末航段原始记录与飞行摘要不一致')
    moving = [r for r in rows if r.get('kind') == 'gazebo' and r.get('phase', '').startswith('leg_')]
    commands = [r for r in rows if r.get('kind') == 'px4' and r.get('phase', '').startswith('leg_')]
    require(moving and commands, '缺少实际运动原始样本')
    for kind in ('px4', 'gazebo'):
        selected = [r for r in rows if r.get('kind') == kind]
        stamps = [r.get('sim_time_s') for r in selected]
        require(all(finite(stamp) for stamp in stamps) and all(a < b for a, b in zip(stamps, stamps[1:])),
                f'{kind}仿真采样时间必须有限且严格递增')
    command_z = commands[0]['command_z_m']
    scored_truth, scored_commands = [], []
    for index, leg in enumerate(legs, 1):
        phase = f'leg_{index}_to_{"B" if index % 2 else "A"}'
        target = settings['leg_length_m'] if index % 2 else 0.
        require(leg.get('phase') == phase and close(leg.get('target_north_from_start_m'), target),
                '航段顺序或目标位置不一致')
        # 末航段结束后，切换LAND的回执等待仍可能沿用旧phase标签；
        # 航段计分以对应leg_result写入位置为边界，不把后续模式切换误计入。
        end_index = next(i for i, row in enumerate(rows) if row is leg)
        truth = [r for r in rows[:end_index] if r.get('kind') == 'gazebo' and r.get('phase') == phase]
        scored_truth.extend(truth)
        scored_commands.extend(r for r in rows[:end_index] if r.get('kind') == 'px4' and r.get('phase') == phase)
        require(truth and len(truth) == leg.get('samples'), '航段Gazebo原始采样数量不一致')
        values = [v for r in truth for key in ('position_ned_m', 'velocity_ned_m_s') for v in r[key]]
        require(all(finite(v) for v in values), '航段真值包含非有限位置或速度')
        altitude_error = max(abs(r['position_ned_m'][2]-command_z) for r in truth)
        cross_track = max(abs(r['position_ned_m'][1]) for r in truth)
        margin = settings['speed_m_s']**2/(2*settings['acceleration_m_s2'])+20
        cruise = [r for r in truth if margin < r['position_ned_m'][0] < settings['leg_length_m']-margin]
        require(cruise, '航段缺少实际巡航样本')
        speeds = [math.hypot(*r['velocity_ned_m_s'][:2]) for r in cruise]
        within = sum(abs(speed-settings['speed_m_s']) <= settings['speed_tolerance_m_s'] for speed in speeds)
        fraction = within/len(cruise)
        endpoint = abs(leg['truth_endpoint_ned_m'][0]-target)
        require(leg.get('passed') is True and altitude_error <= settings['altitude_tolerance_m']
                and cross_track <= settings['cross_track_tolerance_m']
                and endpoint <= settings['truth_endpoint_tolerance_m'] and fraction >= .95,
                '航段原始真值未满足实际飞行验收门槛')
        for key, value in [('max_altitude_error_m', altitude_error), ('max_cross_track_error_m', cross_track),
                           ('truth_endpoint_error_m', endpoint), ('cruise_mean_speed_m_s', sum(speeds)/len(speeds)),
                           ('cruise_speed_sum_m_s', sum(speeds)), ('cruise_speed_within_tolerance_fraction', fraction),
                           ('cruise_samples', len(cruise)), ('cruise_samples_within_tolerance', within)]:
            require(close(leg.get(key), value), f'航段摘要与原始采样不一致：{key}')
    execution = read_json(run / 'execution_fingerprints.json')
    validate_source_hashes(execution.get('source_sha256'), '启动记录')
    required_sources = {
        'models/gimbals/two_axis_pod/controller/build/lib/libgz-two-axis-pod-controller.so',
        'models/gimbals/two_axis_pod/model.sdf',
        'scenarios/geographic_rocket_two_axis_pod/flight_patrol.py',
    }
    require(required_sources <= execution['source_sha256'].keys(), '启动记录缺少实际飞行脚本、模型或控制库指纹')
    library = ROOT / 'models/gimbals/two_axis_pod/controller/build/lib/libgz-two-axis-pod-controller.so'
    require(execution.get('controller_library_sha256') == sha(library), '启动后控制插件二进制变化')
    captured = execution.get('captured_at_unix_s')
    require(finite(captured) and 0 < captured <= start, '缺少飞行前捕获的执行指纹')
    return manifest, execution, legs, scored_truth, scored_commands


def validate_tracking(run, flight, tracking, rows):
    require(tracking.get('result') == 'passed' and not tracking.get('failures'), '独立图像跟踪未通过')
    samples = [r for r in rows if r.get('kind') == 'sample']
    require(len(samples) == tracking.get('samples') and len(samples) >= 30, '跟踪原始采样数量与摘要不一致')
    result_rows = [r for r in rows if r.get('kind') == 'event' and r.get('event') == 'result']
    require(result_rows and all(result_rows[-1].get(k) == v for k, v in tracking.items()
                                if k not in ('acceptance_settings', 'source_sha256', 'execution_metadata')),
            '跟踪结果事件与摘要不一致')
    stamps = [r['stamp'] for r in samples]
    require(all(finite(t) for t in stamps) and all(a < b for a, b in zip(stamps, stamps[1:])),
            '跟踪帧时间必须有限且严格递增')
    require(close(stamps[-1]-stamps[0], tracking.get('synchronized_sim_seconds')), '跟踪观测仿真时长与原始采样不一致')
    require(all(flight['started_at_unix_s'] <= row.get('wall', 0) <= flight['finished_at_unix_s'] for row in samples),
            '跟踪原始样本不属于本次飞行时间窗口')
    validate_source_hashes(tracking.get('source_sha256'), '跟踪记录')
    require(isinstance(tracking.get('acceptance_settings'), dict) and tracking['acceptance_settings'],
            '跟踪记录缺少实际评分设置')
    if tracking.get('acceptance_settings'):
        from types import SimpleNamespace
        spec = importlib.util.spec_from_file_location('geo_archive_tracking', HERE / 'check_tracking.py')
        module = importlib.util.module_from_spec(spec)
        sys.path.insert(0, str(HERE))
        try:
            spec.loader.exec_module(module)
        finally:
            sys.path.remove(str(HERE))
        actual = module.summarize(samples, SimpleNamespace(**tracking['acceptance_settings']))
        require(actual.get('result') == 'passed' and all(actual[k] == tracking.get(k) for k in actual),
                '跟踪摘要与原始图像独立评分不一致')
    return samples


def validate_binding(directory, flight):
    require(directory is not None, '完整链路归档必须提供 --binding-dir 的真实RTSP绑定证据')
    directory = Path(directory).resolve()
    report = read_json(directory / 'binding_summary.json')
    require(report.get('result') == 'passed' and report.get('source_fingerprints_current') is True
            and not report.get('error') and not report.get('identity_errors'), '网关身份与实际RTSP视频检查未通过')
    validate_source_hashes(report.get('source_sha256'), 'RTSP验证')
    rows = read_rows(directory / 'binding_samples.jsonl')
    require(rows and all(flight['started_at_unix_s'] <= r.get('wall', 0) <= flight['finished_at_unix_s'] for r in rows),
            'RTSP绑定证据不属于本次飞行时间窗口')
    videos = [r for r in rows if r.get('kind') == 'video']
    hellos = [r for r in rows if r.get('kind') == 'hello']
    telemetry = [r for r in rows if r.get('kind') == 'telemetry']
    require(len(videos) == report.get('frames') and len(hellos) == report.get('hello_count')
            and len(telemetry) == report.get('telemetry_count'), 'RTSP原始采样数量与摘要不一致')
    require(len(videos) >= 20 and len(telemetry) >= 20 and hellos, 'RTSP原始视频或遥测样本不足')
    from check_rtsp_binding import validate_identity
    sequences, tables, domains = set(), set(), set()
    for row in hellos+telemetry:
        payload = row['payload']
        validate_identity(payload, report['aircraft_id'], report['expected_path'])
        require(payload.get('angle_convention') == 'motion_head_joint_right_positive', 'RTSP绑定记录角度协议不一致')
        version = payload.get('version') if row['kind'] == 'hello' else payload.get('protocol_version')
        require(version == 8, 'RTSP绑定记录必须使用协议v8')
        if payload.get('clock_domain'):
            domains.add(payload['clock_domain'])
        tables.update(int(item[0]) & 0xffffff for item in payload.get('frame_table', []))
    decoded = [row for row in videos if row.get('seq') is not None]
    sequences.update(row['seq'] for row in decoded)
    decoded_ratio = len(decoded)/len(videos)
    matched = len(sequences & tables)
    match_ratio = matched/max(1, len(sequences))
    require(decoded_ratio >= .95 and match_ratio >= .95 and len(domains) == 1,
            'RTSP原始帧号或帧表对应关系未达到既定95%门槛')
    for key, value in [('decoded_frames', len(decoded)), ('unique_video_sequences', len(sequences)),
                       ('matched_frame_table_sequences', matched), ('decoded_ratio', decoded_ratio),
                       ('frame_table_match_ratio', match_ratio)]:
        require(close(report.get(key), value), f'RTSP原始记录与摘要不一致：{key}')
    require(report.get('clock_domains') == sorted(domains), 'RTSP时钟域与原始记录不一致')
    guard = report.get('station_identity_guard')
    if guard is not None:
        require(guard.get('correct_identity_ready') is True and guard.get('correct_public_angles_ready') is True
                and guard.get('mismatched_identity_ready') is False and guard.get('attempted_control_datagrams') == 0
                and guard.get('command_api_results') == [-1]*4, 'Station身份门禁证据未通过')
    return report


def validate_world(run, tracking):
    from prepare_world import absolute_resources, set_text
    from target_fixture import TARGET_NAME
    world = ET.parse(run / 'world.sdf').find('world')
    model_path = run / 'models/rocket_two_axis_pod/model.sdf'
    expected = ET.parse(model_path).find('model')
    actual = world.find("model[@name='rocket_two_axis_pod']")
    require(actual is not None, '运行世界缺少组合实体')
    absolute_resources(expected, model_path)
    # 运行世界会显式覆盖基础吊舱的默认 sim_truth，以保留本场景的 CSRT 验收。
    gimbal = expected.find(".//plugin[@name='two_axis::GimbalController']")
    if gimbal is not None:
        set_text(gimbal, 'tracking_backend', 'csrt')
        set_text(gimbal, 'los_target_name', TARGET_NAME)
    for item in (actual, expected):
        pose = item.find('pose')
        if pose is not None:
            item.remove(pose)
    check = assembly_tools()
    require(check.canonical(actual) == check.canonical(expected),
            '运行世界组合模型与实际生成资产不一致')
    target = world.find("model[@name='AERIAL_TRACKING_TARGET']")
    require(target is not None, '运行世界缺少空中视觉靶标')
    position = [float(value) for value in target.findtext('pose').split()[:3]]
    size = [float(value) for value in target.findtext("link/visual[@name='dark_core']/geometry/box/size").split()]
    require(tracking.get('target_enu') == position and all(close(tracking.get('target_size_m'), value) for value in size),
            '跟踪独立真值靶标与实际运行世界不一致')


def curve_png(flight, tracking, moving, commands, target_rows):
    plt.rcParams.update({'font.family': ['Noto Sans CJK SC', 'DejaVu Sans'], 'axes.unicode_minus': False})
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
    fig.suptitle(f"当前组合体 | {altitude:g} m / {flight['settings']['speed_m_s']:g} m/s / {flight['settings']['leg_length_m']:g} m\n"
                 f"{flight['completed_legs']} 个真实航段及降落通过；图像跟踪 {tracking['synchronized_sim_seconds']:.1f} 仿真秒，独立确认 {tracking['independently_confirmed_ratio']:.0%}")
    data = io.BytesIO()
    fig.savefig(data, format='png', dpi=160)
    plt.close(fig)
    return data.getvalue()


def generate(run, output, binding_dir=None, *, evidence_dir=None, renderer=curve_png):
    run, output = Path(run).resolve(), Path(output).resolve()
    flight = read_json(run / 'flight_result.json')
    tracking = read_json(run / 'tracking/summary.json')
    rows = read_rows(run / 'flight_result.jsonl')
    target_rows = read_rows(run / 'tracking/tracking.jsonl')
    manifest, execution, legs, moving, commands = validate_flight(run, flight, rows)
    samples = validate_tracking(run, flight, tracking, target_rows)
    binding = validate_binding(binding_dir, flight)
    validate_world(run, tracking)
    binding_dir = Path(binding_dir).resolve()
    # 所有核验、截图读取及绘图均先完成，失败时不覆盖入库的既有验收证据。
    curve = renderer(flight, tracking, moving, commands, samples)
    video = (binding_dir / 'rtsp_snapshot.png').read_bytes()
    snapshots = sorted((run / 'tracking').glob('frame_*.png'))
    require(snapshots, '跟踪验收缺少实际图像快照')
    picture = snapshots[len(snapshots)//2].read_bytes()
    raw_files = {
        'combined_flight.json.gz': run / 'flight_result.json',
        'combined_samples.jsonl.gz': run / 'flight_result.jsonl',
        'tracking_summary.json.gz': run / 'tracking/summary.json',
        'tracking_samples.jsonl.gz': run / 'tracking/tracking.jsonl',
        'rtsp_binding_samples.jsonl.gz': binding_dir / 'binding_samples.jsonl',
        'rtsp_binding_summary.json.gz': binding_dir / 'binding_summary.json',
        'execution_fingerprints.json.gz': run / 'execution_fingerprints.json',
        'world.sdf.gz': run / 'world.sdf',
        'assembled_model.sdf.gz': run / 'models/rocket_two_axis_pod/model.sdf',
    }
    metadata = run / 'tracking/execution_metadata.json'
    if metadata.is_file():
        raw_files['tracking_execution_metadata.json.gz'] = metadata
    archives = {name: gzip.compress(path.read_bytes(), mtime=0) for name, path in raw_files.items()}
    summary = dict(passed=True, landing_confirmed=True,
        scope='当前组合体：PX4实际往返飞行、真实图像空中目标跟踪及RTSP身份绑定',
        settings=flight['settings'], completed_legs=flight['completed_legs'], legs=legs,
        flight_started_at_unix_s=flight['started_at_unix_s'], flight_finished_at_unix_s=flight['finished_at_unix_s'],
        model_config_sha256=flight['model_config_sha256'],
        tracking=tracking, assembly_manifest=manifest, execution_fingerprints=execution,
        controller_library_sha256=execution['controller_library_sha256'], rtsp_binding=binding,
        evidence_sha256={'tests/results/'+name: hashlib.sha256(data).hexdigest() for name, data in archives.items()},
        archive_tool_sha256=sha(Path(__file__)),
        limits='仅覆盖当前无风地理世界及实际可达的静止空中靶标；人工交互验收独立，未验证任意目标或所有载机姿态。')
    evidence = Path(evidence_dir) if evidence_dir is not None else HERE / 'tests/results'
    evidence.mkdir(parents=True, exist_ok=True)
    output.mkdir(parents=True, exist_ok=True)
    for name, data in archives.items():
        (evidence / name).write_bytes(data)
    (output / 'pod_video.png').write_bytes(video)
    (output / 'tracking_snapshot.png').write_bytes(picture)
    (output / 'flight_tracking.png').write_bytes(curve)
    (output / 'validation.json').write_text(json.dumps(summary, ensure_ascii=False, allow_nan=False, indent=2)+'\n')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, default=HERE / 'docs')
    parser.add_argument('--binding-dir', type=Path, required=True, help='本次飞行期间实际通过的网关ID、RTSP路径及帧表关联证据')
    args = parser.parse_args()
    generate(args.run_dir.resolve(), args.output_dir.resolve(), args.binding_dir)
    print(f'组合体验收摘要与实际曲线：{args.output_dir.resolve()}')
