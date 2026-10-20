#!/usr/bin/env python3
"""复制当前原生模型、世界与 PX4 配置，保存实际运行参数和资产指纹。"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import shlex
import stat
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
NAME = 'rocket_racing_quad'


def asset_paths(source):
    source = Path(source)
    paths = [source / name for name in ('model.sdf', 'model.config', 'tests/world.sdf')]
    for directory in ('meshes', 'materials', 'textures', 'px4'):
        paths.extend(path for path in sorted((source / directory).rglob('*')) if path.is_file())
    return paths


def source_fingerprints(source=ROOT):
    source = Path(source)
    paths = asset_paths(source)
    paths += [source / name for name in ('parameters.json', 'tools/prepare_runtime.py',
              'tests/run.sh', 'tests/run_check.sh', 'tests/flight_check.py', 'tests/gazebo_truth.py')
              if (source / name).is_file()]
    return {str(path.relative_to(source)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in paths}


def number(text):
    value = float(text)
    if not math.isfinite(value):
        raise ValueError(f'配置数值必须有限：{text}')
    return value


def parameter_values(text):
    parameters = {}
    for line in text.splitlines():
        parts = shlex.split(line, comments=True)
        if len(parts) >= 2 and parts[0] == 'param' and parts[1] in ('set', 'set-default'):
            if len(parts) != 4:
                raise ValueError(f'机体参数必须显式写为 param set-default NAME VALUE：{line}')
            parameters[parts[2]] = number(parts[3])
    if 'MPC_XY_VEL_MAX' not in parameters:
        raise ValueError('PX4 机体文件须显式配置 MPC_XY_VEL_MAX，以便核对验收速度上限')
    return parameters


def plugin_values(plugin):
    values = {}
    for child in plugin:
        value = child.text.strip() if child.text else ''
        try:
            values[child.tag] = number(value)
        except ValueError:
            values[child.tag] = value
    return values


def runtime_snapshot(files, config, airframe):
    model = ET.fromstring(files['model.sdf']).find('model')
    if model is None or model.get('name') != NAME:
        raise ValueError(f'model.sdf 必须包含模型 {NAME}')
    links = {}
    for link in model.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            raise ValueError(f"link {link.get('name')} 缺少显式 inertial")
        links[link.get('name')] = {
            'pose': link.findtext('pose', '0 0 0 0 0 0'),
            'inertial_pose': inertial.findtext('pose', '0 0 0 0 0 0'),
            'mass_kg': number(inertial.findtext('mass')),
            'inertia': {axis: number(inertial.findtext(f'inertia/{axis}'))
                        for axis in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')},
        }
    motors = [plugin_values(p) for p in model.findall('plugin')
              if p.get('name') == 'gz::sim::systems::MulticopterMotorModel']
    aero = [plugin_values(p) for p in model.findall('plugin')
            if p.get('name') == 'gz::sim::systems::AdvancedLiftDrag']
    world = ET.fromstring(files['tests/world.sdf']).find('world')
    if world is None or world.get('name') != 'rocket_racing_quad_test':
        raise ValueError('自验证世界名称必须为 rocket_racing_quad_test，与启动器话题一致')
    timeout = config['simulation']['imu_wait_timeout_s']
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('simulation.imu_wait_timeout_s 必须为有限正数')
    if not isinstance(config['flight_test'], dict):
        raise ValueError('flight_test 必须为验收设置对象')
    parameters = parameter_values(files[airframe].decode())
    return {
        'flight_test': config['flight_test'],
        'simulation': {
            'imu_wait_timeout_s': timeout,
            'world_name': world.get('name'),
            'physics': {child.tag: child.text for child in world.find('physics')},
            'gravity': world.findtext('gravity'),
            'spawn_pose': world.findtext("include[uri='model://rocket_racing_quad']/pose"),
        },
        'px4': {'airframe_id': int(Path(airframe).name.split('_')[0]), 'parameters': parameters},
        'model': {'name': NAME, 'links': links, 'motors': motors, 'aerodynamics': aero,
                  'total_mass_kg': sum(link['mass_kg'] for link in links.values())},
    }


def prepare(output_dir, *, source=ROOT, config_path=None):
    source, output = Path(source).resolve(), Path(output_dir).resolve()
    if output == source or output.is_relative_to(source) or source.is_relative_to(output):
        raise ValueError('运行副本必须另存到模型目录之外')
    if output.exists() and any(output.iterdir()):
        raise ValueError('运行模型输出目录必须为空，以免混入旧资产')
    config_path = Path(config_path).resolve() if config_path else source / 'parameters.json'
    config_bytes = config_path.read_bytes()
    config = json.loads(config_bytes)
    before = source_fingerprints(source)
    files = {str(path.relative_to(source)): path.read_bytes() for path in asset_paths(source)}
    airframes = [name for name in files if re.fullmatch(r'px4/[0-9]+_gz_rocket_racing_quad', name)]
    if len(airframes) != 1:
        raise ValueError('px4/ 必须包含唯一的 <编号>_gz_rocket_racing_quad 机体文件')
    snapshot = runtime_snapshot(files, config, airframes[0])
    if before != source_fingerprints(source) or config_bytes != config_path.read_bytes():
        raise RuntimeError('准备期间源配置发生变化，请重新启动')
    for name, content in files.items():
        if hashlib.sha256(content).hexdigest() != before[name]:
            raise RuntimeError(f'准备期间源资产发生变化：{name}')
    snapshot['runtime'] = {
        'source_sha256': before,
        'flight_config_source': str(config_path),
        'flight_config_sha256': hashlib.sha256(config_bytes).hexdigest(),
    }
    for name, content in files.items():
        target = output / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(content)
        target.chmod(stat.S_IMODE((source / name).stat().st_mode))
    (output / 'parameters.json').write_text(json.dumps(snapshot, indent=2, ensure_ascii=False) + '\n')
    return snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', type=Path, default=ROOT)
    parser.add_argument('--config', type=Path, help='只读取 flight_test 和 simulation.imu_wait_timeout_s')
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    try:
        snapshot = prepare(args.output_dir, source=args.source_dir, config_path=args.config)
    except (ValueError, OSError, KeyError, TypeError, ET.ParseError) as error:
        parser.error(str(error))
    print(f"运行资产：{args.output_dir.resolve()}；质量 {snapshot['model']['total_mass_kg']:.3f} kg；使用原生 SDF 与 PX4 配置")


if __name__ == '__main__':
    main()
