#!/usr/bin/env python3
"""在完整无风地理世界中放置一架待测飞机，保存运行资产指纹。"""

import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
WORLD = 'rocket_speed_comparison'
SOURCE = ROOT / 'worlds/geo_32_217857_119_105878/world.sdf'
VARIANTS = {'standalone': 'rocket_racing_quad', 'assembled': 'rocket_two_axis_pod'}
_spec = importlib.util.spec_from_file_location(
    'rocket_world_support', ROOT / 'scenarios/geographic_rocket_two_axis_pod/prepare_world.py')
# 既有准备脚本的辅助导入需要其所在目录；不执行其 main。
sys.path.insert(0, str(Path(_spec.origin).parent))
support = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(support)
sys.path.pop(0)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(output, model_path, variant, *, view='overall', near_clip=.005):
    if not math.isfinite(near_clip) or near_clip <= 0:
        raise ValueError('浏览相机近裁剪距离必须为有限正数')
    expected = VARIANTS[variant]
    model_path = model_path.resolve()
    tree = ET.parse(SOURCE)
    world = tree.getroot().find('world')
    world.set('name', WORLD)
    support.absolute_resources(world, SOURCE)
    wind = world.find('wind')
    if wind is None:
        wind = ET.SubElement(world, 'wind')
    support.set_text(wind, 'linear_velocity', '0 0 0')
    model = ET.parse(model_path).getroot().find('model')
    if model is None or model.get('name') != expected:
        raise ValueError(f'需要 {variant} 模型 {expected}')
    if model.findtext('static', 'false').lower() in ('true', '1'):
        raise ValueError('需要动态飞行模型')
    support.absolute_resources(model, model_path)
    support.set_text(model, 'pose', '0 0 .24 0 0 0')
    world.append(model)
    marker = support.location_marker(0., 0.)
    marker.set('name', 'SPEED_TEST_TAKEOFF_MARKER')
    world.append(marker)
    scene = world.find('./gui/plugin[@filename="MinimalScene"]')
    if scene is not None:
        clip = scene.find('camera_clip')
        if clip is None:
            clip = ET.SubElement(scene, 'camera_clip')
        support.set_text(clip, 'near', near_clip)
        support.set_text(scene, 'camera_pose',
                         '.43 -.56 .62 0 .25 2.23' if view == 'close'
                         else '-3 -4 2.24 0 .32 .927295')
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(output, encoding='utf-8', xml_declaration=True)
    # 被测飞机允许不同；风、地形、标志、浏览相机与世界物理配置必须相同。
    environment = copy.deepcopy(world)
    environment.remove(environment.find(f'model[@name="{expected}"]'))
    canonical_environment = ET.canonicalize(
        ET.tostring(environment, encoding='unicode'), strip_text=True)
    source_paths = [SOURCE, ROOT / 'models/vehicles/rocket_racing_quad/parameters.json',
                    ROOT / 'models/vehicles/rocket_racing_quad/tools/generate_model.py']
    scene = Path(__file__).resolve().parent
    source_paths += sorted(path for path in scene.iterdir()
                           if path.is_file() and path.suffix in ('.py', '.sh', '.json'))
    if variant == 'assembled':
        source_paths += [ROOT / 'assembledmodels/rocket_two_axis_pod/parameters.json',
                         ROOT / 'assembledmodels/rocket_two_axis_pod/tools/generate_dynamic_model.py',
                         ROOT / 'models/gimbals/two_axis_pod/model.sdf',
                         ROOT / 'models/gimbals/two_axis_pod/controller/build/lib/libgz-two-axis-pod-controller.so']
    generated = {str(path.relative_to(model_path.parent)): sha256(path)
                 for path in sorted(model_path.parent.rglob('*')) if path.is_file()}
    manifest = {
        'variant': variant, 'model_name': expected, 'world_name': WORLD,
        'source_world': str(SOURCE.relative_to(ROOT)), 'world_sha256': sha256(output),
        'environment_sha256': hashlib.sha256(canonical_environment.encode()).hexdigest(),
        'wind_m_s': [0, 0, 0], 'camera_sensors_retained': True,
        'spawn_pose': [0, 0, .24, 0, 0, 0],
        'gimbal_controller_retained': variant == 'assembled',
        'physics_parameters_modified': False,
        'source_sha256': {str(path.relative_to(ROOT)): sha256(path)
                          for path in source_paths if path.is_file()},
        'generated_sha256': generated,
    }
    runtime_config = output.parent / 'config.json'
    if runtime_config.is_file():
        manifest['runtime_config_sha256'] = sha256(runtime_config)
    (output.parent / 'asset_manifest.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--variant', choices=VARIANTS, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--view', choices=('overall', 'close'), default='overall')
    parser.add_argument('--near-clip', type=float, default=.005)
    args = parser.parse_args()
    try:
        print(prepare(args.output, args.model, args.variant,
                      view=args.view, near_clip=args.near_clip))
    except (ValueError, OSError, ET.ParseError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
