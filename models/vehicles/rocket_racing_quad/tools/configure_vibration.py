#!/usr/bin/env python3
"""Opt-in vibration profile for runtime copies; never edits the native model."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT / 'config' / 'flight_hover.json'
PLUGIN_NAME = 'rocket::FlightVibration'

def load_profile(path):
    p = json.loads(Path(path).read_text())
    if p.get('schema_version') != 1:
        raise ValueError('vibration schema_version must be 1')
    ranges = {'imu_rate_hz': (500, 4000), 'max_step_size_s': (0.00005, 0.001),
              'rotor_reference_rad_s': (1, 2000), 'minimum_rotor_rad_s': (1, 2000),
              'rotor_velocity_slowdown': (1, 100), 'airborne_height_m': (0.05, 20),
              'ramp_time_s': (0.1, 30), 'fundamental_hz': (1, 200),
              'frequency_scale_min': (0.1, 1), 'frequency_scale_max': (1, 4)}
    for key, (lo, hi) in ranges.items():
        v = p.get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
            raise ValueError(f'invalid {key}: expected finite value in [{lo}, {hi}]')
    if p['minimum_rotor_rad_s'] >= p['rotor_reference_rad_s']:
        raise ValueError('minimum rotor speed must be below hover reference')
    parameters = p.get('px4_parameters', {})
    allowed = {'IMU_GYRO_CUTOFF', 'IMU_DGYRO_CUTOFF'}
    if not isinstance(parameters, dict) or not set(parameters).issubset(allowed):
        raise ValueError('px4_parameters only accepts IMU_GYRO_CUTOFF and IMU_DGYRO_CUTOFF')
    for key, value in parameters.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 5 <= value <= 1000:
            raise ValueError(f'{key} must be finite and within 5..1000 Hz')
    joints = p.get('rotor_joints')
    if not isinstance(joints, list) or len(joints) != 4 or len(set(joints)) != 4 or not all(isinstance(j, str) and j for j in joints):
        raise ValueError('exactly four distinct rotor joint names are required')
    highest = 0
    for group, amplitude, bound in [('torque_components', 'amplitude_nm', 20),
                                    ('force_components', 'amplitude_n', 50),
                                    ('gyro_components', 'amplitude_rad_s', 10),
                                    ('accel_components', 'amplitude_m_s2', 100)]:
        if not isinstance(p.get(group), list) or not 1 <= len(p[group]) <= 16:
            raise ValueError(f'{group} must contain 1..16 components')
        for c in p[group]:
            for key, limit in [(amplitude, bound), ('phase_rad', 1000)]:
                v = c.get(key)
                if not isinstance(v, list) or len(v) != 3 or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) and abs(x) <= limit for x in v):
                    raise ValueError(f'{group}.{key} must be a bounded finite triple')
            if group in ('torque_components', 'force_components'):
                f = c.get('frequency_hz')
                if not isinstance(f, (float, int)) or not math.isfinite(f) or not 0 < f <= 20:
                    raise ValueError('torque frequency must be in (0,20] Hz')
            else:
                h = c.get('harmonic')
                if not isinstance(h, int) or isinstance(h, bool) or not 1 <= h <= 4:
                    raise ValueError('sensor harmonic must be an integer 1..4')
                highest = max(highest, p['fundamental_hz'] * h * p['frequency_scale_max'])
    if highest >= p['imu_rate_hz'] / 2 or highest >= 1 / (2 * p['max_step_size_s']):
        raise ValueError('configured harmonics exceed sensor or simulation Nyquist frequency')
    return p

def child(parent, tag, value):
    element = parent.find(tag)
    if element is None:
        element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element

def configure_model(model, profile_path=DEFAULT_PROFILE, world_name=None):
    p = load_profile(profile_path)
    name = model.get('name')
    if not name or not world_name or '/' in name or '/' in world_name:
        raise ValueError('world and model names are required and must not contain /')
    if model.find(f"plugin[@name='{PLUGIN_NAME}']") is not None:
        raise ValueError('model already has flight vibration configured')
    sensor = model.find("link[@name='base_link']/sensor[@name='imu_sensor']")
    if sensor is None or sensor.get('type') != 'imu':
        raise ValueError('base_link/imu_sensor IMU required')
    canonical = f'/world/{world_name}/model/{name}/link/base_link/sensor/imu_sensor/imu'
    raw = canonical + '_vibration_raw'
    plugin = ET.Element('plugin', {'filename': 'libgz-rocket-vibration.so', 'name': PLUGIN_NAME})
    for key, value in p.items():
        if key in ('description', 'schema_version', 'px4_parameters'):
            continue
        if key == 'rotor_joints':
            for v in value:
                child(ET.SubElement(plugin, 'rotor'), 'joint', v)
        elif key.endswith('_components'):
            for component in value:
                el = ET.SubElement(plugin, key.removesuffix('s'))
                for k, v in component.items():
                    child(el, k, ' '.join(map(str, v)) if isinstance(v, list) else v)
        else:
            child(plugin, key, value)
    child(plugin, 'raw_imu_topic', raw)
    child(plugin, 'output_imu_topic', canonical)
    # Mutate only after all validation has passed.
    child(sensor, 'topic', raw)
    child(sensor, 'update_rate', p['imu_rate_hz'])
    model.append(plugin)
    return {'model': name, 'raw_imu_topic': raw, 'output_imu_topic': canonical,
            'profile': str(Path(profile_path).resolve()), 'imu_rate_hz': p['imu_rate_hz']}

def configure_world(world, profile_path=DEFAULT_PROFILE):
    p = load_profile(profile_path)
    physics = world.find('physics')
    if physics is None:
        raise ValueError('world requires explicit physics')
    step = min(float(physics.findtext('max_step_size', str(p['max_step_size_s']))), p['max_step_size_s'])
    if not math.isfinite(step) or step <= 0:
        raise ValueError('invalid existing world timestep')
    child(physics, 'max_step_size', step)
    # This field exists in supported scene worlds; do not invent additional physics.
    if physics.find('real_time_update_rate') is not None:
        child(physics, 'real_time_update_rate', 1 / step)

def configure_airframe(path, profile_path=DEFAULT_PROFILE):
    """Append whitelisted filter defaults to a copied airframe, never native PX4 files."""
    path = Path(path)
    if path.resolve() == ROOT or path.resolve().is_relative_to(ROOT):
        raise ValueError('airframe must be a runtime copy outside the native model directory')
    parameters = load_profile(profile_path).get('px4_parameters', {})
    if not parameters:
        return {}
    text = path.read_text()
    marker = '# Optional flight vibration filter defaults'
    if marker in text:
        raise ValueError('airframe already has flight vibration configured')
    lines = [marker] + [f'param set-default {key} {value:g}' for key, value in sorted(parameters.items())]
    path.write_text(text.rstrip() + '\n\n' + '\n'.join(lines) + '\n')
    return parameters

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--world', type=Path)
    parser.add_argument('--world-name')
    parser.add_argument('--model-name')
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--airframe', type=Path)
    a = parser.parse_args()
    if a.model is None and a.airframe is None:
        parser.error('at least --model or --airframe is required')
    if a.world and a.model is None:
        parser.error('--world requires --model')
    # Runtime copies only. Native model and worlds remain byte-for-byte unchanged.
    for path in [a.model, a.world, a.airframe]:
        if path and (path.resolve() == ROOT or path.resolve().is_relative_to(ROOT)):
            raise ValueError('configure_vibration only accepts runtime files outside the native model directory')
    model_tree = ET.parse(a.model) if a.model else None
    model = model_tree.getroot().find('model') if model_tree is not None else None
    if model_tree is not None and model is None:
        raise ValueError('input must be an SDF with a direct model')
    world_tree = ET.parse(a.world) if a.world else None
    world = world_tree.getroot().find('world') if world_tree is not None else None
    if a.model_name and model is not None:
        old_name = model.get('name')
        model.set('name', a.model_name)
    summary = configure_model(model, a.profile, a.world_name or (world.get('name') if world is not None else None)) if model is not None else {}
    if a.model_name and model is not None:
        model.set('name', old_name)
    if world is not None:
        configure_world(world, a.profile)
    if a.airframe:
        summary['px4_parameters'] = configure_airframe(a.airframe, a.profile)
    if model_tree is not None:
        model_tree.write(a.model, encoding='utf-8', xml_declaration=True)
    if world_tree is not None:
        world_tree.write(a.world, encoding='utf-8', xml_declaration=True)
    print(json.dumps(summary))

if __name__ == '__main__':
    main()
