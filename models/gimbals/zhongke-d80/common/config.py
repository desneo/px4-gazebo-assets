"""读取并严格校验中科INI，向所有进程提供同一配置快照。"""
from __future__ import annotations

import configparser
import copy
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import re
from .optics import derive_optics

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / 'parameters.ini'
SCHEMA = {
    'model': {'mass_kg': float, 'name': str, 'aircraft_id': int},
    'mechanics': dict.fromkeys(('yaw_min_deg', 'yaw_max_deg', 'pitch_min_deg',
        'pitch_max_deg', 'max_rate_dps', 'max_acc_dps2', 'kp', 'stop_margin_deg'), float),
    'optics': {'width_px': int, 'height_px': int, **dict.fromkeys(('fps',
        'focal_length_mm', 'reference_focal_length_mm', 'reference_hfov_deg',
        'reference_vfov_deg', 'near_clip_m', 'far_clip_m'), float)},
    'tracking': {'target_names': str, **dict.fromkeys(('discovery_range_m',
        'point_fallback_radius_px', 'reacquire_period_s', 'lost_timeout_s'), float)},
    'product': dict.fromkeys(('mass_limit_g', 'supply_voltage_v',
        'average_power_limit_w', 'peak_power_limit_w', 'diameter_limit_mm',
        'height_limit_mm', 'pointing_accuracy_deg', 'accuracy_yaw_min_deg',
        'accuracy_yaw_max_deg', 'accuracy_pitch_min_deg', 'accuracy_pitch_max_deg',
        'fov_tolerance_percent'), float),
    'network': {'udp_bind': str, 'udp_port': int, **dict.fromkeys(('telemetry_hz',
        'peer_lease_s', 'manual_timeout_s', 'telemetry_max_age_s'), float)},
    'video': {'rtsp_bind': str, 'rtsp_port': int, 'rtsp_mount': str, 'bitrate_kbps': int},
    'viewer': {'near_clip_m': float, 'sensitivity': float},
    'demo': {'world_name': str, 'marker_name': str},
}


# 仅运行身份和端点允许通过CLI覆盖，机械、光学等性能仍来自源INI。
RUNTIME_FIELDS = {
    'model': ('model', 'name', str),
    'aircraft_id': ('model', 'aircraft_id', int),
    'world': ('demo', 'world_name', str),
    'target_name': ('tracking', 'target_names', list),
    'udp_bind': ('network', 'udp_bind', str),
    'udp_port': ('network', 'udp_port', int),
    'rtsp_bind': ('video', 'rtsp_bind', str),
    'rtsp_port': ('video', 'rtsp_port', int),
    'rtsp_mount': ('video', 'rtsp_mount', str),
}


def add_runtime_arguments(parser):
    """为装配生成器和网关注册相同的显式运行覆盖参数。"""
    for name, (_, _, kind) in RUNTIME_FIELDS.items():
        options = dict(default=None)
        if kind is list:
            options.update(action='append', help='目标实体名称，可重复指定')
        else:
            options['type'] = kind
        parser.add_argument('--' + name.replace('_', '-'), **options)
    return parser


def runtime_overrides_from_args(args):
    """未提供的参数不覆盖源INI；目标名单按命令行整体替换。"""
    overrides = {}
    for name, (section, key, _) in RUNTIME_FIELDS.items():
        value = getattr(args, name, None)
        if value is not None:
            overrides.setdefault(section, {})[key] = value
    return overrides


def apply_runtime_overrides(config, overrides):
    """复制源快照，应用运行字段并校验、重新计算派生值和指纹。"""
    allowed = {}
    for section, key, _ in RUNTIME_FIELDS.values():
        allowed.setdefault(section, set()).add(key)
    effective = copy.deepcopy(config)
    for section, fields in overrides.items():
        if section not in allowed or set(fields) - allowed[section]:
            raise ValueError(f'不支持的中科运行覆盖字段：{section}')
        effective[section].update(copy.deepcopy(fields))
    return refresh_derived(effective)


def _positive(value, name):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name}必须为正有限数')


def validate_config(c: dict) -> None:
    for section, fields in SCHEMA.items():
        for key, kind in fields.items():
            if kind in (int, float):
                value = c[section][key]
                if type(value) not in (int, float) or not math.isfinite(value):
                    raise ValueError(f'{section}.{key}必须为有限数值')
                if kind is int and type(value) is not int:
                    raise ValueError(f'{section}.{key}必须为整数')
    _positive(c['model']['mass_kg'], 'model.mass_kg')
    if c['model']['aircraft_id'] < 0:
        raise ValueError('model.aircraft_id必须非负')
    for section, key in (('model', 'name'), ('demo', 'world_name'), ('demo', 'marker_name')):
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]*', c[section][key]):
            raise ValueError(f'{section}.{key}必须是非空英文标识符')
    m = c['mechanics']
    for axis in ('yaw', 'pitch'):
        lo, hi = m[f'{axis}_min_deg'], m[f'{axis}_max_deg']
        if not -327.68 <= lo < hi <= 327.67 or not lo <= 0 <= hi:
            raise ValueError(f'{axis}行程必须有序、包含零位且在协议角度范围内')
        if m['stop_margin_deg'] < 0 or 2 * m['stop_margin_deg'] >= hi - lo:
            raise ValueError(f'{axis}停车缓冲不得为负或吞没行程')
    if not 0 < m['max_rate_dps'] <= 327.67:
        raise ValueError('mechanics.max_rate_dps必须在(0,327.67]内')
    _positive(m['max_acc_dps2'], 'mechanics.max_acc_dps2')
    _positive(m['kp'], 'mechanics.kp')
    o = c['optics']
    for key in ('width_px', 'height_px'):
        if o[key] <= 0 or o[key] % 2:
            raise ValueError(f'optics.{key}必须为正偶数')
    _positive(o['fps'], 'optics.fps')
    if not 0 < o['near_clip_m'] < o['far_clip_m']:
        raise ValueError('相机裁剪必须满足0<near<far')
    derive_optics(o)
    t = c['tracking']
    for key in ('discovery_range_m', 'reacquire_period_s', 'lost_timeout_s'):
        _positive(t[key], f'tracking.{key}')
    if t['point_fallback_radius_px'] < 0:
        raise ValueError('点选提示半径不得为负')
    if len(t['target_names']) != len(set(t['target_names'])):
        raise ValueError('tracking.target_names不得重复')
    p = c['product']
    for key in ('mass_limit_g', 'supply_voltage_v', 'diameter_limit_mm', 'height_limit_mm'):
        _positive(p[key], f'product.{key}')
    for key in ('average_power_limit_w', 'peak_power_limit_w',
                'pointing_accuracy_deg', 'fov_tolerance_percent'):
        if p[key] < 0:
            raise ValueError(f'product.{key}不得为负')
    if p['average_power_limit_w'] > p['peak_power_limit_w']:
        raise ValueError('平均功耗上限不得大于峰值上限')
    for axis in ('yaw', 'pitch'):
        if p[f'accuracy_{axis}_min_deg'] > p[f'accuracy_{axis}_max_deg']:
            raise ValueError('指向精度适用角度范围倒置')
    for section, key in (('network', 'udp_bind'), ('video', 'rtsp_bind')):
        try:
            ipaddress.ip_address(c[section][key])
        except ValueError as exc:
            raise ValueError(f'{section}.{key}必须为IP字面量') from exc
    for section, key in (('network', 'udp_port'), ('video', 'rtsp_port')):
        if not 1 <= c[section][key] <= 65535:
            raise ValueError(f'{section}.{key}必须在1～65535内')
    for key in ('telemetry_hz', 'peer_lease_s', 'manual_timeout_s', 'telemetry_max_age_s'):
        _positive(c['network'][key], f'network.{key}')
    if not re.fullmatch(r'/[A-Za-z0-9_/-]+', c['video']['rtsp_mount']):
        raise ValueError('video.rtsp_mount必须为合法非空绝对挂载路径')
    if c['video']['bitrate_kbps'] <= 0:
        raise ValueError('video.bitrate_kbps必须为正整数')
    for key in ('near_clip_m', 'sensitivity'):
        _positive(c['viewer'][key], f'viewer.{key}')


def refresh_derived(c: dict) -> dict:
    validate_config(c)
    canonical = {section: c[section] for section in SCHEMA}
    digest = hashlib.sha256(json.dumps(canonical, sort_keys=True,
        ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    c['derived'] = {**derive_optics(c['optics']), 'config_hash': digest}
    c['config_hash'] = digest
    return c


def load_config(path: str | Path = DEFAULT_CONFIG) -> dict:
    path = Path(path).expanduser().resolve()
    parser = configparser.ConfigParser(interpolation=None, strict=True)
    try:
        with path.open(encoding='utf-8') as stream:
            parser.read_file(stream)
        if parser.defaults():
            raise ValueError('不支持DEFAULT配置继承')
        if set(parser.sections()) != set(SCHEMA):
            raise ValueError(f'配置节缺失或未知：{set(parser.sections()) ^ set(SCHEMA)}')
        c = {}
        for section, fields in SCHEMA.items():
            if set(parser[section]) != set(fields):
                raise ValueError(f'{section}配置项缺失或未知：{set(parser[section]) ^ set(fields)}')
            c[section] = {key: kind(parser[section][key].strip()) for key, kind in fields.items()}
        names = c['tracking']['target_names']
        c['tracking']['target_names'] = [x.strip() for x in names.split(',') if x.strip()]
        c['config_path'] = str(path)
        return refresh_derived(c)
    except (OSError, configparser.Error, ValueError, OverflowError) as exc:
        raise ValueError(f'中科配置读取失败：{path}：{exc}') from exc


def to_model_parameters(c: dict) -> dict:
    """把规范INI快照转换为本组件复制生成器的数据结构，不读取原模型配置。"""
    m, o, d = c['mechanics'], c['optics'], c['derived']
    return {
        'housing_diameter_m': .084, 'overall_height_m': .091,
        'body_height_m': .083, 'flange_thickness_m': .008,
        'camera_diameter_m': .048, 'camera_center_m': [.015, 0, .056],
        'mounting_holes': {'count': 6, 'thread_diameter_m': .003,
            'radial': True, 'pilot_diameter_m': .0025, 'axial_position_m': .0034},
        'mass_kg': c['model']['mass_kg'],
        'mass_distribution': 'uniform_effective_density_over_cad_solids',
        'kinematics': {'yaw_origin_m': [0, 0, .056], 'yaw_axis': [1, 0, 0],
            'pitch_axis': [0, -1, 0]},
        'yaw_limits_deg': [m['yaw_min_deg'], m['yaw_max_deg']],
        'pitch_limits_deg': [m['pitch_min_deg'], m['pitch_max_deg']],
        'max_rate_dps': m['max_rate_dps'], 'max_acc_dps2': m['max_acc_dps2'],
        'kp': m['kp'], 'stop_margin_deg': m['stop_margin_deg'],
        'manual_timeout_s': c['network']['manual_timeout_s'],
        'discovery_range_m': c['tracking']['discovery_range_m'],
        'target_names': c['tracking']['target_names'], 'config_hash': c['config_hash'],
        'model_name': c['model']['name'], 'world_name': c['demo']['world_name'],
        'telemetry_hz': c['network']['telemetry_hz'],
        'camera': {'width': o['width_px'], 'height': o['height_px'], 'fps': o['fps'],
            'hfov_deg': d['hfov_deg'], 'vfov_deg': d['vfov_deg'],
            'fx': d['fx'], 'fy': d['fy'], 'cx': d['cx'], 'cy': d['cy'],
            'near_m': o['near_clip_m'], 'far_m': o['far_clip_m'],
            'sensor_offset_m': [0, 0, .0237], 'sensor_rpy_rad': [0, -math.pi/2, 0]},
    }
