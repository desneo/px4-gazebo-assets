#!/usr/bin/env python3
"""从保存的 STEP 导入九个实体，导出安装系网格和精确几何质量属性。"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

from cadgen import read_scene, build123d as bd
from cadgen.geometry import mass_properties
from generate_model import ROOT, write_dae

GROUPS = {
    'mount_link': ((0., 0., 0.), ['mount_ring_6xM3', 'fixed_cowling', 'azimuth_bearing_housing']),
    'yaw_arm_link': ((0., 0., .056), ['azimuth_yoke', 'pitch_pivot_cap_left', 'pitch_pivot_cap_right']),
    'camera_link': ((.015, 0., .056), ['pitch_ball_shell', 'lens_bezel', 'optical_window']),
}
COLORS = dict(zip(
    ['mount_ring_6xM3', 'fixed_cowling', 'azimuth_bearing_housing', 'azimuth_yoke',
     'pitch_pivot_cap_left', 'pitch_pivot_cap_right', 'pitch_ball_shell', 'lens_bezel', 'optical_window'],
    ['667079', '83888B', '525B63', '4D565F', '9EA4AA', '9EA4AA', '38434E', '222C36', '34617A']))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rounded(value):
    if isinstance(value, float):
        return float(format(value, '.12g')) if abs(value) > 1e-15 else 0.
    if isinstance(value, (list, tuple)):
        return [rounded(v) for v in value]
    if isinstance(value, dict):
        return {k: rounded(v) for k, v in value.items()}
    return value


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', type=Path, help='可选的新 STEP；同名 .step.json 一同导入')
    args = ap.parse_args()
    target = ROOT / 'cad/motion_head.step'
    target.parent.mkdir(parents=True, exist_ok=True)
    if args.source and args.source.resolve() != target.resolve():
        shutil.copyfile(args.source, target)
        shutil.copyfile(args.source.with_suffix('.step.json'), target.with_suffix('.step.json'))
    sidecar = json.loads(target.with_suffix('.step.json').read_text())
    if sidecar['documentHash'] != sha(target):
        raise ValueError('STEP 与关节边车的文档哈希不一致')
    mates = {m['name']: m for m in sidecar['kinematics']['mates']}
    for name, origin, axis, limits in [('azimuth', [0, 0, 56], [1, 0, 0], [-60, 60]),
                                       ('elevation', [-15, 0, 56], [0, 1, 0], [-80, 30])]:
        m = mates[name]
        if m['axis']['origin'] != origin or m['axis']['dir'] != axis or m['limits']['value'] != limits:
            raise ValueError(f'CAD 关节与组件安装约定不一致：{name}')
    scene = read_scene(target)
    shapes = {}
    for leaf in scene.leaves():
        shape = leaf.shape()
        if len(shape.solids()) != 1 or not shape.is_valid or shape.volume <= 0:
            raise ValueError(f'需要有效正体积单实体：{leaf.label}')
        shapes[leaf.label] = shape.rotate(bd.Axis.Z, 180)
    if set(shapes) != set(COLORS):
        raise ValueError('CAD 零件清单与两轴组件不一致')
    result = dict(source='motion_head.step', units='m', cad_to_mount_rotation=[[ -1, 0, 0], [0, -1, 0], [0, 0, 1]],
                  tessellation={'linear_tolerance_mm': .035, 'angular_tolerance_rad': .12},
                  groups={}, parts={}, sha256={})
    for name, (origin, names) in GROUPS.items():
        props = mass_properties([(shapes[n], 1.) for n in names])
        volume = sum(shapes[n].volume for n in names)
        center = list(props.center_of_mass)
        result['groups'][name] = dict(origin_m=origin, parts=names, volume_mm3=volume,
            center_local_m=[center[i]/1000-origin[i] for i in range(3)],
            unit_mass_inertia_m2=[[v/volume/1e6 for v in row] for row in props.inertia])
        for n in names:
            shape = shapes[n]
            vertices, faces = shape.tessellate(.035, .12)
            points = [[v/1000-origin[i] for i, v in enumerate(point)] for point in vertices]
            mesh = ROOT / 'meshes' / (n + '.dae')
            write_dae(mesh, points, faces, n)
            color = COLORS[n]
            bounds = shape.bounding_box()
            result['parts'][n] = dict(link=name, mesh=str(mesh.relative_to(ROOT)),
                rgba=[int(color[i:i+2], 16)/255 for i in (0, 2, 4)]+[1.],
                volume_mm3=shape.volume, bounds_mount_m=[[v/1000 for v in bounds.min], [v/1000 for v in bounds.max]],
                vertices=len(vertices), triangles=len(faces))
            result['sha256'][str(mesh.relative_to(ROOT))] = sha(mesh)
    result['volume_mm3'] = sum(g['volume_mm3'] for g in result['groups'].values())
    for p in (target, target.with_suffix('.step.json')):
        result['sha256'][str(p.relative_to(ROOT))] = sha(p)
    (ROOT / 'cad/properties.json').write_text(json.dumps(rounded(result), ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'parts': len(shapes), 'volume_mm3': result['volume_mm3'], 'step_sha256': sha(target)}))


if __name__ == '__main__':
    main()
