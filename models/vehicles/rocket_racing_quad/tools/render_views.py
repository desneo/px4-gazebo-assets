#!/usr/bin/env python3
"""从当前配置与生成模型输出外形评审图，不作为 Gazebo 渲染截图。"""
import argparse
import os
from pathlib import Path
import xml.etree.ElementTree as ET
os.environ.setdefault('MPLCONFIGDIR', '/tmp/rocket-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import generate_meshes
import generate_model


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def transform(triangles, pose):
    xyzrpy = [float(v) for v in (pose or '0 0 0 0 0 0').split()]
    return triangles @ rotation(xyzrpy[3:]).T + xyzrpy[:3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=generate_model.ROOT / 'parameters.json')
    parser.add_argument('--output', type=Path, default=generate_model.ROOT / 'docs/model_views.png')
    args = parser.parse_args()
    config = generate_meshes.load_config(args.config)
    generate_model.validate(config)
    meshes = generate_meshes.build_meshes(config)
    model = generate_model.build_model(config).find('model')
    pieces = []
    for link in model.findall('link'):
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh/uri')
            if mesh is not None:
                vertices = meshes[Path(mesh.text).name]
            else:
                shape = visual.find('geometry/cylinder')
                radius, length = float(shape.findtext('radius')), float(shape.findtext('length'))
                vertices = generate_meshes.revolve_around_x([(-length/2, 0), (-length/2, radius),
                    (length/2, radius), (length/2, 0)], segments=16) @ generate_meshes.X_AXIS_TO_Z_AXIS.T
            vertices = transform(transform(vertices, visual.findtext('pose')), link.findtext('pose'))
            color = '#278e9d' if link.attrib['name'].startswith('rotor') else '#717c89'
            pieces.append((vertices, color))
    fig = plt.figure(figsize=(12, 10), facecolor='white')
    views = [('Front (looking along -Y)', 0, -90), ('Side (looking along +X)', 0, 0),
             ('Top (+Z)', 90, -90), ('Oblique', 22, -55)]
    extent = config['geometry']['diagonal_wheelbase_m'] / 2 + config['geometry']['propeller_diameter_m'] / 2
    for i, (title, elev, azim) in enumerate(views, 1):
        ax = fig.add_subplot(2, 2, i, projection='3d', computed_zorder=False)
        for vertices, color in pieces:
            ax.add_collection3d(Poly3DCollection(vertices, facecolors=color, linewidths=0, shade=True,
                                                zsort='average'))
        ax.set(xlim=(-extent, extent), ylim=(-extent, extent), zlim=(-extent, extent),
               xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)', title=title)
        ax.set_box_aspect((1, 1, 1))
        ax.set_proj_type('ortho')
        ax.view_init(elev=elev, azim=azim)
        ax.set_xticks([-.2, 0, .2]); ax.set_yticks([-.2, 0, .2]); ax.set_zticks([-.2, 0, .2])
        if i == 1:
            ax.set_yticks([]); ax.set_ylabel('')
        elif i == 2:
            ax.set_xticks([]); ax.set_xlabel('')
        elif i == 3:
            ax.set_zticks([]); ax.set_zlabel('')
    g = config['geometry']
    fig.suptitle(f"Rocket racing quad | axial length {g['body_length_m']*1000:.0f} mm | diagonal wheelbase {g['diagonal_wheelbase_m']*1000:.0f} mm | propellers {g['propeller_diameter_m']*1000:.0f} mm\nGeometry preview from generated assets; nose and thrust: +Z", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, .94))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    print(args.output.resolve())


if __name__ == '__main__':
    main()
