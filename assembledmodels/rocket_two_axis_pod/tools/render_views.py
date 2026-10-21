#!/usr/bin/env python3
"""从装配 SDF 生成静态几何评审图；图片不是 Gazebo 截图。

默认输出整机三视图与斜视图、机头近景与机械姿态对比。姿态对比通过同目录的
generate_model.build_model(yaw_deg=..., pitch_deg=...) 生成关节姿态对比。
"""
import argparse
import importlib.util
import os
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

os.environ.setdefault("MPLCONFIGDIR", "/tmp/rocket-pod-matplotlib")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
plt.rcParams.update({"font.family": ["Noto Sans CJK SC", "DejaVu Sans"],
                     "axes.unicode_minus": False, "font.size": 10})


def assembly_generator():
    spec = importlib.util.spec_from_file_location("assembled_preview_generator", ROOT / "tools/generate_model.py")
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    return generator


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def transform(vertices, pose):
    values = np.fromstring(pose or "0 0 0 0 0 0", sep=" ")
    if len(values) != 6:
        raise ValueError(f"Expected xyz/rpy pose, got {pose!r}")
    return vertices @ rotation(values[3:]).T + values[:3]


def resolve_mesh(uri, model_path):
    if uri.startswith("model://"):
        model_name, relative = uri[8:].split("/", 1)
        candidates = [REPO / "assembledmodels" / model_name / relative,
                      REPO / "assemblies" / model_name / relative]
        candidates.extend(directory / model_name / relative for directory in (REPO / "models").iterdir()
                          if directory.is_dir())
        for path in candidates:
            if path.is_file():
                return path
        raise FileNotFoundError(f"Cannot resolve {uri}")
    path = Path(uri.removeprefix("file://"))
    return path if path.is_absolute() else model_path.parent / path


def read_stl(path):
    raw = path.read_bytes()
    if len(raw) >= 84:
        count = struct.unpack_from("<I", raw, 80)[0]
        if len(raw) == 84 + count * 50:
            dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
            return np.frombuffer(raw, dtype=dtype, count=count, offset=84)["vertices"].astype(float)
    vertices = [list(map(float, line.split()[1:])) for line in raw.decode("ascii").splitlines()
                if line.strip().startswith("vertex ")]
    return np.asarray(vertices).reshape(-1, 3, 3)


def read_dae(path):
    """Read the indexed triangle geometry used by repository-generated COLLADA assets."""
    doc = ET.parse(path).getroot()
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    pieces = []
    for mesh in doc.findall(".//c:geometry/c:mesh", ns):
        sources = {}
        for source in mesh.findall("c:source", ns):
            values = np.fromstring(source.findtext("c:float_array", namespaces=ns), sep=" ")
            accessor = source.find("c:technique_common/c:accessor", ns)
            sources[source.attrib["id"]] = values.reshape(-1, int(accessor.attrib.get("stride", "1")))
        vertices = {vertex.attrib["id"]: vertex.find("c:input[@semantic='POSITION']", ns).attrib["source"][1:]
                    for vertex in mesh.findall("c:vertices", ns)}
        for triangles in mesh.findall("c:triangles", ns):
            inputs = triangles.findall("c:input", ns)
            item = next(item for item in inputs if item.attrib["semantic"] == "VERTEX")
            stride = max(int(item.attrib.get("offset", "0")) for item in inputs) + 1
            indices = np.fromstring(triangles.findtext("c:p", namespaces=ns), sep=" ", dtype=int).reshape(-1, stride)
            coordinates = sources[vertices[item.attrib["source"][1:]]][:, :3]
            pieces.append(coordinates[indices[:, int(item.attrib.get("offset", "0"))]].reshape(-1, 3, 3))
    if not pieces:
        raise ValueError(f"No triangle geometry in {path}")
    unit = doc.find("c:asset/c:unit", ns)
    scale = float(unit.attrib.get("meter", "1")) if unit is not None else 1.0
    return np.concatenate(pieces) * scale


def cylinder(radius, length, segments=48):
    a = np.arange(segments) * 2 * np.pi / segments
    ring = np.column_stack((radius * np.cos(a), radius * np.sin(a), np.zeros(segments)))
    lower, upper = ring + [0, 0, -length/2], ring + [0, 0, length/2]
    result = []
    for i in range(segments):
        j = (i + 1) % segments
        result.extend([[lower[i], lower[j], upper[j]], [lower[i], upper[j], upper[i]],
                       [[0, 0, -length/2], lower[j], lower[i]], [[0, 0, length/2], upper[i], upper[j]]])
    return np.asarray(result)


def sphere(radius, segments=48, bands=24):
    a, b = np.meshgrid(np.linspace(0, 2*np.pi, segments+1), np.linspace(0, np.pi, bands+1))
    points = radius*np.stack((np.sin(b)*np.cos(a), np.sin(b)*np.sin(a), np.cos(b)), axis=-1)
    return np.concatenate([np.stack((points[:-1, :-1], points[1:, :-1], points[1:, 1:]), axis=-2).reshape(-1, 3, 3),
                           np.stack((points[:-1, :-1], points[1:, 1:], points[:-1, 1:]), axis=-2).reshape(-1, 3, 3)])


def box(size):
    vertices = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                         [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]]) * np.asarray(size)/2
    faces = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                      [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])
    return vertices[faces]


def load_pieces(sdf, model_path):
    model = sdf if sdf.tag == "model" else sdf.find("model")
    pieces = []
    for link in model.findall("link"):
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")
            mesh = geometry.find("mesh")
            if mesh is not None:
                path = resolve_mesh(mesh.findtext("uri"), model_path)
                if path.suffix.lower() == ".stl":
                    vertices = read_stl(path)
                elif path.suffix.lower() == ".dae":
                    vertices = read_dae(path)
                else:
                    raise ValueError(f"Unsupported mesh format: {path}")
                vertices *= np.fromstring(mesh.findtext("scale", "1 1 1"), sep=" ")
            elif geometry.find("cylinder") is not None:
                shape = geometry.find("cylinder")
                vertices = cylinder(float(shape.findtext("radius")), float(shape.findtext("length")))
            elif geometry.find("sphere") is not None:
                vertices = sphere(float(geometry.findtext("sphere/radius")))
            elif geometry.find("box") is not None:
                vertices = box(np.fromstring(geometry.findtext("box/size"), sep=" "))
            else:
                raise ValueError(f"Unsupported geometry for {visual.attrib['name']}")
            vertices = transform(transform(vertices, visual.findtext("pose")), link.findtext("pose"))
            diffuse = np.fromstring(visual.findtext("material/diffuse", ".6 .6 .6 1"), sep=" ")
            # Bright studio exposure keeps dark source materials legible, without recoloring parts.
            color = np.r_[np.clip(diffuse[:3]**0.42, 0, 1), diffuse[3] if len(diffuse) > 3 else 1]
            pieces.append((vertices * 1000, color, visual.attrib["name"]))
    if not pieces:
        raise ValueError("SDF contains no directly renderable link visuals")
    return pieces


def draw(ax, pieces, limits, elev, azim, title, ticks=True):
    visible, colors = [], []
    elevation, azimuth = np.deg2rad([elev, azim])
    toward_camera = np.array([np.cos(elevation)*np.cos(azimuth),
                              np.cos(elevation)*np.sin(azimuth), np.sin(elevation)])
    light = np.array([-.3, -.5, 1.])
    light /= np.linalg.norm(light)
    for vertices, color, _name in pieces:
        if vertices[:, :, 2].max() < limits[2][0]:
            continue
        normals = np.cross(vertices[:, 1]-vertices[:, 0], vertices[:, 2]-vertices[:, 0])
        keep = normals @ toward_camera > 1e-12
        if not np.any(keep):
            continue
        normals = normals[keep]
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        visible.append(vertices[keep])
        face_colors = np.tile(color, (int(keep.sum()), 1))
        face_colors[:, :3] *= (.80 + .20*np.maximum(0., normals @ light))[:, None]
        colors.append(face_colors)
    # 合并后按所有三角面统一排序，避免罩壳与球头分属 collection 时互相穿透显示。
    ax.add_collection3d(Poly3DCollection(np.concatenate(visible), facecolors=np.concatenate(colors),
                                        linewidths=0, axlim_clip=True, antialiased=False,
                                        shade=False))
    ax.set(xlim=limits[0], ylim=limits[1], zlim=limits[2], title=title)
    ax.set_box_aspect([high-low for low, high in limits])
    ax.set_proj_type("ortho")
    ax.view_init(elev=elev, azim=azim)
    ax.set(xlabel="X (mm)", ylabel="Y (mm)", zlabel="Z (mm)")
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor((.96, .97, .98, 1))
        axis._axinfo["grid"].update(color=(.78, .81, .84, .45), linewidth=.5)
    ax.tick_params(labelsize=8, pad=0)
    if not ticks:
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])


def axis_arrow(ax, origin, direction, length, color, label, label_offset=(0, 0, 0)):
    origin, direction = np.asarray(origin), np.asarray(direction)
    ax.quiver(*origin, *(direction*length), color=color, linewidth=2, arrow_length_ratio=.16)
    ax.text(*(origin+direction*(length+3)+label_offset), label, color=color, fontsize=9, fontweight="bold", zorder=100)


def head_axes(ax, yaw_deg=0, pitch_deg=0):
    generator = assembly_generator()
    mount = generator.dimensions()[1] * 1000
    chain = generator.kinematics.PodKinematics()
    poses = chain.link_poses(yaw_deg, pitch_deg)
    for name, color, label, length in (('yaw_joint', '#1e64b4', 'yaw / −X', 60),
                                       ('pitch_joint', '#148050', 'pitch / −Y′', 55)):
        joint = chain.joints[name]
        parent = joint['parent']
        motion = poses[parent] @ np.linalg.inv(chain.home[parent])
        pivot = motion @ joint['pivot']
        center = pivot[:3, 3]*1000 + [0, 0, mount]
        axis_arrow(ax, center, motion[:3, :3] @ joint['axis'], length, color, label)
    sensor = chain.camera_pose(yaw_deg, pitch_deg, mount/1000)
    axis_arrow(ax, sensor[:3, 3]*1000, sensor[:3, 0], 40, '#dc7722', '视线', (8, -5, 0))
    axis_arrow(ax, [0, 0, mount - 6], [1, 0, 0], 62, "#bd3434", "+X")


def save(fig, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180, facecolor="white")
    plt.close(fig)
    print(path.resolve())


def main_views(pieces, output_dir):
    geometry = assembly_generator().source_geometry()
    min_z, max_z, length = (geometry[key] * 1000 for key in ('min_z_m', 'max_z_m', 'length_m'))
    extent = max(np.abs(vertices[:, :, :2]).max() for vertices, _color, _name in pieces) + 25
    limits = [(-extent, extent), (-extent, extent), (min_z - 20, max_z + 20)]
    fig = plt.figure(figsize=(14, 12))
    for index, (title, elev, azim) in enumerate([
            ("正视 · 从 −Y 看", 0, -90), ("侧视 · 从 +X 看", 0, 0),
            ("俯视 · 从 +Z 看", 90, -90), ("斜视", 20, -55)], 1):
        ax = fig.add_subplot(2, 2, index, projection="3d")
        draw(ax, pieces, limits, elev, azim, title)
        if index == 1:
            ax.set_yticks([]); ax.set_ylabel("")
            offset = -extent*.78
            ax.plot([offset, offset], [0, 0], [min_z, max_z], color="#253345", linewidth=1)
            for z in (min_z, max_z):
                ax.plot([offset-7, offset+7], [0, 0], [z, z], color="#253345")
            ax.text(offset+10, 0, (min_z + max_z) / 2, f"{length:g} mm", ha="left", fontsize=10)
        elif index == 2:
            ax.set_xticks([]); ax.set_xlabel("")
        elif index == 3:
            ax.set_zticks([]); ax.set_zlabel("")
    fig.suptitle(f"火箭穿越机 + 两轴立装吊舱  |  总长 {length:g} mm\n顶部替换式组合；吊舱 +X / +Y / +Z 与机体同向；机械零位视线朝 +Z", fontsize=15, y=.985)
    fig.text(.5, .015, "由装配 SDF 实际几何生成的静态评审图（非 Gazebo 截图） · 沿用源 diffuse 材质，明亮工作室曝光", ha="center", color="#5a6470", fontsize=10)
    fig.subplots_adjust(left=.02, right=.98, top=.92, bottom=.05, wspace=.01, hspace=.10)
    save(fig, output_dir / "model_views.png")


def head_detail(pieces, output_dir):
    generator = assembly_generator()
    geometry = generator.source_geometry()
    mount = generator.dimensions()[1] * 1000
    min_z, max_z = geometry['min_z_m'] * 1000, geometry['max_z_m'] * 1000
    fig = plt.figure(figsize=(14, 8.2))
    limits = [(-70, 85), (-75, 75), (mount - 29, max_z + 20)]
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    draw(ax, pieces, limits, 0, -90, "头部正视 · 机械零位")
    ax.set_yticks([]); ax.set_ylabel("")
    for z in (mount, max_z):
        ax.plot([-50, 50], [0, 0], [z, z], color="#808a97", linestyle="--", linewidth=.8)
    ax.text2D(.08, .72, f"最高点 Z = {max_z:g} mm", transform=ax.transAxes, fontsize=9, color="#3d4856")
    ax.text2D(.08, .34, f"底环安装面 Z = {mount:g} mm", transform=ax.transAxes, fontsize=9, color="#3d4856")
    ax = fig.add_subplot(1, 2, 2, projection="3d")
    draw(ax, pieces, limits, 25, -55, "头部斜视 · 轴向示意")
    head_axes(ax)
    fig.suptitle("顶部替换式安装  |  底环 Ø84 mm  /  机身最大 Ø90 mm  /  吊舱高 91 mm", fontsize=15, y=.96)
    fig.text(.5, .10, f"yaw：绕机体 −X；pitch：绕随 yaw 转动的 −Y′；零位镜头朝 +Z，负 pitch 转向 +X\n机尾 Z = {min_z:g} mm；整机最高点 Z = +{max_z:g} mm；所有尺寸单位为 mm", ha="center", fontsize=11)
    fig.text(.5, .025, "由装配 SDF 实际几何生成的静态评审图（非 Gazebo 截图） · 坐标箭头为评审标注", ha="center", color="#5a6470", fontsize=10)
    fig.subplots_adjust(left=.01, right=.99, top=.90, bottom=.20, wspace=.03)
    save(fig, output_dir / "head_detail.png")


def pose_comparison(output_dir, model_path):
    generator = assembly_generator()
    mount = generator.dimensions()[1] * 1000
    max_z = generator.source_geometry()['max_z_m'] * 1000
    fig = plt.figure(figsize=(14, 13))
    for index, (yaw, pitch, note) in enumerate([(0, 0, "朝 +Z"), (0, -80, "朝 +X 上方 10°"),
                                              (60, -80, "朝 +X/+Y/+Z"), (0, 30, "朝 −X/+Z")], 1):
        sdf = generator.build_model(yaw_deg=yaw, pitch_deg=pitch)
        pieces = load_pieces(sdf, model_path)
        ax = fig.add_subplot(2, 2, index, projection="3d")
        draw(ax, pieces, [(-75, 90), (-75, 90), (mount - 19, max_z + 20)], 22, -55,
             f"yaw = {yaw}° / pitch = {pitch}°\n视线{note}", ticks=False)
        head_axes(ax, yaw, pitch)
    fig.suptitle("两轴机械角与视线对比  |  载机固定，吊舱真实几何随关节旋转", fontsize=15, y=.975)
    fig.text(.5, .06, "yaw 范围 −60°～+60°；pitch 范围 −80°～+30°\n外轴穿过球心；pitch = 0° 时，yaw 使视线在安装 YZ 平面转动", ha="center", fontsize=11)
    fig.text(.5, .025, "由装配生成器实际几何生成的静态评审图（非 Gazebo 截图） · 坐标箭头为评审标注", ha="center", color="#5a6470", fontsize=10)
    fig.subplots_adjust(left=.01, right=.99, top=.89, bottom=.12, wspace=.02, hspace=.18)
    save(fig, output_dir / "mechanical_poses.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "model.sdf")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "docs")
    parser.add_argument("--poses", action=argparse.BooleanOptionalAction, default=True,
                        help="调用装配生成器生成机械姿态对比图；--no-poses 可跳过")
    args = parser.parse_args()
    model_path = args.model.resolve()
    pieces = load_pieces(ET.parse(model_path).getroot(), model_path)
    main_views(pieces, args.output_dir)
    head_detail(pieces, args.output_dir)
    if args.poses:
        pose_comparison(args.output_dir, model_path)


if __name__ == "__main__":
    main()
