#!/usr/bin/env python3
"""读取 parameters.json 的 geometry 字段，生成米制、机头朝 +z 的 STL。

机身和动力舱网格以局部原点居中，旋翼网格以桨轴中心为原点。
动力舱位置由 model.sdf 的位姿给出，尾翼与机身共享原点。
运行：python3 tools/generate_meshes.py [--config /path/to/parameters.json]
导入本模块不会读取配置或写入文件。
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

MODEL_DIR = Path(__file__).resolve().parent.parent
MESH_DIR = MODEL_DIR / "meshes"
X_AXIS_TO_Z_AXIS = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])


def load_config(path: str | Path | None = None) -> dict:
    """加载完整参数文件并检查几何；缺失字段不使用隐含默认值。"""
    config = json.loads(Path(path or MODEL_DIR / "parameters.json").read_text(encoding="utf-8"))
    validate_geometry(config["geometry"])
    return config


def validate_geometry(g: dict) -> None:
    """拒绝无效尺寸、桨盘干涉与超出机头机尾的动力舱。"""
    positive = (
        "body_length_m", "body_radius_m", "nacelle_length_m", "nacelle_radius_m",
        "diagonal_wheelbase_m", "propeller_diameter_m", "fin_span_m",
        "fin_thickness_m", "fin_root_length_m", "fin_tip_chord_m", "arm_radius_m",
    )
    for key in (*positive, "nacelle_z_m", "rotor_z_m", "fin_tip_z_m"):
        value = g[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"geometry.{key} 必须是有限数值")
        if key in positive and value <= 0:
            raise ValueError(f"geometry.{key} 必须大于 0")
    length, radius = g["body_length_m"], g["body_radius_m"]
    if length <= 2 * radius:
        raise ValueError("机身长度必须大于直径")
    if g["fin_root_length_m"] > length:
        raise ValueError("尾翼根弦长度不可超出机身长度")
    if g["fin_thickness_m"] >= min(2 * radius, g["fin_span_m"]):
        raise ValueError("尾翼厚度必须小于机身直径和尾翼展长")
    if abs(g["nacelle_z_m"]) + g["nacelle_length_m"] / 2 > length / 2:
        raise ValueError("动力舱不得超出机头到机尾的轴向包络")
    if abs(g["rotor_z_m"]) + 0.014 > length / 2:
        raise ValueError("桨毂不得超出机头到机尾的轴向包络")
    prop_radius = g["propeller_diameter_m"] / 2
    if prop_radius <= 0.034:
        raise ValueError("桨直径过小，无法容纳当前桨根和轮毂")
    if g["diagonal_wheelbase_m"] / math.sqrt(2) - 2 * prop_radius < 0.005:
        raise ValueError("相邻桨盘需至少保留 5 mm 净空")
    if g["diagonal_wheelbase_m"] / 2 - prop_radius - radius < 0.005:
        raise ValueError("桨盘与机身需至少保留 5 mm 径向净空")
    if g["nacelle_radius_m"] > prop_radius:
        raise ValueError("动力舱半径必须小于桨半径")
    # 下置桨需给桨毂留出净空；翼根尾端在机身内，外露翼面位于桨上方。
    nacelle_bottom = g["nacelle_z_m"] - g["nacelle_length_m"] / 2
    if g["rotor_z_m"] + 0.019 > nacelle_bottom + 1e-12:
        raise ValueError("下置桨毂与动力舱底部需至少保留 5 mm 净空")
    span = radius + g["fin_span_m"]
    if abs(span - g["diagonal_wheelbase_m"] / 2) > g["nacelle_radius_m"]:
        raise ValueError("尾翼外端必须连接到动力舱")
    tip_bottom = g["fin_tip_z_m"] - g["fin_tip_chord_m"] / 2
    tip_top = g["fin_tip_z_m"] + g["fin_tip_chord_m"] / 2
    root_top = -length / 2 + g["fin_root_length_m"]
    if tip_top > root_top or tip_bottom < -length / 2 or tip_top > length / 2:
        raise ValueError("翼尖弦长必须位于机身轴向包络和翼根上端以内")
    shoulder = g["rotor_z_m"] + 0.019
    if tip_bottom < shoulder:
        raise ValueError("外露翼面下缘需位于桨毂上方并保留 5 mm 净空")
    # 计算桨盘最内侧与翼下缘的距离，而非只检查翼尖。
    inner_radius = g["diagonal_wheelbase_m"] / 2 - prop_radius
    lower_at_disc = shoulder + (tip_bottom - shoulder) * (inner_radius - radius) / g["fin_span_m"]
    if lower_at_disc < g["rotor_z_m"] + 0.0085:
        raise ValueError("尾翼与桨叶旋转包络需至少保留 5 mm 净空")


# ---------------------------------------------------------------- STL 输出
def write_stl(path: Path, triangles: np.ndarray) -> None:
    """写出 ASCII STL，triangles 形状为 (n, 3, 3)。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"solid {path.stem}"]
    for triangle in triangles:
        normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
        norm = float(np.linalg.norm(normal))
        normal = normal / norm if norm > 1e-12 else np.array([0.0, 0.0, 1.0])
        lines.append(
            "  facet normal %.6e %.6e %.6e" % (normal[0], normal[1], normal[2])
        )
        lines.append("    outer loop")
        for vertex in triangle:
            lines.append(
                "      vertex %.6e %.6e %.6e" % (vertex[0], vertex[1], vertex[2])
            )
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {path.stem}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ---------------------------------------------------------------- 几何工具
def revolve_around_x(
    profile: list[tuple[float, float]], segments: int = 64
) -> np.ndarray:
    """把 x-r 剖面绕 x 轴旋转成三角面片；r 为 0 处会自动省略退化面片。"""
    angles = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    cos_a, sin_a = np.cos(angles), np.sin(angles)

    rings = [
        np.stack([np.full(segments, x), r * cos_a, r * sin_a], axis=1)
        for x, r in profile
    ]

    triangles = []
    for index in range(len(rings) - 1):
        lower, upper = rings[index], rings[index + 1]
        for j in range(segments):
            k = (j + 1) % segments
            a, b = lower[j], lower[k]
            c, d = upper[j], upper[k]
            if np.allclose(a, b):
                triangles.append(np.array([a, d, c]))
            elif np.allclose(c, d):
                triangles.append(np.array([a, b, c]))
            else:
                triangles.append(np.array([a, b, c]))
                triangles.append(np.array([b, d, c]))
    return np.array(triangles)


def revolve_profile(
    x_start: float,
    x_end: float,
    radius: float,
    nose_ratio: float = 0.28,
    tail_ratio: float = 0.22,
    segments: int = 64,
) -> np.ndarray:
    """生成“锥形头 + 等直段 + 收缩尾”的流线型回转体。"""
    length = x_end - x_start
    body_start = x_start + nose_ratio * length
    body_end = x_end - tail_ratio * length

    profile: list[tuple[float, float]] = [(x_start, 0.0)]
    for i in range(1, 11):
        t = i / 10.0
        profile.append(
            (x_start + nose_ratio * length * t, radius * math.sin(t * math.pi / 2.0))
        )
    profile.append((body_end, radius))
    for i in range(1, 11):
        t = i / 10.0
        profile.append(
            (body_end + tail_ratio * length * t, (0.0 if i == 10 else radius * math.cos(t * math.pi / 2.0)))
        )
    return revolve_around_x(profile, segments)


def prism_from_polygon(
    polygon_xz: list[tuple[float, float]], thickness: float
) -> np.ndarray:
    """把 x-z 多边形沿 y 拉成棱柱；首顶点需能连接各顶点而不越出边界。"""
    half = thickness / 2.0
    front = [np.array([x, -half, z]) for x, z in polygon_xz]
    back = [np.array([x, half, z]) for x, z in polygon_xz]

    triangles = []
    for i in range(1, len(front) - 1):
        triangles.append(np.array([front[0], front[i], front[i + 1]]))
        triangles.append(np.array([back[0], back[i + 1], back[i]]))
    for i in range(len(front)):
        j = (i + 1) % len(front)
        triangles.append(np.array([front[i], back[i], back[j]]))
        triangles.append(np.array([front[i], back[j], front[j]]))
    return np.array(triangles)


def tapered_blade(
    root_center: np.ndarray,
    root_half: tuple[float, float],
    tip_center: np.ndarray,
    tip_half: tuple[float, float],
) -> np.ndarray:
    """生成根部到尖部收缩的桨叶，half 为 (弦向半宽, 厚度半宽)。"""

    def ring(center: np.ndarray, half: tuple[float, float]) -> list[np.ndarray]:
        chord, thick = half
        return [
            center + np.array([+chord, 0.0, +thick]),
            center + np.array([-chord, 0.0, +thick]),
            center + np.array([-chord, 0.0, -thick]),
            center + np.array([+chord, 0.0, -thick]),
        ]

    root, tip = ring(root_center, root_half), ring(tip_center, tip_half)
    triangles = []
    for i in range(4):
        j = (i + 1) % 4
        triangles.append(np.array([root[i], root[j], tip[j]]))
        triangles.append(np.array([root[i], tip[j], tip[i]]))
    triangles.append(np.array([root[0], root[1], root[2]]))
    triangles.append(np.array([root[0], root[2], root[3]]))
    triangles.append(np.array([tip[0], tip[2], tip[1]]))
    triangles.append(np.array([tip[0], tip[3], tip[2]]))
    mesh = np.array(triangles)
    center = (root_center + tip_center) / 2.0
    normals = np.cross(mesh[:, 1] - mesh[:, 0], mesh[:, 2] - mesh[:, 0])
    inward = np.einsum("ij,ij->i", normals, mesh.mean(axis=1) - center) < 0
    mesh[inward] = mesh[inward][:, [0, 2, 1]]
    return mesh


# ---------------------------------------------------------------- 部件
def build_fuselage(g: dict) -> np.ndarray:
    """轴向长度 L，尾部 -z、尖头 +z。"""
    length = g["body_length_m"]
    return revolve_profile(-length / 2, length / 2, g["body_radius_m"],
                           nose_ratio=0.12, tail_ratio=0.27) @ X_AXIS_TO_Z_AXIS.T


def build_nacelle(g: dict) -> np.ndarray:
    """短电机整流罩：底部平面电机座，顶部渐尖；局部原点居中。"""
    length, radius = g["nacelle_length_m"], g["nacelle_radius_m"]
    bottom = -length / 2
    profile = [(bottom, 0.), (bottom, radius), (bottom + 0.27 * length, radius)]
    for i in range(1, 17):
        t = i / 16
        profile.append((bottom + (0.27 + 0.73 * t) * length,
                        0. if i == 16 else radius * math.cos(t * math.pi / 2)))
    return revolve_around_x(profile) @ X_AXIS_TO_Z_AXIS.T


def build_tail(g: dict) -> np.ndarray:
    """四片长三角翼沿电机对角方向连接低位动力舱，外露下缘避开下置桨。"""
    bottom = -g["body_length_m"] / 2
    radius = g["body_radius_m"]
    span = radius + g["fin_span_m"]
    tip_bottom = g["fin_tip_z_m"] - g["fin_tip_chord_m"] / 2
    tip_top = g["fin_tip_z_m"] + g["fin_tip_chord_m"] / 2
    # 以翼根上端为三角扇中心，正确覆盖带尾部收尖的凹多边形。
    fin = prism_from_polygon([
        (0., bottom + g["fin_root_length_m"]), (0., bottom),
        (radius, g["rotor_z_m"] + 0.019),
        (span, tip_bottom), (span, tip_top),
    ], g["fin_thickness_m"])
    fins = []
    for angle in (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4):
        c, s = math.cos(angle), math.sin(angle)
        rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        fins.append(fin @ rotation.T)
    return np.concatenate(fins)


def build_propeller(g: dict) -> np.ndarray:
    """两叶桨：桨叶角点保持在配置的旋转半径内。"""
    radius = g["propeller_diameter_m"] / 2
    tip_chord = min(0.017, radius * 0.14)
    tip_y = math.sqrt(radius ** 2 - tip_chord ** 2)
    hub = revolve_profile(-0.014, 0.014, 0.014) @ X_AXIS_TO_Z_AXIS.T
    blades = [tapered_blade(
        root_center=np.array([0., sign * 0.014, 0.]),
        root_half=(0.030, 0.0035),
        tip_center=np.array([0., sign * tip_y, 0.]),
        tip_half=(tip_chord, 0.0016),
    ) for sign in (1., -1.)]
    return np.concatenate([hub, *blades])


def build_meshes(config: dict) -> dict[str, np.ndarray]:
    """从完整参数字典构建三角网格，不写文件。"""
    g = config["geometry"]
    validate_geometry(g)
    return {
        "fuselage.stl": build_fuselage(g),
        "nacelle.stl": build_nacelle(g),
        "tail.stl": build_tail(g),
        "propeller_10in.stl": build_propeller(g),
    }


def generate(config: dict, mesh_dir: str | Path | None = None) -> dict[str, Path]:
    """根据完整参数字典输出 STL；输出目录默认模型 meshes/。"""
    target = Path(mesh_dir) if mesh_dir is not None else MESH_DIR
    paths = {}
    for name, triangles in build_meshes(config).items():
        path = target / name
        write_stl(path, triangles)
        paths[name] = path
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=MODEL_DIR / "parameters.json")
    args = parser.parse_args()
    for name, path in generate(load_config(args.config)).items():
        print(f"{name}: {path}")


if __name__ == "__main__":
    main()
