#!/usr/bin/env python3
"""生成火箭穿越机与 A2416A 捷联导引头的静态造型资产。"""
from __future__ import annotations

import argparse
import configparser
import copy
import importlib.util
import math
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
ROCKET = REPO / "models/vehicles/rocket_racing_quad"
SEEKER = REPO / "models/gimbals/jielian-A2416A"
NAME = "rocket_jielian_a2416a"


def node(parent, tag, value=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if value is not None:
        child.text = str(value)
    return child


def write_xml(path: Path, root: ET.Element) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def configuration() -> dict:
    parser = configparser.ConfigParser(interpolation=None, strict=True)
    with (ROOT / "parameters.ini").open(encoding="utf-8") as stream:
        parser.read_file(stream)
    result = {
        "assembly": {key: parser.getfloat("assembly", key) for key in parser["assembly"]},
        "component": dict(parser["component"]),
        "dynamic": {
            "airframe_id": parser.getint("dynamic", "airframe_id"),
            "topic_prefix": parser.get("dynamic", "topic_prefix"),
        },
        "view": {},
        "px4": {"parameters": {key.upper(): float(value) for key, value in parser["px4"].items()}},
        "flight_validation": {},
    }
    for key, value in parser["view"].items():
        result["view"][key] = [float(part.strip()) for part in value.split(",")] if key.endswith("pose") else float(value)
    for key, value in parser["flight_validation"].items():
        if value.lower() in {"true", "false"}:
            result["flight_validation"][key] = parser.getboolean("flight_validation", key)
        else:
            result["flight_validation"][key] = float(value)
    assembly = result["assembly"]
    if not math.isclose(assembly["overall_length_m"], 0.590, abs_tol=1e-12):
        raise ValueError("当前装配必须保持 590 mm 总长")
    if not math.isclose(assembly["mount_z_m"] + assembly["seeker_height_m"], 0.380, abs_tol=1e-12):
        raise ValueError("导引头顶部必须保持 z=+0.380 m")
    if not assembly["shoulder_start_z_m"] < assembly["mount_z_m"]:
        raise ValueError("肩部起点必须低于导引头安装面")
    return result


def rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def transform(text):
    values = [float(value) for value in (text or "0 0 0 0 0 0").split()]
    if len(values) != 6:
        raise ValueError("pose 必须包含六个数值")
    matrix = np.eye(4)
    matrix[:3, :3] = rotation(values[3:])
    matrix[:3, 3] = values[:3]
    return matrix


def pose_text(matrix):
    r = matrix[:3, :3]
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if abs(math.cos(pitch)) > 1e-8:
        roll, yaw = math.atan2(r[2, 1], r[2, 2]), math.atan2(r[1, 0], r[0, 0])
    else:
        roll, yaw = 0.0, math.atan2(-r[0, 1], r[1, 1])
    return " ".join(f"{value:.12g}" for value in (*matrix[:3, 3], roll, pitch, yaw))


def read_stl(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    if len(raw) >= 84:
        count = struct.unpack_from("<I", raw, 80)[0]
        if len(raw) == 84 + count * 50:
            dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
            return np.frombuffer(raw, dtype=dtype, count=count, offset=84)["vertices"].astype(float)
    vertices = [list(map(float, line.split()[1:])) for line in raw.decode("ascii").splitlines()
                if line.strip().startswith("vertex ")]
    result = np.asarray(vertices).reshape(-1, 3, 3)
    if not len(result) or not np.isfinite(result).all():
        raise ValueError(f"无效 STL：{path}")
    return result


def source_mesh(visual_name: str):
    model = ET.parse(ROCKET / "model.sdf").getroot().find("model")
    matches = [(link, visual) for link in model.findall("link")
               for visual in link.findall("visual") if visual.get("name") == visual_name]
    if len(matches) != 1:
        raise ValueError(f"基础模型必须有唯一视觉 {visual_name}")
    link, visual = matches[0]
    mesh = visual.find("geometry/mesh")
    uri = mesh.findtext("uri")
    path = ROCKET / uri.split("/", 3)[3] if uri.startswith("model://rocket_racing_quad/") else Path(uri)
    scale = np.fromstring(mesh.findtext("scale", "1 1 1"), sep=" ")
    pose = transform(link.findtext("pose")) @ transform(visual.findtext("pose"))
    vertices = read_stl(path) * scale
    return vertices @ pose[:3, :3].T + pose[:3, 3], pose, scale


def source_geometry():
    fuselage, _, _ = source_mesh("fuselage_visual")
    tail, _, _ = source_mesh("tail_visual")
    return {
        "min_z_m": float(fuselage[:, :, 2].min()),
        "max_z_m": float(fuselage[:, :, 2].max()),
        "length_m": float(fuselage[:, :, 2].max() - fuselage[:, :, 2].min()),
        "fin_max_z_m": float(tail[:, :, 2].max()),
    }


def cut_fuselage() -> np.ndarray:
    config = configuration()["assembly"]
    cut_z = config["shoulder_start_z_m"]
    mount_z = config["mount_z_m"]
    radius = config["seeker_diameter_m"] / 2.0
    vertices, source_pose, source_scale = source_mesh("fuselage_visual")
    triangles, boundary = [], {}
    for triangle in vertices:
        polygon = []
        for index, point in enumerate(triangle):
            following = triangle[(index + 1) % 3]
            inside, next_inside = point[2] <= cut_z, following[2] <= cut_z
            if inside:
                polygon.append(point)
            if inside != next_inside:
                crossing = point + (following - point) * ((cut_z - point[2]) / (following[2] - point[2]))
                crossing[2] = cut_z
                polygon.append(crossing)
        for point in polygon:
            if abs(point[2] - cut_z) < 1e-10:
                boundary[tuple(np.round(point, 11))] = point
        for index in range(1, len(polygon) - 1):
            candidate = np.asarray([polygon[0], polygon[index], polygon[index + 1]])
            if np.linalg.norm(np.cross(candidate[1] - candidate[0], candidate[2] - candidate[0])) > 1e-14:
                triangles.append(candidate)
    if len(boundary) < 3:
        raise ValueError("机身裁切平面没有形成闭合轮廓")
    ring = np.asarray(sorted(boundary.values(), key=lambda point: math.atan2(point[1], point[0])))
    radii = np.linalg.norm(ring[:, :2], axis=1)
    upper = ring.copy()
    upper[:, :2] *= (radius / radii)[:, None]
    upper[:, 2] = mount_z
    center = np.array([0.0, 0.0, mount_z])
    for index in range(len(ring)):
        following = (index + 1) % len(ring)
        triangles.extend(([ring[index], ring[following], upper[following]],
                          [ring[index], upper[following], upper[index]],
                          [center, upper[index], upper[following]]))
    return ((np.asarray(triangles) - source_pose[:3, 3]) @ source_pose[:3, :3]) / source_scale


def seeker_model():
    path = SEEKER / "model.sdf"
    if not path.is_file():
        raise FileNotFoundError(f"请先生成基础导引头模型：{path}")
    return ET.parse(path).getroot().find("model")


def build_model() -> ET.Element:
    config = configuration()
    mount_z = config["assembly"]["mount_z_m"]
    root = ET.Element("sdf", version="1.10")
    model = node(root, "model", name=NAME)
    node(model, "static", "true")
    destination = node(model, "link", name="appearance_link")
    rocket = ET.parse(ROCKET / "model.sdf").getroot().find("model")
    for source_link in rocket.findall("link"):
        link_pose = transform(source_link.findtext("pose"))
        for source_visual in source_link.findall("visual"):
            visual = copy.deepcopy(source_visual)
            visual.set("name", f"rocket_{source_link.get('name')}_{source_visual.get('name')}")
            source_pose = transform(source_visual.findtext("pose"))
            set_pose = visual.find("pose")
            if set_pose is None:
                set_pose = ET.SubElement(visual, "pose")
            set_pose.text = pose_text(link_pose @ source_pose)
            mesh = visual.find("geometry/mesh")
            if source_visual.get("name") == "fuselage_visual":
                mesh.find("uri").text = f"model://{NAME}/meshes/fuselage_cut.stl"
            elif mesh is not None:
                uri = mesh.findtext("uri")
                if uri.startswith("model://rocket_racing_quad/meshes/"):
                    mesh.find("uri").text = f"model://{NAME}/meshes/rocket_{uri.rsplit('/', 1)[1]}"
            destination.append(visual)
    translate = np.eye(4)
    translate[2, 3] = mount_z
    for source_link in seeker_model().findall("link"):
        link_pose = translate @ transform(source_link.findtext("pose"))
        for source_visual in source_link.findall("visual"):
            visual = copy.deepcopy(source_visual)
            visual.set("name", f"seeker_{source_link.get('name')}_{source_visual.get('name')}")
            set_pose = visual.find("pose")
            if set_pose is None:
                set_pose = ET.SubElement(visual, "pose")
            set_pose.text = pose_text(link_pose @ transform(source_visual.findtext("pose")))
            destination.append(visual)
    return root


def marker(world, spawn_z):
    model = node(world, "model", name="JIELIAN_A2416A_LOCATION_MARKER")
    node(model, "static", "true")
    link = node(model, "link", name="arrow")
    height = spawn_z + 0.36
    for name, pose, size in (
        ("shaft", f"-.20 0 {height} 0 0 0", ".12 .009 .009"),
        ("head_upper", f"-.125 0 {height + .01} 0 .7853981634 0", ".04 .009 .009"),
        ("head_lower", f"-.125 0 {height - .01} 0 -.7853981634 0", ".04 .009 .009"),
    ):
        visual = node(link, "visual", name=name)
        node(visual, "pose", pose)
        node(node(node(visual, "geometry"), "box"), "size", size)
        material = node(visual, "material")
        node(material, "ambient", "1 .2 .02 1")
        node(material, "diffuse", "1 .2 .02 1")


def build_world(close=False) -> ET.Element:
    config = configuration()
    view = config["view"]
    geometry = source_geometry()
    spawn_z = -geometry["min_z_m"] + 0.015
    root = ET.Element("sdf", version="1.10")
    world = node(root, "world", name=f"{NAME}_preview")
    node(world, "gravity", "0 0 -9.8")
    physics = node(world, "physics", name="physics", type="ignored")
    node(physics, "max_step_size", ".004")
    node(physics, "real_time_factor", "1")
    for filename, class_name in (("physics", "Physics"), ("user-commands", "UserCommands"),
                                 ("scene-broadcaster", "SceneBroadcaster")):
        node(world, "plugin", filename=f"gz-sim-{filename}-system", name=f"gz::sim::systems::{class_name}")
    scene = node(world, "scene")
    node(scene, "ambient", ".7 .7 .7 1")
    node(scene, "background", ".84 .88 .93 1")
    node(scene, "grid", "false")
    gui = node(world, "gui", fullscreen="false")
    render = node(gui, "plugin", filename="MinimalScene", name="3D View")
    node(render, "engine", "ogre2")
    node(render, "scene", "scene")
    pose = view["close_camera_pose" if close else "camera_pose"]
    node(render, "camera_pose", " ".join(str(value) for value in pose))
    clip = node(render, "camera_clip")
    node(clip, "near", view["near_clip_m"])
    node(clip, "far", view["far_clip_m"])
    for filename, label in (("GzSceneManager", "Scene Manager"),
                            ("InteractiveViewControl", "Interactive view control"),
                            ("CameraTracking", "Camera Tracking"),
                            ("EntityTree", "Entity tree"), ("WorldControl", "World control")):
        node(gui, "plugin", filename=filename, name=label)
    light = node(world, "light", name="sun", type="directional")
    node(light, "pose", "0 0 3 0 0 0")
    node(light, "diffuse", ".8 .8 .8 1")
    node(light, "direction", "-.4 .5 -1")
    include = node(world, "include")
    node(include, "uri", f"model://{NAME}")
    node(include, "name", "ROCKET_JIELIAN_A2416A")
    node(include, "pose", f"0 0 {spawn_z:.12g} 0 0 0")
    floor = node(world, "model", name="preview_floor")
    node(floor, "static", "true")
    visual = node(node(floor, "link", name="floor"), "visual", name="floor")
    plane = node(node(visual, "geometry"), "plane")
    node(plane, "normal", "0 0 1")
    node(plane, "size", "4 4")
    marker(world, spawn_z)
    return root


def mesh_tools():
    spec = importlib.util.spec_from_file_location("rocket_mesh_tools", ROCKET / "tools/generate_meshes.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate(output_dir: Path = ROOT) -> None:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "meshes").mkdir(exist_ok=True)
    mesh_tools().write_stl(output_dir / "meshes/fuselage_cut.stl", cut_fuselage())
    rocket = ET.parse(ROCKET / "model.sdf").getroot()
    for uri in rocket.findall(".//visual/geometry/mesh/uri"):
        if uri.text.startswith("model://rocket_racing_quad/meshes/") and not uri.text.endswith("fuselage.stl"):
            source = ROCKET / uri.text.split("/", 3)[3]
            shutil.copy2(source, output_dir / "meshes" / f"rocket_{source.name}")
    write_xml(output_dir / "model.sdf", build_model())
    write_xml(output_dir / "tests/world.sdf", build_world(close=False))
    write_xml(output_dir / "tests/closeup.sdf", build_world(close=True))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT)
    args = parser.parse_args()
    generate(args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
