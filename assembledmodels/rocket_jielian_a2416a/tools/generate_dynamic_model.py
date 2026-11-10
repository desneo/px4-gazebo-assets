#!/usr/bin/env python3
"""从当前火箭基础资产生成固定 A2416A 导引头的动态 PX4 装配。"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
ROCKET = REPO / "models/vehicles/rocket_racing_quad"
SEEKER = REPO / "models/gimbals/jielian-A2416A"
NAME = "rocket_jielian_a2416a"


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


appearance = load_module("rocket_jielian_appearance", ROOT / "tools/generate_model.py")
generic = load_module("rocket_aisheng_dynamic_helpers",
                      REPO / "assembledmodels/rocket_aisheng_seeker/tools/generate_dynamic_model.py")
generic.NAME = NAME
native = generic.native


def set_text(parent, tag, value):
    element = parent.find(tag)
    if element is None:
        element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element


def settings(airframe_id=None, topic_prefix=None):
    config = appearance.configuration()
    assembly = config["assembly"]
    source = ET.parse(ROCKET / "model.sdf").getroot()
    airframes = sorted((ROCKET / "px4").glob("*_gz_rocket_racing_quad"))
    if len(airframes) != 1:
        raise ValueError("火箭基础模型必须包含唯一 PX4 airframe")
    text = airframes[0].read_text(encoding="utf-8")
    parameters = native.parameter_values(text)
    selected_airframe = config["dynamic"]["airframe_id"] if airframe_id is None else airframe_id
    if isinstance(selected_airframe, bool) or not isinstance(selected_airframe, int) or selected_airframe <= 0:
        raise ValueError("airframe_id 必须为正整数")
    prefix = config["dynamic"]["topic_prefix"] if topic_prefix is None else topic_prefix
    if not isinstance(prefix, str) or not re.fullmatch(r"/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*", prefix):
        raise ValueError("topic_prefix 必须为无末尾斜线的绝对 Gazebo 话题")
    defaults = json.loads((ROCKET / "parameters.json").read_text(encoding="utf-8"))
    return {
        "config": config,
        "source_model": source,
        "source_airframe": airframes[0],
        "airframe_text": text,
        "px4_parameters": parameters,
        "px4_overrides": config["px4"]["parameters"],
        "flight_test": defaults["flight_test"],
        "imu_wait_timeout_s": defaults["simulation"]["imu_wait_timeout_s"],
        "airframe_id": selected_airframe,
        "topic_prefix": prefix,
        "mount_z_m": assembly["mount_z_m"],
        "retained_body_length_m": assembly["mount_z_m"] - appearance.source_geometry()["min_z_m"],
    }


def build_model(config):
    root = copy.deepcopy(config["source_model"])
    model = root.find("model")
    model.set("name", NAME)
    model.set("canonical_link", "base_link")
    set_text(model, "static", "false")
    geometry = appearance.source_geometry()
    base = model.find("link[@name='base_link']")
    collision = base.find("collision[@name='body_collision']")
    pose = [float(value) for value in collision.findtext("pose", "0 0 0 0 0 0").split()]
    pose[2] = (geometry["min_z_m"] + config["mount_z_m"]) / 2.0
    set_text(collision, "pose", " ".join(f"{value:.12g}" for value in pose))
    set_text(collision.find("geometry/cylinder"), "length", f"{config['retained_body_length_m']:.12g}")
    base.find("visual[@name='fuselage_visual']/geometry/mesh/uri").text = f"model://{NAME}/meshes/fuselage_cut.stl"

    component = ET.parse(SEEKER / "model.sdf").getroot().find("model")
    links = component.findall("link")
    if len(links) != 1 or links[0].get("name") != "seeker_link":
        raise ValueError("A2416A 基础模型必须只含 seeker_link")
    seeker_link = copy.deepcopy(links[0])
    transform = appearance.transform(seeker_link.findtext("pose"))
    transform[2, 3] += config["mount_z_m"]
    set_text(seeker_link, "pose", appearance.pose_text(transform))
    camera = seeker_link.find("sensor[@name='camera']")
    if camera is None:
        raise ValueError("A2416A 基础模型缺少 camera 传感器")
    set_text(camera, "topic", config["topic_prefix"] + "/image")
    set_text(camera.find("camera"), "camera_info_topic", config["topic_prefix"] + "/camera_info")
    model.append(seeker_link)

    plugins = component.findall("plugin")
    if len(plugins) != 1 or plugins[0].get("name") != "jielian_a2416a::StrapdownController":
        raise ValueError("A2416A 基础模型必须包含唯一捷联控制器")
    plugin = copy.deepcopy(plugins[0])
    for key in ("command", "telemetry", "command_result"):
        set_text(plugin, key + "_topic", config["topic_prefix"] + "/" + key)
    set_text(plugin, "image_topic", config["topic_prefix"] + "/image")
    set_text(plugin, "camera_link", "seeker_link")
    model.append(plugin)

    fixed = ET.SubElement(model, "joint", name="seeker_mount_joint", type="fixed")
    set_text(fixed, "parent", "base_link")
    set_text(fixed, "child", "seeker_link")
    for uri in model.findall(".//mesh/uri"):
        prefix = "model://rocket_racing_quad/meshes/"
        if uri.text.startswith(prefix):
            uri.text = f"model://{NAME}/meshes/rocket_{uri.text[len(prefix):]}"
    return root


def source_fingerprints():
    paths = [ROOT / "parameters.ini", ROOT / "tools/generate_model.py", Path(__file__),
             ROCKET / "model.sdf", ROCKET / "parameters.json", ROCKET / "tools/prepare_runtime.py",
             SEEKER / "model.sdf", SEEKER / "parameters.ini"]
    paths.extend(sorted((ROCKET / "meshes").glob("*")))
    return {str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in paths if path.is_file()}


def component_world():
    root = ET.parse(ROCKET / "tests/world.sdf").getroot()
    world = root.find("world")
    world.set("name", NAME + "_test")
    include = world.find("include[uri='model://rocket_racing_quad']")
    if include is None:
        raise ValueError("火箭自验证世界缺少基础模型 include")
    include.find("uri").text = f"model://{NAME}"
    # 飞行验收使用固定飞机真值夹具。光轴初始沿 +Z，目标位于发射点上方；
    # 该实体只用于检测/跟踪链路，不参与遮挡或外部场景集成。
    target = ET.SubElement(world, "model", name="target_aircraft")
    ET.SubElement(target, "static").text = "true"
    ET.SubElement(target, "pose").text = "0 0 500 0 0 0"
    link = ET.SubElement(target, "link", name="target_link")
    for tag in ("visual", "collision"):
        shape = ET.SubElement(link, tag, name=f"target_{tag}")
        geometry = ET.SubElement(shape, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = "6 3 1.5"
        if tag == "visual":
            material = ET.SubElement(shape, "material")
            ET.SubElement(material, "ambient").text = "1 .18 .02 1"
            ET.SubElement(material, "diffuse").text = "1 .18 .02 1"
    return root


def model_config():
    root = ET.Element("model")
    ET.SubElement(root, "name").text = "Rocket Jielian A2416A"
    ET.SubElement(root, "version").text = "1.0"
    sdf = ET.SubElement(root, "sdf", version="1.10")
    sdf.text = "model.sdf"
    ET.SubElement(root, "description").text = "Dynamic rocket with fixed A2416A strapdown seeker"
    return root


def generate(output_dir, *, airframe_id=None, topic_prefix=None):
    output = Path(output_dir).resolve()
    if output == ROOT.resolve() or ROOT.resolve().is_relative_to(output) or output.is_relative_to((REPO / "models").resolve()):
        raise ValueError("动态输出不得覆盖静态装配或基础组件")
    before = source_fingerprints()
    config = settings(airframe_id, topic_prefix)
    root = build_model(config)
    world = component_world()
    gravity = math.sqrt(sum(float(value) ** 2 for value in world.findtext("world/gravity").split()))
    hover = generic.hover_configuration(root.find("model"), config["px4_parameters"], gravity)
    reference = generic.px4_reference_configuration(
        config["source_model"].find("model"), root.find("model"), config["px4_parameters"])
    airframe_text = generic.assembly_airframe(
        config["airframe_text"], hover["px4_hover_thrust"], reference["parameters"], config["px4_overrides"])
    airframe_text = airframe_text.replace("Gazebo rocket with aisheng seeker",
                                          "Gazebo rocket with A2416A strapdown seeker")
    parameters = native.parameter_values(airframe_text)
    model = root.find("model")
    manifest = {
        "model_name": NAME,
        "airframe_id": config["airframe_id"],
        "topic_prefix": config["topic_prefix"],
        "image_topic": config["topic_prefix"] + "/image",
        "camera_info_topic": config["topic_prefix"] + "/camera_info",
        "mount_z_m": config["mount_z_m"],
        "overall_length_m": config["config"]["assembly"]["overall_length_m"],
        "seeker_mass_kg": native.number(model.find("link[@name='seeker_link']/inertial/mass").text),
        "total_mass_kg": hover["total_mass_kg"],
        "px4_reference": reference,
        "source_sha256": before,
    }
    world_element = world.find("world")
    spawn_pose = world_element.findtext(f"include[uri='model://{NAME}']/pose")
    if not spawn_pose:
        raise ValueError("动态验收世界缺少A2416A装配机出生位姿")
    snapshot = {
        "model": generic.model_snapshot(model),
        "px4": {"airframe_id": config["airframe_id"], "parameters": parameters},
        "flight_test": config["flight_test"],
        "flight_validation": config["config"]["flight_validation"],
        "simulation": {"imu_wait_timeout_s": config["imu_wait_timeout_s"],
                       "world_name": world_element.get("name"),
                       "spawn_pose": spawn_pose},
        "assembly": manifest,
    }
    if before != source_fingerprints():
        raise RuntimeError("生成期间基础资产发生变化，请重新生成")
    output.mkdir(parents=True, exist_ok=True)
    (output / "meshes").mkdir(exist_ok=True)
    appearance.mesh_tools().write_stl(output / "meshes/fuselage_cut.stl", appearance.cut_fuselage())
    for uri in config["source_model"].findall(".//mesh/uri"):
        prefix = "model://rocket_racing_quad/meshes/"
        if uri.text.startswith(prefix) and not uri.text.endswith("fuselage.stl"):
            source = ROCKET / "meshes" / uri.text[len(prefix):]
            shutil.copy2(source, output / "meshes" / f"rocket_{source.name}")
    appearance.write_xml(output / "model.sdf", root)
    appearance.write_xml(output / "tests/world.sdf", world)
    model_cfg = model_config()
    ET.indent(model_cfg, space="  ")
    ET.ElementTree(model_cfg).write(output / "model.config", encoding="utf-8", xml_declaration=True)
    (output / "px4").mkdir(exist_ok=True)
    (output / "px4" / f"{config['airframe_id']}_gz_{NAME}").write_text(airframe_text, encoding="utf-8")
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (output / "parameters.json").write_text(json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "runtime" / NAME)
    parser.add_argument("--airframe-id", type=int)
    parser.add_argument("--topic-prefix")
    args = parser.parse_args()
    print(json.dumps(generate(args.output_dir, airframe_id=args.airframe_id,
                              topic_prefix=args.topic_prefix), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
