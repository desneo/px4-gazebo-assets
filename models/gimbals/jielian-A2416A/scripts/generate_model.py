#!/usr/bin/env python3
"""由 parameters.ini 确定性生成 A2416A 模型和近景验证世界。"""

from __future__ import annotations

import argparse
import configparser
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODEL_URI = "model://jielian-A2416A"


def add(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def number(value):
    return format(float(value), ".15g")


def parse_list(value):
    result = []
    for item in value.split(","):
        item = item.strip()
        if item and item not in result:
            result.append(item)
    return result


def load_config(path=ROOT / "parameters.ini"):
    cfg = configparser.ConfigParser(interpolation=None)
    if not cfg.read(path, encoding="utf-8"):
        raise ValueError(f"无法读取配置：{path}")
    required = ("model", "optics", "tracking", "protocol", "viewer", "demo")
    if any(section not in cfg for section in required):
        raise ValueError("parameters.ini 缺少必要分区")
    model = cfg["model"]
    optics = cfg["optics"]
    tracking = cfg["tracking"]
    values = {
        "name": model.get("name", "").strip(),
        "diameter": model.getfloat("diameter_m"),
        "height": model.getfloat("height_m"),
        "mass": model.getfloat("mass_kg"),
        "width": optics.getint("width_px"),
        "height_px": optics.getint("height_px"),
        "fps": optics.getfloat("fps"),
        "hfov": optics.getfloat("hfov_deg"),
        "vfov": optics.getfloat("vfov_deg"),
        "fx": optics.getfloat("fx_px"),
        "fy": optics.getfloat("fy_px"),
        "cx": optics.getfloat("cx_px"),
        "cy": optics.getfloat("cy_px"),
        "near": optics.getfloat("near_clip_m"),
        "far": optics.getfloat("far_clip_m"),
        "targets": parse_list(tracking.get("target_names", "")),
        "measurement_hz": tracking.getfloat("measurement_hz"),
        "auto_range": tracking.getfloat("auto_detection_range_m"),
        "manual_range": tracking.getfloat("manual_tracking_range_m"),
        "lost_timeout": tracking.getfloat("lost_timeout_s"),
        "max_targets": tracking.getint("max_targets"),
        "misalignment_limit": tracking.getfloat("misalignment_limit_deg"),
        "crosshair_speed": tracking.getfloat("crosshair_max_speed_px_s"),
        "crosshair_timeout": tracking.getfloat("crosshair_timeout_s"),
        "gate_min": tracking.getfloat("gate_scale_min"),
        "gate_max": tracking.getfloat("gate_scale_max"),
        "command_topic": cfg["protocol"].get("command_topic"),
        "telemetry_topic": cfg["protocol"].get("telemetry_topic"),
        "result_topic": cfg["protocol"].get("command_result_topic"),
        "viewer_near": cfg["viewer"].getfloat("near_clip_m"),
        "sensitivity": cfg["viewer"].getfloat("sensitivity"),
        "world_name": cfg["demo"].get("world_name"),
        "marker_name": cfg["demo"].get("marker_name"),
    }
    finite_positive = ("diameter", "height", "mass", "fps", "fx", "fy", "near",
                       "far", "measurement_hz", "auto_range", "manual_range",
                       "lost_timeout", "misalignment_limit", "crosshair_speed",
                       "crosshair_timeout", "gate_min", "gate_max")
    if not values["name"] or any(not math.isfinite(values[key]) or values[key] <= 0
                                 for key in finite_positive):
        raise ValueError("模型、光学或跟踪参数必须为有限正数")
    if values["width"] <= 0 or values["height_px"] <= 0 or values["max_targets"] != 4:
        raise ValueError("图像尺寸必须为正，max_targets 必须为 4")
    if not 0 < values["hfov"] < 180 or not 0 < values["vfov"] < 180:
        raise ValueError("视场角必须位于 (0, 180)")
    if values["far"] <= values["near"] or values["manual_range"] < values["auto_range"]:
        raise ValueError("裁剪距离或跟踪距离顺序无效")
    expected_fx = values["width"] / (2 * math.tan(math.radians(values["hfov"]) / 2))
    expected_fy = values["height_px"] / (2 * math.tan(math.radians(values["vfov"]) / 2))
    if abs(values["fx"] - expected_fx) > 1e-6 or abs(values["fy"] - expected_fy) > 1e-6:
        raise ValueError("fx/fy 必须与标称水平/垂直视场一致")
    if values["targets"] == []:
        raise ValueError("至少配置一个真值目标")
    return values


def geometry(parent, kind, values):
    shape = add(add(parent, "geometry"), kind)
    for key, value in values.items():
        add(shape, key, value)


def material(visual, color, emissive=None):
    mat = add(visual, "material")
    add(mat, "ambient", color)
    add(mat, "diffuse", color)
    add(mat, "specular", ".35 .35 .35 1")
    if emissive:
        add(mat, "emissive", emissive)


def visual_box(link, name, size, pose, color, emissive=None):
    visual = add(link, "visual", name=name)
    add(visual, "pose", pose)
    geometry(visual, "box", {"size": size})
    material(visual, color, emissive)
    return visual


def make_model(cfg):
    sdf = ET.Element("sdf", version="1.9")
    model = add(sdf, "model", name=cfg["name"], canonical_link="seeker_link")
    add(model, "self_collide", "false")
    link = add(model, "link", name="seeker_link")
    inertial = add(link, "inertial")
    add(inertial, "pose", f"0 0 {number(cfg['height'] / 2)} 0 0 0")
    add(inertial, "mass", number(cfg["mass"]))
    tensor = add(inertial, "inertia")
    radius = cfg["diameter"] / 2
    ixy = cfg["mass"] * (3 * radius * radius + cfg["height"] ** 2) / 12
    izz = cfg["mass"] * radius * radius / 2
    for key, value in (("ixx", ixy), ("iyy", ixy), ("izz", izz),
                       ("ixy", 0), ("ixz", 0), ("iyz", 0)):
        add(tensor, key, number(value))

    for tag in ("visual", "collision"):
        body = add(link, tag, name=f"housing_{tag}")
        add(body, "pose", f"0 0 {number(cfg['height'] / 2)} 0 0 0")
        geometry(body, "cylinder", {"radius": number(radius), "length": number(cfg["height"])})
        if tag == "visual":
            material(body, ".055 .060 .065 1")
    rim = add(link, "visual", name="lens_rim_visual")
    add(rim, "pose", f"0 0 {number(cfg['height'] + .0005)} 0 0 0")
    geometry(rim, "cylinder", {"radius": ".012", "length": ".001"})
    material(rim, ".08 .09 .10 1")
    window = add(link, "visual", name="lens_window_visual")
    add(window, "pose", f"0 0 {number(cfg['height'] + .0011)} 0 0 0")
    geometry(window, "cylinder", {"radius": ".009", "length": ".0005"})
    material(window, ".05 .20 .28 1", ".01 .04 .06 1")

    sensor = add(link, "sensor", name="camera", type="camera")
    # Gazebo 相机光轴为局部 +X；绕 Y -90°后与圆柱/载机 +Z 重合。
    add(sensor, "pose", f"0 0 {number(cfg['height'] + .0015)} 0 -1.5707963267948966 0")
    add(sensor, "always_on", "true")
    add(sensor, "update_rate", number(cfg["fps"]))
    add(sensor, "topic", f"/{cfg['name']}/camera")
    camera = add(sensor, "camera")
    add(camera, "horizontal_fov", number(math.radians(cfg["hfov"])))
    image = add(camera, "image")
    add(image, "width", cfg["width"])
    add(image, "height", cfg["height_px"])
    add(image, "format", "R8G8B8")
    clip = add(camera, "clip")
    add(clip, "near", number(cfg["near"]))
    add(clip, "far", number(cfg["far"]))
    intrinsics = add(add(camera, "lens"), "intrinsics")
    for key in ("fx", "fy", "cx", "cy"):
        add(intrinsics, key, number(cfg[key]))
    add(intrinsics, "s", "0")
    add(camera, "camera_info_topic", f"/{cfg['name']}/camera_info")

    plugin = add(model, "plugin", filename="gz-jielian-a2416a-controller",
                 name="jielian_a2416a::StrapdownController")
    settings = {
        "camera_link": "seeker_link", "camera_sensor": "camera",
        "image_topic": f"/{cfg['name']}/camera",
        "command_topic": cfg["command_topic"], "telemetry_topic": cfg["telemetry_topic"],
        "command_result_topic": cfg["result_topic"],
        "measurement_hz": cfg["measurement_hz"],
        "auto_detection_range_m": cfg["auto_range"],
        "manual_tracking_range_m": cfg["manual_range"],
        "lost_timeout_s": cfg["lost_timeout"], "max_targets": cfg["max_targets"],
        "misalignment_limit_deg": cfg["misalignment_limit"],
        "crosshair_max_speed_px_s": cfg["crosshair_speed"],
        "crosshair_timeout_s": cfg["crosshair_timeout"],
        "gate_scale_min": cfg["gate_min"], "gate_scale_max": cfg["gate_max"],
    }
    for key, value in settings.items():
        add(plugin, key, number(value) if isinstance(value, float) else value)
    for target in cfg["targets"]:
        add(plugin, "sim_truth_target_name", target)
    return sdf


def aircraft(world, name, position, color):
    model = add(world, "model", name=name)
    add(model, "static", "true")
    add(model, "pose", f"{position[0]} {position[1]} {position[2]} 0 0 0")
    link = add(model, "link", name="link")
    visual_box(link, "body", "6 1.1 1.1", "0 0 0 0 0 0", color)
    visual_box(link, "wing", "2.4 12 .20", "-.3 0 0 0 0 0", ".80 .82 .85 1")
    visual_box(link, "tail", "1.5 3.2 .18", "-3 0 .35 0 0 0", color)


def make_world(cfg):
    sdf = ET.Element("sdf", version="1.9")
    world = add(sdf, "world", name=cfg["world_name"])
    add(world, "gravity", "0 0 -9.8")
    spherical = add(world, "spherical_coordinates")
    add(spherical, "surface_model", "EARTH_WGS84")
    add(spherical, "world_frame_orientation", "ENU")
    add(spherical, "latitude_deg", "47.397971057728974")
    add(spherical, "longitude_deg", "8.546163739800146")
    add(spherical, "elevation", "0")
    physics = add(world, "physics", name="1ms", type="ode")
    add(physics, "max_step_size", ".001")
    add(physics, "real_time_factor", "1")
    for filename, name in (("gz-sim-physics-system", "gz::sim::systems::Physics"),
                           ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
                           ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster")):
        add(world, "plugin", filename=filename, name=name)
    sensors = add(world, "plugin", filename="gz-sim-sensors-system", name="gz::sim::systems::Sensors")
    add(sensors, "render_engine", "ogre2")
    scene = add(world, "scene")
    add(scene, "ambient", ".65 .65 .65 1")
    add(scene, "background", ".72 .80 .90 1")
    add(scene, "shadows", "true")
    gui = add(world, "gui", fullscreen="false")
    view = add(gui, "plugin", filename="MinimalScene", name="3D View")
    add(view, "engine", "ogre2")
    add(view, "scene", "scene")
    add(view, "camera_pose", ".40 -.48 .32 0 .20 2.30")
    camera_clip = add(view, "camera_clip")
    add(camera_clip, "near", number(cfg["viewer_near"]))
    add(camera_clip, "far", "30000")
    for filename, name in (("GzSceneManager", "Scene Manager"),
                           ("InteractiveViewControl", "Interactive view control"),
                           ("CameraTracking", "Camera Tracking"),
                           ("EntityTree", "Entity tree"),
                           ("WorldControl", "World control")):
        add(gui, "plugin", filename=filename, name=name)

    sun = add(world, "light", name="sun", type="directional")
    add(sun, "direction", "-.4 .3 -1")
    add(sun, "diffuse", ".9 .9 .9 1")
    include = add(world, "include")
    add(include, "uri", MODEL_URI)
    add(include, "name", cfg["name"])
    add(include, "pose", "0 0 0 0 0 0")
    add(include, "static", "true")

    target_positions = ((0, 0, 500), (45, 15, 620), (-60, -10, 700), (75, -25, 760))
    colors = (".95 .30 .05 1", ".10 .55 .95 1", ".20 .75 .35 1", ".80 .35 .85 1")
    for name, position, color in zip(cfg["targets"], target_positions, colors):
        aircraft(world, name, position, color)

    marker = add(world, "model", name=cfg["marker_name"])
    add(marker, "static", "true")
    add(marker, "pose", ".16 -.15 .11 0 0 0")
    link = add(marker, "link", name="link")
    visual_box(link, "shaft", ".02 .24 .02", "0 0 0 0 0 0", "1 .20 .02 1", "1 .05 0 1")
    head = add(link, "visual", name="arrowhead_visual")
    add(head, "pose", "0 .16 0 -1.5707963267948966 0 0")
    geometry(head, "cone", {"radius": ".055", "length": ".12"})
    material(head, "1 .20 .02 1", "1 .05 0 1")
    return sdf


def write_xml(tree, path):
    ET.indent(tree, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(tree).write(path, encoding="utf-8", xml_declaration=True)


def generate(config_path=ROOT / "parameters.ini", output_dir=ROOT):
    cfg = load_config(config_path)
    write_xml(make_model(cfg), Path(output_dir) / "model.sdf")
    write_xml(make_world(cfg), Path(output_dir) / "worlds" / "preview.sdf")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=ROOT / "parameters.ini")
    parser.add_argument("--output-dir", type=Path, default=ROOT)
    args = parser.parse_args()
    generate(args.config, args.output_dir)
