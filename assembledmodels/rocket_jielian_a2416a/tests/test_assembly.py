#!/usr/bin/env python3
"""A2416A 捷联导引头火箭静态和动态装配契约。"""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


appearance = load("rocket_jielian_appearance_test", ROOT / "tools/generate_model.py")
dynamic = load("rocket_jielian_dynamic_test", ROOT / "tools/generate_dynamic_model.py")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class AssemblyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source_before = {
            path: sha(path) for path in (
                dynamic.ROCKET / "model.sdf", dynamic.SEEKER / "model.sdf",
                dynamic.ROCKET / "parameters.json", dynamic.SEEKER / "parameters.ini")
        }
        cls.temp = tempfile.TemporaryDirectory(prefix="rocket-jielian-a2416a-")
        cls.static_output = Path(cls.temp.name) / "static" / appearance.NAME
        appearance.generate(cls.static_output)
        cls.dynamic_output = Path(cls.temp.name) / "dynamic" / dynamic.NAME
        cls.manifest = dynamic.generate(cls.dynamic_output)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_source_assets_are_unchanged(self):
        self.assertEqual(self.source_before, {path: sha(path) for path in self.source_before})

    def test_static_geometry_and_marker(self):
        model = ET.parse(self.static_output / "model.sdf").getroot().find("model")
        self.assertEqual(model.get("name"), appearance.NAME)
        self.assertEqual(model.findtext("static"), "true")
        self.assertFalse(model.findall("joint"))
        self.assertFalse(model.findall(".//collision"))
        self.assertTrue((self.static_output / "meshes/fuselage_cut.stl").is_file())
        for world_name in ("world.sdf", "closeup.sdf"):
            world = ET.parse(self.static_output / "tests" / world_name).getroot().find("world")
            marker = world.find("model[@name='JIELIAN_A2416A_LOCATION_MARKER']")
            self.assertIsNotNone(marker)
            self.assertFalse(marker.findall(".//collision"))
            self.assertAlmostEqual(float(world.findtext("gui/plugin/camera_clip/near")), 0.005)

    def test_dynamic_mass_mount_and_topics(self):
        self.assertEqual(self.manifest["airframe_id"], 4026)
        self.assertEqual(self.manifest["topic_prefix"], "/rocket_jielian_a2416a")
        self.assertAlmostEqual(self.manifest["overall_length_m"], 0.590, places=12)
        self.assertAlmostEqual(self.manifest["mount_z_m"], 0.320, places=12)
        self.assertAlmostEqual(self.manifest["seeker_mass_kg"], 0.05, places=12)
        self.assertAlmostEqual(self.manifest["total_mass_kg"], 4.81, places=9)
        model = ET.parse(self.dynamic_output / "model.sdf").getroot().find("model")
        link = model.find("link[@name='seeker_link']")
        self.assertIsNotNone(link)
        self.assertAlmostEqual(float(link.findtext("pose").split()[2]), 0.320, places=12)
        joint = model.find("joint[@name='seeker_mount_joint']")
        self.assertEqual(joint.get("type"), "fixed")
        self.assertEqual(joint.findtext("parent"), "base_link")
        self.assertEqual(joint.findtext("child"), "seeker_link")
        plugin = model.find("plugin[@name='jielian_a2416a::StrapdownController']")
        self.assertEqual(plugin.findtext("command_topic"), "/rocket_jielian_a2416a/command")
        self.assertEqual(plugin.findtext("telemetry_topic"), "/rocket_jielian_a2416a/telemetry")
        self.assertEqual(plugin.findtext("image_topic"), "/rocket_jielian_a2416a/image")
        camera = link.find("sensor[@name='camera']")
        self.assertEqual(camera.findtext("topic"), "/rocket_jielian_a2416a/image")

    def test_airframe_and_asset_closure(self):
        airframe = self.dynamic_output / "px4/4026_gz_rocket_jielian_a2416a"
        self.assertTrue(airframe.is_file())
        text = airframe.read_text(encoding="utf-8")
        self.assertIn("gz_rocket_jielian_a2416a", text)
        self.assertIn("A2416A strapdown seeker", text)
        for key in ("MPC_THR_HOVER", "CA_ROTOR0_PX", "EKF2_IMU_POS_X", "EKF2_GPS_POS_Z"):
            self.assertIn(f"param set-default {key} ", text)
        model = ET.parse(self.dynamic_output / "model.sdf").getroot().find("model")
        for uri in model.findall(".//mesh/uri"):
            self.assertTrue(uri.text.startswith(f"model://{dynamic.NAME}/meshes/"))
            relative = uri.text.removeprefix(f"model://{dynamic.NAME}/")
            self.assertTrue((self.dynamic_output / relative).is_file(), relative)

    def test_dynamic_world_contains_truth_aircraft_fixture(self):
        world = ET.parse(self.dynamic_output / "tests/world.sdf").getroot().find("world")
        snapshot = json.loads(
            (self.dynamic_output / "parameters.json").read_text(encoding="utf-8")
        )
        self.assertEqual(snapshot["simulation"]["spawn_pose"], "0 0 0.215 0 0 0")
        target = world.find("model[@name='target_aircraft']")
        self.assertIsNotNone(target)
        self.assertEqual(target.findtext("static"), "true")
        self.assertEqual(target.findtext("pose"), "0 0 500 0 0 0")
        self.assertIsNotNone(target.find(".//visual"))
        self.assertIsNotNone(target.find(".//collision"))

    def test_flight_launcher_is_shell_syntax_valid(self):
        script = ROOT / "tests/run_flight_check.sh"
        completed = subprocess.run(["bash", "-n", str(script)], capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        text = script.read_text(encoding="utf-8")
        self.assertIn("capture_tracking_telemetry.py", text)
        self.assertIn("validate_flight_tracking.py", text)
        self.assertIn("recognition_on", text)
        self.assertIn("auto_lock_on", text)

    def test_runtime_overrides_are_scoped(self):
        with tempfile.TemporaryDirectory(prefix="rocket-jielian-prefix-") as temporary:
            output = Path(temporary) / dynamic.NAME
            manifest = dynamic.generate(output, airframe_id=4027, topic_prefix="/uav_7/seeker")
            self.assertEqual(manifest["airframe_id"], 4027)
            self.assertEqual(manifest["topic_prefix"], "/uav_7/seeker")
            model = ET.parse(output / "model.sdf").getroot().find("model")
            self.assertEqual(model.find("plugin[@name='jielian_a2416a::StrapdownController']")
                             .findtext("command_topic"), "/uav_7/seeker/command")

    def test_dangerous_output_and_bad_prefix_rejected(self):
        with self.assertRaises(ValueError):
            dynamic.generate(ROOT)
        for prefix in ("relative", "/bad/", "/bad value"):
            with self.subTest(prefix=prefix), self.assertRaises(ValueError):
                dynamic.settings(topic_prefix=prefix)


if __name__ == "__main__":
    unittest.main(verbosity=2)
