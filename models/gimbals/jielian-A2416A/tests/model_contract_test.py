"""验证 A2416A 单刚体外形、光学参数及近景场景契约。"""

from pathlib import Path
import importlib.util
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class ModelContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("jielian_generator", ROOT / "scripts/generate_model.py")
        cls.generator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.generator)
        cls.temp = tempfile.TemporaryDirectory()
        cls.output = Path(cls.temp.name)
        cls.generator.generate(output_dir=cls.output)
        cls.model = ET.parse(cls.output / "model.sdf").getroot().find("model")
        cls.world = ET.parse(cls.output / "worlds/preview.sdf").getroot().find("world")
        cls.cfg = cls.generator.load_config()

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_one_physical_link_and_no_joint(self):
        links = self.model.findall("link")
        self.assertEqual([link.get("name") for link in links], ["seeker_link"])
        self.assertEqual(self.model.findall("joint"), [])
        self.assertEqual(self.model.get("canonical_link"), "seeker_link")

    def test_placeholder_geometry_mass_and_inertia(self):
        link = self.model.find("link")
        self.assertEqual(link.findtext("visual[@name='housing_visual']/pose"), "0 0 0.03 0 0 0")
        self.assertEqual(link.findtext("collision[@name='housing_collision']/pose"), "0 0 0.03 0 0 0")
        self.assertEqual(link.findtext("inertial/pose"), "0 0 0.03 0 0 0")
        self.assertAlmostEqual(float(link.findtext("inertial/mass")), .05)
        self.assertAlmostEqual(float(link.findtext("inertial/inertia/ixx")), 2.625e-5)
        self.assertAlmostEqual(float(link.findtext("inertial/inertia/iyy")), 2.625e-5)
        self.assertAlmostEqual(float(link.findtext("inertial/inertia/izz")), 2.25e-5)
        self.assertAlmostEqual(float(link.findtext("visual[@name='housing_visual']/geometry/cylinder/radius")), .03)
        self.assertAlmostEqual(float(link.findtext("visual[@name='housing_visual']/geometry/cylinder/length")), .06)

    def test_single_camera_exact_intrinsics_and_positive_z_axis(self):
        sensors = self.model.findall(".//sensor")
        self.assertEqual(len(sensors), 1)
        sensor = sensors[0]
        self.assertEqual(sensor.get("name"), "camera")
        pose = [float(value) for value in sensor.findtext("pose").split()]
        self.assertGreater(pose[2], .061)
        self.assertAlmostEqual(pose[4], -math.pi / 2)
        camera = sensor.find("camera")
        self.assertEqual(camera.findtext("image/format"), "R8G8B8")
        self.assertEqual(int(camera.findtext("image/width")), 1920)
        self.assertEqual(int(camera.findtext("image/height")), 1080)
        self.assertEqual(float(sensor.findtext("update_rate")), 25)
        for key in ("fx", "fy", "cx", "cy"):
            self.assertAlmostEqual(float(camera.findtext(f"lens/intrinsics/{key}")), self.cfg[key], places=10)
        self.assertAlmostEqual(math.degrees(2 * math.atan(1920 / (2 * self.cfg["fx"]))), 37, places=10)
        self.assertAlmostEqual(math.degrees(2 * math.atan(1080 / (2 * self.cfg["fy"]))), 21, places=10)

    def test_plugin_has_discoverable_contract(self):
        plugin = self.model.find("plugin")
        self.assertEqual(plugin.get("filename"), "gz-jielian-a2416a-controller")
        self.assertEqual(plugin.get("name"), "jielian_a2416a::StrapdownController")
        self.assertEqual(plugin.findtext("camera_link"), "seeker_link")
        self.assertEqual(plugin.findtext("camera_sensor"), "camera")
        for field in ("command_topic", "telemetry_topic", "command_result_topic", "image_topic"):
            self.assertTrue(plugin.findtext(field).startswith("/jielian_a2416a/"))
        self.assertEqual(float(plugin.findtext("measurement_hz")), 50)
        self.assertEqual(float(plugin.findtext("auto_detection_range_m")), 780)
        self.assertEqual(float(plugin.findtext("manual_tracking_range_m")), 1200)
        self.assertEqual(float(plugin.findtext("lost_timeout_s")), 2.5)
        self.assertEqual(len(plugin.findall("sim_truth_target_name")), 4)

    def test_marker_is_saved_without_collision(self):
        marker = self.world.find("model[@name='JIELIAN_A2416A_LOCATION_MARKER']")
        self.assertIsNotNone(marker)
        self.assertEqual(marker.findall(".//collision"), [])
        self.assertIsNotNone(marker.find(".//visual[@name='arrowhead_visual']"))
        view = self.world.find("gui/plugin[@name='3D View']")
        self.assertEqual(float(view.findtext("camera_clip/near")), .005)
        self.assertEqual(self.world.findtext("spherical_coordinates/world_frame_orientation"), "ENU")

    def test_generation_is_deterministic(self):
        paths = (self.output / "model.sdf", self.output / "worlds/preview.sdf")
        before = [path.read_bytes() for path in paths]
        self.generator.generate(output_dir=self.output)
        self.assertEqual(before, [path.read_bytes() for path in paths])

    def test_preview_runner_uses_configured_sensitivity_service(self):
        text = (ROOT / "scripts/run_preview.py").read_text(encoding="utf-8")
        self.assertIn("/gui/camera/view_control/sensitivity", text)
        self.assertIn("cfg['sensitivity']", text)


if __name__ == "__main__":
    unittest.main()
