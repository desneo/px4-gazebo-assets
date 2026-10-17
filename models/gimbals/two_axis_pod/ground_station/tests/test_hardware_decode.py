#!/usr/bin/env python3
"""验证地面站解码管线只接受 NVIDIA NVDEC，不产生软件回退。"""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "ground_station.py"
tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))


class FakePipeline:
    def get_by_name(self, name):
        return object() if name == "sink" else None


class FakeFactory:
    available = True

    @classmethod
    def find(cls, name):
        return object() if cls.available and name == "nvh264dec" else None


class HardwareDecodeTests(unittest.TestCase):
    def setUp(self):
        self.descriptions = []

        def parse_launch(description):
            self.descriptions.append(description)
            return FakePipeline()

        fake_gst = SimpleNamespace(ElementFactory=FakeFactory, parse_launch=parse_launch)
        namespace = {
            "_HAVE_GST": True,
            "_GST_ERR": "",
            "DECODER_ELEMENTS": {"nvdec": "nvh264dec", "nvenc": "nvh264dec"},
            "Gst": fake_gst,
        }
        node = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == "build_pipeline")
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), "exec"), namespace)
        self.build_pipeline = namespace["build_pipeline"]
        FakeFactory.available = True

    def test_pipeline_contains_only_nvidia_decoder(self):
        _, _, decoder = self.build_pipeline("rtsp://127.0.0.1/test", "nvdec", verbose=False)
        self.assertEqual(decoder, "nvdec")
        self.assertIn("nvh264dec", self.descriptions[0])
        self.assertNotIn("avdec", self.descriptions[0])

    def test_software_decoder_configuration_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "仅允许 nvh264dec"):
            self.build_pipeline("rtsp://127.0.0.1/test", "avdec", verbose=False)
        self.assertFalse(self.descriptions)

    def test_missing_nvidia_plugin_is_fatal(self):
        FakeFactory.available = False
        with self.assertRaisesRegex(RuntimeError, "缺少 NVIDIA 硬解元件"):
            self.build_pipeline("rtsp://127.0.0.1/test", "nvdec", verbose=False)
        self.assertFalse(self.descriptions)


if __name__ == "__main__":
    unittest.main(verbosity=2)
