#!/usr/bin/env python3
"""Station INI 配置读取测试。"""
from pathlib import Path
import sys
import tempfile
import unittest


GROUND_STATION = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(GROUND_STATION))

import ground_station  # noqa: E402


class StationConfigTests(unittest.TestCase):
    def test_annotated_ini_loads_typed_values(self):
        config = ground_station.load_config(ground_station.DEFAULT_CONFIG)
        self.assertEqual(config["gimbal_host"], "192.168.1.56")
        self.assertEqual(config["udp_port"], 8770)
        self.assertIs(type(config["udp_port"]), int)
        self.assertEqual(config["aircraft_id"], 1)
        self.assertEqual(
            ground_station.rtsp_url(config),
            "rtsp://192.168.1.56:8555/two_axis_pod_1")
        self.assertEqual(config["display_scale"], 0.5)
        self.assertIs(type(config["display_scale"]), float)
        self.assertIs(config["reconnect"], True)

    def test_json_config_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "station.json"
            path.write_text('{"udp_port": 9000}\n', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "仅支持 INI"):
                ground_station.load_config(str(path))


if __name__ == "__main__":
    unittest.main(verbosity=2)
