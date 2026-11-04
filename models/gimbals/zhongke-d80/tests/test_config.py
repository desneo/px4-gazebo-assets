"""统一INI边界、未知配置和规范化快照契约。"""
import configparser
import argparse
from pathlib import Path
import tempfile
import unittest
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common.config import (load_config, to_model_parameters, add_runtime_arguments,
    runtime_overrides_from_args, apply_runtime_overrides)


class ConfigurationTests(unittest.TestCase):
    def changed(self, section, key, value):
        parser = configparser.ConfigParser(interpolation=None)
        parser.read(ROOT / 'parameters.ini')
        parser[section][key] = value
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'parameters.ini'
            with path.open('w') as stream:
                parser.write(stream)
            return load_config(path)

    def test_invalid_configuration(self):
        cases = [('model', 'mass_kg', '0'), ('model', 'mass_kg', 'nan'),
            ('mechanics', 'yaw_min_deg', '91'), ('mechanics', 'pitch_min_deg', '1'),
            ('mechanics', 'max_rate_dps', '327.68'), ('mechanics', 'max_rate_dps', 'inf'),
            ('mechanics', 'stop_margin_deg', '100'), ('network', 'udp_port', '65536'),
            ('network', 'telemetry_hz', '0'), ('network', 'udp_bind', 'bad'),
            ('optics', 'focal_length_mm', '-1'), ('optics', 'reference_vfov_deg', '180'),
            ('optics', 'focal_length_mm', '30000'), ('optics', 'width_px', '1919'),
            ('optics', 'height_px', '0'), ('optics', 'hfov_deg', '9'),
            ('tracking', 'discovery_range_m', '0'),
            ('tracking', 'target_names', 'a,a'), ('video', 'rtsp_port', '0')]
        for section, key, value in cases:
            with self.subTest(section=section, key=key, value=value):
                with self.assertRaisesRegex(ValueError, '中科配置读取失败'):
                    self.changed(section, key, value)

    def test_repeated_and_inherited_values_rejected(self):
        original = (ROOT / 'parameters.ini').read_text()
        for content in (original + '\n[model]\nmass_kg=1\n',
                        '[DEFAULT]\nextra=1\n' + original):
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'bad.ini'
                path.write_text(content)
                with self.assertRaises(ValueError):
                    load_config(path)

    def test_rates_and_mass_reach_generator(self):
        for rate in (10, 20, 30):
            c = self.changed('mechanics', 'max_rate_dps', str(rate))
            self.assertEqual(to_model_parameters(c)['max_rate_dps'], rate)
        c = self.changed('model', 'mass_kg', '.3')
        self.assertEqual(to_model_parameters(c)['mass_kg'], .3)

    def test_hash_is_path_independent(self):
        source = load_config()
        same = self.changed('model', 'mass_kg', '0.1950')
        self.assertEqual(source['config_hash'], same['config_hash'])
        changed = self.changed('mechanics', 'max_rate_dps', '30')
        self.assertNotEqual(source['config_hash'], changed['config_hash'])

    def test_runtime_cli_is_explicit_and_source_snapshot_is_immutable(self):
        parser = add_runtime_arguments(argparse.ArgumentParser())
        self.assertEqual(runtime_overrides_from_args(parser.parse_args([])), {})
        args = parser.parse_args(['--model', 'rocket_zhongke_d80_2', '--aircraft-id', '2',
            '--world', 'two_axis_moving_target', '--target-name', 'target_a',
            '--target-name', 'target_b', '--udp-bind', '0.0.0.0', '--udp-port', '8881',
            '--rtsp-bind', '127.0.0.1', '--rtsp-port', '8567',
            '--rtsp-mount', '/zhongke_d80_2'])
        source = load_config()
        before = (ROOT / 'parameters.ini').read_bytes()
        effective = apply_runtime_overrides(source, runtime_overrides_from_args(args))
        self.assertEqual(effective['model']['name'], 'rocket_zhongke_d80_2')
        self.assertEqual(effective['model']['aircraft_id'], 2)
        self.assertEqual(effective['tracking']['target_names'], ['target_a', 'target_b'])
        self.assertEqual(effective['network']['udp_port'], 8881)
        self.assertEqual(effective['video']['rtsp_port'], 8567)
        self.assertEqual(effective['video']['rtsp_mount'], '/zhongke_d80_2')
        self.assertNotEqual(source['config_hash'], effective['config_hash'])
        self.assertEqual(source, load_config())
        self.assertEqual(before, (ROOT / 'parameters.ini').read_bytes())
        self.assertEqual(effective['config_hash'],
            apply_runtime_overrides(load_config(), runtime_overrides_from_args(args))['config_hash'])

    def test_runtime_overrides_reject_invalid_values_and_performance_fields(self):
        for overrides in ({'network': {'udp_port': 0}}, {'model': {'aircraft_id': -1}},
                {'tracking': {'target_names': ['same', 'same']}},
                {'video': {'rtsp_mount': 'relative'}}, {'mechanics': {'kp': 1}}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                apply_runtime_overrides(load_config(), overrides)

    def test_each_ini_key_has_chinese_comment(self):
        previous = ''
        for line in (ROOT / 'parameters.ini').read_text().splitlines():
            if line.strip() and not line.lstrip().startswith(('#', ';', '[')) and '=' in line:
                self.assertTrue(previous.startswith('#'), line)
                self.assertRegex(previous, '[\u4e00-\u9fff]')
            previous = line.strip()


if __name__ == '__main__':
    unittest.main()
