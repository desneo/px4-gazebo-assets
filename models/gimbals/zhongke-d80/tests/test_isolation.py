"""核验生成前后参考资产不变和组件资源边界；既有验收基线由独立工具核验。"""
from __future__ import annotations
import importlib.util
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from common.config import load_config

spec=importlib.util.spec_from_file_location('d80_reference_baseline',ROOT/'tests/reference_baseline.py')
baseline_module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(baseline_module)


class IsolationTest(unittest.TestCase):
    def test_generation_preserves_reference_assets_px4_and_working_changes(self):
        before=baseline_module.capture()
        self.assertTrue(before['files'],'参考资产不能为空')
        with tempfile.TemporaryDirectory(prefix='zhongke-isolation-') as directory:
            generated=subprocess.run([sys.executable,str(ROOT/'tools/generate_model.py'),
                '--output-dir',directory],capture_output=True,text=True,timeout=15)
            self.assertEqual(generated.returncode,0,generated.stdout+generated.stderr)
        self.assertEqual(baseline_module.capture(),before,
                         '生成中科模型不能修改参考文件或已有工作区状态')

    def test_source_dependencies_and_symlinks_stay_inside_component(self):
        for folder in ('common','controller','ground_station','scripts','tools','meshes','cad'):
            for path in (ROOT/folder).rglob('*'):
                if path.is_symlink():
                    self.assertTrue(path.resolve().is_relative_to(ROOT),f'外部符号链接：{path}')
        cmake=(ROOT/'controller/CMakeLists.txt').read_text()
        self.assertNotRegex(cmake,r'\.\./(?:\.\./)?(?:two_axis_pod|three_axis_pod|aisheng-ASD)')
        self.assertIn('vendor/gimbal_core',cmake)

    def test_model_library_topics_meshes_and_default_endpoints_are_independent(self):
        config=load_config(ROOT/'parameters.ini')
        self.assertEqual(config['model']['name'],'zhongke_d80')
        self.assertEqual(config['network']['udp_port'],8870)
        self.assertEqual(config['video']['rtsp_port'],8556)
        self.assertEqual(config['video']['rtsp_mount'],'/zhongke_d80_1')
        model=ET.parse(ROOT/'model.sdf').getroot().find('model')
        self.assertEqual(model.attrib['name'],'zhongke_d80')
        plugins=[element for element in model.findall('plugin')
                 if 'controller' in element.attrib.get('filename','')]
        self.assertEqual(len(plugins),1)
        self.assertIn('zhongke-d80',plugins[0].attrib['filename'])
        for uri in model.iter('uri'):
            self.assertTrue(uri.text.startswith('model://zhongke-d80/'),uri.text)
        for tag in ('command_topic','telemetry_topic','command_result_topic'):
            value=plugins[0].findtext(tag)
            if value is not None:
                self.assertTrue(value.startswith('/zhongke_d80/'),value)
        self.assertEqual(config['demo']['marker_name'],'ZHONGKE_D80_LOCATION_MARKER')

    def test_stop_entrypoints_do_not_use_broad_process_kills(self):
        scripts=list((ROOT/'scripts').glob('*.sh'))+list((ROOT/'ground_station').glob('*.sh'))
        self.assertTrue(scripts)
        for path in scripts:
            content=path.read_text()
            self.assertNotRegex(content,r'\b(?:pkill|killall)\b',str(path))


if __name__=='__main__':
    unittest.main()
