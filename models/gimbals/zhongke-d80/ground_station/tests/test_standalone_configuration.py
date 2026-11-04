"""独立view入口、连接优先级、相对性能路径和固定本地端口。"""
import configparser
from contextlib import chdir
import json
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from common.config import load_config
from ground_station.configuration import (parse_arguments,resolve_configuration,
                                          client_manifest_path)
from ground_station.binary_client import BinaryClient
from ground_station.diagnostics import DisabledDiagnostics
from ground_station import lifecycle
from ground_station.tests.test_telemetry_json import golden_frame


class StandaloneConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.directory=Path(self.temp.name)
        shutil.copy2(ROOT/'parameters.ini',self.directory/'parameters.ini')
        (self.directory/'station').mkdir()
        self.station=self.directory/'station/config.ini'
        self.parser=configparser.ConfigParser(interpolation=None)
        self.parser['connection']=dict(udp_host='127.0.0.2',udp_port='9880',local_bind='127.0.0.1',
            local_port='43210',rtsp_url='rtsp://127.0.0.2:9556/test_camera',parameters_file='../parameters.ini')
        self.write()

    def tearDown(self):self.temp.cleanup()

    def write(self):
        with self.station.open('w') as stream:self.parser.write(stream)

    def test_exact_legacy_view_cli_and_relative_parameters_without_world_lookup(self):
        with chdir(self.station.parent):args=parse_arguments(['view','config.ini'])
        self.assertIsNone(args.session)
        self.assertEqual(args.station_config,str(self.station))
        with patch('scripts.session.load_session',side_effect=AssertionError('禁止寻找demo')):
            cfg,prefs,session=resolve_configuration(args)
        self.assertIsNone(session)
        self.assertEqual(args.config,str(self.directory/'parameters.ini'))
        self.assertEqual(cfg['network']['udp_bind'],'127.0.0.2')
        self.assertEqual(cfg['network']['udp_port'],9880)
        self.assertEqual(cfg['_rtsp_url'],'rtsp://127.0.0.2:9556/test_camera')
        self.assertEqual(prefs['local_port'],43210)

    def test_no_arguments_and_explicit_station_option_do_not_invent_session(self):
        default=parse_arguments([])
        self.assertIsNone(default.session)
        self.assertEqual(default.station_config,str(ROOT/'ground_station/config.ini'))
        explicit=parse_arguments(['--station-config',str(self.station)])
        self.assertIsNone(explicit.session)
        self.assertEqual(explicit.station_config,str(self.station))
        with patch('scripts.session.load_session',side_effect=AssertionError('禁止寻找demo')):
            resolve_configuration(explicit)

    def test_explicit_session_endpoints_and_parameters_win_but_keep_local_binding(self):
        config=load_config(ROOT/'parameters.ini')
        manifest=dict(config_path=str(ROOT/'parameters.ini'),config_hash=config['config_hash'],
            udp_host='127.0.0.3',udp_port=9990,rtsp_url='rtsp://127.0.0.3:9557/session',
            session_path=str(ROOT/'runs/test/session.ini'),gz_partition='explicit_only')
        args=parse_arguments(['--station-config',str(self.station),'--session','explicit'])
        with patch('scripts.session.load_session',return_value=manifest) as load:
            cfg,prefs,session=resolve_configuration(args)
        load.assert_called_once_with('explicit')
        self.assertIs(session,manifest)
        self.assertEqual(cfg['network']['udp_bind'],'127.0.0.3')
        self.assertEqual(cfg['network']['udp_port'],9990)
        self.assertEqual(cfg['_rtsp_url'],manifest['rtsp_url'])
        self.assertEqual(prefs['parameters_file'],str(ROOT/'parameters.ini'))
        self.assertEqual(prefs['local_bind'],'127.0.0.1')
        self.assertEqual(prefs['local_port'],43210)

    def test_explicit_performance_override_wins_in_independent_mode(self):
        args=parse_arguments(['view',str(self.station),'--config',str(ROOT/'parameters.ini')])
        cfg,prefs,session=resolve_configuration(args)
        self.assertEqual(args.config,str(ROOT/'parameters.ini'))
        self.assertEqual(prefs['parameters_file'],str(ROOT/'parameters.ini'))
        self.assertIsNone(session)

    def test_invalid_addresses_and_ports_report_chinese(self):
        for key,value in (('udp_host','not-an-ip'),('udp_port','65536'),
                          ('local_bind','invalid'),('local_port','-1'),
                          ('rtsp_url','http://127.0.0.1/video')):
            with self.subTest(key=key):
                original=self.parser['connection'][key]
                self.parser['connection'][key]=value;self.write()
                with self.assertRaisesRegex(ValueError,'地面站配置读取失败'):
                    resolve_configuration(parse_arguments(['view',str(self.station)]))
                self.parser['connection'][key]=original

    def test_configuration_import_has_no_graphics_or_gazebo_dependency(self):
        code='import sys;sys.path.insert(0,sys.argv[1]);import ground_station.configuration;assert not any(k in sys.modules for k in ("cv2","gi","gz.transport13"))'
        result=subprocess.run([sys.executable,'-c',code,str(ROOT)],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_registry_identity_uses_config_path_and_stop_never_looks_up_demo(self):
        first=client_manifest_path(self.station)
        second=client_manifest_path(self.station.parent/'../station/config.ini')
        self.assertEqual(first,second)
        self.assertNotEqual(first,client_manifest_path(self.station.parent/'other.ini'))
        self.assertTrue(first.is_relative_to(ROOT/'runs'))
        with patch.object(lifecycle,'stop_session',return_value=0) as stop,\
             patch.object(lifecycle,'load_session',side_effect=AssertionError('不能寻找世界')):
            self.assertEqual(lifecycle.main(['stop','view',str(self.station)]),0)
        stop.assert_called_once_with(str(first),only='ground_station')

    def test_legacy_launcher_executes_standalone_without_session_argument(self):
        manifest=self.directory/'client/session.ini'
        with patch.object(lifecycle,'client_manifest_path',return_value=manifest),\
             patch.object(lifecycle,'write_session') as write,patch.object(lifecycle,'record_process') as record,\
             patch.object(lifecycle.os,'setsid'),patch.object(lifecycle.os,'execv') as execute:
            lifecycle.main(['run','view',str(self.station),'--duration','1'])
        command=execute.call_args.args[1]
        self.assertNotIn('--session',command)
        self.assertIn('--station-config',command)
        self.assertIn(str(self.station),command)
        self.assertEqual(write.call_args.args[1]['client_mode'],'independent')
        self.assertEqual(record.call_args.args[:2],(manifest,'ground_station'))

    def test_disabled_diagnostics_never_imports_or_subscribes_gazebo(self):
        subscriber=DisabledDiagnostics()
        self.assertIsNone(subscriber.snapshot())
        subscriber.close()


class FixedLocalPortTest(unittest.TestCase):
    def test_real_udp_uses_requested_local_port_for_send_and_receive(self):
        server=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        server.bind(('127.0.0.1',0));server.settimeout(1)
        reserve=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        reserve.bind(('127.0.0.1',0));port=reserve.getsockname()[1];reserve.close()
        samples=[]
        client=BinaryClient(*server.getsockname(),local_bind='127.0.0.1',local_port=port,
                            on_telemetry=samples.append)
        try:
            self.assertEqual(client.local_endpoint,('127.0.0.1',port))
            client.start()
            command,source=server.recvfrom(128)
            self.assertEqual(source,client.local_endpoint)
            self.assertEqual(command[:3],b'\xeb\x90\0')
            server.sendto(golden_frame(),source)
            end=time.monotonic()+1
            while not samples and time.monotonic()<end:time.sleep(.005)
            self.assertTrue(samples)
            self.assertEqual(samples[0].fields['yaw_rate'],-45.67)
            self.assertEqual(samples[0].source,server.getsockname())
        finally:client.stop();server.close()

    def test_busy_local_port_and_invalid_ip_are_clear_chinese_errors(self):
        owner=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        owner.bind(('127.0.0.1',0))
        try:
            with self.assertRaisesRegex(ValueError,'本地接收端口绑定失败'):
                BinaryClient('127.0.0.1',8870,local_bind='127.0.0.1',local_port=owner.getsockname()[1])
            with self.assertRaisesRegex(ValueError,'合法IP'):
                BinaryClient('not-an-ip',8870)
        finally:owner.close()


if __name__=='__main__':unittest.main()
