"""无参数停止中科客户端，保留场景服务、其他模型与PID重用进程。"""
import contextlib
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from ground_station import lifecycle
from scripts import session as sessions


class StopTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.script=self.root/'ground_station/ground_station.py'
        self.script.parent.mkdir()
        self.script.write_text('import time\ntime.sleep(60)\n')
        self.children=[]
        self.patches=[patch.object(lifecycle,'ROOT',self.root),patch.object(sessions,'ROOT',self.root)]
        for item in self.patches:item.start()

    def tearDown(self):
        for child in self.children:
            if child.poll() is None:child.terminate()
            child.wait(timeout=3)
        for item in reversed(self.patches):item.stop()
        self.temp.cleanup()

    def child(self,script=None):
        process=subprocess.Popen([sys.executable,str(script or self.script)],start_new_session=True)
        self.children.append(process)
        return process

    def register(self,name,process,key='ground_station'):
        sessions.write_session(name,dict(state='running',station_config='/tmp/02/config/ground_station_1.ini'))
        return sessions.record_process(name,key,process.pid)

    def test_no_arguments_discovers_clients_without_reading_default_connection(self):
        with patch.object(lifecycle,'stop_running_ground_stations') as stop,\
             patch.object(lifecycle,'resolve_configuration',side_effect=AssertionError('停止不依赖连接配置')),\
             patch.object(lifecycle,'stop_session',side_effect=AssertionError('不局限默认配置')):
            self.assertEqual(lifecycle.main(['stop']),0)
        stop.assert_called_once_with()

    def test_real_clients_stop_across_configs_and_sessions_without_stopping_other_models_or_gateway(self):
        first,second=self.child(),self.child()
        self.register('station_external_config',first)
        self.register('demo_session',second)
        other_script=self.root/'other_model/ground_station.py'
        other_script.parent.mkdir();other_script.write_text(self.script.read_text())
        other=self.child(other_script)
        self.register('other_model',other)
        gateway=self.child()
        self.register('gateway_only',gateway,'gateway')
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(lifecycle.stop_running_ground_stations(),2)
        first.wait(timeout=3);second.wait(timeout=3)
        self.assertIsNone(other.poll())
        self.assertIsNone(gateway.poll())
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(lifecycle.stop_running_ground_stations(),0)

    def test_reused_pid_start_identity_is_not_stopped(self):
        station=self.child()
        record=self.register('stale',station)
        with sessions.edit_session(sessions.session_path('stale')) as manifest:
            manifest['process ground_station']['start_ticks']=str(int(record['start_ticks'])+1)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(lifecycle.stop_running_ground_stations(),0)
        self.assertIsNone(station.poll())

    def test_duplicate_records_stop_only_pid_once_and_never_signal_its_group(self):
        station=self.child()
        self.register('first',station)
        self.register('duplicate',station)
        with patch.object(lifecycle,'stop_process',return_value=True) as stop,\
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(lifecycle.stop_running_ground_stations(),1)
        stop.assert_called_once()
        self.assertEqual(stop.call_args.args[0]['pid'],str(station.pid))
        self.assertEqual(stop.call_args.args[0]['pgid'],'0')

    def test_borrowed_script_argument_and_same_name_other_model_are_rejected(self):
        record=dict(pid='999',start_ticks='1')
        for command in ([sys.executable,'-c','pass',str(self.script)],
                        [sys.executable,str(self.root/'other/ground_station.py')],
                        ['/bin/echo',str(self.script)]):
            with self.subTest(command=command),patch.object(lifecycle,'process_matches',return_value=True),\
                 patch.object(Path,'read_bytes',return_value=os.fsencode('\0'.join(command)+'\0')):
                self.assertFalse(lifecycle.is_ground_station_process(record))
        with patch.object(lifecycle,'process_matches',return_value=True),\
             patch.object(Path,'read_bytes',return_value=os.fsencode(sys.executable+'\0-u\0'+str(self.script)+'\0')):
            self.assertTrue(lifecycle.is_ground_station_process(record))

    def test_bad_and_outside_manifests_do_not_block_other_registered_clients(self):
        station=self.child();self.register('valid',station)
        malformed=self.root/'runs/malformed/session.ini';malformed.parent.mkdir()
        malformed.write_text('this is not ini')
        outside=self.root/'outside.ini';outside.write_text('[session]\nstate=running\n')
        symlink=self.root/'runs/outside/session.ini';symlink.parent.mkdir();symlink.symlink_to(outside)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(lifecycle.stop_running_ground_stations(),1)
        station.wait(timeout=3)

    def test_explicit_session_preserves_directed_stop(self):
        with patch.object(lifecycle,'stop_session') as stop,\
             patch.object(lifecycle,'stop_running_ground_stations') as all_clients:
            self.assertEqual(lifecycle.main(['stop','--session','manual']),0)
        stop.assert_called_once_with('manual',only='ground_station')
        all_clients.assert_not_called()

    def test_explicit_config_preserves_directed_stop(self):
        config=self.root/'external/config.ini'
        with patch.object(lifecycle,'stop_session') as stop,\
             patch.object(lifecycle,'stop_running_ground_stations') as all_clients:
            self.assertEqual(lifecycle.main(['stop','--station-config',str(config)]),0)
        stop.assert_called_once_with(str(lifecycle.client_manifest_path(config)),only='ground_station')
        all_clients.assert_not_called()


if __name__=='__main__':unittest.main()
