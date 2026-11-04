"""只终止自身会话的身份与并发清单契约。"""
import configparser
import multiprocessing
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import unittest
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.session import (load_session, write_session, record_process,
    process_matches, stop_process, stop_session, session_path, start_ticks)


def register(name, key, pid):
    record_process(name, key, pid)


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.name = 'lifecycle_' + uuid.uuid4().hex[:12]
        write_session(self.name, {'state':'testing', 'udp_port':8870})
        self.children = []

    def tearDown(self):
        for child in self.children:
            if child.poll() is None:
                child.terminate()
            child.wait(timeout=5)

    def child(self):
        p = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                             start_new_session=True)
        self.children.append(p)
        return p

    def test_pid_start_identity_prevents_stopping_unrelated_process(self):
        p = self.child()
        record = record_process(self.name, 'owned', p.pid)
        self.assertTrue(process_matches(record))
        altered = {**record, 'start_ticks': str(int(record['start_ticks'])+1)}
        self.assertFalse(stop_process(altered))
        self.assertIsNone(p.poll())
        self.assertTrue(stop_process(record, timeout=.4))
        p.wait(timeout=2)
        self.assertFalse(stop_process(record))

    def test_concurrent_registration_preserves_every_process(self):
        p = self.child()
        workers = [multiprocessing.Process(target=register, args=(self.name, f'worker{i}', p.pid))
                   for i in range(8)]
        for w in workers:
            w.start()
        for w in workers:
            w.join(5)
            self.assertEqual(w.exitcode,0)
        self.assertEqual(set(load_session(self.name)['processes']), {f'worker{i}' for i in range(8)})

    def test_only_ground_station_stop_preserves_other_owned_service(self):
        station, service = self.child(), self.child()
        record_process(self.name,'ground_station',station.pid)
        record_process(self.name,'gateway',service.pid)
        self.assertEqual(stop_session(self.name, only='ground_station'),1)
        station.wait(timeout=2)
        self.assertIsNone(service.poll())
        self.assertEqual(load_session(self.name)['state'],'testing')
        self.assertEqual(stop_session(self.name),1)
        service.wait(timeout=2)
        self.assertEqual(stop_session(self.name),0)
        self.assertEqual(load_session(self.name)['state'],'stopped')

    def test_port_conflict_refuses_start_without_stopping_occupant(self):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as occupied:
            occupied.bind(('127.0.0.1',0))
            parser = configparser.ConfigParser(interpolation=None)
            parser.read(ROOT/'parameters.ini')
            parser['network']['udp_port']=str(occupied.getsockname()[1])
            config = session_path(self.name).parent/'conflict.ini'
            with config.open('w') as stream:
                parser.write(stream)
            command = subprocess.run([sys.executable,str(ROOT/'scripts/runtime.py'),
                '--headless','--config',str(config),'--session',self.name+'_conflict'],
                capture_output=True,text=True,timeout=10)
            self.assertNotEqual(command.returncode,0)
            self.assertIn('占用',command.stderr)
            occupied.sendto(b'still_owned',occupied.getsockname())
            self.assertEqual(occupied.recv(32),b'still_owned')


if __name__=='__main__':
    unittest.main()
