"""只读诊断无法注入测量或控制；标准帧独立处理并保持有效时钟。"""
import json
from pathlib import Path
import socket
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from ground_station.binary_client import BinaryClient
from ground_station.diagnostics import DiagnosticSubscriber
from ground_station.telemetry_forward import TelemetryForwarder,FIELDS
from ground_station.tests.test_telemetry_json import golden_frame


class DiagnosticBoundaryTest(unittest.TestCase):
    def test_subscriber_only_subscribes_and_unsubscribes_never_advertises_or_requests(self):
        node=Mock();node.subscribe.return_value=True
        with patch('gz.transport13.Node',return_value=node):
            subscriber=DiagnosticSubscriber('current')
        self.assertEqual(node.subscribe.call_count,1)
        node.advertise.assert_not_called();node.request.assert_not_called()
        subscriber._on_message(SimpleNamespace(data=json.dumps(dict(
            model_name='zhongke_d80',config_hash='current',cmd='manual',yaw=999,pitch=-999))))
        node.advertise.assert_not_called();node.request.assert_not_called()
        node.publish.assert_not_called()
        subscriber.close()
        node.unsubscribe.assert_called_once_with('/zhongke_d80/diagnostics')

    def test_injected_diagnostic_values_never_change_binary_json_or_emit_control(self):
        source=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        source.bind(('127.0.0.1',0));source.settimeout(.05)
        dest=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        dest.bind(('127.0.0.1',0));dest.settimeout(1)
        forward=TelemetryForwarder();forward.start(*dest.getsockname())
        client=BinaryClient(*source.getsockname(),on_telemetry=forward.offer)
        subscriber=DiagnosticSubscriber.__new__(DiagnosticSubscriber)
        subscriber.config_hash='current';subscriber.model_name='zhongke_d80'
        subscriber.max_age_s=1;subscriber._lock=threading.Lock()
        subscriber._latest=None;subscriber._received_at=0;subscriber.error=''
        try:
            self.assertIsNone(subscriber.snapshot())
            client.start()
            deadline=time.monotonic()+1
            while True:
                try:_,peer=source.recvfrom(128);break
                except socket.timeout:
                    self.assertLess(time.monotonic(),deadline)
            injection=dict(model_name='zhongke_d80',config_hash='current',target_name='injected',
                frame_yaw=999,yaw_rate=123,status1=255,timestamp=1,
                cmd='manual',yaw=100,pitch=100,fields={'yaw_rate':123})
            subscriber._on_message(SimpleNamespace(data=json.dumps(injection)))
            raw=golden_frame();source.sendto(raw,peer)
            payload=json.loads(dest.recvfrom(4096)[0])
            self.assertEqual(set(payload),set(FIELDS)|{'timestamp'})
            self.assertEqual(payload['frame_yaw'],-234.56)
            self.assertEqual(payload['yaw_rate'],-45.67)
            self.assertEqual(payload['status1'],0xF4)
            self.assertNotIn('target_name',payload)
            previous=client.snapshot()
            bad=bytearray(raw);bad[-1]^=1
            source.sendto(bad,peer)
            deadline=time.monotonic()+1
            while client.stats()['rejected']<1 and time.monotonic()<deadline:
                time.sleep(.005)
            self.assertEqual(client.stats()['received'],1)
            self.assertEqual(client.snapshot().received_monotonic,previous.received_monotonic)
            self.assertEqual(client.snapshot().raw,raw)
            # 诊断注入期间仅允许客户端既定空指令保活，无任何运动帧。
            end=time.monotonic()+.25
            while time.monotonic()<end:
                try:command,_=source.recvfrom(128)
                except socket.timeout:continue
                self.assertEqual(len(command),16)
                self.assertEqual(command[2],0)
            self.assertEqual(forward.snapshot()['sent'],1)
        finally:
            client.stop();forward.close();source.close();dest.close()


if __name__=='__main__':
    unittest.main()
