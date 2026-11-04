"""发送按钮真实交互、最新16样本内容和目的端切换/停止竞争。"""
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
import cv2
import numpy as np
import ground_station.telemetry_forward as forwarding
from ground_station.ground_station import GroundStation
from ground_station.telemetry_forward import TelemetryForwarder,ForwardInputs
from ground_station.tests.test_telemetry_json import sample


def receiver():
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1',0));sock.settimeout(.15)
    return sock


class GatedForwarder(TelemetryForwarder):
    def __init__(self):
        self.gate=threading.Event()
        super().__init__()
    def _run(self):
        self.gate.wait(2)
        super()._run()
    def close(self):
        self.gate.set()
        super().close()


class ForwardUiTest(unittest.TestCase):
    def setUp(self):
        self.old,self.new=receiver(),receiver()
        self.forward=TelemetryForwarder()
        self.ui=GroundStation.__new__(GroundStation)
        self.ui.forwarder=self.forward
        self.ui.tele=Mock()
        self.ui.tele.peer=('127.0.0.1',8870)
        self.ui.tele.stats.return_value={'connected':True}
        self.ui.manual=Mock()
        self.ui.joystick=None
        self.ui.joystick_values=(0,0)
        self.ui.display_size=(960,540)
        self.ui.forward_inputs=ForwardInputs(*self.old.getsockname())
        self.ui.forward_note='未发送'
        self.ui._forward_pointer=self.ui._forward_pressed=False
        self.ui.preferences={}
        self.directory=tempfile.TemporaryDirectory()
        self.config=Path(self.directory.name)/'config.ini'
        self.config.write_text('# 保留中文说明\n[forward]\nip = 127.0.0.1\nport = 9000\n',encoding='utf-8')
        self.ui.args=SimpleNamespace(station_config=str(self.config))

    def tearDown(self):
        self.forward.close();self.old.close();self.new.close();self.directory.cleanup()

    def click(self,name):
        x,y,w,h=self.ui._forward_geometry()[name]
        self.ui._forward_mouse(cv2.EVENT_LBUTTONDOWN,x+2,y+2)
        self.ui._forward_mouse(cv2.EVENT_LBUTTONUP,x+2,y+2)

    def test_edit_click_pauses_and_button_alone_enables_saved_new_destination(self):
        self.click('toggle')
        self.assertTrue(self.forward.snapshot()['active'])
        self.click('port')
        self.assertFalse(self.forward.snapshot()['active'])
        self.assertEqual(self.forward.snapshot()['queued'],0)
        self.ui.forward_inputs.values['port']=str(self.new.getsockname()[1])
        self.click('toggle')
        self.forward.offer(sample())
        received=json.loads(self.new.recvfrom(4096)[0])
        self.assertEqual(received['frame_yaw'],-234.56)
        with self.assertRaises(socket.timeout):
            self.old.recvfrom(4096)
        self.assertIn(f'port = {self.new.getsockname()[1]}',self.config.read_text())
        self.assertIn('# 保留中文说明',self.config.read_text())

    def test_redrawing_forward_panel_does_not_count_or_send_samples(self):
        self.ui.font=None
        self.ui._draw_rate_panel=Mock()
        self.click('toggle')
        self.forward.offer(sample())
        self.old.recvfrom(4096)
        canvas=np.zeros((820,960,3),dtype=np.uint8)
        for _ in range(30):
            self.ui._draw_forward_panel(canvas)
        self.assertEqual(self.forward.snapshot()['sent'],1)
        with self.assertRaises(socket.timeout):
            self.old.recvfrom(4096)

    def test_gui_close_closes_sender_and_rejects_later_samples(self):
        self.click('toggle')
        self.ui.diagnostics=Mock();self.ui.pointer_guard=Mock();self.ui.pipeline=None
        with patch('cv2.destroyAllWindows'):
            self.ui.shutdown()
        self.assertFalse(self.forward.snapshot()['active'])
        self.assertFalse(self.forward._thread.is_alive())
        self.forward.offer(sample())
        with self.assertRaises(socket.timeout):
            self.old.recvfrom(4096)

    def test_same_control_endpoint_cannot_be_enabled_as_json_destination(self):
        self.ui.forward_inputs.values=dict(ip='127.0.0.1',port='8870')
        self.ui._toggle_forward()
        self.assertFalse(self.forward.snapshot()['active'])
        self.assertIn('控制端口',self.ui.forward_note)


class ForwardRaceTest(unittest.TestCase):
    def test_overload_keeps_latest_sixteen_samples_in_receive_order(self):
        dest=receiver();forward=GatedForwarder()
        try:
            forward.start(*dest.getsockname())
            for seq in range(1,41):
                forward.offer(sample(seq,yaw_raw=seq*100))
            forward.gate.set()
            values=[json.loads(dest.recvfrom(4096)[0])['frame_yaw'] for _ in range(16)]
            self.assertEqual(values,list(range(25,41)))
            self.assertEqual(forward.snapshot()['dropped'],24)
        finally:
            forward.close();dest.close()

    def test_start_new_destination_clears_pending_old_session_and_keeps_rx_dedup(self):
        old,new=receiver(),receiver();forward=GatedForwarder()
        try:
            forward.start(*old.getsockname())
            forward.offer(sample(1,yaw_raw=100))
            forward.start(*new.getsockname())
            forward.offer(sample(1,yaw_raw=100))
            forward.offer(sample(2,yaw_raw=200))
            forward.gate.set()
            self.assertEqual(json.loads(new.recvfrom(4096)[0])['frame_yaw'],2)
            for dest in (old,new):
                with self.assertRaises(socket.timeout):
                    dest.recvfrom(4096)
        finally:
            forward.close();old.close();new.close()

    def test_stop_serializes_with_inflight_send_before_new_session(self):
        old,new=receiver(),receiver();forward=TelemetryForwarder()
        entered,release,stopped=threading.Event(),threading.Event(),threading.Event()
        original=forwarding.json_measurement
        def slow(sample_value):
            entered.set()
            if not release.wait(2):
                raise ValueError('测试发送锁等待超时')
            return original(sample_value)
        def stop():
            forward.stop();stopped.set()
        worker=None
        try:
            forward.start(*old.getsockname())
            with patch.object(forwarding,'json_measurement',side_effect=slow):
                forward.offer(sample(1,yaw_raw=100))
                self.assertTrue(entered.wait(1))
                worker=threading.Thread(target=stop);worker.start()
                self.assertFalse(stopped.wait(.03))
                release.set()
                self.assertTrue(stopped.wait(1))
            old.recvfrom(4096)
            forward.start(*new.getsockname())
            forward.offer(sample(1,yaw_raw=100))
            forward.offer(sample(2,yaw_raw=200))
            self.assertEqual(json.loads(new.recvfrom(4096)[0])['frame_yaw'],2)
            with self.assertRaises(socket.timeout):
                new.recvfrom(4096)
        finally:
            release.set()
            if worker:worker.join(timeout=1)
            forward.close();old.close();new.close()


if __name__=='__main__':
    unittest.main()
