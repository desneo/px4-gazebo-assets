"""双击像素符号、人工接管、单轴回零与编辑转发地址行为。"""
from pathlib import Path
from types import SimpleNamespace
import sys
import time
import unittest
from unittest.mock import Mock

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
import cv2
import numpy as np
from common.config import load_config
from ground_station.ground_station import GroundStation, ManualController
from ground_station.telemetry_forward import ForwardInputs


class UiTest(unittest.TestCase):
    def setUp(self):
        self.ui=GroundStation.__new__(GroundStation)
        self.ui.tele=Mock()
        self.ui.tele.stats.return_value={'connected':True}
        self.ui.tele.snapshot.return_value=SimpleNamespace(fields={'frame_yaw':12.34,'frame_pitch':-5.67})
        self.ui.manual=Mock()
        self.ui.frame_size=(1920,1080)
        self.ui.display_size=(960,540)
        self.ui.last_frame_mono=time.monotonic()
        self.ui.forward_inputs=ForwardInputs('127.0.0.1',9000)
        self.ui._forward_pointer=self.ui._forward_pressed=False
        self.ui.forwarder=Mock()
        self.ui.forwarder.snapshot.return_value={'active':True}
        self.ui.buttons=[]
        self.ui.joystick=None
        self.ui.joystick_values=(0,0)
        self.ui.max_rate=35
        self.ui.last_cmd=''
        self.ui.diagnostics=Mock()
        self.ui.diagnostics.snapshot.return_value=None
        self.ui.pointer_guard=Mock()

    def test_double_click_uses_original_pixels_center_origin_y_down(self):
        self.ui.on_mouse(cv2.EVENT_LBUTTONDBLCLK,240,400,0,None)
        self.ui.tele.send.assert_called_once_with(0x0D,x=-480,y=260)
        self.ui.manual.cancel.assert_called_once()

    def test_changed_resolution_and_half_pixel_away_from_zero(self):
        self.ui.frame_size=(1280,720)
        self.ui.display_size=(1280,720)
        self.ui.on_mouse(cv2.EVENT_LBUTTONDBLCLK,639.5,359.5,0,None)
        self.ui.tele.send.assert_called_once_with(0x0D,x=-1,y=-1)

    def test_stale_video_rejects_point_and_drag_never_submits_track(self):
        self.ui.last_frame_mono=time.monotonic()-2
        self.ui.on_mouse(cv2.EVENT_LBUTTONDBLCLK,240,400,0,None)
        self.ui.on_mouse(cv2.EVENT_LBUTTONDOWN,240,400,0,None)
        self.ui.on_mouse(cv2.EVENT_MOUSEMOVE,300,450,cv2.EVENT_FLAG_LBUTTON,None)
        self.ui.on_mouse(cv2.EVENT_LBUTTONUP,300,450,0,None)
        self.ui.tele.send.assert_not_called()

    def test_single_axis_zero_preserves_other_fresh_measured_axis(self):
        self.ui._action('yaw_zero')
        self.ui.tele.send.assert_called_once_with(0x26,x=0,y=-567)
        self.ui.tele.send.reset_mock()
        self.ui._action('pitch_zero')
        self.ui.tele.send.assert_called_once_with(0x26,x=1234,y=0)
        self.ui.tele.stats.return_value={'connected':False}
        self.ui.tele.send.reset_mock()
        self.ui._action('yaw_zero')
        self.ui.tele.send.assert_not_called()

    def test_manual_uses_configured_max_and_stop_sends_both_frames(self):
        self.ui._manual('yaw',1)
        self.ui.manual.hold.assert_called_once_with('yaw',35,.3)
        self.assertEqual(self.ui.tele.send.call_args_list[-1].args,(0x0E,))
        self.ui.tele.send.reset_mock()
        self.ui._action('stop')
        self.assertEqual([c.args for c in self.ui.tele.send.call_args_list],[(0x0E,),(0x24,)])

    def test_control_row_has_three_centered_buttons_and_stop_remains_clickable(self):
        self.ui.cfg=load_config(ROOT/'parameters.ini')
        self.ui.font=None
        self.ui.hover=None
        self.ui._draw_forward_panel=Mock()
        self.ui.draw(np.zeros((540,960,3),dtype=np.uint8),None)
        buttons=[button for button in self.ui.buttons if 540<=button[1][1]<570]
        self.assertEqual([button[0] for button in buttons],['双击选取','自动锁定','停止'])
        left=buttons[0][1][0]
        right=buttons[-1][1][0]+buttons[-1][1][2]
        self.assertLessEqual(abs(left+right-960),1)
        _,(x,y,w,h),_,_=buttons[-1]
        self.ui.on_mouse(cv2.EVENT_LBUTTONDOWN,x+w//2,y+h//2,0,None)
        self.ui.on_mouse(cv2.EVENT_LBUTTONUP,x+w//2,y+h//2,0,None)
        self.assertEqual([call.args for call in self.ui.tele.send.call_args_list],[(0x0E,),(0x24,)])

    def test_enter_and_escape_only_finish_address_edit(self):
        for key in (13,27):
            self.ui.forward_inputs.activate('ip')
            self.assertTrue(self.ui._handle_forward_key(key))
            self.assertIsNone(self.ui.forward_inputs.focus)
        self.assertEqual(self.ui.forwarder.stop.call_count,2)
        self.ui.forwarder.start.assert_not_called()

    def test_auto_toggles_from_fresh_diagnostics_and_unknown_only_requests_auto(self):
        self.ui._action('auto')
        self.ui.tele.send.assert_called_once_with(0x0F)
        for mode in ('AUTO','auto'):
            self.ui.tele.send.reset_mock()
            self.ui.diagnostics.snapshot.return_value={'selection_mode':mode}
            self.ui._action('auto')
            self.ui.tele.send.assert_called_once_with(0x0E)

    def test_focus_loss_cancels_keyboard_and_joystick_manual_input(self):
        self.ui.pointer_guard.focused.return_value=False
        for joystick in (None,'aim'):
            self.ui.joystick=joystick
            self.ui._check_joystick()
            self.assertIsNone(self.ui.joystick)
        self.assertEqual(self.ui.manual.cancel.call_count,2)


class ManualTest(unittest.TestCase):
    def test_manual_stream_uses_configured_limit_and_release_sends_zero(self):
        client=Mock()
        manual=ManualController(client,35)
        manual.hold('yaw',99,.25)
        manual.start()
        end=time.monotonic()+1
        while not client.send.called and time.monotonic()<end:
            time.sleep(.005)
        self.assertEqual(client.send.call_args.kwargs['x'],350)
        manual.cancel()
        self.assertEqual(client.send.call_args.kwargs,dict(x=0,y=0))
        manual.stop()


if __name__=='__main__':
    unittest.main()
