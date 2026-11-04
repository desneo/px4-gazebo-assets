"""Qt窗口标识、初始化清理及X11原生焦点保护的崩溃回归。"""
import ctypes
import os
from pathlib import Path
import subprocess
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
import ground_station.ground_station as gui
from common.config import load_config


class FakeFunction:
    def __init__(self,callback=lambda *_:1):
        self.callback=callback
        self.argtypes=self.restype=None
        self.calls=[]
    def __call__(self,*args):
        self.calls.append(args)
        return self.callback(*args)


class FakeX11:
    def __init__(self,title):
        self.buffer=ctypes.create_string_buffer(title.encode('utf-8'))
        self.XOpenDisplay=FakeFunction(lambda *_:0x1234567887654321)
        self.XDefaultRootWindow=FakeFunction(lambda *_:518)
        self.XInternAtom=FakeFunction(lambda *_:99)
        self.XGetInputFocus=FakeFunction(self.focus)
        self.XGetWindowProperty=FakeFunction(self.title)
        for name in ('XFetchName','XQueryTree','XQueryPointer','XFree','XCloseDisplay'):
            setattr(self,name,FakeFunction())
    def focus(self,display,window,revert):
        window._obj.value=100
        return 1
    def title(self,*args):
        args[7]._obj.value=31
        args[8]._obj.value=8
        args[9]._obj.value=len(self.buffer.value)
        args[11]._obj.value=ctypes.addressof(self.buffer)
        return 0


class WindowTest(unittest.TestCase):
    def test_internal_window_key_is_ascii_with_separate_chinese_caption(self):
        cfg=load_config(ROOT/'parameters.ini')
        preferences=gui.station_preferences(ROOT/'ground_station/config.ini')
        preferences['window_title']='中科 D80 测试窗口'
        with patch.object(gui,'BinaryClient'),patch.object(gui,'TelemetryForwarder'),\
             patch.object(gui,'DiagnosticSubscriber'),patch.object(gui,'PointerGuard') as guard:
            station=gui.GroundStation(cfg,preferences,SimpleNamespace())
        self.assertTrue(station.win.isascii())
        self.assertEqual(station.title,'中科 D80 测试窗口')
        guard.assert_called_once_with(station.title)

    def test_window_callback_initialization_failure_always_shuts_down(self):
        station=gui.GroundStation.__new__(gui.GroundStation)
        station.tele=Mock();station.manual=Mock();station.shutdown=Mock()
        station.win='zhongke_test';station.title='中文显示标题'
        with patch.object(gui,'_HAVE_GST',True),patch.object(gui.cv2,'namedWindow') as named,\
             patch.object(gui.cv2,'setWindowTitle') as caption,\
             patch.object(gui.cv2,'setMouseCallback',side_effect=gui.cv2.error('初始化失败')):
            with self.assertRaises(gui.cv2.error):station.run()
        named.assert_called_once_with('zhongke_test',gui.cv2.WINDOW_AUTOSIZE)
        caption.assert_called_once_with('zhongke_test','中文显示标题')
        station.shutdown.assert_called_once()

    def test_native_pointer_widths_utf8_caption_and_return_types(self):
        native=FakeX11('中科 D80 中文标题')
        with patch.object(gui,'_X11',native):
            guard=gui.PointerGuard('中科 D80 中文标题')
        self.assertTrue(guard.focused())
        self.assertFalse(native.XFetchName.calls)
        self.assertIs(native.XOpenDisplay.restype,ctypes.c_void_p)
        self.assertIs(native.XInternAtom.restype,ctypes.c_ulong)
        for name in ('XGetInputFocus','XFetchName','XQueryTree','XQueryPointer',
                     'XGetWindowProperty','XFree','XCloseDisplay'):
            self.assertIs(getattr(native,name).restype,ctypes.c_int)
        guard.close()
        self.assertEqual(native.XCloseDisplay.calls[0][0],0x1234567887654321)
        self.assertIsNone(guard.focused())

    def test_close_cannot_free_display_while_focus_query_is_inflight(self):
        native=FakeX11('title')
        entered,release,close_requested=threading.Event(),threading.Event(),threading.Event()
        def blocked(display,window,revert):
            entered.set();release.wait(2);window._obj.value=0
            return 1
        native.XGetInputFocus.callback=blocked
        with patch.object(gui,'_X11',native):guard=gui.PointerGuard('title')
        query=threading.Thread(target=guard.focused)
        def close():close_requested.set();guard.close()
        closing=threading.Thread(target=close)
        try:
            query.start();self.assertTrue(entered.wait(1))
            closing.start();self.assertTrue(close_requested.wait(1))
            self.assertFalse(native.XCloseDisplay.calls)
        finally:
            release.set();query.join(1);closing.join(1);guard.close()
        self.assertEqual(len(native.XCloseDisplay.calls),1)

    @unittest.skipUnless(os.environ.get('DISPLAY'),'真实Qt窗口回归需要DISPLAY；其余原生ABI测试仍执行')
    def test_real_qt_window_caption_callback_poll_and_teardown_without_udp(self):
        code='''
import os,sys
sys.path.insert(0,sys.argv[1])
from ground_station.ground_station import cv2,np,PointerGuard
os.environ['QT_QPA_FONTDIR']='/usr/share/fonts/truetype/dejavu'
key='zhongke_gui_regression_'+str(os.getpid());title='中科 D80 窗口回归'
guard=PointerGuard(title)
try:
    cv2.namedWindow(key,cv2.WINDOW_AUTOSIZE)
    cv2.setWindowTitle(key,title)
    cv2.setMouseCallback(key,lambda *args:None)
    cv2.imshow(key,np.zeros((160,400,3),dtype=np.uint8))
    for _ in range(10):
        cv2.waitKey(20)
        assert guard.focused() in (True,False,None)
        assert guard.held_in_window() in (True,False,None)
    assert cv2.getWindowProperty(key,cv2.WND_PROP_VISIBLE)==1
finally:
    guard.close();cv2.destroyWindow(key);cv2.waitKey(1)
print('真实窗口初始化、焦点查询和清理通过，无UDP客户端')
'''
        result=subprocess.run([sys.executable,'-X','faulthandler','-c',code,str(ROOT)],
                              capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)


if __name__=='__main__':unittest.main()
