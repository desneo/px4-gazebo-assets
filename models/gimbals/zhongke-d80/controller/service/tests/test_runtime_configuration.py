"""网关CLI使用与生成器一致的有效配置，不生成临时INI。"""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from common.config import apply_runtime_overrides,load_config
from controller.service import gimbal_bridge


class RuntimeConfigurationTest(unittest.TestCase):
    def test_gateway_receives_effective_configuration_and_existing_topic_options(self):
        command=['--model','rocket_zhongke_d80_2','--aircraft-id','2',
            '--world','two_axis_moving_target','--target-name','target_a',
            '--udp-bind','0.0.0.0','--udp-port','8881',
            '--rtsp-bind','127.0.0.1','--rtsp-port','8567',
            '--rtsp-mount','/zhongke_d80_2','--topic-prefix','/uav_2/pod',
            '--image-topic','/uav_2/pod/camera/image','--no-video']
        expected=apply_runtime_overrides(load_config(),dict(
            model=dict(name='rocket_zhongke_d80_2',aircraft_id=2),
            demo=dict(world_name='two_axis_moving_target'),tracking=dict(target_names=['target_a']),
            network=dict(udp_bind='0.0.0.0',udp_port=8881),
            video=dict(rtsp_bind='127.0.0.1',rtsp_port=8567,rtsp_mount='/zhongke_d80_2')))
        with patch.object(gimbal_bridge,'Gateway') as gateway:
            self.assertEqual(gimbal_bridge.main(command),0)
        self.assertEqual(gateway.call_args.args[0],expected)
        self.assertEqual(gateway.call_args.kwargs['topic_prefix'],'/uav_2/pod')
        self.assertFalse(gateway.call_args.kwargs['video'])
        gateway.return_value.run.assert_called_once_with()

    def test_invalid_runtime_configuration_fails_before_gateway_is_created(self):
        with patch.object(gimbal_bridge,'Gateway') as gateway:
            self.assertEqual(gimbal_bridge.main(['--udp-port','0']),1)
        gateway.assert_not_called()


if __name__=='__main__':unittest.main()
