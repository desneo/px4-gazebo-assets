"""界面区分机械gyro与目标LOS，并沿用外发单位及无效零值。"""
import unittest

from test_joystick import GroundStation
from test_forward import telemetry


class AttitudeDisplayTests(unittest.TestCase):
    def test_gyro_and_target_los_have_distinct_sources_and_matching_output_limits(self):
        value = telemetry()
        value['attitude_deg']['yaw'].update(pos=12.3, vel=-7.5)
        value['attitude_deg']['pitch'].update(pos=-41.2, vel=3.25)
        value['target_los'].update(valid=True, target_name='target',
                                   yaw_rate_dps=35., pitch_rate_dps=-2.5)
        angles, gyro, los, vector = GroundStation._attitude_panel_lines(value, True)
        self.assertIn('yaw +12.30  pitch -41.20', angles)
        self.assertIn('陀螺仪角速度(°/s): yaw -7.50  pitch +3.25', gyro)
        self.assertIn('目标LOS角速度(°/s): yaw +20.00  pitch -2.50', los)
        self.assertIn('N +7.12  E -28.50  D +32.75', vector)

    def test_invalid_los_displays_zero_while_gyro_continues(self):
        value = telemetry()
        value['attitude_deg']['yaw']['vel'] = 10.
        value['target_los'].update(valid=False, yaw_rate_dps=None,
                                   pitch_rate_dps=None, reason='target_unbound')
        _, gyro, los, vector = GroundStation._attitude_panel_lines(value, True)
        self.assertIn('yaw +10.00', gyro)
        self.assertIn('yaw +0.00  pitch +0.00', los)
        self.assertIn('N +7.12  E -28.50  D +32.75', vector)

    def test_missing_los_and_unready_attitude_do_not_display_plausible_measurements(self):
        value = telemetry()
        del value['target_los']
        self.assertIn('yaw --  pitch --', GroundStation._attitude_panel_lines(value, True)[2])
        lines = GroundStation._attitude_panel_lines(value, False)
        self.assertIn('待确认', lines[0])
        self.assertTrue(all('yaw --  pitch --' in line for line in lines[1:3]))
        self.assertIn('N --  E --  D --', lines[3])

    def test_vector_display_independent_validity_and_no_twenty_degree_clipping(self):
        value = telemetry()
        value['target_los'].update(valid=False, pitch_rate_dps=None, yaw_rate_dps=None,
                                   reason='vertical_singularity', omega_ned_dps=[35., -42., 3.])
        lines = GroundStation._attitude_panel_lines(value, True)
        self.assertIn('yaw +0.00  pitch +0.00', lines[2])
        self.assertIn('N +35.00  E -42.00  D +3.00', lines[3])
        value['target_los'].update(omega_valid=False, omega_ned_dps=None, omega_reason='no_target')
        self.assertIn('N +0.00  E +0.00  D +0.00', GroundStation._attitude_panel_lines(value, True)[3])
        del value['target_los']['omega_valid']
        self.assertIn('N --  E --  D --', GroundStation._attitude_panel_lines(value, True)[3])


if __name__ == '__main__':
    unittest.main()
