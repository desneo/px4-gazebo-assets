import unittest

from check_rtsp_binding import validate_identity


class BindingIdentityTests(unittest.TestCase):
    def test_nondefault_aircraft_and_path_match(self):
        validate_identity(dict(aircraft_id=23, video_rtsp_path='/two_axis_pod_23'),
                          23, '/two_axis_pod_23')

    def test_other_aircraft_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'ID'):
            validate_identity(dict(aircraft_id=24, video_rtsp_path='/two_axis_pod_23'),
                              23, '/two_axis_pod_23')

    def test_other_path_is_rejected(self):
        with self.assertRaisesRegex(ValueError, '路径'):
            validate_identity(dict(aircraft_id=23, video_rtsp_path='/two_axis_pod_24'),
                              23, '/two_axis_pod_23')

    def test_missing_or_malformed_identity_is_rejected(self):
        for identity in (None, True, '23', 23.):
            with self.subTest(identity=identity), self.assertRaises(ValueError):
                validate_identity(dict(aircraft_id=identity, video_rtsp_path='/two_axis_pod_23'),
                                  23, '/two_axis_pod_23')


if __name__ == '__main__':
    unittest.main()
