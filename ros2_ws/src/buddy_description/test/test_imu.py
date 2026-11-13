#!/usr/bin/env python3
"""Protocol, calibration, clock wrap and reset regressions; no ROS needed."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from buddy_imu import ImuProcessor, parse_imu_line


def sample(t, gz=131):
    return parse_imu_line(f'I:{t},0,0,16384,0,0,{gz}')


class TestImu(unittest.TestCase):
    def test_units_and_axis_sign(self):
        s = parse_imu_line('I:123,16384,-16384,0,131,-131,0')
        self.assertAlmostEqual(s.acceleration[0], 9.80665)
        self.assertAlmostEqual(s.acceleration[1], -9.80665)
        self.assertAlmostEqual(s.gyro[0], math.pi/180)
        self.assertAlmostEqual(s.gyro[1], -math.pi/180)

    def test_malformed_and_out_of_range(self):
        for line in ('E:1,2', 'I:0,1,2', 'I:0,0,0,0,0,nan,0',
                     'I:-1,0,0,0,0,0,0', 'I:0,0,0,0,0,0,32768'):
            with self.assertRaises(ValueError):
                parse_imu_line(line)

    def test_motion_resets_calibration_window(self):
        p = ImuProcessor(samples=3, warmup=0)
        p.update(sample(0), 1_000_000_000, True)
        p.update(sample(20_000), 1_020_000_000, True)
        p.update(sample(40_000), 1_040_000_000, False)
        self.assertIsNone(p.bias)
        for t in (60_000, 80_000, 100_000):
            _, corrected = p.update(sample(t), 1_000_000_000+t*1000, True)
        self.assertAlmostEqual(corrected[2], 0)
        # Once calibrated, a real turn must not be learned as a new bias.
        _, corrected = p.update(sample(120_000, 1310), 1_120_000_000, False)
        self.assertAlmostEqual(corrected[2], math.radians(9))

    def test_wrap_keeps_bias_but_reboot_discards_it(self):
        p = ImuProcessor(samples=2, warmup=0)
        p.update(sample(2**32-40_000), 1_000_000_000, True)
        p.update(sample(2**32-20_000), 1_020_000_000, True)
        stamp, _ = p.update(sample(0), 1_040_000_000, False)
        self.assertEqual(stamp, 1_040_000_000)
        self.assertIsNotNone(p.bias)
        p.update(sample(20_000), 1_060_000_000, False)
        p.update(sample(100), 1_080_000_000, False)
        self.assertIsNone(p.bias)

    def test_no_calibration_under_high_rotation(self):
        p = ImuProcessor(samples=2, warmup=0)
        for t in range(0, 100_000, 20_000):
            p.update(sample(t, 10_000), 1_000_000_000+t*1000, True)
        self.assertIsNone(p.bias)


if __name__ == '__main__':
    unittest.main()
