import unittest
from utils import motor_angle2value, motor_value2angle

class TestMotorFunctions(unittest.TestCase):

    def test_motor_angle2value(self):
        self.assertEqual(motor_angle2value(0), 0)
        self.assertEqual(motor_angle2value(360), 4095)
        self.assertAlmostEqual(motor_angle2value(180), 2047.5, places=1)
        self.assertAlmostEqual(motor_angle2value(90), 1023.75, places=1)

    def test_motor_value2angle(self):
        self.assertEqual(motor_value2angle(0), 0)
        self.assertEqual(motor_value2angle(4095), 360)
        self.assertAlmostEqual(motor_value2angle(2047.5), 180, places=1)
        self.assertAlmostEqual(motor_value2angle(1023.75), 90, places=1)

    def test_round_trip(self):
        for angle in [0, 90, 180, 270, 360]:
            value = motor_angle2value(angle)
            result_angle = motor_value2angle(value)
            self.assertAlmostEqual(result_angle, angle, places=1)

        for value in [0, 1023.75, 2047.5, 3071.25, 4095]:
            angle = motor_value2angle(value)
            result_value = motor_angle2value(angle)
            self.assertAlmostEqual(result_value, value, places=1)

if __name__ == '__main__':
    unittest.main()
