import math
import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from as5600 import (  # noqa: E402
    AS5600,
    EncoderError,
    MultiTurnEncoder,
    REG_RAW_ANGLE,
    REG_STATUS,
)


class FakeI2C:
    def __init__(self, angles, status=0x20):
        self.angles = list(angles)
        self.status = status

    def readfrom_mem(self, _address, register, count):
        if register == REG_STATUS:
            return bytes((self.status,))
        if register == REG_RAW_ANGLE and count == 2:
            angle = self.angles[0]
            if len(self.angles) > 1:
                self.angles.pop(0)
            return bytes(((angle >> 8) & 0x0F, angle & 0xFF))
        raise AssertionError("unexpected register read")


class AS5600Tests(unittest.TestCase):
    def test_raw_angle_is_12_bit(self):
        sensor = AS5600(FakeI2C([0x0ABC]))

        self.assertEqual(sensor.read_raw(), 0x0ABC)

    def test_missing_or_bad_magnet_is_rejected(self):
        for status, state in ((0x00, "missing"), (0x10, "weak"), (0x08, "strong")):
            sensor = AS5600(FakeI2C([0], status=status))
            self.assertEqual(sensor.magnet_state(), state)
            with self.assertRaises(EncoderError):
                sensor.require_magnet("test")

    def test_multiturn_encoder_unwraps_forward_boundary(self):
        sensor = AS5600(FakeI2C([4090] * 7 + [5]))
        encoder = MultiTurnEncoder(sensor)

        encoder.calibrate(0.0)
        measured = encoder.update()

        self.assertAlmostEqual(measured, 11 * 2 * math.pi / 4096)

    def test_multiturn_encoder_unwraps_reverse_boundary(self):
        sensor = AS5600(FakeI2C([5] * 7 + [4090]))
        encoder = MultiTurnEncoder(sensor)

        encoder.calibrate(0.0)
        measured = encoder.update()

        self.assertAlmostEqual(measured, -11 * 2 * math.pi / 4096)

    def test_encoder_sign_changes_logical_direction(self):
        sensor = AS5600(FakeI2C([100] * 7 + [110]))
        encoder = MultiTurnEncoder(sensor, motor_sign=-1)

        encoder.calibrate(1.0)

        self.assertLess(encoder.update(), 1.0)

    def test_calibration_rejects_unstable_angle_samples(self):
        sensor = AS5600(FakeI2C([100, 100, 100, 500, 100, 500, 100]))
        encoder = MultiTurnEncoder(sensor, name="J1 encoder")

        with self.assertRaisesRegex(EncoderError, "reading is unstable"):
            encoder.calibrate(0.0)

    def test_calibration_ignores_initial_stale_samples(self):
        sensor = AS5600(FakeI2C([900, 400, 100, 101, 100, 101, 100, 100]))
        encoder = MultiTurnEncoder(sensor)

        encoder.calibrate(0.0)

        self.assertAlmostEqual(encoder.update(), 0.0)


if __name__ == "__main__":
    unittest.main()
