import importlib
import math
import pathlib
import sys
import time
import types
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


class FakePin:
    OUT = 1
    IN = 2
    PULL_UP = 3

    def __init__(self, pin, mode, value=0):
        self.pin = pin
        self.mode = mode
        self._value = value

    def value(self, new_value=None):
        if new_value is not None:
            self._value = new_value
        return self._value


sys.modules["machine"] = types.SimpleNamespace(Pin=FakePin)
time.sleep_us = lambda _value: None
time.ticks_us = lambda: 0
time.ticks_diff = lambda first, second: first - second
time.ticks_add = lambda value, delta: value + delta

scara_arm = importlib.import_module("scara_arm")


class ScaraArmTests(unittest.TestCase):
    def test_inverse_normalizes_equivalent_shoulder_angle(self):
        arm = scara_arm.ScaraArm()

        t1, t2 = arm.inverse(-300, -1, elbow="up")

        self.assertGreaterEqual(math.degrees(t1), scara_arm.J1_MIN)
        self.assertLessEqual(math.degrees(t1), scara_arm.J1_MAX)
        x, y = arm.forward(t1, t2)
        self.assertAlmostEqual(x, -300, places=6)
        self.assertAlmostEqual(y, -1, places=6)

    def test_line_crossing_inner_hole_is_rejected_before_motion(self):
        arm = scara_arm.ScaraArm()
        arm.L1 = 500
        arm.L2 = 100
        moves = []
        arm.move_to_xy = lambda *args, **kwargs: moves.append(("xy", args))
        arm.move_to_angles = lambda *args, **kwargs: moves.append(("angles", args))

        with self.assertRaisesRegex(ValueError, "crosses unreachable"):
            arm.cut_line(600, 0, -600, 0, seg_mm=1000)

        self.assertEqual(moves, [])

    def test_interrupted_move_resynchronizes_joint_angles(self):
        arm = scara_arm.ScaraArm()

        def interrupted_execute(_d1, _d2, _ramp_in, _ramp_out):
            arm.j1.pos = 100
            arm.j2.pos = -200
            raise KeyboardInterrupt()

        arm._execute = interrupted_execute
        with self.assertRaises(KeyboardInterrupt):
            arm.move_to_angles(30, -45)

        expected_t1 = arm.j1.pos / arm.j1.steps_per_rad
        expected_t2 = (
            arm.j2.pos / arm.j2.steps_per_rad - arm.coupling * expected_t1
        )
        self.assertAlmostEqual(arm.t1, expected_t1)
        self.assertAlmostEqual(arm.t2, expected_t2)

    def test_cut_line_rejects_nonpositive_segment_length(self):
        arm = scara_arm.ScaraArm()

        with self.assertRaisesRegex(ValueError, "greater than zero"):
            arm.cut_line(100, 100, 200, 100, seg_mm=0)


if __name__ == "__main__":
    unittest.main()
