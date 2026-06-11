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
        self.history = [value]

    def value(self, new_value=None):
        if new_value is not None:
            self._value = new_value
            self.history.append(new_value)
        return self._value


sys.modules["machine"] = types.SimpleNamespace(Pin=FakePin)
time.sleep_us = lambda _value: None
time.ticks_us = lambda: 0
time.ticks_diff = lambda first, second: first - second
time.ticks_add = lambda value, delta: value + delta

swivelcut = importlib.import_module("swivelcut")


class SwivelCutTests(unittest.TestCase):
    def test_startup_state_is_folded(self):
        arm = swivelcut.SwivelCut()

        x, y, t1, t2 = arm.position()

        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(t1, 0.0)
        self.assertAlmostEqual(t2, 180.0)

    def test_step_outputs_idle_high(self):
        arm = swivelcut.SwivelCut()

        self.assertEqual(arm.j1.step.value(), swivelcut.TB6600_INACTIVE)
        self.assertEqual(arm.j2.step.value(), swivelcut.TB6600_INACTIVE)

    def test_common_anode_step_pulse_goes_low_then_returns_high(self):
        arm = swivelcut.SwivelCut()
        original_ticks_us = swivelcut.ticks_us
        now = [0]

        def advancing_ticks_us():
            now[0] += 100000
            return now[0]

        try:
            swivelcut.ticks_us = advancing_ticks_us
            arm._execute(1, 0)
        finally:
            swivelcut.ticks_us = original_ticks_us

        self.assertEqual(
            arm.j1.step.history,
            [
                swivelcut.TB6600_INACTIVE,
                swivelcut.TB6600_ACTIVE,
                swivelcut.TB6600_INACTIVE,
            ],
        )

    def test_common_anode_enable_is_high_and_disable_is_low(self):
        arm = swivelcut.SwivelCut()

        self.assertEqual(arm.en.value(), swivelcut.TB6600_OUTPUTS_ENABLED)
        arm.enable()
        self.assertEqual(arm.en.value(), swivelcut.TB6600_OUTPUTS_ENABLED)
        arm.disable()
        self.assertEqual(arm.en.value(), swivelcut.TB6600_OUTPUTS_DISABLED)

    def test_only_j2_motor_direction_is_inverted(self):
        arm = swivelcut.SwivelCut()

        self.assertFalse(arm.j1.invert)
        self.assertTrue(arm.j2.invert)

    def test_xy_zero_400_is_straight_ahead(self):
        arm = swivelcut.SwivelCut()

        t1, t2 = arm.inverse(0, 400, elbow="up")

        self.assertAlmostEqual(math.degrees(t1), 0.0)
        self.assertAlmostEqual(math.degrees(t2), 0.0)
        x, y = arm.forward(t1, t2)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 400.0, places=6)

    def test_forward_reports_x_sideways_and_y_forward(self):
        arm = swivelcut.SwivelCut()

        x, y = arm.forward(0, 0)

        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 400.0, places=6)

    def test_positive_x_points_to_physical_right(self):
        arm = swivelcut.SwivelCut()

        t1, t2 = arm.inverse(400, 0)
        x, y = arm.forward(t1, t2)

        self.assertAlmostEqual(math.degrees(t1), 90.0, places=6)
        self.assertAlmostEqual(math.degrees(t2), 0.0, places=6)
        self.assertAlmostEqual(x, 400.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)

    def test_near_origin_uses_in_limit_folded_solution(self):
        arm = swivelcut.SwivelCut()

        t1, t2 = arm.inverse(20, 20)

        self.assertAlmostEqual(math.degrees(t1), -40.945, places=3)
        self.assertAlmostEqual(math.degrees(t2), 171.890, places=3)

    def test_line_crossing_inner_hole_is_rejected_before_motion(self):
        arm = swivelcut.SwivelCut()
        arm.L1 = 500
        arm.L2 = 100
        moves = []
        arm.move_to_xy = lambda *args, **kwargs: moves.append(("xy", args))
        arm.move_to_angles = lambda *args, **kwargs: moves.append(("angles", args))

        with self.assertRaisesRegex(ValueError, "crosses unreachable"):
            arm.cut_line(600, 0, -600, 0, seg_mm=1000)

        self.assertEqual(moves, [])

    def test_interrupted_move_resynchronizes_joint_angles(self):
        arm = swivelcut.SwivelCut()

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

    def test_xy_can_move_only_j1(self):
        arm = swivelcut.SwivelCut()
        arm.t1 = math.radians(10)
        arm.t2 = math.radians(120)
        moves = []
        arm.move_to_angles = lambda t1, t2: moves.append((t1, t2))
        solved_t1, _ = arm.inverse(200, 100, "up")

        arm.move_joint_to_xy(1, 200, 100, "up")

        self.assertAlmostEqual(moves[0][0], math.degrees(solved_t1))
        self.assertAlmostEqual(moves[0][1], 120)

    def test_xy_can_move_only_j2(self):
        arm = swivelcut.SwivelCut()
        arm.t1 = math.radians(10)
        arm.t2 = math.radians(120)
        moves = []
        arm.move_to_angles = lambda t1, t2: moves.append((t1, t2))
        _, solved_t2 = arm.inverse(200, 100, "down")

        arm.move_joint_to_xy(2, 200, 100, "down")

        self.assertAlmostEqual(moves[0][0], 10)
        self.assertAlmostEqual(moves[0][1], math.degrees(solved_t2))

    def test_cut_line_rejects_nonpositive_segment_length(self):
        arm = swivelcut.SwivelCut()

        with self.assertRaisesRegex(ValueError, "greater than zero"):
            arm.cut_line(100, 100, 200, 100, seg_mm=0)

    def test_straight_cut_requires_unfolding_first(self):
        arm = swivelcut.SwivelCut()

        with self.assertRaisesRegex(ValueError, r"first use XY <x> <y>"):
            arm.cut_line(100, 100, 200, 100)


if __name__ == "__main__":
    unittest.main()
