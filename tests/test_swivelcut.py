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

    def __init__(self, pin, mode=None, value=0):
        self.pin = pin
        self.mode = mode
        self._value = value

    def value(self, new_value=None):
        if new_value is not None:
            self._value = new_value
        return self._value


class FakeSoftI2C:
    def __init__(self, **_kwargs):
        pass


sys.modules["machine"] = types.SimpleNamespace(Pin=FakePin, SoftI2C=FakeSoftI2C)
time.sleep_ms = lambda _value: None
time.sleep_us = lambda _value: None
time.ticks_ms = lambda: 0
time.ticks_us = lambda: 0
time.ticks_diff = lambda first, second: first - second
time.ticks_add = lambda value, delta: value + delta

swivelcut = importlib.import_module("swivelcut")


class SwivelCutTests(unittest.TestCase):
    def test_startup_state_is_folded(self):
        arm = swivelcut.SwivelCut(auto_encoders=False)

        x, y, t1, t2 = arm.position()

        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(t1, 0.0)
        self.assertAlmostEqual(t2, 180.0)

    def test_step_outputs_idle_high(self):
        arm = swivelcut.SwivelCut(auto_encoders=False)

        self.assertEqual(arm.j1.step.value(), 1)
        self.assertEqual(arm.j2.step.value(), 1)

    def test_only_j2_motor_direction_is_inverted(self):
        arm = swivelcut.SwivelCut(auto_encoders=False)

        self.assertFalse(arm.j1.invert)
        self.assertTrue(arm.j2.invert)

    def test_xy_zero_400_is_straight_ahead(self):
        arm = swivelcut.SwivelCut(auto_encoders=False)

        t1, t2 = arm.inverse(0, 400, elbow="up")

        self.assertAlmostEqual(math.degrees(t1), 0.0)
        self.assertAlmostEqual(math.degrees(t2), 0.0)
        x, y = arm.forward(t1, t2)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 400.0, places=6)

    def test_forward_reports_x_sideways_and_y_forward(self):
        arm = swivelcut.SwivelCut(auto_encoders=False)

        x, y = arm.forward(0, 0)

        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 400.0, places=6)

    def test_positive_x_points_to_physical_right(self):
        arm = swivelcut.SwivelCut(auto_encoders=False)

        t1, t2 = arm.inverse(400, 0)
        x, y = arm.forward(t1, t2)

        self.assertAlmostEqual(math.degrees(t1), 90.0, places=6)
        self.assertAlmostEqual(math.degrees(t2), 0.0, places=6)
        self.assertAlmostEqual(x, 400.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)

    def test_near_origin_uses_in_limit_folded_solution(self):
        arm = swivelcut.SwivelCut(auto_encoders=False)

        t1, t2 = arm.inverse(20, 20)

        self.assertAlmostEqual(math.degrees(t1), -40.945, places=3)
        self.assertAlmostEqual(math.degrees(t2), 171.890, places=3)

    def test_line_crossing_inner_hole_is_rejected_before_motion(self):
        arm = swivelcut.SwivelCut(auto_encoders=False)
        arm.L1 = 500
        arm.L2 = 100
        moves = []
        arm.move_to_xy = lambda *args, **kwargs: moves.append(("xy", args))
        arm.move_to_angles = lambda *args, **kwargs: moves.append(("angles", args))

        with self.assertRaisesRegex(ValueError, "crosses unreachable"):
            arm.cut_line(600, 0, -600, 0, seg_mm=1000)

        self.assertEqual(moves, [])

    def test_interrupted_move_resynchronizes_joint_angles(self):
        arm = swivelcut.SwivelCut(auto_encoders=False)

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
        arm = swivelcut.SwivelCut(auto_encoders=False)
        arm.t1 = math.radians(10)
        arm.t2 = math.radians(120)
        moves = []
        arm.move_to_angles = lambda t1, t2: moves.append((t1, t2))
        solved_t1, _ = arm.inverse(200, 100, "up")

        arm.move_joint_to_xy(1, 200, 100, "up")

        self.assertAlmostEqual(moves[0][0], math.degrees(solved_t1))
        self.assertAlmostEqual(moves[0][1], 120)

    def test_xy_can_move_only_j2(self):
        arm = swivelcut.SwivelCut(auto_encoders=False)
        arm.t1 = math.radians(10)
        arm.t2 = math.radians(120)
        moves = []
        arm.move_to_angles = lambda t1, t2: moves.append((t1, t2))
        _, solved_t2 = arm.inverse(200, 100, "down")

        arm.move_joint_to_xy(2, 200, 100, "down")

        self.assertAlmostEqual(moves[0][0], 10)
        self.assertAlmostEqual(moves[0][1], math.degrees(solved_t2))

    def test_cut_line_rejects_nonpositive_segment_length(self):
        arm = swivelcut.SwivelCut(auto_encoders=False)

        with self.assertRaisesRegex(ValueError, "greater than zero"):
            arm.cut_line(100, 100, 200, 100, seg_mm=0)

    def test_straight_cut_requires_unfolding_first(self):
        arm = swivelcut.SwivelCut(auto_encoders=False)

        with self.assertRaisesRegex(ValueError, r"first use XY <x> <y>"):
            arm.cut_line(100, 100, 200, 100)

    def test_small_encoder_error_is_corrected(self):
        target_t1 = math.radians(10)
        target_t2 = math.radians(120)

        class FakeEncoder:
            def __init__(self, values):
                self.values = list(values)

            def update(self):
                return self.values.pop(0)

        j1_values = [
            math.radians(9) * swivelcut.GEAR_J1,
            target_t1 * swivelcut.GEAR_J1,
        ]
        j2_values = [
            target_t2 * swivelcut.GEAR_J2,
            target_t2 * swivelcut.GEAR_J2,
        ]
        arm = swivelcut.SwivelCut(
            encoders=(FakeEncoder(j1_values), FakeEncoder(j2_values))
        )
        arm.encoder_calibrated = True
        corrections = []

        def execute(d1, d2, _ramp_in=True, _ramp_out=True):
            corrections.append((d1, d2))
            arm.j1.pos += d1
            arm.j2.pos += d2

        arm._execute = execute
        arm._settle_feedback(target_t1, target_t2)

        self.assertEqual(len(corrections), 1)
        self.assertGreater(corrections[0][0], 0)
        self.assertIsNone(arm.feedback_fault)

    def test_j1_only_mode_calibrates_and_never_reads_j2(self):
        class FakeEncoder:
            def __init__(self, value):
                self.value = value
                self.calibrated = False
                self.updates = 0

            def calibrate(self, _known_angle):
                self.calibrated = True

            def update(self):
                self.updates += 1
                return self.value

            def magnet_state(self):
                return "ok"

        j1 = FakeEncoder(0.0)
        j2 = FakeEncoder(math.radians(180) * swivelcut.GEAR_J2)
        arm = swivelcut.SwivelCut(encoders=(j1, j2))

        arm.calibrate_j1_encoder()

        self.assertTrue(j1.calibrated)
        self.assertFalse(j2.calibrated)
        self.assertEqual(j2.updates, 0)
        self.assertEqual(arm.encoder_status(), ("ok", "not installed"))

    def test_j1_only_mode_blocks_j2_motion(self):
        class FakeEncoder:
            def calibrate(self, _known_angle):
                pass

            def update(self):
                return 0.0

        arm = swivelcut.SwivelCut(encoders=(FakeEncoder(), FakeEncoder()))
        arm.calibrate_j1_encoder()

        with self.assertRaisesRegex(Exception, "blocks J2 motion"):
            arm.move_joint(2, 170)

    def test_large_encoder_error_disables_drivers(self):
        class FakeEncoder:
            def __init__(self, value):
                self.value = value

            def update(self):
                return self.value

        arm = swivelcut.SwivelCut(
            encoders=(
                FakeEncoder(math.radians(-20) * swivelcut.GEAR_J1),
                FakeEncoder(math.radians(180) * swivelcut.GEAR_J2),
            )
        )
        arm.encoder_calibrated = True
        arm.enable()

        with self.assertRaisesRegex(Exception, "too large"):
            arm._settle_feedback(0.0, math.radians(180))

        self.assertEqual(arm.en.value(), 1)
        self.assertIsNotNone(arm.feedback_fault)
        self.assertFalse(arm.encoder_calibrated)

    def test_correction_that_increases_error_is_stopped(self):
        class FakeEncoder:
            def __init__(self, values):
                self.values = list(values)

            def update(self):
                return self.values.pop(0)

        arm = swivelcut.SwivelCut(
            encoders=(
                FakeEncoder(
                    [
                        math.radians(-1) * swivelcut.GEAR_J1,
                        math.radians(-3) * swivelcut.GEAR_J1,
                    ]
                ),
                FakeEncoder(
                    [
                        math.radians(180) * swivelcut.GEAR_J2,
                        math.radians(180) * swivelcut.GEAR_J2,
                    ]
                ),
            )
        )
        arm.encoder_calibrated = True

        def execute(d1, d2, _ramp_in=True, _ramp_out=True):
            arm.j1.pos += d1
            arm.j2.pos += d2

        arm._execute = execute

        with self.assertRaisesRegex(Exception, "check encoder signs"):
            arm._settle_feedback(0.0, math.radians(180))

        self.assertEqual(arm.en.value(), 1)


if __name__ == "__main__":
    unittest.main()
