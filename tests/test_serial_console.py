import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

# test_swivelcut installs desktop substitutes for the MicroPython modules.
import test_swivelcut  # noqa: F401
from serial_console import SwivelCutConsole


class FakeArm:
    def __init__(self):
        self.calls = []
        self.enabled = False
        self.pose = (0.0, 0.0, 0.0, 180.0)
        self.encoder_pose = (0.0, 180.0)
        self.taught = []

    def enable(self):
        self.enabled = True
        self.calls.append(("enable",))

    def disable(self):
        self.enabled = False
        self.calls.append(("disable",))

    def set_folded_start(self):
        self.calls.append(("folded",))

    def calibrate_encoders(self):
        self.calls.append(("calibrate",))

    def calibrate_j1_encoder(self):
        self.calls.append(("calibrate_j1",))

    def encoder_status(self):
        return "ok", "ok"

    def encoder_angles(self):
        return self.encoder_pose

    def move_joint(self, joint, angle):
        self.calls.append(("joint", joint, angle))

    def move_to_angles(self, t1, t2):
        self.calls.append(("angles", t1, t2))

    def move_to_xy(self, x, y, elbow):
        self.calls.append(("xy", x, y, elbow))

    def move_joint_to_xy(self, joint, x, y, elbow):
        self.calls.append(("xy_joint", joint, x, y, elbow))

    def cut_line(self, x0, y0, x1, y1, elbow):
        self.calls.append(("cut", x0, y0, x1, y1, elbow))

    def record_teach(self, duration, sample_hz, j1_only=False):
        self.calls.append(("teach", duration, sample_hz, j1_only))
        self.enabled = False
        self.taught = [(0, 0, 180), (1000, 10, 160)]
        return len(self.taught)

    def replay_teach(self):
        self.calls.append(("play",))
        self.enabled = True
        return len(self.taught)

    def clear_teach(self):
        self.calls.append(("clear",))
        self.taught = []

    def position(self):
        return self.pose


class SwivelCutConsoleTests(unittest.TestCase):
    def setUp(self):
        self.arm = FakeArm()
        self.output = []
        self.console = SwivelCutConsole(self.arm, self.output.append)

    def test_motion_is_rejected_until_folded_pose_is_confirmed(self):
        with self.assertRaisesRegex(ValueError, "ARM FOLDED"):
            self.console.execute("J1 10")

    def test_arm_folded_resets_state_before_enabling(self):
        self.console.execute("ARM FOLDED")

        self.assertTrue(self.console.armed)
        self.assertEqual(
            self.arm.calls,
            [("disable",), ("folded",), ("calibrate",), ("enable",)],
        )

    def test_arm_j1_calibrates_only_j1_and_blocks_other_motion(self):
        self.console.execute("ARM J1")
        self.console.execute("J1 10")

        self.assertEqual(
            self.arm.calls,
            [
                ("disable",),
                ("folded",),
                ("calibrate_j1",),
                ("enable",),
                ("joint", 1, 10.0),
            ],
        )
        with self.assertRaisesRegex(ValueError, "only J1"):
            self.console.execute("J2 10")
        with self.assertRaisesRegex(ValueError, "only J1"):
            self.console.execute("XYJ1 100 100")

    def test_arm_j1_can_teach_and_replay(self):
        self.console.execute("ARM J1")
        with self.assertRaisesRegex(ValueError, "TEACH J1"):
            self.console.execute("TEACH 2 25")
        self.console.execute("TEACH J1 2 25")

        self.assertFalse(self.console.armed)
        self.assertEqual(self.console.arm_mode, "j1")
        self.assertIn(("teach", 2.0, 25.0, True), self.arm.calls)

        self.console.execute("PLAY")

        self.assertTrue(self.console.armed)
        self.assertEqual(self.console.arm_mode, "j1")
        self.assertIn(("play",), self.arm.calls)

    def test_degree_and_cartesian_commands(self):
        self.console.execute("ARM FOLDED")
        self.console.execute("J1 30")
        self.console.execute("J2 -45")
        self.console.execute("ANGLES 20 120")
        self.console.execute("XY 200 100 DOWN")
        self.console.execute("XYJ1 150 100 UP")
        self.console.execute("XYJ2 150 100 DOWN")
        self.console.execute("CUT 100 100 250 100 UP")

        self.assertIn(("joint", 1, 30.0), self.arm.calls)
        self.assertIn(("joint", 2, -45.0), self.arm.calls)
        self.assertIn(("angles", 20.0, 120.0), self.arm.calls)
        self.assertIn(("xy", 200.0, 100.0, "down"), self.arm.calls)
        self.assertIn(("xy_joint", 1, 150.0, 100.0, "up"), self.arm.calls)
        self.assertIn(("xy_joint", 2, 150.0, 100.0, "down"), self.arm.calls)
        self.assertIn(("cut", 100.0, 100.0, 250.0, 100.0, "up"), self.arm.calls)

    def test_cartesian_commands_default_to_in_limit_branch(self):
        self.console.execute("ARM FOLDED")
        self.console.execute("XY 20 20")
        self.console.execute("CUT 20 20 30 20")

        self.assertIn(("xy", 20.0, 20.0, "up"), self.arm.calls)
        self.assertIn(("cut", 20.0, 20.0, 30.0, 20.0, "up"), self.arm.calls)

    def test_shutdown_disables_drivers(self):
        self.console.execute("ARM FOLDED")
        self.console.shutdown()

        self.assertFalse(self.console.armed)
        self.assertFalse(self.arm.enabled)

    def test_encoder_status_is_reported(self):
        self.console.execute("ENC")

        self.assertEqual(self.output[-1], "ENC J1=ok J2=ok J1=0.00 J2=180.00")

    def test_teach_disarms_then_play_rearms(self):
        self.console.execute("ARM FOLDED")
        self.console.execute("TEACH 2 25")

        self.assertFalse(self.console.armed)
        self.assertIn(("teach", 2.0, 25.0, False), self.arm.calls)

        self.console.execute("PLAY")

        self.assertTrue(self.console.armed)
        self.assertIn(("play",), self.arm.calls)

    def test_clear_erases_taught_path(self):
        self.console.execute("CLEAR")

        self.assertIn(("clear",), self.arm.calls)


if __name__ == "__main__":
    unittest.main()
