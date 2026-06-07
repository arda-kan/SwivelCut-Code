import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

# test_scara_arm installs the desktop machine/time substitutes before import.
import test_scara_arm  # noqa: F401
from serial_console import ArmConsole


class FakeArm:
    def __init__(self):
        self.calls = []
        self.enabled = False
        self.pose = (0.0, 0.0, 0.0, 180.0)

    def enable(self):
        self.enabled = True
        self.calls.append(("enable",))

    def disable(self):
        self.enabled = False
        self.calls.append(("disable",))

    def set_folded_start(self):
        self.calls.append(("folded",))

    def move_joint(self, joint, angle):
        self.calls.append(("joint", joint, angle))

    def move_to_angles(self, t1, t2):
        self.calls.append(("angles", t1, t2))

    def move_to_xy(self, x, y, elbow):
        self.calls.append(("xy", x, y, elbow))

    def cut_line(self, x0, y0, x1, y1, elbow):
        self.calls.append(("cut", x0, y0, x1, y1, elbow))

    def position(self):
        return self.pose


class ArmConsoleTests(unittest.TestCase):
    def setUp(self):
        self.arm = FakeArm()
        self.output = []
        self.console = ArmConsole(self.arm, self.output.append)

    def test_motion_is_rejected_until_folded_pose_is_confirmed(self):
        with self.assertRaisesRegex(ValueError, "ARM FOLDED"):
            self.console.execute("J1 10")

    def test_arm_folded_resets_state_before_enabling(self):
        self.console.execute("ARM FOLDED")

        self.assertTrue(self.console.armed)
        self.assertEqual(
            self.arm.calls, [("disable",), ("folded",), ("enable",)]
        )

    def test_degree_and_cartesian_commands(self):
        self.console.execute("ARM FOLDED")
        self.console.execute("J1 30")
        self.console.execute("J2 -45")
        self.console.execute("ANGLES 20 120")
        self.console.execute("XY 200 100 DOWN")
        self.console.execute("CUT 100 100 250 100 UP")

        self.assertIn(("joint", 1, 30.0), self.arm.calls)
        self.assertIn(("joint", 2, -45.0), self.arm.calls)
        self.assertIn(("angles", 20.0, 120.0), self.arm.calls)
        self.assertIn(("xy", 200.0, 100.0, "down"), self.arm.calls)
        self.assertIn(("cut", 100.0, 100.0, 250.0, 100.0, "up"), self.arm.calls)

    def test_shutdown_disables_drivers(self):
        self.console.execute("ARM FOLDED")
        self.console.shutdown()

        self.assertFalse(self.console.armed)
        self.assertFalse(self.arm.enabled)


if __name__ == "__main__":
    unittest.main()
