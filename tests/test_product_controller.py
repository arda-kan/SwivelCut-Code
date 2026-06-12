import importlib
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

    def __init__(self, pin=0, mode=None, value=1):
        self.pin = pin
        self.mode = mode
        self._value = value
        self.history = [value]

    def value(self, new_value=None):
        if new_value is not None:
            self._value = new_value
            self.history.append(new_value)
        return self._value


class FakeADC:
    ATTN_11DB = 3

    def __init__(self, _pin=None, values=None):
        self.values = list(values or [65535])

    def atten(self, _value):
        pass

    def read_u16(self):
        if len(self.values) > 1:
            return self.values.pop(0)
        return self.values[0]


class FakeSoftI2C:
    def __init__(self, **_kwargs):
        pass


sys.modules["machine"] = types.SimpleNamespace(
    ADC=FakeADC, Pin=FakePin, SoftI2C=FakeSoftI2C
)
time.sleep_ms = lambda _value: None
time.sleep_us = lambda _value: None
time.ticks_ms = lambda: 0
time.ticks_us = lambda: 0
time.ticks_diff = lambda first, second: first - second
time.ticks_add = lambda value, delta: value + delta

product_controller = importlib.import_module("product_controller")


class StubButton:
    def __init__(self):
        self.events = []

    def update(self, _now=None):
        return self.events.pop(0) if self.events else None


class StubHeadDetector:
    def __init__(self, head):
        self.head = head

    def update(self):
        return self.head


class StubBlade:
    def __init__(self):
        self.extended = False
        self.actions = []

    def extend(self):
        self.extended = True
        self.actions.append("extend")

    def retract(self):
        self.extended = False
        self.actions.append("retract")

    def stop(self):
        self.actions.append("stop")

    def home(self):
        self.actions.append("home")
        self.extended = False


class StubArm:
    def __init__(self):
        self.encoder_calibrated = True
        self.teach_points = []
        self.teach_mode = None
        self.motion_abort_check = None
        self.disabled = 0
        self.replays = 0
        self.samples = [(1.0, 179.0), (2.0, 178.0), (3.0, 177.0)]

    def disable(self):
        self.disabled += 1

    def encoder_angles(self):
        if len(self.samples) > 1:
            return self.samples.pop(0)
        return self.samples[0]

    def _validate_angles(self, _j1, _j2):
        pass

    def sync_from_encoders(self):
        return self.encoder_angles()

    @staticmethod
    def stabilize_teach_points(points, _smoothing, _deviation):
        return [
            (elapsed, j1 + 0.25, j2 - 0.25)
            for elapsed, j1, j2 in points
        ]

    def replay_teach(self, before_path=None):
        if before_path is not None:
            before_path()
        self.replays += 1


class ProductControllerTests(unittest.TestCase):
    def make_controller(self, head):
        self.arm = StubArm()
        self.blade = StubBlade()
        self.start = StubButton()
        self.stabilization = StubButton()
        self.repeat = StubButton()
        self.messages = []
        controller = product_controller.ProductController(
            arm=self.arm,
            head_detector=StubHeadDetector(head),
            blade=self.blade,
            start_button=self.start,
            stabilization_button=self.stabilization,
            repeat_button=self.repeat,
            output=self.messages.append,
        )
        controller.head = head
        controller.last_head_sample = 0
        return controller

    def test_head_detector_classifies_and_debounces_resistor_ids(self):
        detector = product_controller.HeadDetector(
            FakeADC(values=[32000, 32000, 32000]), stable_reads=3
        )

        self.assertEqual(detector.update(), product_controller.HEAD_UNKNOWN)
        self.assertEqual(detector.update(), product_controller.HEAD_UNKNOWN)
        self.assertEqual(detector.update(), product_controller.HEAD_TRACING)
        self.assertEqual(
            product_controller.HeadDetector.classify(11000),
            product_controller.HEAD_CUTTING,
        )
        self.assertEqual(
            product_controller.HeadDetector.classify(62000),
            product_controller.HEAD_DISCONNECTED,
        )

    def test_button_emits_one_press_after_debounce(self):
        pin = FakePin(value=1)
        button = product_controller.DebouncedButton(pin, debounce_ms=35)

        pin.value(0)
        self.assertIsNone(button.update(10))
        self.assertIsNone(button.update(44))
        self.assertEqual(button.update(45), "pressed")
        self.assertIsNone(button.update(80))

    def test_start_stop_records_raw_path_with_tracing_head(self):
        controller = self.make_controller(product_controller.HEAD_TRACING)

        controller.start_teaching(now=100)
        controller._record_sample(150)
        controller.stop_teaching(now=200)

        self.assertEqual(controller.state, product_controller.STATE_IDLE)
        self.assertGreaterEqual(len(controller.raw_teach_points), 2)
        self.assertIn("TAUGHT", self.messages[-1])

    def test_stabilization_is_applied_only_when_enabled(self):
        controller = self.make_controller(product_controller.HEAD_CUTTING)
        controller.raw_teach_points = [
            (0, 1.0, 179.0),
            (50, 2.0, 178.0),
        ]

        raw = controller._path_for_cut()
        controller.stabilization_enabled = True
        stable = controller._path_for_cut()

        self.assertEqual(raw[0][1], 1.0)
        self.assertEqual(stable[0][1], 1.25)

    def test_cut_extends_replays_retracts_and_disables(self):
        controller = self.make_controller(product_controller.HEAD_CUTTING)
        controller.raw_teach_points = [
            (0, 1.0, 179.0),
            (50, 2.0, 178.0),
        ]

        controller.cut()

        self.assertEqual(self.arm.replays, 1)
        self.assertEqual(self.blade.actions, ["extend", "retract"])
        self.assertGreater(self.arm.disabled, 0)
        self.assertEqual(controller.state, product_controller.STATE_IDLE)
        self.assertIsNone(self.arm.motion_abort_check)

    def test_repeat_requires_cutting_head(self):
        controller = self.make_controller(product_controller.HEAD_TRACING)
        controller.raw_teach_points = [
            (0, 1.0, 179.0),
            (50, 2.0, 178.0),
        ]

        with self.assertRaisesRegex(ValueError, "cutting head"):
            controller._handle_idle_press("repeat", 0)

    def test_blade_retracts_when_replay_raises(self):
        controller = self.make_controller(product_controller.HEAD_CUTTING)
        controller.raw_teach_points = [
            (0, 1.0, 179.0),
            (50, 2.0, 178.0),
        ]

        def fail(before_path=None):
            if before_path is not None:
                before_path()
            raise product_controller.MotionAborted("stop")

        self.arm.replay_teach = fail
        with self.assertRaises(product_controller.MotionAborted):
            controller.cut()

        self.assertEqual(self.blade.actions, ["extend", "retract"])
        self.assertFalse(self.blade.extended)


if __name__ == "__main__":
    unittest.main()
