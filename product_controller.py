"""Four-button SwivelCut product controls for MicroPython on ESP32."""

from machine import ADC, Pin
from time import sleep_ms, ticks_add, ticks_diff, ticks_ms

from as5600 import EncoderError
from swivelcut import (
    MotionAborted,
    SwivelCut,
    TEACH_DEFAULT_HZ,
    TEACH_MAX_SECONDS,
    TEACH_MAX_DEVIATION_DEG,
    TEACH_SMOOTHING_MS,
)

# The power button is a physical latching switch and is intentionally not read
# by the ESP32. These are the three momentary logic buttons, wired to ground.
PIN_START_STOP = 18
PIN_STABILIZATION = 19
PIN_REPEAT = 23

# Head ID wiring: 3V3 -> 10k pull-up -> GPIO34 -> attachment resistor -> GND.
# Fit 10k in the tracing head and 2.2k in the cutting head.
PIN_HEAD_ID = 34
HEAD_TRACING = "tracing"
HEAD_CUTTING = "cutting"
HEAD_UNKNOWN = "unknown"
HEAD_DISCONNECTED = "disconnected"
TRACING_ADC_MIN = 24000
TRACING_ADC_MAX = 41000
CUTTING_ADC_MIN = 6500
CUTTING_ADC_MAX = 18000
DISCONNECTED_ADC_MIN = 56000
HEAD_STABLE_READS = 4

# A small H-bridge drives the blade actuator. Set BLADE_REVERSE_TO_RETRACT to
# False for a one-direction cam mechanism that advances once per activation.
PIN_BLADE_A = 13
PIN_BLADE_B = 14
BLADE_DRIVE_MS = 500
BLADE_REVERSE_TO_RETRACT = True

BUTTON_DEBOUNCE_MS = 35
MAIN_LOOP_MS = 10
HEAD_SAMPLE_MS = 20

STATE_IDLE = "idle"
STATE_TEACHING = "teaching"
STATE_CUTTING = "cutting"
STATE_ERROR = "error"


class DebouncedButton:
    def __init__(self, pin, debounce_ms=BUTTON_DEBOUNCE_MS):
        self.pin = pin if hasattr(pin, "value") else Pin(
            pin, Pin.IN, Pin.PULL_UP
        )
        self.debounce_ms = debounce_ms
        self.raw_pressed = self.pin.value() == 0
        self.pressed = self.raw_pressed
        self.changed_at = ticks_ms()

    def update(self, now=None):
        if now is None:
            now = ticks_ms()
        raw_pressed = self.pin.value() == 0
        if raw_pressed != self.raw_pressed:
            self.raw_pressed = raw_pressed
            self.changed_at = now
        if (
            raw_pressed != self.pressed
            and ticks_diff(now, self.changed_at) >= self.debounce_ms
        ):
            self.pressed = raw_pressed
            return "pressed" if self.pressed else "released"
        return None


class HeadDetector:
    def __init__(self, adc=None, stable_reads=HEAD_STABLE_READS):
        self.adc = adc if adc is not None else ADC(Pin(PIN_HEAD_ID))
        if hasattr(self.adc, "atten") and hasattr(ADC, "ATTN_11DB"):
            self.adc.atten(ADC.ATTN_11DB)
        self.stable_reads = stable_reads
        self.candidate = None
        self.candidate_count = 0
        self.head = HEAD_UNKNOWN
        self.last_value = None

    @staticmethod
    def classify(value):
        if value >= DISCONNECTED_ADC_MIN:
            return HEAD_DISCONNECTED
        if CUTTING_ADC_MIN <= value <= CUTTING_ADC_MAX:
            return HEAD_CUTTING
        if TRACING_ADC_MIN <= value <= TRACING_ADC_MAX:
            return HEAD_TRACING
        return HEAD_UNKNOWN

    def update(self):
        value = self.adc.read_u16()
        self.last_value = value
        candidate = self.classify(value)
        if candidate == self.candidate:
            self.candidate_count += 1
        else:
            self.candidate = candidate
            self.candidate_count = 1
        if self.candidate_count >= self.stable_reads:
            self.head = candidate
        return self.head


class BladeActuator:
    def __init__(
        self,
        pin_a=PIN_BLADE_A,
        pin_b=PIN_BLADE_B,
        drive_ms=BLADE_DRIVE_MS,
        reverse_to_retract=BLADE_REVERSE_TO_RETRACT,
    ):
        self.a = pin_a if hasattr(pin_a, "value") else Pin(
            pin_a, Pin.OUT, value=0
        )
        self.b = pin_b if hasattr(pin_b, "value") else Pin(
            pin_b, Pin.OUT, value=0
        )
        self.drive_ms = drive_ms
        self.reverse_to_retract = reverse_to_retract
        self.extended = False
        self.stop()

    def _drive(self, a, b):
        self.a.value(a)
        self.b.value(b)
        sleep_ms(self.drive_ms)
        self.stop()

    def stop(self):
        self.a.value(0)
        self.b.value(0)

    def extend(self):
        self._drive(1, 0)
        self.extended = True

    def retract(self):
        if self.reverse_to_retract:
            self._drive(0, 1)
        else:
            self._drive(1, 0)
        self.extended = False

    def home(self):
        if self.reverse_to_retract:
            self.retract()
        else:
            self.stop()
            self.extended = False


class ProductController:
    def __init__(
        self,
        arm=None,
        head_detector=None,
        blade=None,
        start_button=None,
        stabilization_button=None,
        repeat_button=None,
        output=print,
    ):
        self.arm = arm if arm is not None else SwivelCut()
        self.head_detector = (
            head_detector if head_detector is not None else HeadDetector()
        )
        self.blade = blade if blade is not None else BladeActuator()
        self.start_button = (
            start_button if start_button is not None
            else DebouncedButton(PIN_START_STOP)
        )
        self.stabilization_button = (
            stabilization_button if stabilization_button is not None
            else DebouncedButton(PIN_STABILIZATION)
        )
        self.repeat_button = (
            repeat_button if repeat_button is not None
            else DebouncedButton(PIN_REPEAT)
        )
        self.output = output
        self.state = STATE_IDLE
        self.stabilization_enabled = False
        self.raw_teach_points = []
        self.recording_started_at = None
        self.next_teach_sample = None
        self.last_head_sample = None
        self.head = HEAD_UNKNOWN
        self.abort_requested = False

    def initialize(self):
        self.arm.disable()
        self.blade.home()
        self.arm.set_folded_start()
        self.arm.calibrate_encoders()
        self.output(
            "READY: folded pose calibrated; connect tracing or cutting head"
        )

    def _sample_head(self, now, force=False):
        if (
            force
            or self.last_head_sample is None
            or ticks_diff(now, self.last_head_sample) >= HEAD_SAMPLE_MS
        ):
            self.head = self.head_detector.update()
            self.last_head_sample = now
        return self.head

    def _record_sample(self, now):
        if ticks_diff(now, self.next_teach_sample) < 0:
            return
        t1_deg, t2_deg = self.arm.encoder_angles()
        self.arm._validate_angles(t1_deg, t2_deg)
        elapsed = ticks_diff(now, self.recording_started_at)
        self.raw_teach_points.append((elapsed, t1_deg, t2_deg))
        interval_ms = max(1, int(round(1000.0 / TEACH_DEFAULT_HZ)))
        self.next_teach_sample = ticks_add(self.next_teach_sample, interval_ms)

    def start_teaching(self, now=None):
        if self.head != HEAD_TRACING:
            raise ValueError("tracing head is required to teach a path")
        if now is None:
            now = ticks_ms()
        self.arm.disable()
        self.raw_teach_points = []
        self.recording_started_at = now
        self.next_teach_sample = now
        self.state = STATE_TEACHING
        self._record_sample(now)
        self.output("TEACHING: press Start/Stop to finish")

    def stop_teaching(self, now=None):
        if self.state != STATE_TEACHING:
            return
        if now is None:
            now = ticks_ms()
        self._record_sample(now)
        if len(self.raw_teach_points) < 2:
            self.raw_teach_points = []
            self.state = STATE_IDLE
            raise ValueError("taught path is too short")
        self.arm.sync_from_encoders()
        self.state = STATE_IDLE
        self.output(
            "TAUGHT: {} points; install cutting head".format(
                len(self.raw_teach_points)
            )
        )

    def _path_for_cut(self):
        if len(self.raw_teach_points) < 2:
            raise ValueError("no taught path")
        if self.stabilization_enabled:
            return self.arm.stabilize_teach_points(
                self.raw_teach_points,
                TEACH_SMOOTHING_MS,
                TEACH_MAX_DEVIATION_DEG,
            )
        return list(self.raw_teach_points)

    def _motion_should_abort(self):
        now = ticks_ms()
        if self.start_button.update(now) == "pressed":
            self.abort_requested = True
        head = self._sample_head(now, force=True)
        return self.abort_requested or head != HEAD_CUTTING

    def cut(self):
        if self.head != HEAD_CUTTING:
            raise ValueError("cutting head is required to cut")
        self.arm.teach_points = self._path_for_cut()
        self.arm.teach_mode = "dual"
        self.abort_requested = False
        self.state = STATE_CUTTING
        self.arm.motion_abort_check = self._motion_should_abort
        self.output(
            "CUTTING: stabilization {}; press Start/Stop to abort".format(
                "ON" if self.stabilization_enabled else "OFF"
            )
        )
        try:
            self.arm.replay_teach(before_path=self.blade.extend)
            self.output("CUT COMPLETE")
        finally:
            self.arm.motion_abort_check = None
            self.arm.disable()
            self.blade.retract()
            self.state = STATE_IDLE

    def _handle_idle_press(self, button, now):
        if button == "stabilization":
            self.stabilization_enabled = not self.stabilization_enabled
            self.output(
                "STABILIZATION {}".format(
                    "ON" if self.stabilization_enabled else "OFF"
                )
            )
        elif button == "start":
            if self.head == HEAD_TRACING:
                self.start_teaching(now)
            elif self.head == HEAD_CUTTING:
                self.cut()
            else:
                raise ValueError("connect a recognized head")
        elif button == "repeat":
            self.cut()

    def update(self, now=None):
        if now is None:
            now = ticks_ms()
        self._sample_head(now)

        start_event = self.start_button.update(now)
        stabilization_event = self.stabilization_button.update(now)
        repeat_event = self.repeat_button.update(now)

        if self.state == STATE_TEACHING:
            if self.head != HEAD_TRACING:
                self.arm.disable()
                self.raw_teach_points = []
                self.state = STATE_IDLE
                raise MotionAborted("tracing head removed; recording discarded")
            if start_event == "pressed":
                self.stop_teaching(now)
            elif ticks_diff(now, self.recording_started_at) >= int(
                TEACH_MAX_SECONDS * 1000
            ):
                self.stop_teaching(now)
                self.output("TEACHING STOPPED: maximum duration reached")
            else:
                self._record_sample(now)
            return

        if self.state != STATE_IDLE:
            return
        if stabilization_event == "pressed":
            self._handle_idle_press("stabilization", now)
        if start_event == "pressed":
            self._handle_idle_press("start", now)
        if repeat_event == "pressed":
            self._handle_idle_press("repeat", now)

    def shutdown(self):
        self.arm.motion_abort_check = None
        self.arm.disable()
        self.blade.stop()
        if self.blade.extended:
            self.blade.retract()
        self.state = STATE_IDLE

    def run(self):
        self.initialize()
        try:
            while True:
                try:
                    self.update()
                except MotionAborted as error:
                    self.output("STOPPED: {}".format(error))
                except (EncoderError, ValueError, OSError) as error:
                    self.arm.disable()
                    if self.blade.extended:
                        self.blade.retract()
                    self.state = STATE_ERROR
                    self.output("ERROR: {}".format(error))
                    self.state = STATE_IDLE
                sleep_ms(MAIN_LOOP_MS)
        finally:
            self.shutdown()


def run_product_controller():
    ProductController().run()
