"""SwivelCut two-joint motion controller for MicroPython on ESP32."""

import math
from machine import Pin
from time import sleep_us, ticks_us, ticks_diff, ticks_add

# Machine dimensions and drive ratios
L1 = 200.0          # shoulder pivot -> elbow pivot
L2 = 200.0          # elbow pivot -> cutting tip

STEPS_PER_MOTOR_REV = 200      # 1.8-degree NEMA 23 and NEMA 17 motors
MICROSTEP = 32                 # TB6600 DIP switches must also be set to 32
GEAR_J1 = 6.0                  # motor turns per shoulder revolution
GEAR_J2 = 9.0                  # motor turns per elbow revolution

COUPLING = 0.0

# Known pose at power-on
START_T1_DEG = 0.0
START_T2_DEG = -180.0
FOLDED_RADIUS_MM = 1.0

# Set either value to True if that joint runs backwards.
INVERT_J1 = False
INVERT_J2 = False

# Motion timing
MAX_STEP_RATE = 1500.0         # cruise ceiling [microsteps / s]
ACCEL = 3000.0                 # acceleration [microsteps / s^2]
PULSE_US = 10                  # conservative pulse width for TB6600 inputs
DIR_SETUP_US = 10              # direction setup time before the first pulse

# Software travel limits
J1_MIN, J1_MAX = -90.0, 90.0
J2_MIN, J2_MAX = -180.0, 180.0

# ESP32 pins
PIN_J1_STEP = 25
PIN_J1_DIR  = 26
PIN_J2_STEP = 32
PIN_J2_DIR  = 33
PIN_ENABLE  = 27               # shared /EN, active-LOW. Set to None if unused.

LINE_SEG_MM = 2.0              # straight cuts are split into segments this long

TWO_PI = 2.0 * math.pi
STEPS_PER_RAD_J1 = STEPS_PER_MOTOR_REV * MICROSTEP * GEAR_J1 / TWO_PI
STEPS_PER_RAD_J2 = STEPS_PER_MOTOR_REV * MICROSTEP * GEAR_J2 / TWO_PI


class StepperAxis:
    def __init__(self, step_pin, dir_pin, steps_per_rad, invert=False):
        self.step = Pin(step_pin, Pin.OUT, value=1)
        self.dir  = Pin(dir_pin, Pin.OUT, value=0)
        self.steps_per_rad = steps_per_rad
        self.invert = invert
        self.pos = 0

    def set_dir(self, sign):
        forward = (sign > 0)
        if self.invert:
            forward = not forward
        self.dir.value(1 if forward else 0)


class SwivelCut:
    def __init__(self):
        self.j1 = StepperAxis(PIN_J1_STEP, PIN_J1_DIR, STEPS_PER_RAD_J1, INVERT_J1)
        self.j2 = StepperAxis(PIN_J2_STEP, PIN_J2_DIR, STEPS_PER_RAD_J2, INVERT_J2)
        self.en = Pin(PIN_ENABLE, Pin.OUT, value=1) if PIN_ENABLE is not None else None
        self.L1 = L1
        self.L2 = L2
        self.coupling = COUPLING
        self.set_folded_start()

    def enable(self):
        if self.en:
            self.en.value(0)
        sleep_us(2000)

    def disable(self):
        if self.en:
            self.en.value(1)

    def forward(self, t1, t2):
        """Joint angles (rad) -> tip (x, y) mm."""
        forward = self.L1 * math.cos(t1) + self.L2 * math.cos(t1 + t2)
        sideways = self.L1 * math.sin(t1) + self.L2 * math.sin(t1 + t2)
        # Positive machine X is physically to the right.
        return -sideways, forward

    def inverse(self, x, y, elbow="down"):
        """Tip (x, y) mm -> joint angles (t1, t2) rad. Raises if unreachable."""
        L1, L2 = self.L1, self.L2
        r2 = x * x + y * y
        c2 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        if c2 < -1.0 - 1e-12 or c2 > 1.0 + 1e-12:
            raise ValueError("unreachable point ({:.1f}, {:.1f})".format(x, y))
        c2 = max(-1.0, min(1.0, c2))
        s2 = math.sqrt(1.0 - c2 * c2)