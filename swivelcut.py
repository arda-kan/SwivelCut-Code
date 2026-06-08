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
START_T2_DEG = 180.0
FOLDED_RADIUS_MM = 1.0

# Set either value to True if that joint runs backwards.
INVERT_J1 = False
INVERT_J2 = True

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
        return sideways, forward

    def inverse(self, x, y, elbow="up"):
        """Tip (x, y) mm -> joint angles (t1, t2) rad. Raises if unreachable."""
        L1, L2 = self.L1, self.L2
        r2 = x * x + y * y
        c2 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        if c2 < -1.0 - 1e-12 or c2 > 1.0 + 1e-12:
            raise ValueError("unreachable point ({:.1f}, {:.1f})".format(x, y))
        c2 = max(-1.0, min(1.0, c2))
        s2 = math.sqrt(1.0 - c2 * c2)
        if elbow == "down":
            s2 = -s2
        t2 = math.atan2(s2, c2)
        t1 = math.atan2(x, y) - math.atan2(L2 * s2, L1 + L2 * c2)
        return self._normalize_angle(t1), self._normalize_angle(t2)

    def reachable(self, x, y):
        r = math.sqrt(x * x + y * y)
        return abs(self.L1 - self.L2) <= r <= (self.L1 + self.L2)

    def line_reachable(self, x0, y0, x1, y1):
        """Whether every point on a segment lies in the reachable annulus."""
        if not self.reachable(x0, y0) or not self.reachable(x1, y1):
            return False
        dx = x1 - x0
        dy = y1 - y0
        length2 = dx * dx + dy * dy
        if length2 == 0:
            return True
        projection = -(x0 * dx + y0 * dy) / length2
        projection = max(0.0, min(1.0, projection))
        nearest_x = x0 + projection * dx
        nearest_y = y0 + projection * dy
        nearest_r = math.sqrt(nearest_x * nearest_x + nearest_y * nearest_y)
        return nearest_r + 1e-9 >= abs(self.L1 - self.L2)

    def _angle_to_steps(self, t1, t2):
        s1 = round(t1 * self.j1.steps_per_rad)
        s2 = round((t2 + self.coupling * t1) * self.j2.steps_per_rad)
        return s1, s2

    @staticmethod
    def _normalize_angle(angle):
        """Return an equivalent angle in the conventional [-pi, pi] range."""
        normalized = (angle + math.pi) % TWO_PI - math.pi
        return math.pi if normalized == -math.pi and angle > 0 else normalized

    @staticmethod
    def _validate_angles(t1_deg, t2_deg):
        if not (J1_MIN <= t1_deg <= J1_MAX):
            raise ValueError("J1 {:.1f} out of [{}, {}]".format(t1_deg, J1_MIN, J1_MAX))
        if not (J2_MIN <= t2_deg <= J2_MAX):
            raise ValueError("J2 {:.1f} out of [{}, {}]".format(t2_deg, J2_MIN, J2_MAX))

    def _sync_angles_from_steps(self):
        """Reconstruct joint state from emitted steps, including belt coupling."""
        self.t1 = self.j1.pos / self.j1.steps_per_rad
        motor2_angle = self.j2.pos / self.j2.steps_per_rad
        self.t2 = motor2_angle - self.coupling * self.t1

    def set_folded_start(self):
        """Reset software state to the required physical startup pose."""
        self.t1 = math.radians(START_T1_DEG)
        self.t2 = math.radians(START_T2_DEG)
        self.j1.pos, self.j2.pos = self._angle_to_steps(self.t1, self.t2)

    def position(self):
        x, y = self.forward(self.t1, self.t2)
        return x, y, math.degrees(self.t1), math.degrees(self.t2)

    def move_to_angles(self, t1_deg, t2_deg, ramp_in=True, ramp_out=True):
        """Coordinated move of BOTH joints to absolute angles (deg)."""
        self._validate_angles(t1_deg, t2_deg)
        t1 = math.radians(t1_deg)
        t2 = math.radians(t2_deg)
        tgt1, tgt2 = self._angle_to_steps(t1, t2)
        d1 = tgt1 - self.j1.pos
        d2 = tgt2 - self.j2.pos
        try:
            self._execute(d1, d2, ramp_in, ramp_out)
        finally:
            self._sync_angles_from_steps()

    def move_joint(self, joint, angle_deg, relative=False):
        """Move one joint and hold the other."""
        if joint == 1:
            t1d = math.degrees(self.t1) + angle_deg if relative else angle_deg
            self.move_to_angles(t1d, math.degrees(self.t2))
        elif joint == 2:
            t2d = math.degrees(self.t2) + angle_deg if relative else angle_deg
            self.move_to_angles(math.degrees(self.t1), t2d)
        else:
            raise ValueError("joint must be 1 or 2")

    def move_to_xy(self, x, y, elbow="up"):
        """Inverse-kinematics point-to-point move to a Cartesian tip position."""
        t1, t2 = self.inverse(x, y, elbow)
        self.move_to_angles(math.degrees(t1), math.degrees(t2))

    def move_joint_to_xy(self, joint, x, y, elbow="up"):
        """Use an XY solution for one joint and hold the other joint still."""
        if joint not in (1, 2):
            raise ValueError("joint must be 1 or 2")
        t1, t2 = self.inverse(x, y, elbow)
        if joint == 1:
            self.move_to_angles(math.degrees(t1), math.degrees(self.t2))
        else:
            self.move_to_angles(math.degrees(self.t1), math.degrees(t2))

    def cut_line(self, x0, y0, x1, y1, elbow="up", seg_mm=LINE_SEG_MM):
        """Move the blade along a straight XY line."""
        if seg_mm <= 0:
            raise ValueError("seg_mm must be greater than zero")
        current_x, current_y = self.forward(self.t1, self.t2)
        if math.sqrt(current_x * current_x + current_y * current_y) < FOLDED_RADIUS_MM:
            raise ValueError(
                "arm is folded at singular XY (0, 0); first use XY <x> <y>, "
                "then CUT from that same nonzero point"
            )
        if not self.line_reachable(x0, y0, x1, y1):
            raise ValueError("line crosses unreachable workspace")
        dist = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
        n = max(1, int(math.ceil(dist / seg_mm)))

        # Check every segment before the motors move.
        for i in range(n + 1):
            f = i / float(n)
            xi = x0 + (x1 - x0) * f
            yi = y0 + (y1 - y0) * f
            t1, t2 = self.inverse(xi, yi, elbow)
            self._validate_angles(math.degrees(t1), math.degrees(t2))

        self.move_to_xy(x0, y0, elbow)
        for i in range(1, n + 1):
            f = i / float(n)
            xi = x0 + (x1 - x0) * f
            yi = y0 + (y1 - y0) * f
            t1, t2 = self.inverse(xi, yi, elbow)
            ramp_in = (i == 1)
            ramp_out = (i == n)
            self.move_to_angles(math.degrees(t1), math.degrees(t2),
                                ramp_in=ramp_in, ramp_out=ramp_out)

    def _execute(self, d1, d2, ramp_in=True, ramp_out=True):
        """Send a coordinated pair of step counts to the drivers."""
        s1 = 1 if d1 >= 0 else -1
        s2 = 1 if d2 >= 0 else -1
        n1 = abs(d1)
        n2 = abs(d2)
        total = n1 if n1 >= n2 else n2
        if total == 0:
            return

        self.j1.set_dir(s1)
        self.j2.set_dir(s2)
        sleep_us(DIR_SETUP_US)

        major_is_1 = (n1 >= n2)
        minor = n2 if major_is_1 else n1
        err = total // 2

        min_interval = 1.0e6 / MAX_STEP_RATE
        c0 = 0.676 * 1.0e6 * math.sqrt(2.0 / ACCEL)        # first-step interval
        n_ramp = int((MAX_STEP_RATE * MAX_STEP_RATE) / (2.0 * ACCEL))
        n_acc = n_ramp if ramp_in else 0
        n_dec = n_ramp if ramp_out else 0
        if n_acc + n_dec > total:
            if ramp_in and ramp_out:
                n_acc = total // 2
                n_dec = total - n_acc
            elif ramp_in:
                n_acc = total
            else:
                n_dec = total

        c = c0 if ramp_in else min_interval
        ramp = 0
        step1 = self.j1.step
        step2 = self.j2.step

        t_next = ticks_us()
        for i in range(total):
            # The major axis steps every cycle. Error tracking spaces out the
            # minor-axis steps so both joints finish together.
            do2 = False
            if major_is_1:
                do1 = True
                err -= minor
                if err < 0:
                    do2 = True
                    err += total
            else:
                do2 = True
                err -= minor
                if err < 0:
                    do1 = True
                    err += total
                else:
                    do1 = False
            if major_is_1:
                step1.value(0)
                if do2:
                    step2.value(0)
            else:
                step2.value(0)
                if do1:
                    step1.value(0)
            sleep_us(PULSE_US)
            step1.value(1)
            step2.value(1)

            if major_is_1:
                self.j1.pos += s1
                if do2:
                    self.j2.pos += s2
            else:
                self.j2.pos += s2
                if do1:
                    self.j1.pos += s1

            t_next = ticks_add(t_next, int(c))
            while ticks_diff(ticks_us(), t_next) < 0:
                pass

            if i < n_acc:
                ramp += 1
                c = c - (2.0 * c) / (4.0 * ramp + 1.0)
                if c < min_interval:
                    c = min_interval
            elif i >= total - n_dec:
                m = total - i
                c = c + (2.0 * c) / (4.0 * m + 1.0)
            else:
                c = min_interval



if __name__ == "__main__":
    pass
