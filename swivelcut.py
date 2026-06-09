"""SwivelCut two-joint motion controller for MicroPython on ESP32."""

import math
from machine import Pin, SoftI2C
from time import sleep_ms, sleep_us, ticks_ms, ticks_us, ticks_diff, ticks_add

from as5600 import AS5600, EncoderError, MultiTurnEncoder

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

# Each AS5600 has the fixed address 0x36, so the modules use separate buses.
PIN_J1_ENCODER_SDA = 21
PIN_J1_ENCODER_SCL = 22
PIN_J2_ENCODER_SDA = 18
PIN_J2_ENCODER_SCL = 19
ENCODER_I2C_HZ = 400000

# Change a sign if increasing the logical joint angle decreases its raw count.
ENCODER_J1_SIGN = 1
ENCODER_J2_SIGN = 1

ENCODER_SAMPLE_STEPS = 256
FEEDBACK_TOLERANCE_DEG = 0.25
FEEDBACK_MAX_ERROR_DEG = 10.0
FEEDBACK_MAX_CORRECTIONS = 3

TEACH_DEFAULT_HZ = 20
TEACH_MAX_HZ = 50
TEACH_MAX_SECONDS = 60.0

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
    def __init__(self, encoders=None, auto_encoders=True):
        self.j1 = StepperAxis(PIN_J1_STEP, PIN_J1_DIR, STEPS_PER_RAD_J1, INVERT_J1)
        self.j2 = StepperAxis(PIN_J2_STEP, PIN_J2_DIR, STEPS_PER_RAD_J2, INVERT_J2)
        self.en = Pin(PIN_ENABLE, Pin.OUT, value=1) if PIN_ENABLE is not None else None
        self.L1 = L1
        self.L2 = L2
        self.coupling = COUPLING
        self.encoder_i2c = None
        if encoders is not None:
            self.encoders = encoders
        elif auto_encoders:
            self.encoders = self._create_default_encoders()
        else:
            self.encoders = None
        self.encoder_calibrated = False
        self.feedback_fault = None
        self.teach_points = []
        self.set_folded_start()

    def _create_default_encoders(self):
        j1_i2c = SoftI2C(
            scl=Pin(PIN_J1_ENCODER_SCL),
            sda=Pin(PIN_J1_ENCODER_SDA),
            freq=ENCODER_I2C_HZ,
        )
        j2_i2c = SoftI2C(
            scl=Pin(PIN_J2_ENCODER_SCL),
            sda=Pin(PIN_J2_ENCODER_SDA),
            freq=ENCODER_I2C_HZ,
        )
        self.encoder_i2c = (j1_i2c, j2_i2c)
        return (
            MultiTurnEncoder(AS5600(j1_i2c), ENCODER_J1_SIGN, "J1 encoder"),
            MultiTurnEncoder(AS5600(j2_i2c), ENCODER_J2_SIGN, "J2 encoder"),
        )

    def enable(self):
        if self.feedback_fault:
            raise EncoderError("feedback fault: " + self.feedback_fault)
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

    def _require_encoders(self):
        if not self.encoders:
            raise EncoderError("encoders are not configured")

    def calibrate_encoders(self):
        """Define the current encoder readings as the known folded pose."""
        self._require_encoders()
        self.encoder_calibrated = False
        motor1 = self.t1 * GEAR_J1
        motor2 = (self.t2 + self.coupling * self.t1) * GEAR_J2
        try:
            self.encoders[0].calibrate(motor1)
            self.encoders[1].calibrate(motor2)
        except (OSError, EncoderError) as error:
            self._feedback_fail(str(error))
        self.encoder_calibrated = True
        self.feedback_fault = None
        return self.encoder_angles()

    def encoder_status(self):
        self._require_encoders()
        try:
            return (
                self.encoders[0].magnet_state(),
                self.encoders[1].magnet_state(),
            )
        except OSError as error:
            raise EncoderError("encoder I2C read failed: {}".format(error))

    def _sample_encoders(self):
        if not self.encoder_calibrated:
            return None
        try:
            motor1 = self.encoders[0].update()
            motor2 = self.encoders[1].update()
        except (OSError, EncoderError) as error:
            self._feedback_fail(str(error))
        t1 = motor1 / GEAR_J1
        t2 = motor2 / GEAR_J2 - self.coupling * t1
        return t1, t2

    def encoder_angles(self):
        """Return measured J1 and J2 angles in degrees."""
        measured = self._sample_encoders()
        if measured is None:
            raise EncoderError("encoders are not calibrated")
        return math.degrees(measured[0]), math.degrees(measured[1])

    def sync_from_encoders(self):
        """Use measured shaft positions as the controller's current state."""
        t1_deg, t2_deg = self.encoder_angles()
        self._validate_angles(t1_deg, t2_deg)
        self.t1 = math.radians(t1_deg)
        self.t2 = math.radians(t2_deg)
        self.j1.pos, self.j2.pos = self._angle_to_steps(self.t1, self.t2)
        return t1_deg, t2_deg

    def _feedback_fail(self, message):
        self.feedback_fault = message
        self.encoder_calibrated = False
        self.disable()
        raise EncoderError(message)

    def _settle_feedback(self, target_t1, target_t2):
        if not self.encoders:
            return
        if not self.encoder_calibrated:
            self._feedback_fail("encoders are not calibrated")

        tolerance = math.radians(FEEDBACK_TOLERANCE_DEG)
        maximum = math.radians(FEEDBACK_MAX_ERROR_DEG)
        previous_error = None
        for attempt in range(FEEDBACK_MAX_CORRECTIONS + 1):
            measured_t1, measured_t2 = self._sample_encoders()
            error1 = target_t1 - measured_t1
            error2 = target_t2 - measured_t2
            worst_error = max(abs(error1), abs(error2))
            if worst_error <= tolerance:
                self.t1, self.t2 = measured_t1, measured_t2
                self.j1.pos, self.j2.pos = self._angle_to_steps(self.t1, self.t2)
                return
            if worst_error > maximum:
                self._feedback_fail(
                    "encoder error too large: J1={:.2f} deg J2={:.2f} deg".format(
                        math.degrees(error1), math.degrees(error2)
                    )
                )
            if previous_error is not None and worst_error >= previous_error:
                self._feedback_fail(
                    "correction increased encoder error; check encoder signs "
                    "and mechanics"
                )
            if attempt == FEEDBACK_MAX_CORRECTIONS:
                break

            previous_error = worst_error
            self.t1, self.t2 = measured_t1, measured_t2
            self.j1.pos, self.j2.pos = self._angle_to_steps(self.t1, self.t2)
            target1, target2 = self._angle_to_steps(target_t1, target_t2)
            self._execute(target1 - self.j1.pos, target2 - self.j2.pos)
            self._sync_angles_from_steps()

        self._feedback_fail(
            "position did not settle within {:.2f} deg".format(
                FEEDBACK_TOLERANCE_DEG
            )
        )

    def position(self):
        x, y = self.forward(self.t1, self.t2)
        return x, y, math.degrees(self.t1), math.degrees(self.t2)

    def _move_to_angles_once(self, t1_deg, t2_deg, ramp_in=True, ramp_out=True):
        self._validate_angles(t1_deg, t2_deg)
        t1 = math.radians(t1_deg)
        t2 = math.radians(t2_deg)
        tgt1, tgt2 = self._angle_to_steps(t1, t2)
        try:
            self._execute(tgt1 - self.j1.pos, tgt2 - self.j2.pos, ramp_in, ramp_out)
        finally:
            self._sync_angles_from_steps()
        return t1, t2

    def move_to_angles(
        self, t1_deg, t2_deg, ramp_in=True, ramp_out=True, feedback=True
    ):
        """Coordinated move of BOTH joints to absolute angles (deg)."""
        if self.encoders and not self.encoder_calibrated:
            self._feedback_fail("encoders are not calibrated")
        t1, t2 = self._move_to_angles_once(
            t1_deg, t2_deg, ramp_in=ramp_in, ramp_out=ramp_out
        )
        if feedback:
            self._settle_feedback(t1, t2)

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
                                ramp_in=ramp_in, ramp_out=ramp_out,
                                feedback=ramp_out)

    def record_teach(self, duration_s, sample_hz=TEACH_DEFAULT_HZ):
        """Record a hand-guided joint trajectory while the drivers are off."""
        if not self.encoder_calibrated:
            raise EncoderError("encoders are not calibrated")
        if duration_s <= 0 or duration_s > TEACH_MAX_SECONDS:
            raise ValueError(
                "teach duration must be in (0, {}] seconds".format(
                    int(TEACH_MAX_SECONDS)
                )
            )
        if sample_hz <= 0 or sample_hz > TEACH_MAX_HZ:
            raise ValueError(
                "teach sample rate must be in (0, {}] Hz".format(TEACH_MAX_HZ)
            )

        self.disable()
        interval_ms = max(1, int(round(1000.0 / sample_hz)))
        duration_ms = int(round(duration_s * 1000.0))
        start = ticks_ms()
        next_sample = start
        points = []
        while True:
            now = ticks_ms()
            remaining = ticks_diff(next_sample, now)
            if remaining > 0:
                sleep_ms(remaining)
                now = ticks_ms()
            elapsed = ticks_diff(now, start)
            t1_deg, t2_deg = self.encoder_angles()
            self._validate_angles(t1_deg, t2_deg)
            points.append((elapsed, t1_deg, t2_deg))
            if elapsed >= duration_ms:
                break
            next_sample = ticks_add(next_sample, interval_ms)

        self.teach_points = points
        self.sync_from_encoders()
        return len(points)

    def replay_teach(self):
        """Return to the recorded start and reproduce the taught trajectory."""
        if len(self.teach_points) < 2:
            raise ValueError("no taught movement; use TEACH first")
        if not self.encoder_calibrated:
            raise EncoderError("encoders are not calibrated")

        for _elapsed, t1_deg, t2_deg in self.teach_points:
            self._validate_angles(t1_deg, t2_deg)

        self.enable()
        first = self.teach_points[0]
        self.move_to_angles(first[1], first[2])
        replay_start = ticks_ms()
        for index in range(1, len(self.teach_points)):
            elapsed, t1_deg, t2_deg = self.teach_points[index]
            final = index == len(self.teach_points) - 1
            self._move_to_angles_once(
                t1_deg, t2_deg, ramp_in=False, ramp_out=final
            )
            wait_ms = ticks_diff(ticks_add(replay_start, elapsed), ticks_ms())
            if wait_ms > 0:
                sleep_ms(wait_ms)

        target = self.teach_points[-1]
        self._settle_feedback(math.radians(target[1]), math.radians(target[2]))
        return len(self.teach_points)

    def clear_teach(self):
        self.teach_points = []

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

            if self.encoder_calibrated and (
                (i + 1) % ENCODER_SAMPLE_STEPS == 0 or i + 1 == total
            ):
                self._sample_encoders()

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
