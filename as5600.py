"""AS5600 magnetic encoder support for MicroPython."""

import math


AS5600_ADDRESS = 0x36
REG_STATUS = 0x0B
REG_RAW_ANGLE = 0x0C

STATUS_MAGNET_DETECTED = 0x20
STATUS_MAGNET_TOO_WEAK = 0x10
STATUS_MAGNET_TOO_STRONG = 0x08

COUNTS_PER_REV = 4096
HALF_COUNTS = COUNTS_PER_REV // 2
TWO_PI = 2.0 * math.pi


class EncoderError(RuntimeError):
    """Raised when encoder feedback is unavailable or unsafe to use."""


class AS5600:
    def __init__(self, i2c, address=AS5600_ADDRESS):
        self.i2c = i2c
        self.address = address

    def read_raw(self):
        data = self.i2c.readfrom_mem(self.address, REG_RAW_ANGLE, 2)
        return ((data[0] << 8) | data[1]) & 0x0FFF

    def read_status(self):
        return self.i2c.readfrom_mem(self.address, REG_STATUS, 1)[0]

    def magnet_state(self):
        status = self.read_status()
        if status & STATUS_MAGNET_TOO_STRONG:
            return "strong"
        if status & STATUS_MAGNET_TOO_WEAK:
            return "weak"
        if status & STATUS_MAGNET_DETECTED:
            return "ok"
        return "missing"

    def require_magnet(self, name="encoder"):
        state = self.magnet_state()
        if state != "ok":
            raise EncoderError("{} magnet is {}".format(name, state))


class MultiTurnEncoder:
    """Unwrap one AS5600 and express it in logical motor radians."""

    def __init__(self, sensor, motor_sign=1, name="encoder"):
        if motor_sign not in (-1, 1):
            raise ValueError("motor_sign must be -1 or 1")
        self.sensor = sensor
        self.motor_sign = motor_sign
        self.name = name
        self.calibrated = False
        self.last_raw = 0
        self.unwrapped_counts = 0
        self.origin_motor_rad = 0.0

    def calibrate(self, known_motor_rad):
        self.sensor.require_magnet(self.name)
        self.last_raw = self.sensor.read_raw()
        self.unwrapped_counts = 0
        self.origin_motor_rad = known_motor_rad
        self.calibrated = True
        return known_motor_rad

    def update(self):
        if not self.calibrated:
            raise EncoderError("{} is not calibrated".format(self.name))
        raw = self.sensor.read_raw()
        delta = raw - self.last_raw
        if delta > HALF_COUNTS:
            delta -= COUNTS_PER_REV
        elif delta < -HALF_COUNTS:
            delta += COUNTS_PER_REV
        self.unwrapped_counts += delta
        self.last_raw = raw
        return self.motor_radians()

    def motor_radians(self):
        delta_rad = self.unwrapped_counts * TWO_PI / COUNTS_PER_REV
        return self.origin_motor_rad + self.motor_sign * delta_rad

    def magnet_state(self):
        return self.sensor.magnet_state()
