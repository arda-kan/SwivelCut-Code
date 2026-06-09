"""Standalone dual-AS5600 wiring and magnet test for an ESP32."""

from machine import Pin, SoftI2C
from time import sleep_ms


AS5600_ADDRESS = 0x36
REG_STATUS = 0x0B
REG_RAW_ANGLE = 0x0C

ENCODERS = (
    ("J1", 16, 17),  # name, SDA, SCL
    ("J2", 14, 13),
)


def read_raw(i2c):
    data = i2c.readfrom_mem(AS5600_ADDRESS, REG_RAW_ANGLE, 2)
    return ((data[0] << 8) | data[1]) & 0x0FFF


def magnet_state(i2c):
    status = i2c.readfrom_mem(AS5600_ADDRESS, REG_STATUS, 1)[0]
    if status & 0x08:
        return "TOO STRONG"
    if status & 0x10:
        return "TOO WEAK"
    if status & 0x20:
        return "OK"
    return "NOT DETECTED"


def create_bus(name, sda, scl):
    i2c = SoftI2C(
        sda=Pin(sda),
        scl=Pin(scl),
        freq=100000,
        timeout=50000,
    )
    devices = i2c.scan()
    print(
        "{} bus: SDA=GPIO{} SCL=GPIO{} devices={}".format(
            name, sda, scl, ["0x{:02X}".format(value) for value in devices]
        )
    )
    if AS5600_ADDRESS not in devices:
        print("{} ERROR: AS5600 not found; expected address 0x36".format(name))
        print(
            "{} CHECK: 3V3->VCC, GND->GND, GPIO{}->SDA, GPIO{}->SCL".format(
                name, sda, scl
            )
        )
        return None
    return i2c


def main():
    print("SwivelCut AS5600 test")
    print("Keep motor power OFF and rotate the shafts slowly by hand.")
    buses = []
    for name, sda, scl in ENCODERS:
        i2c = create_bus(name, sda, scl)
        if i2c is not None:
            buses.append((name, i2c))

    if len(buses) != len(ENCODERS):
        print("Fix the missing encoder wiring, disconnect USB, then test again.")
    if not buses:
        print("No encoders found. Test one module at a time.")
        return

    print("{} encoder(s) found. Press Ctrl-C to stop.".format(len(buses)))
    try:
        while True:
            values = []
            for name, i2c in buses:
                raw = read_raw(i2c)
                degrees = raw * 360.0 / 4096.0
                values.append(
                    "{} raw={:4d} angle={:7.2f} magnet={}".format(
                        name, raw, degrees, magnet_state(i2c)
                    )
                )
            print(" | ".join(values))
            sleep_ms(250)
    except KeyboardInterrupt:
        print("\nEncoder test stopped.")


main()
