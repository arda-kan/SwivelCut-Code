# SwivelCut Code

Controller and motion visualizer for the two-joint SwivelCut cardboard cutter.

## Fixed machine

- 200 mm shoulder link and 200 mm elbow link
- NEMA 23 shoulder motor with 6:1 pulley reduction
- NEMA 17 elbow motor mounted on link 1 with 9:1 pulley reduction
- 200 full steps per motor revolution
- TB6600 drivers set to 1/32 microstepping
- Required startup pose: J1 = 0 degrees, J2 = 180 degrees (fully folded)
- J1 software travel: -90 to +90 degrees
- J2 software travel: -180 to +180 degrees

Two AS5600 magnetic encoders measure the stepper motor shafts. Before every
power-on, place the arm in the folded startup pose. `ARM FOLDED` checks both
magnets and defines the current encoder readings as J1 = 0 degrees and
J2 = 180 degrees.

The folded pose is a kinematic singularity at `(0, 0)`. Use a joint-space or
point-to-point move to unfold the arm before calling `cut_line()`.

## Current capabilities

- Coordinated absolute shoulder and elbow moves with acceleration ramps
- Forward and inverse kinematics for absolute XY moves
- Elbow-up and elbow-down inverse-kinematics branches
- Straight Cartesian cuts split into validated 2 mm segments
- Software travel and workspace checks before motion
- Continuous AS5600 sampling during motor movement
- Bounded post-move correction and automatic shutdown on feedback faults
- Timed hand-guided joint recording with return-to-start replay
- Browser geometry and motion visualizer

## ESP32 to TB6600 pins

- GPIO 25: J1 pulse
- GPIO 26: J1 direction
- GPIO 32: J2 pulse
- GPIO 33: J2 direction
- GPIO 27: shared active-low enable

Positive joint angles are counterclockwise when viewed from above the cutting
plane. J1 uses the normal driver direction and J2 is inverted to match the
installed motor and belt direction. This changes only the electrical direction
signal; the reported joint angles and Cartesian calculations stay unchanged.

Coordinates use positive `X = physical right` and positive `Y = forward`.
With both links straight, `J1=0`, `J2=0` is `(X=0, Y=400)`.

The STEP outputs idle HIGH and pulse LOW. This matches the common-anode wiring
shown earlier, where the interface sinks `PUL-` during a step.

## ESP32 to AS5600 wiring

Both AS5600 modules have the fixed I2C address `0x36`. They cannot be connected
to the same SDA/SCL pair without an I2C multiplexer, so this controller uses two
independent software-I2C buses:

| Encoder | SDA | SCL |
| --- | ---: | ---: |
| J1 motor shaft | GPIO 21 | GPIO 22 |
| J2 motor shaft | GPIO 18 | GPIO 19 |

Connect both module grounds to ESP32 ground. Power the modules from 3.3 V so
their I2C pull-ups cannot expose the ESP32 pins to 5 V. Confirm the exact pin
labels and supply requirements on the purchased modules before applying power.

The supplied diametric magnet must be centered on the motor shaft, parallel to
the sensor face, and held at a stable gap. `ARM FOLDED` refuses to arm if either
AS5600 reports a missing, weak, or excessively strong magnetic field.

The encoder direction depends on which magnet pole faces the sensor and how the
module is mounted. Set `ENCODER_J1_SIGN` and `ENCODER_J2_SIGN` in
`swivelcut.py` to `1` or `-1` so increasing logical motor angle also increases
the measured logical angle. The controller stops if a correction makes the
error worse, but direction must still be checked carefully with the blade
removed.

### Motor-shaft measurement limitation

An encoder on the stepper shaft can detect missed motor motion and can record a
back-driven motor. It cannot directly measure belt stretch, pulley-to-shaft
slip, backlash, frame flex, or motion lost after that motor shaft. Measuring
those errors requires an encoder on each joint output shaft. The code in this
branch assumes motor-shaft mounting and divides measured motor rotation by the
6:1 and 9:1 drive ratios.

## Files

- `as5600.py`: AS5600 register access and wrap-safe multi-turn tracking.
- `swivelcut.py`: MicroPython controller for an ESP32 and two stepper axes.
- `serial_console.py`: guarded USB serial command parser.
- `main.py`: starts the serial console automatically when the ESP32 boots.
- `swivelcut_visualizer.html`: browser visualizer for geometry, IK, and motion.

## Running it on the ESP32

### 1. Wire and configure the hardware

Use two TB6600 drivers: one for the NEMA 23 and one for the NEMA 17. Both motors
are 200 full steps per revolution; set both TB6600 drivers to 1/32
microstepping. Set each driver's current limit for its own motor; do not assume
both motors use the same current.

Connect the ESP32 signals using the pin table above. The ESP32, both signal
interfaces and the motor power supply must have the required common reference
for the input wiring method used by your TB6600 modules. Do not power the motors
from the ESP32. Confirm that your TB6600 inputs reliably accept 3.3 V logic;
otherwise use a suitable transistor or level-shifting interface.

Test initially with the blade removed, low motor current, and an accessible
emergency power switch.

### 2. Install the computer tools

On macOS Terminal:

```sh
python3 -m pip install --user esptool mpremote
```

Download the stable generic ESP32/WROOM MicroPython `.bin` firmware from:

https://micropython.org/download/ESP32_GENERIC/

### 3. Connect and identify the ESP32

Connect the ESP32 over a data-capable USB cable:

```sh
python3 -m mpremote connect list
```

The following commands use automatic port detection. If several serial devices
are connected, replace `connect auto` with the device shown by the list command.

### 4. Flash MicroPython

Put the ESP32 into bootloader mode if required by holding `BOOT`, pressing and
releasing `RESET`, then releasing `BOOT`.

Replace `firmware.bin` with the downloaded file path:

```sh
python3 -m esptool erase-flash
python3 -m esptool --chip esp32 write-flash 0x1000 firmware.bin
```

### 5. Upload the arm program

Run this from the repository directory:

```sh
python3 -m mpremote connect auto fs cp swivelcut.py :swivelcut.py
python3 -m mpremote connect auto fs cp as5600.py :as5600.py
python3 -m mpremote connect auto fs cp serial_console.py :serial_console.py
python3 -m mpremote connect auto fs cp main.py :main.py
python3 -m mpremote connect auto reset
```

MicroPython automatically executes `main.py` after boot.

### 6. Open the command terminal

```sh
python3 -m mpremote connect auto repl
```

Exit the terminal later with `Ctrl-]` or `Ctrl-X`.

### 7. Confirm startup and test slowly

Physically place the arm in the fully folded pose before every reset. Then type:

```text
ARM FOLDED
```

The drivers remain disabled until both magnets pass their AS5600 diagnostics
and this exact confirmation is received.

Before powered movement, verify encoder directions with the blade removed:

1. Type `ARM FOLDED`, then `DISARM`.
2. Move J1 by hand no more than 5 degrees counterclockwise and type `ENC`.
   The reported J1 angle must be positive.
3. Return to folded. Move J2 slightly in the negative direction and type
   `ENC`. The reported J2 angle must decrease below 180 degrees.
4. If either direction is wrong, change that encoder's `ENCODER_*_SIGN`,
   upload again, reset, and repeat.
5. Return the arm exactly to folded and type `ARM FOLDED` again.

The small movement limit matters because each AS5600 is single-turn and shaft
turns are unwrapped from consecutive samples.

Begin powered testing with small moves and keep clear of the mechanism:

```text
J2 175
J1 5
J1 0
J2 180
```

If a positive command turns a joint clockwise rather than counterclockwise,
disconnect motor power and change that joint's `INVERT_J1` or `INVERT_J2`
setting in `swivelcut.py`, upload the file again, and reset.

### 8. Normal commands

```text
ANGLES 30 120
J1 -20
J2 90
XY 200 100
XY 200 100 DOWN
XYJ1 200 100
XYJ2 200 100 DOWN
CUT 100 100 250 100
ENC
TEACH 5
PLAY
CLEAR
POS
DISARM
HELP
```

Angles are absolute degrees, not relative movements. `ANGLES 30 120` means
"move J1 to +30 degrees and J2 to +120 degrees." `XY` values are absolute
millimetres from the shoulder pivot. `XYJ1` and `XYJ2` calculate the normal XY
solution but move only the named joint. Because the other joint stays still,
the blade will usually not finish at the requested XY point. Before `CUT`, move
to a safely unfolded starting position. A cut cannot begin at exact `(0, 0)`
because the fully folded arm is a singularity. For a forward centerline cut,
start slightly away from the origin:

```text
XY 0 20 UP
CUT 0 20 0 400 UP
```

For example, this moves both links into the straight-ahead pose:

```text
XY 0 400
```

Pressing `Ctrl-C` while connected stops the Python program and disables the
drivers through the console cleanup handler. Cutting motor power with the
physical emergency switch remains the primary emergency stop.

## Teach and replay

Start from the folded calibration workflow, move to the desired starting pose,
then record a five-second hand-guided movement:

```text
ARM FOLDED
ANGLES 20 120
TEACH 5 20
```

`TEACH` immediately disables the drivers. Guide the arm during the requested
window, keeping both joints inside their software limits. When recording ends,
the drivers remain disabled. Replay with:

```text
PLAY
```

The arm first drives back to the recorded starting pose, then follows the
recorded joint samples and applies final encoder correction. `CLEAR` erases the
stored movement. Recordings live only in RAM and are lost on reset.

The configured 1500 microstep/s ceiling gives maximum joint speeds of about
14.1 degrees/s for J1 and 9.4 degrees/s for J2. Hand-guided motion faster than
those limits cannot be reproduced at the original timing; replay preserves the
joint path but will take longer. The default recording rate is 20 Hz, the
allowed range is 1-50 Hz, and the maximum duration is 60 seconds.

Keep holding clear after `PLAY`: returning from the taught end pose to its
starting pose is an automatic motor move. Teach/replay is joint-space motion,
not a guarantee that the blade follows a straight Cartesian path.

## Tests

The controller's hardware imports are replaced with small test doubles:

```sh
python3 -m unittest discover -s tests -v
node tests/test_swivelcut_visualizer.js
```
