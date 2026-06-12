# SwivelCut Code

Controller and motion visualizer for the two-joint SwivelCut cardboard cutter.

## Arduino IDE firmware

The primary ESP32 firmware is now
`SwivelCutArduino/SwivelCutArduino.ino`. Open that file in Arduino IDE, select
an ESP32 board, and upload it. Use `115200` baud for Serial Monitor. The older
MicroPython files remain in the branch as migration reference.

The Arduino sketch drives each TB6600 directly with the proven DFRobot-style
waveform: STEP is HIGH for 1500 microseconds and LOW for 1500 microseconds.
GPIO 25/26/27 are J1 pulse, direction, and enable; GPIO 32/33 are J2 pulse and
direction. Start with the blade removed and fold the arm before switching on or
resetting the ESP32. For bench testing, use `TEST J1 800` or `TEST J2 800`.

## Fixed machine

- 200 mm shoulder link and 200 mm elbow link
- NEMA 23 shoulder motor with 6:1 pulley reduction
- NEMA 17 elbow motor mounted on link 1 with 9:1 pulley reduction
- 200 full steps per motor revolution
- TB6600 drivers set to 1/4 microstepping
- Required startup pose: J1 = 0 degrees, J2 = 180 degrees (fully folded)
- J1 software travel: -90 to +90 degrees
- J2 software travel: -180 to +180 degrees

Two AS5600 magnetic encoders measure the stepper motor shafts. Before every
power-on, place the arm in the folded startup pose. `ARM FOLDED` checks both
magnets and defines the current encoder readings as J1 = 0 degrees and
J2 = 180 degrees.

The folded pose is a kinematic singularity at `(0, 0)`. Use a joint-space or
point-to-point move to unfold the arm before calling `cut_line()`.

For temporary bench testing with only the J1 encoder installed, fold the arm
and use `ARM J1`. This mode calibrates and monitors only J1. It accepts direct
`J1 <deg>` moves and explicit `TEACH J1`/`PLAY`, while holding J2 fixed. J2,
Cartesian, and cutting commands remain blocked. Normal `ARM FOLDED` operation
still requires both encoders.

`ARM J1` permits an unstable initial angle reading so temporary hardware can
still be tested. This bypasses only the calibration stability check; large
feedback errors during powered movement still disable the driver.

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
- GPIO 27: shared enable

Positive joint angles are counterclockwise when viewed from above the cutting
plane. J1 uses the normal driver direction and J2 is inverted to match the
installed motor and belt direction. This changes only the electrical direction
signal; the reported joint angles and Cartesian calculations stay unchanged.

Coordinates use positive `X = physical right` and positive `Y = forward`.
With both links straight, `J1=0`, `J2=0` is `(X=0, Y=400)`.

The Arduino sketch defaults to the same signal levels as the supplied DFRobot
test: STEP pulses HIGH, idles LOW, and GPIO 27 HIGH enables the outputs. Wire
the TB6600 input side to match that common-cathode signal convention. If the
existing electronics are wired common-anode, change `STEP_ACTIVE`,
`STEP_IDLE`, `OUTPUTS_ENABLED`, and `OUTPUTS_DISABLED` at the top of the sketch
before applying motor power.

## ESP32 to AS5600 wiring

Both AS5600 modules have the fixed I2C address `0x36`. They cannot be connected
to the same SDA/SCL pair without an I2C multiplexer, so this controller uses two
independent software-I2C buses:

| Encoder | SDA | SCL |
| --- | ---: | ---: |
| J1 motor shaft | GPIO 16 | GPIO 17 |
| J2 motor shaft | GPIO 21 | GPIO 22 |

Wire the module pins as follows:

| J1 AS5600 pin | Connect to |
| --- | --- |
| VCC | ESP32 `3V3` |
| GND | ESP32 `GND` |
| SDA | ESP32 `GPIO 16` |
| SCL | ESP32 `GPIO 17` |
| DIR | Leave disconnected |
| OUT | Leave disconnected |

| J2 AS5600 pin | Connect to |
| --- | --- |
| VCC | ESP32 `3V3` |
| GND | ESP32 `GND` |
| SDA | ESP32 `GPIO 21` |
| SCL | ESP32 `GPIO 22` |
| DIR | Leave disconnected |
| OUT | Leave disconnected |

Both modules share the ESP32's `3V3` and `GND`, but their SDA/SCL pairs remain
separate. Power the modules from 3.3 V so their onboard I2C pull-ups cannot
expose the ESP32 pins to 5 V. Do not connect VCC to the motor power supply.
Confirm the exact pin labels printed on the purchased modules before applying
power; some boards arrange the header pins differently.

## Product buttons and attachment detection

The product controller is implemented directly in
`SwivelCutArduino/SwivelCutArduino.ino`. The on/off control is a hardware
latching power switch and is not connected to an ESP32 input. Use a switch,
contactor, or power module rated for the actual motor-supply current; the
physical power disconnect remains the primary emergency stop.

The three momentary logic buttons connect between these pins and ground. The
firmware enables each ESP32 input's internal pull-up.

| Button | ESP32 pin | Behavior |
| --- | ---: | --- |
| Start/Stop | GPIO 18 | Start/stop teaching, start cutting, or abort a cut |
| Stabilization | GPIO 19 | Toggle recorded-path smoothing for the next cut |
| Repeat | GPIO 23 | Repeat the last taught cut with the cutting head fitted |

The attachment needs only an ID contact and ground contact. Fit an external
`10 kOhm` pull-up from `3V3` to GPIO 34 on the fixed arm, then connect GPIO 34
and ground through the pogo pins:

| Attachment | Resistor inside attachment |
| --- | ---: |
| Tracing head | `10 kOhm` from ID to ground |
| Cutting head | `2.2 kOhm` from ID to ground |

This produces approximately half-scale ADC for the tracing head, approximately
18% scale for the cutting head, and full scale when disconnected. GPIO 34 has
no internal pull-up. The ranges in `SwivelCutArduino.ino` intentionally leave
gaps so a loose, damaged, or unknown attachment cannot be treated as a cutting
head. The Arduino sketch uses 12-bit `analogRead()` values from 0 to 4095.
Check the assembled values and adjust `TRACING_ADC_*`, `CUTTING_ADC_*`, and
`DISCONNECTED_ADC_MIN` if resistor tolerance, wiring, or the ESP32 ADC shifts
them. Type `HEAD` in Serial Monitor to print the detected head and raw ADC
reading.

The blade actuator uses an H-bridge:

| H-bridge input | ESP32 pin |
| --- | ---: |
| Blade A | GPIO 13 |
| Blade B | GPIO 14 |

The default is a 500 ms forward pulse to extend and a 500 ms reverse pulse to
retract. Set `BLADE_DRIVE_MS` for the real mechanism. If a one-direction cam
extends and retracts on successive activations, set
`BLADE_REVERSE_TO_RETRACT = False`. Limit switches or position feedback are
strongly recommended because timing alone cannot prove the razor is retracted.
With a one-direction cam, place the mechanism in its retracted phase before
power-on because firmware cannot infer the cam phase after power is removed.

### Button workflow

1. Fold the arm to J1 = 0 degrees and J2 = 180 degrees, then switch power on.
2. Fit the tracing head and press Start/Stop to begin teaching.
3. Guide the arm, then press Start/Stop again to retain the raw recording.
4. Fit the cutting head and optionally press Stabilization.
5. Press Start/Stop to extend the razor and replay the path.
6. Press Repeat to run the same cut again.

Stabilization is applied from the untouched raw recording immediately before
each cut. Removing the active head or pressing Start/Stop during cutting stops
step generation, disables the arm drivers, and retracts the blade. Completion,
feedback errors, and other exceptions also retract the blade. Teaching is
limited to 60 seconds to bound ESP32 RAM use.

The supplied diametric magnet must be centered on the motor shaft, parallel to
the sensor face, and held at a stable gap. `ARM FOLDED` refuses to arm if either
AS5600 reports a missing, weak, or excessively strong magnetic field.

The encoder direction depends on which magnet pole faces the sensor and how the
module is mounted. Set `ENCODER_J1_SIGN` and `ENCODER_J2_SIGN` in the Arduino
sketch to `1` or `-1` so increasing logical motor angle also increases
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

- `SwivelCutArduino/SwivelCutArduino.ino`: primary Arduino IDE firmware,
  including buttons, head ID, blade actuation, teaching, smoothing, and replay.
- `as5600.py`: AS5600 register access and wrap-safe multi-turn tracking.
- `encoder_test.py`: standalone wiring, angle, and magnet diagnostic.
- `swivelcut.py`, `product_controller.py`, `serial_console.py`, and `main.py`:
  older MicroPython implementation retained as a reference.
- `swivelcut_visualizer.html`: browser visualizer for geometry, IK, and motion.

## Running it on the ESP32

### 1. Wire and configure the hardware

Use two TB6600 drivers: one for the NEMA 23 and one for the NEMA 17. Both motors
are 200 full steps per revolution; set both TB6600 drivers to 1/4
microstepping. Set each driver's current limit for its own motor; do not assume
both motors use the same current.

Connect the ESP32 signals using the pin table above. The ESP32, both signal
interfaces and the motor power supply must have the required common reference
for the input wiring method used by your TB6600 modules. Do not power the motors
from the ESP32. Confirm that your TB6600 inputs reliably accept 3.3 V logic;
otherwise use a suitable transistor or level-shifting interface.

Test initially with the blade removed, low motor current, and an accessible
emergency power switch.

### Test the encoders before motor operation

Keep the blade removed and motor power off for the first test. Upload the
Arduino sketch with the arm folded and open Serial Monitor at `115200` baud.
The sketch reports a warning if either AS5600 address is missing and reports an
error if either magnet check fails.

After successful folded calibration, type `ENC`, move one shaft a very small
amount by hand, and type `ENC` again. The matching joint angle must change
smoothly in the expected direction while the other remains steady. If an
encoder is missing, disconnect power and check its 3.3 V, ground, SDA, SCL,
magnet centering, and sensor gap before continuing.

### 2. Install and configure Arduino IDE

1. Install Arduino IDE 2.x.
2. Add Espressif's ESP32 board package through Boards Manager.
3. Open `SwivelCutArduino/SwivelCutArduino.ino`.
4. Select the matching ESP32 board and USB port.
5. Compile and upload the sketch.
6. Open Serial Monitor at `115200` baud for status and bench commands.

### 3. Confirm startup and test slowly

Physically place the arm in the fully folded pose before every reset or
power-on. The Arduino product controller checks both magnets and calibrates
that pose automatically. It leaves the arm drivers disabled until a cut starts.
The Serial Monitor remains available for diagnostics and manual bench commands.

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
setting in `SwivelCutArduino.ino`, upload the sketch again, and reset.

### 4. Normal commands

For continuous encoder output, calibrate first and opt in:

```text
ARM FOLDED
STREAM RATE 10
STREAM ON
```

The serial monitor then prints `ENC_STREAM J1=... J2=...` continuously,
including while the motors move or remain disarmed. Use `STREAM OFF` to stop.
The accepted stream rate is 1-50 Hz.

For controlled open-loop testing, `FEEDBACK OFF` disables encoder position
faults and corrective motor pulses while leaving encoder streaming and teaching
available. Restore normal protection with `FEEDBACK ON`; feedback defaults to
on after every reset. `FEEDBACK STATUS` prints the current mode.

```text
ARM J1
J1 5
TEACH J1 5
PLAY
DISARM
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
the drivers remain disabled. The recorded joint path is stabilized before it is
stored: smoothing runs both forward and backward, so jitter is reduced without
adding timing lag, and the first and last positions are kept exactly. Replay
with:

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

Stabilization is tuneable from the `TEACH` command:

```text
TEACH <seconds> [Hz] [smooth_ms] [max_deviation_deg]
```

The defaults are `150 ms` smoothing and `1.0 degree` maximum deviation.

- `smooth_ms` controls how much time-neighbouring samples influence each other.
  Increase it to remove slower hand tremor; decrease it to retain sharper,
  quicker details. Start around `100-200 ms`. Values around `300-500 ms` are
  stronger but will soften deliberate corners.
- `max_deviation_deg` limits how far either stabilized joint angle may move from
  its measured value. Decrease it to preserve the taught shape more strictly;
  increase it when encoder noise is larger. A useful starting range is
  `0.5-1.0` degrees.
- Set either value to `0` to disable stabilization and retain raw samples.
- Tune at the normal recording rate. A higher sample rate captures more detail
  but does not by itself remove jitter.

For example, use stronger smoothing with tighter shape protection:

```text
TEACH 5 30 250 0.5
```

Stabilization filters the recorded command path; it cannot remove mechanical
play, blade flex, or encoder errors that are not visible at the motor shafts.

Keep holding clear after `PLAY`: returning from the taught end pose to its
starting pose is an automatic motor move. Teach/replay is joint-space motion,
not a guarantee that the blade follows a straight Cartesian path.

## Tests

The controller's hardware imports are replaced with small test doubles:

```sh
python3 -m unittest discover -s tests -v
node tests/test_swivelcut_visualizer.js
```
