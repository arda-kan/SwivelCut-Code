# SwivelCut Code

Controller and motion visualizer for the two-joint SwivelCut cardboard cutter.

## Arduino IDE firmware

The primary ESP32 firmware is now
`SwivelCutArduino/SwivelCutArduino.ino`. Open that file in Arduino IDE, select
an ESP32 board, and upload it at 115200 baud. The older MicroPython files remain
in the branch as migration reference.

The Arduino sketch drives each TB6600 directly with the proven DFRobot-style
waveform: STEP is HIGH for 1500 microseconds and LOW for 1500 microseconds.
GPIO 25/26/27 are J1 pulse, direction, and enable; GPIO 32/33 are J2 pulse and
direction. Start with the blade removed, fold the arm, send `ARM FOLDED`, then
use `TEST J1 800` or `TEST J2 800` for a controlled base motor test.

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
- GPIO 13/14: blade H-bridge inputs
- GPIO 18: Start/Stop button, normally open to GND
- GPIO 19: Stabilization button, normally open to GND
- GPIO 23: Repeat button, normally open to GND
- GPIO 34: head-ID voltage-divider ADC input

GPIO18, GPIO19, and GPIO23 use the ESP32 internal pull-ups. A pressed button
reads LOW and a released button reads HIGH. The firmware debounces each input
for 35 ms and prints named `PRESSED` and `RELEASED` events. The latching power
switch cuts hardware power directly and is not connected to a GPIO.

The head-ID divider is:

```text
3.3V -- 10k fixed resistor -- GPIO34 -- head resistor -- GND
```

The firmware uses 12-bit ADC readings and requires five consecutive matching
classifications before reporting a head change:

| Head | Head resistor | Accepted ADC range |
| --- | ---: | ---: |
| Cutting | 2.2k | 400-1125 |
| Tracing | 10k | 1500-2550 |
| Disconnected | Open | 3500-4095 |
| Unknown | Any other reading | Outside the ranges above |

Use `CONTROLS` in the serial console to print all three button states and the
current head type/ADC value. Button and stable head changes are also printed
automatically.

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

- `as5600.py`: AS5600 register access and wrap-safe multi-turn tracking.
- `encoder_test.py`: standalone wiring, angle, and magnet diagnostic.
- `swivelcut.py`: MicroPython controller for an ESP32 and two stepper axes.
- `serial_console.py`: guarded USB serial command parser.
- `main.py`: starts the serial console automatically when the ESP32 boots.
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

Keep the TB6600 motor supply switched off. From the repository directory, run:

```sh
python3 -m mpremote connect auto run encoder_test.py
```

The expected startup output contains address `0x36` on both buses:

```text
J1 bus: SDA=GPIO16 SCL=GPIO17 devices=['0x36']
J2 bus: SDA=GPIO21 SCL=GPIO22 devices=['0x36']
Both encoders found. Press Ctrl-C to stop.
```

Slowly rotate each motor shaft by hand. Its `raw` value and angle should change
smoothly while the other encoder remains steady. The angle wraps between about
359.9 and 0 degrees once per motor-shaft revolution.

The magnet result should be `OK`:

- `NOT DETECTED`: magnet absent, too far away, badly off-center, or incorrect
  magnet type.
- `TOO WEAK`: reduce the sensor-to-magnet gap or improve centering.
- `TOO STRONG`: increase the sensor-to-magnet gap.

If one bus does not list `0x36`, switch power off and check VCC, GND, SDA, and
SCL for that module. Do not move wiring while it is powered. Stop the test with
`Ctrl-C`.

For `devices=[]`, use this order:

1. Disconnect USB and motor power.
2. Remove J2 completely and test only J1.
3. Check continuity from the ESP32 pin itself to the sensor header; do not rely
   only on breadboard row alignment.
4. Confirm the sensor receives about 3.3 V between its VCC and GND pins.
5. Confirm SDA and SCL are not swapped and neither is shorted to GND or 3.3 V.
6. Try the other AS5600 module on the same J1 wires. If the second module works,
   the first module or its solder joints are faulty.

Do not connect external pull-up resistors until the module has been inspected.
Most AS5600 breakout boards already include I2C pull-ups.

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

### 8. Serial command reference

Commands are case-insensitive. Values are separated by spaces, angles are
absolute degrees, and Cartesian positions are absolute millimetres from the
shoulder pivot. Positive `X` points physically right, positive `Y` points
forward, and positive joint rotation is counterclockwise when viewed from
above.

| Command | What it does | Important behavior |
| --- | --- | --- |
| `ARM FOLDED` | Calibrates both encoders at `J1=0`, `J2=180` and enables the drivers. | Physically fold the arm first. This command resets the controller's assumed pose. |
| `ARM J1` | Calibrates only the J1 encoder at `J1=0` for single-motor testing and teaching. | Physically fold the arm first. Use `TEACH J1`, not ordinary `TEACH`. |
| `ARM J2` | Calibrates only the J2 encoder at the folded `J2=180` home and enables J2. | Physically fold the arm first. Only `J2 <deg>` motion is allowed in this mode. |
| `DISARM` | Immediately disables both motor drivers. | To resume ordinary motion, physically fold the arm and use `ARM FOLDED` again. |
| `ENC` | Reports whether each AS5600 is found and whether its magnet is OK, weak, strong, or missing. | Calibrated encoder angles are included when available. |
| `STREAM ON` | Continuously prints measured encoder angles and raw values. | `RAW1` and `RAW2` are signed 12-bit counts (`-2048` to `2047`), so decreasing rotation crosses smoothly through zero. |
| `STREAM OFF` | Stops continuous encoder output. | Does not change motor state. |
| `STREAM RATE <Hz>` | Sets continuous output from 1 to 50 Hz. | Example: `STREAM RATE 20`. |
| `FEEDBACK OFF` | Disables encoder position faults and corrective pulses. | Motor motion becomes open-loop; use only for controlled testing. |
| `FEEDBACK ON` | Restores encoder correction and position-fault shutdown. | This is the default after every reset. |
| `FEEDBACK STATUS` | Prints the current feedback mode. | Does not change motor state. |
| `CONTROLS` | Prints all three debounced button states, the hardware-only power switch note, and the stable head ID with its ADC reading. | Button and stable head changes are also printed automatically. |
| `POS` | Prints the controller's current `X`, `Y`, `J1`, and `J2` state. | This is software state. It is not refreshed by arbitrary hand movement while disarmed. |
| `J1 <deg>` | Moves J1 to an absolute shoulder angle while holding J2. | Allowed range is `-90` to `+90` degrees. |
| `J2 <deg>` | Moves J2 to an absolute elbow angle while holding J1. | Allowed range is `-180` to `+180` degrees. |
| `ANGLES <j1> <j2>` | Moves both joints together to absolute angles. | This is a joint-space move; the blade path is generally curved. |
| `XY <x> <y> [UP\|DOWN]` | Solves inverse kinematics and moves both joints to the requested tip position. | Defaults to `UP`. This is point-to-point motion, not a straight cut. |
| `XYJ1 <x> <y> [UP\|DOWN]` | Solves the requested XY pose but applies only its J1 angle. | J2 stays fixed, so the final blade position will normally not equal the entered XY point. |
| `XYJ2 <x> <y> [UP\|DOWN]` | Solves the requested XY pose but applies only its J2 angle. | J1 stays fixed, so the final blade position will normally not equal the entered XY point. |
| `CUT <x0> <y0> <x1> <y1> [UP\|DOWN]` | Moves to `(x0,y0)`, then follows a validated straight line to `(x1,y1)`. | The arm must already be unfolded. The controller checks the whole line before moving. |
| `TEACH <seconds> [Hz]` | Disables the drivers and records both joint encoders while the arm is guided by hand. | This original mode still requires both encoders. |
| `TEACH J1 <seconds> [Hz]` | Records only J1 after `ARM J1`. | J2 remains fixed during replay. |
| `PLAY` | Enables the drivers, returns to the taught start, and replays the recording. | Keep clear: the return-to-start move happens automatically. |
| `CLEAR` | Deletes the taught path from RAM. | Recordings are also lost on reset or power loss. |
| `HELP` | Prints the command list in the serial terminal. | Useful for checking the exact accepted syntax. |

#### Joint-space moves

Use `J1`, `J2`, or `ANGLES` when the joint angles themselves matter or when
unfolding from the singular startup pose:

```text
J2 175
J1 5
ANGLES 20 120
```

These commands do not promise a straight blade path. `ANGLES 30 120` means
"move J1 to +30 degrees and J2 to +120 degrees", not move each joint by that
amount.

#### Point-to-point XY moves

`XY` chooses joint angles that place the blade at the requested coordinate:

```text
XY 0 400 UP
XY 200 100 DOWN
```

`UP` and `DOWN` select the two inverse-kinematics branches. They can reach the
same XY point with different joint poses. A branch is rejected if either
resulting angle violates its software limit. With both links straight,
`XY 0 400` corresponds to `J1=0`, `J2=0`.

`XYJ1` and `XYJ2` are diagnostic or setup tools, not partial Cartesian moves.
For example, `XYJ1 200 100 UP` calculates the complete solution for
`(200,100)` but moves only the shoulder to that solution's J1 angle. The elbow
remains where it was.

#### Straight cuts

`CUT` has an explicit start and end:

```text
XY 0 20 UP
CUT 0 20 0 400 UP
```

The first command unfolds the arm. The `CUT` command then:

1. Rejects the operation if the arm is still folded at `(0,0)`.
2. Checks that both endpoints and the entire line are in the reachable annulus.
3. Solves every 2 mm point along the line and checks joint limits before motion.
4. Moves point-to-point to `(x0,y0)`, even if the blade is currently elsewhere.
5. Follows the straight segment from `(x0,y0)` to `(x1,y1)`.

Therefore, use the blade's intended entry point as `(x0,y0)`. Do not assume a
command such as `CUT 100 100 250 100` cuts from the current position to
`(250,100)`; it first moves to `(100,100)`.

#### State and safety commands

`ENC` and `POS` answer different questions:

```text
ENC
POS
```

- `ENC` samples the physical motor-shaft encoders.
- `POS` prints the controller's tracked pose.
- After a normal powered move or completed `TEACH`, they should closely agree.
- After hand-moving a disarmed arm, `ENC` changes but `POS` may remain stale
  until the controller explicitly synchronizes from the encoders.

`DISARM` removes holding torque. The arm may move under hand force or gravity,
so do not rely on the previous software pose after disarming. Use the required
folded calibration procedure before returning to ordinary powered commands.

Typical safe powered session:

```text
ARM FOLDED
ENC
J2 175
J1 5
XY 0 20 UP
CUT 0 20 0 300 UP
POS
DISARM
```

If a command is malformed, unreachable, outside a joint limit, or unsafe from
the folded singularity, the console prints `ERROR:` and does not execute that
command. An encoder feedback failure prints `FEEDBACK FAULT:`, disables the
drivers, and requires the encoder issue to be corrected before folding and
arming again.

Pressing `Ctrl-C` while connected stops the Python program and disables the
drivers through the console cleanup handler. Cutting motor power with the
physical emergency switch remains the primary emergency stop.

## Browser visualizer

Open `swivelcut_visualizer.html` in a browser to preview the motion commands
without hardware. The command selector mirrors `J1`, `J2`, `ANGLES`, `XY`,
`XYJ1`, `XYJ2`, and `CUT`, and shows the equivalent serial command.

The visualizer uses the same conventions and checks as `swivelcut.py`:

- `X` is sideways/right and `Y` is forward.
- `J1=0`, `J2=0` points both links forward to `(0,400)`.
- The folded start is `J1=0`, `J2=180` at `(0,0)`.
- Elbow `UP`/`DOWN`, angle normalization, software limits, reachability,
  coupling-aware motor steps, and 2 mm `CUT` validation match the controller.
- `CUT` first moves to its explicit start point and then traces the line.

The visualizer predicts commanded geometry and step counts. It does not model
encoder noise, missed steps, acceleration timing, backlash, belt flex, or
post-move feedback correction, so it is a planning aid rather than a hardware
safety check.

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
