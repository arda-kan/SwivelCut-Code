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

There are currently no encoders or homing switches. Before every power-on, place
the arm in the folded startup pose. The controller then treats that pose as its
known position.

The folded pose is a kinematic singularity at `(0, 0)`. Use a joint-space or
point-to-point move to unfold the arm before calling `cut_line()`.

## ESP32 to TB6600 pins

- GPIO 25: J1 pulse
- GPIO 26: J1 direction
- GPIO 32: J2 pulse
- GPIO 33: J2 direction
- GPIO 27: shared active-low enable

Positive joint angles are counterclockwise when viewed from above the cutting
plane. The current direction settings assume a high DIR signal produces that
motion. Reverse the relevant motor connector if the installed wiring moves a
joint clockwise for a positive command.

Coordinates use `X = sideways` and `Y = forward`. With both links straight,
`J1=0`, `J2=0` is `(X=0, Y=400)`.

The STEP outputs idle HIGH and pulse LOW. This matches the common-anode wiring
shown earlier, where the interface sinks `PUL-` during a step.

## Files

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

The drivers remain disabled until this exact confirmation is received. Begin
with small moves and keep clear of the mechanism:

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
POS
DISARM
HELP
```

Angles are absolute degrees, not relative movements. `ANGLES 30 120` means
"move J1 to +30 degrees and J2 to +120 degrees." `XY` values are absolute
millimetres from the shoulder pivot. `XYJ1` and `XYJ2` calculate the normal XY
solution but move only the named joint. Because the other joint stays still,
the blade will usually not finish at the requested XY point. Before `CUT`, move
to a safely unfolded starting position.

For example, this moves both links into the straight-ahead pose:

```text
XY 0 400
```

Pressing `Ctrl-C` while connected stops the Python program and disables the
drivers through the console cleanup handler. Cutting motor power with the
physical emergency switch remains the primary emergency stop.

## Tests

The controller's hardware imports are replaced with small test doubles:

```sh
python3 -m unittest discover -s tests -v
node tests/test_swivelcut_visualizer.js
```
