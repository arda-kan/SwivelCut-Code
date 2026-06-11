# SwivelCut Arduino IDE version

Open `SwivelCutArduino/SwivelCutArduino.ino` in Arduino IDE and select
`ESP32 Dev Module`.

The sketch uses direct GPIO pulse generation like the Arduino example:

```cpp
digitalWrite(PUL, LOW);
delayMicroseconds(1500);
digitalWrite(PUL, HIGH);
delayMicroseconds(1500);
```

It does not require AccelStepper or another stepper library.

## TB6600 settings and wiring

- Set both TB6600 drivers to 1/4 microstepping.
- `PUL+`, `DIR+`, and `ENA+` connect to ESP32 `3.3V`.
- J1 `PUL-` connects to GPIO 25.
- J1 `DIR-` connects to GPIO 26.
- J2 `PUL-` connects to GPIO 32.
- J2 `DIR-` connects to GPIO 33.
- Both `ENA-` terminals connect to GPIO 27.
- ESP32 ground, driver signal ground, and power-supply reference must match the
  requirements of the specific TB6600 modules.

The measured driver behavior is encoded as:

- `ENA-` HIGH: motor outputs enabled.
- `ENA-` LOW: motor outputs disabled.
- `PUL-` LOW: active half of the step pulse.
- `PUL-` HIGH: idle half of the step pulse.

## First test

Remove the blade and belts or other mechanical load. Open Serial Monitor at
115200 baud and use:

```text
ARM FOLDED
TEST J1 800
TEST J1 -800
TEST J2 800
TEST J2 -800
DISARM
```

At 1/4 microstepping, 800 pulses are one motor-shaft revolution. The 1500 us
LOW plus 1500 us HIGH waveform is approximately 333 pulses per second, so one
unloaded revolution takes about 2.4 seconds.

The `J1`, `J2`, and `ANGLES` commands use the configured 6:1 and 9:1 reductions:

```text
ARM FOLDED
J2 175
J1 5
ANGLES 0 180
POS
DISARM
```
