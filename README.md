# SwivelCut Code

ESP32 firmware and browser tools for the two-joint SwivelCut cardboard cutter.

## Files

- `SwivelCutArduino/SwivelCutArduino.ino`: ESP32 firmware.
- `swivelcut_visualizer.html`: SVG path converter, reachability checker, and
  `LOAD POINTS` package generator.
- `swivelcut_firmware_simulator.html`: kinematics and firmware serial-console
  simulator.
- `tests/test_swivelcut_visualizer.js`: SVG converter math tests.
- `tests/test_firmware_simulator.js`: firmware simulator behavior tests.

Both HTML tools are self-contained and can be opened directly in a browser.

## Machine Configuration

- Link lengths: 200 mm + 200 mm
- J1 gear ratio: 6:1
- J2 gear ratio: 9:1
- Motors: 200 full steps/revolution
- TB6600 microstepping: 1/4
- Folded startup pose: J1 = 0 degrees, J2 = 180 degrees
- J1 limits: -90 to +90 degrees
- J2 limits: -180 to +180 degrees
- Arm coordinates: `J1=-90, J2=0` is `X=400, Y=0`;
  `J1=90, J2=0` is `X=-400, Y=0`

Coordinates use positive X to the physical right and positive Y forward.

## ESP32 Pins

| Function | GPIO |
| --- | ---: |
| J1 STEP | 25 |
| J1 DIR | 26 |
| J2 STEP | 32 |
| J2 DIR | 33 |
| Shared driver enable | 27 |
| Blade H-bridge inputs | 13, 14 |
| J1 AS5600 SDA, SCL | 18, 19 |
| J2 AS5600 SDA, SCL | 16, 17 |
| Start/Stop button | 5 |
| Stabilization button | 22 |
| Repeat button | 23 |
| Head-ID ADC | 34 |

Each AS5600 uses address `0x36` on its own ESP32 I2C controller. Power both
encoder modules from 3.3 V. The buses run at 100 kHz and retry transient reads
three times before declaring a feedback fault.

Buttons are normally open to GND and use `INPUT_PULLUP`. Pressed is LOW.

## Head Identification

The divider is:

```text
3.3V -- 10k fixed resistor -- GPIO34 -- head resistor -- GND
```

| Head | Resistor | ADC range |
| --- | ---: | ---: |
| Cutter | 2.2k | 400-1125 |
| Tracer | 10k | 1500-2550 |
| Disconnected | Open | 3500-4095 |

Five consistent readings are required before a head change is accepted.

## Firmware Workflow

1. Fold the arm and send `ARM FOLDED`.
2. Install the tracer head.
3. Press Start/Stop once to begin recording and press it again to stop.
4. Install the cutter head.
5. Optionally toggle Stabilization while idle.
6. Press Start/Stop once to run the complete cut.
7. Press Repeat to repeat the last completed cut.

Head removal during teaching discards the incomplete trace. A confirmed cutter
head removal during a cut aborts motion and retracts the blade. Product buttons
are ignored whenever the motors are moving, including cuts and repeats;
Stabilization can only change while the machine is idle and stationary.
After a successful cut, Start/Stop will not run the same trace again; teach a
new movement or use Repeat for another pass.

The firmware prints a `REPORT` after operations with software J1/J2, XY,
encoder J1/J2, and raw encoder counts.

## Serial Commands

Key commands include:

```text
ARM FOLDED
ARM J1
ARM J2
DISARM
ENC
POS
ANGLES <j1> <j2>
XY <x> <y> [UP|DOWN]
CUT <x0> <y0> <x1> <y1> [UP|DOWN]
TEACH [J1] <seconds> [Hz] [smooth_ms] [max_dev]
PLAY
CLEAR
LOAD POINTS <N>
CUT LOADED
CONTROL TEST ON
CONTROL TEST OFF
STATE TEST ON
STATE TEST OFF
HELP
```

After `LOAD POINTS <N>`, send exactly N lines containing:

```text
<j1Deg> <j2Deg>
```

Use `swivelcut_visualizer.html` to generate this package from an SVG.

## Replay Mode

`CONTINUOUS_TRAJECTORY_REPLAY` near the top of the firmware selects the replay
executor. Its default `true` value paces the taught samples as one continuous
step stream, with periodic safety feedback checks and closed-loop settling only
at the final point. Set it to `false` to restore point-by-point motion and
feedback settling. The continuous executor automatically slows the full
trajectory when a recorded segment requests more steps than the configured
pulse timing permits.

## Stabilization

Joint-space smoothing remains the default. Optional XY smoothing can be enabled
with `XY_SMOOTHING_IMPLEMENTED`. Its deviation clamp is scaled per point using
the point's actual reach, with a 20 mm minimum radius, and it retries reduced
time windows if a smoothed point cannot be converted through IK.

## Build

Open `SwivelCutArduino/SwivelCutArduino.ino` in Arduino IDE and select an ESP32
board. The current configuration compiles without PSRAM and uses approximately
29% of ESP32 global dynamic memory.

## Safety

Test with the blade removed first. The blade actuator is timed open-loop and
does not have position feedback. Limit switches or equivalent blade-position
feedback are recommended before production use.
