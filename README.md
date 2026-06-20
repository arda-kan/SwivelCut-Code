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
- Motor direction: J1 normal, J2 inverted
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
| Start/Stop button | 2 |
| Stabilization button (VP, external pull-up) | 36 |
| Repeat button (VN, external pull-up) | 39 |
| Relay toggle button | 0 |
| Relay control (HIGH = connected) | 15 |
| Four WS2812 LED data | 4 |
| Head-ID ADC | 34 |

Each AS5600 uses address `0x36` on its own ESP32 I2C controller. Power both
encoder modules from 3.3 V. The buses run at 100 kHz and retry transient reads
three times before declaring a feedback fault.

Buttons are normally open to GND and pressed is LOW. GPIO36 and GPIO39 have no
internal pull-ups and require the external 10 kΩ pull-ups used by the supplied
button test. GPIO0 and GPIO2 use `INPUT_PULLUP`. Button 4 toggles the relay:
GPIO15 HIGH connects it and LOW disconnects it. At startup the relay and motor
drivers are off. Turning button 4 ON requires the arm to be physically folded;
the firmware then zeros the folded pose and calibrates both encoders, but leaves
the motor drivers disabled so the arms remain manually movable while waiting.
Replay or cutting enables the arms only when motion starts. Turning power OFF
keeps the arms disabled before disconnecting the relay.

The four indicators are WS2812 addressable LEDs on GPIO4, using GRB order and
brightness 64. The physical strip runs in reverse panel order: Start/Stop uses
pixel 1, Stabilization pixel 2, Repeat pixel 3, and Relay pixel 0. Red means off
and green means on. When the relay/on-off button is OFF, all four pixels are
completely black; turning it ON restores the normal red/green indicators.

`CONTROL TEST ON` is intentionally different: button 4 only exercises the
relay and LEDs, while the motor drivers remain disabled.

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

Set `ASSUME_CUTTER_UNLESS_TRACER` near the top of the firmware to `true` for
the optional override mode. In that mode, readings inside the tracer ADC range
remain `TRACER`, while cutter, disconnected, and unknown readings are all
treated as `CUTTER`. It defaults to `false`, preserving normal detection.

## Firmware Workflow

1. Fold the arm and send `ARM FOLDED`.
2. Install the tracer head.
3. Press Start/Stop once to begin recording and press it again to stop.
4. Install the cutter head.
5. Optionally toggle Stabilization while idle.
6. Press Start/Stop once to run the complete cut.
7. With the cutter attached, press Repeat to repeat the last completed cut.
   With the tracer attached, press Repeat to replay and draw over the recorded
   trace without operating the blade.

Head removal during teaching discards the incomplete trace. A confirmed cutter
head removal during a cut aborts motion and retracts the blade. Product buttons
are ignored whenever the motors are moving, including cuts and repeats;
Stabilization can only change while the machine is idle and stationary.
After a successful cut, Start/Stop will not run the same trace again; teach a
new movement or use Repeat for another pass.

Repeat is head-sensitive. A cutter requires a previously completed cut and
performs the normal blade cycle. A tracer only requires a recorded path and
replays it without cutter-length compensation or blade movement. Stabilization
is applied to either replay when enabled.

The firmware prints a `REPORT` after operations with software J1/J2, XY,
encoder J1/J2, and raw encoder counts.

## Attachment Length Compensation

`CUTTER_EXTRA_LENGTH_MM` near the top of the firmware defaults to `0.0`. It is
the amount by which the cutter extends farther from J2 than the tracer. Before
each physical cut or repeat, the traced joint path is converted to XY using the
tracer length and solved again using the longer cutter length. This preserves
the traced tip path without accumulating an offset between repeat cuts. Set it
to a positive measured difference, such as `5.0`, if a future cutter attachment
extends beyond the tracer.

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
BLADE RETRACTED
BLADE DOWN
CONTROL TEST ON
CONTROL TEST OFF
STATE TEST ON
STATE TEST OFF
CONTROLS
LEDS
PINS
RELAY ON
RELAY OFF
RELAY STATUS
HELP
```

`CONTROL TEST ON` disables the motors and blade, prints debounced button
press/release levels, toggles the corresponding red/green LED on every press,
and allows button 4 to exercise the relay. After its initial snapshot, output
is printed only when a button, LED, relay, or head state changes. `CONTROLS`,
`LEDS`, and `PINS` provide one-shot serial wiring reports.

When `XY` omits `UP` or `DOWN`, the firmware automatically prefers `UP` for
positive X and `DOWN` for negative X, then tries the other branch if the
preferred solution violates a joint limit. `CUT` continues to default to `UP`
when its branch is omitted so one continuous cut never changes elbow branch.

After `LOAD POINTS <N>`, send exactly N lines containing:

```text
<j1Deg> <j2Deg>
```

Use `swivelcut_visualizer.html` to generate this package from an SVG.

The blade actuator exposes only two serial commands: `BLADE RETRACTED` and
`BLADE DOWN`. Both commands drive the actuator for the timed duration configured
near the top of the firmware. Cut operations force the blade retracted before
moving to the start point, drive it down for the cutting pass, and retract it
again on completion or stop.

## Replay Mode

`CONTINUOUS_TRAJECTORY_REPLAY` near the top of the firmware selects the replay
executor. Its default `true` value paces the taught samples as one continuous
step stream, with periodic safety feedback checks and closed-loop settling only
at the final point. Set it to `false` to restore point-by-point motion and
feedback settling. The continuous executor automatically slows the full
trajectory when a recorded segment requests more steps than the configured
pulse timing permits.

## Stepper Timing

The firmware uses two independent ESP32 hardware timers, one for each TB6600
STEP pin. Timer ISRs emit active/idle pulse phases and atomically update the
software joint positions as pulses are generated. J1 and J2 therefore run at
different frequencies within the same trajectory segment while controls,
encoder streaming, feedback checks, and cut aborts continue to be serviced.

The implementation targets the Arduino-ESP32 3.x `timerBegin(frequency)` API.
`STEPPER_TIMER_HZ`, `STEPPER_MIN_HALF_PERIOD_US`, and
`DEFAULT_STEP_RATE_HZ` near the top of the sketch control timer resolution,
maximum pulse rate, and ordinary point-move speed.

## Stabilization

XY smoothing is enabled by default with `XY_SMOOTHING_IMPLEMENTED`. Its
deviation clamp is scaled per point using the point's actual reach, with a
20 mm minimum radius, and it retries reduced time windows if a smoothed point
cannot be converted through IK. Set `XY_SMOOTHING_IMPLEMENTED` to `false` to
return to joint-space angle smoothing.

## Build

Open `SwivelCutArduino/SwivelCutArduino.ino` in Arduino IDE and select an ESP32
board. The current configuration compiles without PSRAM and uses approximately
29% of ESP32 global dynamic memory.

## Safety

Test with the blade removed first. The blade actuator is timed open-loop and
does not have position feedback. Tune `BLADE_DOWN_SECONDS` and
`BLADE_RETRACT_SECONDS` on the machine before cutting material. Limit switches
or equivalent blade-position feedback are recommended before production use.
