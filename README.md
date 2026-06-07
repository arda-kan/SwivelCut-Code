# SwivelCut Code

Controller and motion visualizer for a two-joint planar SCARA-style cardboard
cutting arm.

## Fixed machine

- 200 mm shoulder link and 200 mm elbow link
- NEMA 23 shoulder motor with 6:1 pulley reduction
- NEMA 17 elbow motor mounted on link 1 with 9:1 pulley reduction
- 200 full steps per motor revolution
- TB6600 drivers set to 1/32 microstepping
- Required startup pose: J1 = 0 degrees, J2 = 180 degrees (fully folded)
- One revolution of software travel per joint: -180 to +180 degrees

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

## Files

- `scara_arm.py`: MicroPython controller for an ESP32 and two stepper axes.
- `arm_visualizer.html`: standalone browser visualizer for geometry, IK, and
  joint motion.

## Tests

The controller's hardware imports are replaced with small test doubles:

```sh
python3 -m unittest discover -s tests -v
node tests/test_visualizer.js
```
