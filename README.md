# SwivelCut Code

Controller and motion visualizer for a two-joint planar SCARA-style cardboard
cutting arm.

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
