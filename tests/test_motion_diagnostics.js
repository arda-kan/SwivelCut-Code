const assert = require("assert");
const fs = require("fs");

const firmware = fs.readFileSync(
  "SwivelCutArduino/SwivelCutArduino.ino",
  "utf8",
);

assert.match(firmware, /constexpr bool MOTION_DIAGNOSTICS = false;/);
assert.match(firmware, /void printMotionDiagnostic\(/);
assert.match(firmware, /MOTION_DIAG stage=/);
assert.match(firmware, / correction=/);
assert.match(firmware, / dir_J1=/);
assert.match(firmware, / dir_J2=/);
assert.match(firmware, / target_X=/);
assert.match(firmware, / measured_X=/);
assert.match(firmware, / error_X=/);
assert.match(firmware, /CALIBRATION_DIAG raw_J1=/);
assert.match(firmware, /MOTION_DIAG cutter_point=/);
assert.match(
  firmware,
  /if \(MOTION_DIAGNOSTICS\) \{\s+printMotionDiagnostic\(\s*"CORRECTION"/,
);

console.log("motion diagnostics tests: OK");
