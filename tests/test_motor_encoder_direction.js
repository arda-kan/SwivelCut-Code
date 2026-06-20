const assert = require("assert");
const fs = require("fs");

const firmware = fs.readFileSync(
  "SwivelCutArduino/SwivelCutArduino.ino",
  "utf8",
);

// Bench replay showed J1 tracking its commanded direction while J2 moved by
// nearly the same amount in the opposite direction. Manual tracing showed the
// J2 encoder sign was already coherent, so only the motor direction is flipped.
assert.match(firmware, /constexpr bool INVERT_J1 = false;/);
assert.match(firmware, /constexpr bool INVERT_J2 = true;/);
assert.match(firmware, /constexpr int ENCODER_J2_SIGN = 1;/);
assert.match(firmware, /float normalizeJointDegrees\(float degreesValue\)/);
assert.match(
  firmware,
  /j2Deg = normalizeJointDegrees\(\s*180\.0f \+ motor2Deg \/ J2_GEAR_RATIO\);/,
);

console.log("motor/encoder direction tests: OK");
