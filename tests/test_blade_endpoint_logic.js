const assert = require("assert");
const fs = require("fs");

const firmware = fs.readFileSync(
  "SwivelCutArduino/SwivelCutArduino.ino",
  "utf8"
);

assert.ok(firmware.includes("constexpr float BLADE_DOWN_SECONDS"));
assert.ok(firmware.includes("constexpr float BLADE_RETRACT_SECONDS"));
assert.ok(firmware.includes("enum class BladePosition"));
assert.ok(firmware.includes("bool bladeIsDown();"));
assert.ok(firmware.includes("void bladeRetracted(bool force = false);"));
assert.ok(firmware.includes('command == "BLADE RETRACTED"'));
assert.ok(firmware.includes('command == "BLADE DOWN"'));
assert.ok(!firmware.includes('command == "BLADE UP"'));
assert.ok(firmware.includes("if (operateBlade) bladeRetracted(true);"));
assert.ok(firmware.includes("if (operateBlade) bladeDown();"));
assert.ok(firmware.includes("bladeRetracted(true);\n  if (!moveToXY(x0, y0, elbowDown))"));
assert.ok(firmware.includes("if (bladeIsDown()) bladeRetracted();"));

console.log("blade endpoint logic tests: OK");
