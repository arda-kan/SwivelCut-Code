const assert = require("assert");
const fs = require("fs");

const firmware = fs.readFileSync(
  "SwivelCutArduino/SwivelCutArduino.ino",
  "utf8"
);

assert.ok(firmware.includes("constexpr float BLADE_DOWN_SECONDS"));
assert.ok(firmware.includes("constexpr float BLADE_RETRACT_SECONDS"));
assert.ok(firmware.includes("constexpr int BLADE_PWM_PIN = 13;"));
assert.ok(firmware.includes("constexpr int BLADE_DIR_PIN = 14;"));
assert.ok(firmware.includes("constexpr int BLADE_PWM_FREQUENCY_HZ = 1000;"));
assert.ok(firmware.includes("constexpr int BLADE_PWM_RESOLUTION_BITS = 8;"));
assert.ok(firmware.includes("constexpr uint8_t BLADE_PWM_DUTY = 200;"));
assert.ok(firmware.includes("constexpr uint8_t BLADE_DOWN_DIRECTION = HIGH;"));
assert.ok(firmware.includes("constexpr uint8_t BLADE_RETRACT_DIRECTION = LOW;"));
assert.ok(firmware.includes("enum class BladePosition"));
assert.ok(firmware.includes("void stopBlade();"));
assert.ok(firmware.includes("bool bladeIsDown();"));
assert.ok(firmware.includes("void bladeRetracted(bool force = false);"));
assert.ok(firmware.includes('command == "BLADE RETRACTED"'));
assert.ok(firmware.includes('command == "BLADE DOWN"'));
assert.ok(!firmware.includes('command == "BLADE UP"'));
assert.ok(firmware.includes("if (operateBlade) bladeRetracted(true);"));
assert.ok(firmware.includes("if (operateBlade) bladeDown();"));
assert.ok(firmware.includes("bladeRetracted(true);\n  if (!moveToXY(x0, y0, elbowDown))"));
assert.ok(firmware.includes("if (bladeIsDown()) bladeRetracted();"));
assert.ok(firmware.includes("driveBlade(BLADE_DOWN_DIRECTION, BLADE_DOWN_SECONDS);"));
assert.ok(firmware.includes("driveBlade(BLADE_RETRACT_DIRECTION, BLADE_RETRACT_SECONDS);"));
assert.ok(!firmware.includes("BLADE_IN1_PIN"));
assert.ok(!firmware.includes("BLADE_IN2_PIN"));
assert.ok(!firmware.includes("BLADE_REVERSE_TO_RETRACT"));

console.log("blade endpoint logic tests: OK");
