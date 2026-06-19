const assert = require("assert");
const fs = require("fs");

const firmware = fs.readFileSync(
  "SwivelCutArduino/SwivelCutArduino.ino",
  "utf8",
);

assert.match(firmware, /constexpr int RELAY_PIN = 15;/);
assert.match(firmware, /constexpr uint8_t RELAY_CONNECTED_LEVEL = HIGH;/);
assert.match(firmware, /constexpr uint8_t RELAY_DISCONNECTED_LEVEL = LOW;/);
assert.match(
  firmware,
  /if \(button\.number == 4\) \{\s+if \(!pressed\) return;\s+setRelayConnected\(!relayConnected\);/,
);
assert.match(firmware, /BUTTON_LED_RED_PINS\[\]/);
assert.match(firmware, /BUTTON_LED_GREEN_PINS\[\]/);
assert.match(firmware, /command == "LEDS"/);
assert.match(firmware, /command == "PINS"/);
assert.match(firmware, /command == "RELAY ON"/);
assert.match(firmware, /command == "RELAY OFF"/);
assert.doesNotMatch(firmware, /ARM_TOGGLE_BUTTON_PIN/);
assert.doesNotMatch(firmware, /CONTROL_TEST_REPORT_MS/);
assert.doesNotMatch(firmware, /nextControlTestReportMs/);

console.log("panel control tests: OK");
