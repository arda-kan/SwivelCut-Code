const assert = require("assert");
const fs = require("fs");

const firmware = fs.readFileSync(
  "SwivelCutArduino/SwivelCutArduino.ino",
  "utf8",
);

assert.match(firmware, /constexpr int RELAY_PIN = 15;/);
assert.match(firmware, /constexpr int START_STOP_BUTTON_PIN = 2;/);
assert.match(firmware, /constexpr int STABILIZATION_BUTTON_PIN = 36;/);
assert.match(firmware, /constexpr int REPEAT_BUTTON_PIN = 39;/);
assert.match(firmware, /constexpr int RELAY_BUTTON_PIN = 0;/);
assert.match(firmware, /constexpr uint8_t RELAY_CONNECTED_LEVEL = HIGH;/);
assert.match(firmware, /constexpr uint8_t RELAY_DISCONNECTED_LEVEL = LOW;/);
assert.match(
  firmware,
  /if \(button\.number == 4\) \{\s+if \(!pressed\) return;\s+setMachinePower\(!relayConnected\);/,
);
assert.match(firmware, /#include <FastLED\.h>/);
assert.match(firmware, /constexpr int BUTTON_LED_DATA_PIN = 4;/);
assert.match(firmware, /constexpr int BUTTON_LED_COUNT = 4;/);
assert.match(firmware, /FastLED\.addLeds<WS2812, BUTTON_LED_DATA_PIN, GRB>/);
assert.match(firmware, /pinMode\(START_STOP_BUTTON_PIN, INPUT_PULLUP\);/);
assert.match(firmware, /pinMode\(STABILIZATION_BUTTON_PIN, INPUT\);/);
assert.match(firmware, /pinMode\(REPEAT_BUTTON_PIN, INPUT\);/);
assert.match(
  firmware,
  /ButtonLed buttonLeds\[\] = \{\s+\{1, 1, "START_STOP"[\s\S]*\{2, 2, "STABILIZATION"[\s\S]*\{3, 3, "REPEAT"[\s\S]*\{0, 4, "RELAY"/,
);
assert.match(firmware, /command == "LEDS"/);
assert.match(firmware, /command == "PINS"/);
assert.match(firmware, /command == "RELAY ON"/);
assert.match(firmware, /command == "RELAY OFF"/);
assert.doesNotMatch(firmware, /ARM_TOGGLE_BUTTON_PIN/);
assert.doesNotMatch(firmware, /CONTROL_TEST_REPORT_MS/);
assert.doesNotMatch(firmware, /nextControlTestReportMs/);
assert.match(
  firmware,
  /void refreshButtonLeds\(\) \{\s+if \(!relayConnected\) \{[\s\S]*CRGB::Black/,
);
assert.match(
  firmware,
  /void setMachinePower\(bool enabled\) \{[\s\S]*disableDrivers\(\);[\s\S]*setRelayConnected\(false\);[\s\S]*setRelayConnected\(true\);[\s\S]*armAtFoldedPose\(AxisMode::DUAL\);/,
);
assert.match(firmware, /command == "RELAY ON"\) return setMachinePower\(true\)/);
assert.match(firmware, /command == "RELAY OFF"\) return setMachinePower\(false\)/);
assert.match(firmware, /constexpr float CUTTER_EXTRA_LENGTH_MM = 5\.0f;/);
assert.match(firmware, /constexpr float CUTTER_LINK_2_MM = LINK_2_MM \+ CUTTER_EXTRA_LENGTH_MM;/);
assert.match(firmware, /bool compensateTaughtPathForCutter\(\)/);
assert.match(
  firmware,
  /prepareTaughtPath\([\s\S]*if \(!compensateTaughtPathForCutter\(\)\) return;/,
);

console.log("panel control tests: OK");
