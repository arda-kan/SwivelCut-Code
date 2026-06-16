const assert = require("assert");
const fs = require("fs");

const firmware = fs.readFileSync(
  "SwivelCutArduino/SwivelCutArduino.ino",
  "utf8"
);

assert.ok(firmware.includes("timerBegin(STEPPER_TIMER_HZ)"));
assert.ok(firmware.includes("timerAttachInterruptArg("));
assert.ok(firmware.includes("stepAxisTimerIsr"));
assert.ok(firmware.includes("startSegment("));
assert.ok(firmware.includes("segmentInProgress()"));
assert.ok(firmware.includes("volatile long j1PositionSteps"));
assert.ok(firmware.includes("volatile long j2PositionSteps"));
assert.ok(firmware.includes("__atomic_add_fetch("));
assert.ok(!firmware.includes("pulseSelectedAxes("));
assert.ok(!firmware.includes("waitForContinuousDeadline("));
assert.ok(!firmware.includes("STEP_HALF_PERIOD_US"));

console.log("interrupt stepper core tests: OK");
