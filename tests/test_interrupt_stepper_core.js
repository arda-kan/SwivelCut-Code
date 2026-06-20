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
assert.ok(firmware.includes("constexpr float REPLAY_STEP_RATE_HZ = 120.0f;"));
assert.ok(firmware.includes("constexpr float REPLAY_MAX_ACCEL_STEPS_PER_S2 = 60.0f;"));
assert.ok(firmware.includes("replaySegmentDurationUs("));
assert.ok(firmware.includes("trajectoryDirectionReversesAfter("));
assert.ok(firmware.includes("PLAY MODE: CONSTANT_PACE RATE_HZ="));
assert.ok(!firmware.includes("continuousTrajectoryTimeScale("));

console.log("interrupt stepper core tests: OK");
