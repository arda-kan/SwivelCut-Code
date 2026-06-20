const assert = require("assert");
const fs = require("fs");

const firmware = fs.readFileSync(
  "SwivelCutArduino/SwivelCutArduino.ino",
  "utf8",
);

const start = firmware.indexOf("void runTracerReplay()");
const end = firmware.indexOf("\nvoid loadPointsFromSerial", start);
assert.ok(start >= 0 && end > start);
const replay = firmware.slice(start, end);

assert.match(replay, /stableHeadType != HeadType::TRACING/);
assert.match(replay, /taughtCount < 2/);
assert.match(replay, /prepareTaughtPath/);
assert.match(replay, /prepareTracerReplayFromNearestEndpoint/);
assert.match(replay, /replayTeach\(false\)/);
assert.doesNotMatch(replay, /compensateTaughtPathForCutter/);
assert.doesNotMatch(replay, /bladeDown/);
assert.doesNotMatch(replay, /productHasLastCut/);

assert.match(firmware, /void reversePreparedTaughtPath\(\)/);
assert.match(
  firmware,
  /if \(lastDistance < firstDistance\) \{\s+reversePreparedTaughtPath\(\);/,
);

console.log("tracer replay logic tests: OK");
