const assert = require("assert");
const fs = require("fs");

const firmware = fs.readFileSync(
  "SwivelCutArduino/SwivelCutArduino.ino",
  "utf8",
);

assert.match(firmware, /void printReplayEndpointReport\(int targetPoint, bool completed\)/);
assert.match(firmware, /Serial\.print\("REPLAY_ENDPOINT status="\)/);
assert.match(firmware, /Serial\.print\(" target_J1="\)/);
assert.match(firmware, /Serial\.print\(" target_J2="\)/);
assert.match(firmware, /Serial\.print\(" measured_J1="\)/);
assert.match(firmware, /Serial\.print\(" measured_J2="\)/);
assert.match(firmware, /Serial\.print\(" error_J1="\)/);
assert.match(firmware, /Serial\.print\(" error_J2="\)/);
assert.match(
  firmware,
  /printReplayEndpointReport\(stoppedPoint, false\);[\s\S]*disableDrivers\(\);/,
);
assert.match(
  firmware,
  /printReplayEndpointReport\(taughtCount - 1, true\);[\s\S]*disableDrivers\(\);/,
);

console.log("replay endpoint report tests: OK");
