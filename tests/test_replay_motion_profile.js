const assert = require("assert");

function segmentDuration(stepEvents, entryRate, remainingSteps, reverses, {
  rate = 120,
  acceleration = 60,
} = {}) {
  const reachable = Math.sqrt(
    entryRate * entryRate + 2 * acceleration * stepEvents,
  );
  const stopping = Math.sqrt(2 * acceleration * remainingSteps);
  let exitRate = Math.min(rate, reachable, stopping);
  if (reverses || remainingSteps === 0) exitRate = 0;
  const sum = entryRate + exitRate;
  const seconds = sum > 0.001
    ? 2 * stepEvents / sum
    : 2 * Math.sqrt(stepEvents / acceleration);
  return { seconds, exitRate };
}

let first = segmentDuration(60, 0, 300, false);
assert.ok(first.exitRate <= 120);
assert.ok(first.exitRate <= Math.sqrt(2 * 60 * 60));

let corner = segmentDuration(60, first.exitRate, 240, true);
assert.strictEqual(corner.exitRate, 0);

let final = segmentDuration(60, 60, 0, false);
assert.strictEqual(final.exitRate, 0);
assert.ok(final.seconds > 0);

let gentler = segmentDuration(60, 0, 300, false, {
  rate: 120,
  acceleration: 30,
});
assert.ok(gentler.seconds > first.seconds);

console.log("replay motion profile tests: OK");
