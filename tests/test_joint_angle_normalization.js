const assert = require("assert");

function normalizeJointDegrees(value) {
  while (value > 180) value -= 360;
  while (value < -180) value += 360;
  return value;
}

function shortestJointDelta(target, current) {
  return normalizeJointDegrees(target - current);
}

assert.strictEqual(normalizeJointDegrees(180), 180);
assert.strictEqual(normalizeJointDegrees(196.06), -163.94);
assert.strictEqual(normalizeJointDegrees(-196.06), 163.94);
assert.strictEqual(normalizeJointDegrees(540), 180);
assert.strictEqual(normalizeJointDegrees(-540), -180);
assert.ok(Math.abs(shortestJointDelta(-179.8, 179.7) - 0.5) < 1e-9);
assert.ok(Math.abs(shortestJointDelta(179.7, -179.8) + 0.5) < 1e-9);

console.log("joint angle normalization tests: OK");
