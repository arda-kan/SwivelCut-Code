const assert = require("assert");

function normalizeJointDegrees(value) {
  while (value > 180) value -= 360;
  while (value < -180) value += 360;
  return value;
}

assert.strictEqual(normalizeJointDegrees(180), 180);
assert.strictEqual(normalizeJointDegrees(196.06), -163.94);
assert.strictEqual(normalizeJointDegrees(-196.06), 163.94);
assert.strictEqual(normalizeJointDegrees(540), 180);
assert.strictEqual(normalizeJointDegrees(-540), -180);

console.log("joint angle normalization tests: OK");
