const assert = require("assert");

const L1 = 260;
const tracerL2 = 255;
const cutterL2 = 260;

function forward(j1Deg, j2Deg, l2) {
  const t1 = j1Deg * Math.PI / 180;
  const t2 = j2Deg * Math.PI / 180;
  return {
    y: L1 * Math.cos(t1) + l2 * Math.cos(t2 - t1),
    x: -L1 * Math.sin(t1) + l2 * Math.sin(t2 - t1),
  };
}

function inverse(x, y, elbowDown, l2) {
  const c2 = (x * x + y * y - L1 * L1 - l2 * l2) / (2 * L1 * l2);
  assert.ok(c2 >= -1 && c2 <= 1);
  let s2 = Math.sqrt(Math.max(0, 1 - c2 * c2));
  if (elbowDown) s2 = -s2;
  const t2 = Math.atan2(s2, c2);
  const t1 = Math.atan2(x, y) - Math.atan2(l2 * s2, L1 + l2 * c2);
  return { j1Deg: -t1 * 180 / Math.PI, j2Deg: t2 * 180 / Math.PI };
}

for (const traced of [
  { j1Deg: -30, j2Deg: 90 },
  { j1Deg: 20, j2Deg: 120 },
  { j1Deg: 10, j2Deg: -90 },
]) {
  const target = forward(traced.j1Deg, traced.j2Deg, tracerL2);
  const cutter = inverse(target.x, target.y, traced.j2Deg < 0, cutterL2);
  const actual = forward(cutter.j1Deg, cutter.j2Deg, cutterL2);
  assert.ok(Math.abs(actual.x - target.x) < 1e-9);
  assert.ok(Math.abs(actual.y - target.y) < 1e-9);
}

console.log("tool length compensation tests: OK");
