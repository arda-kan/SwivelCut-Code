const assert = require("assert");

function reverse(points) {
  const total = points.at(-1).seconds;
  return points
    .slice()
    .reverse()
    .map(point => ({
      seconds: total - point.seconds,
      j1Deg: point.j1Deg,
      j2Deg: point.j2Deg,
    }));
}

const original = [
  { seconds: 0.00, j1Deg: 1, j2Deg: 10 },
  { seconds: 0.05, j1Deg: 2, j2Deg: 20 },
  { seconds: 0.15, j1Deg: 3, j2Deg: 30 },
  { seconds: 0.30, j1Deg: 4, j2Deg: 40 },
];
const reversed = reverse(original);

assert.deepStrictEqual(reversed, [
  { seconds: 0.00, j1Deg: 4, j2Deg: 40 },
  { seconds: 0.15, j1Deg: 3, j2Deg: 30 },
  { seconds: 0.25, j1Deg: 2, j2Deg: 20 },
  { seconds: 0.30, j1Deg: 1, j2Deg: 10 },
]);
for (let i = 1; i < reversed.length; ++i) {
  assert.ok(reversed[i].seconds >= reversed[i - 1].seconds);
}

console.log("reverse trace timing tests: OK");
