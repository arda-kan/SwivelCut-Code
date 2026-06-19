const assert = require("assert");

const CUTTER_MIN = 400;
const CUTTER_MAX = 1125;
const TRACER_MIN = 1500;
const TRACER_MAX = 2550;
const DISCONNECTED_MIN = 3500;

function classify(adc, assumeCutterUnlessTracer) {
  if (adc >= TRACER_MIN && adc <= TRACER_MAX) return "TRACER";
  if (assumeCutterUnlessTracer) return "CUTTER";
  if (adc >= DISCONNECTED_MIN) return "DISCONNECTED";
  if (adc >= CUTTER_MIN && adc <= CUTTER_MAX) return "CUTTER";
  return "UNKNOWN";
}

assert.strictEqual(classify(2000, false), "TRACER");
assert.strictEqual(classify(2000, true), "TRACER");
assert.strictEqual(classify(700, false), "CUTTER");
assert.strictEqual(classify(700, true), "CUTTER");
assert.strictEqual(classify(4095, false), "DISCONNECTED");
assert.strictEqual(classify(4095, true), "CUTTER");
assert.strictEqual(classify(1300, false), "UNKNOWN");
assert.strictEqual(classify(1300, true), "CUTTER");

console.log("head override logic tests: OK");
