const assert = require("assert");
const fs = require("fs");
const vm = require("vm");

const html = fs.readFileSync("swivelcut_firmware_simulator.html", "utf8");
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];

function classList() {
  const values = new Set();
  return {
    add(value) { values.add(value); },
    remove(value) { values.delete(value); },
    toggle(value, force) {
      if (force === undefined ? !values.has(value) : force) values.add(value);
      else values.delete(value);
    },
    contains(value) { return values.has(value); },
  };
}

const elements = new Map();
function element(id) {
  if (!elements.has(id)) {
    elements.set(id, {
      id,
      value: "",
      checked: false,
      disabled: false,
      style: {},
      dataset: {},
      className: "",
      classList: classList(),
      textContent: "",
      innerHTML: "",
      children: [],
      attributes: {},
      addEventListener() {},
      append(...nodes) { this.children.push(...nodes); },
      appendChild(node) { this.children.push(node); return node; },
      setAttribute(name, value) { this.attributes[name] = String(value); },
      getAttribute(name) { return this.attributes[name]; },
      setPointerCapture() {},
      releasePointerCapture() {},
      createSVGPoint() {
        return { x: 0, y: 0, matrixTransform() { return this; } };
      },
      getScreenCTM() { return { inverse() { return {}; } }; },
      scrollTop: 0,
      scrollHeight: 0,
    });
  }
  return elements.get(id);
}

const document = {
  getElementById: element,
  createElement() { return element(`created-${elements.size}`); },
  querySelectorAll() { return []; },
};

let now = 0;
const sandbox = {
  console,
  assert,
  document,
  performance: { now: () => now },
  setTimeout(fn) { now += 1; fn(); return 1; },
  clearTimeout() {},
  setInterval() { return 1; },
  clearInterval() {},
  requestAnimationFrame(fn) { fn(); return 1; },
};
vm.createContext(sandbox);

const tests = `
assert.strictEqual(J1_STEPS_PER_DEG(), 13.333333333333334);
assert.strictEqual(J2_STEPS_PER_DEG(), 20);
assert.strictEqual(state.j2PositionSteps, 3600);
assert.strictEqual(currentJ2Deg(), 180);

let straight = forwardKinematics(0, 0);
assert.ok(Math.abs(straight.x) < 1e-9);
assert.ok(Math.abs(straight.y - 400) < 1e-9);
let folded = forwardKinematics(0, 180);
assert.ok(Math.abs(folded.x) < 1e-9);
assert.ok(Math.abs(folded.y) < 1e-9);

let up = inverseKinematics(200, 200, false);
assert.ok(up);
assert.ok(Math.abs(up.j1Deg) < 1e-9);
assert.ok(Math.abs(up.j2Deg - 90) < 1e-9);
let down = inverseKinematics(200, 200, true);
assert.ok(down);
assert.ok(Math.abs(down.j1Deg + 90) < 1e-9);
assert.ok(Math.abs(down.j2Deg + 90) < 1e-9);
let left = inverseKinematics(-200, 200, true);
assert.ok(left);
assert.ok(Math.abs(left.j1Deg) < 1e-9);
assert.ok(Math.abs(left.j2Deg + 90) < 1e-9);
let rightExtended = inverseKinematics(400, 0, false);
assert.ok(rightExtended);
assert.ok(Math.abs(rightExtended.j1Deg + 90) < 1e-9);
assert.ok(Math.abs(rightExtended.j2Deg) < 1e-9);
let leftExtended = inverseKinematics(-400, 0, false);
assert.ok(leftExtended);
assert.ok(Math.abs(leftExtended.j1Deg - 90) < 1e-9);
assert.ok(Math.abs(leftExtended.j2Deg) < 1e-9);
let autoNegative = inverseKinematicsAuto(-100, 100);
assert.ok(autoNegative);
assert.strictEqual(autoNegative.elbowDown, true);
assert.ok(Math.abs(autoNegative.j1Deg + 24.295188945364572) < 1e-9);
assert.ok(Math.abs(autoNegative.j2Deg + 138.59037789072914) < 1e-9);
let autoPositive = inverseKinematicsAuto(100, 100);
assert.ok(autoPositive);
assert.strictEqual(autoPositive.elbowDown, false);
assert.strictEqual(inverseKinematics(401, 0, false), null);

assert.deepStrictEqual(Array.from(scan("ANGLES 1.5 -2", "ANGLES %f %f")), [1.5, -2]);
assert.deepStrictEqual(Array.from(scan("XY 10 20 DOWN", "XY %f %f %7s")), [10, 20, "DOWN"]);
assert.deepStrictEqual(Array.from(scan("TEST J1 -800", "TEST %2s %ld")), ["J1", -800]);
assert.deepStrictEqual(Array.from(scan("TEACH 2", "TEACH %f")), [2]);

armAtFoldedPose(AxisMode.DUAL);
assert.strictEqual(state.armed, true);
assert.strictEqual(state.encodersCalibrated, true);
let enc = encoderJointAngles();
assert.strictEqual(enc.j1Deg, 0);
assert.strictEqual(enc.j2Deg, 180);

C.ENCODER_J1_SIGN = 1;
state.j1PositionSteps = lroundf(10 * J1_STEPS_PER_DEG());
state.trueJ1Steps = state.j1PositionSteps;
enc = encoderJointAngles();
assert.ok(enc.j1Deg < -9.9);
C.ENCODER_J1_SIGN = -1;

state.rawTaught = [
  {seconds:0,j1Deg:0,j2Deg:180},
  {seconds:.05,j1Deg:3,j2Deg:177},
  {seconds:.1,j1Deg:0,j2Deg:180},
];
prepareTaughtPath(150, 1);
assert.strictEqual(state.taught[0].j1Deg, 0);
assert.strictEqual(state.taught[2].j1Deg, 0);
assert.ok(Math.abs(state.taught[1].j1Deg - 2) < 1e-9);

state.j1PositionSteps = 0;
state.j2PositionSteps = 3600;
state.trueJ1Steps = 0;
state.trueJ2Steps = 3600;
state.encoderOffsetJ1 = 0;
state.encoderOffsetJ2 = 0;
`;

vm.runInContext(`${script}\n${tests}`, sandbox);

(async () => {
  await vm.runInContext("executeSteps(7, -3)", sandbox);
  assert.strictEqual(vm.runInContext("state.j1PositionSteps", sandbox), 7);
  assert.strictEqual(vm.runInContext("state.j2PositionSteps", sandbox), 3597);
  assert.strictEqual(vm.runInContext("state.trueJ1Steps", sandbox), 7);
  assert.strictEqual(vm.runInContext("state.trueJ2Steps", sandbox), 3597);
  console.log("visualizer tests: OK");
})().catch(error => {
  console.error(error);
  process.exitCode = 1;
});
