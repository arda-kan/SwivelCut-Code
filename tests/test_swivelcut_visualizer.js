const assert = require("assert");
const fs = require("fs");
const vm = require("vm");

const html = fs.readFileSync("swivelcut_visualizer.html", "utf8");
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];

const elements = new Map();
function element(id) {
  if (!elements.has(id)) {
    elements.set(id, {
      value: id === "scale" ? "1" : "0",
      checked: id === "flipY",
      disabled: false,
      className: "",
      textContent: "",
      innerHTML: "",
      style: {},
      addEventListener() {},
      replaceChildren() {},
      appendChild() {},
      querySelector() { return null; },
    });
  }
  return elements.get(id);
}
element("elbow").value = "UP";

const sandbox = {
  console,
  assert,
  document: {
    getElementById: element,
    execCommand() {},
  },
  navigator: { clipboard: { async writeText() {} } },
  setTimeout() {},
};
vm.createContext(sandbox);

const tests = `
assert.strictEqual(LINK_1_MM, 200);
assert.strictEqual(LINK_2_MM, 200);
assert.strictEqual(MAX_TEACH_POINTS, 3000);

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

let far = inverseKinematicsDetailed(401, 0, false);
assert.strictEqual(far.ok, false);
assert.strictEqual(far.type, "annulus");
assert.ok(far.reason.includes("too far"));
let limited = inverseKinematicsDetailed(-300, 0, false);
assert.strictEqual(limited.ok, false);
assert.strictEqual(limited.type, "joint-limit");
assert.ok(limited.reason.includes("J1"));

let mapped = transformPoint({x:10,y:20}, {
  scale:2, offsetX:5, offsetY:7, flipY:true, elbowDown:false
});
assert.deepStrictEqual(mapped, {x:25,y:-33});

let transformed = transformedSvgPoint({
  getCTM() { return {a:2,b:0,c:0,d:3,e:5,f:7}; },
  ownerSVGElement: null
}, {x:10,y:20});
assert.deepStrictEqual(transformed, {x:25,y:67});

let relative = elementMatrixInSvg({
  getCTM() { return {a:4,b:0,c:0,d:6,e:15,f:27}; },
  ownerSVGElement: {
    getCTM() { return {a:2,b:0,c:0,d:3,e:5,f:6}; }
  }
});
assert.deepStrictEqual(relative, {a:2,b:0,c:0,d:2,e:5,f:7});

let flat = flattenSequences([
  {points:[{x:1,y:2},{x:3,y:4}]},
  {points:[{x:5,y:6}]}
]);
assert.strictEqual(flat.length, 3);
assert.strictEqual(flat[2].sequenceIndex, 1);

let validation = validatePoints([
  {x:200,y:200,sequenceIndex:0,pointIndex:0},
  {x:0,y:400,sequenceIndex:0,pointIndex:1}
], false);
assert.strictEqual(validation.allReachable, true);
let output = generateLoadPointsPackage(validation);
assert.ok(output.startsWith("LOAD POINTS 2\\n"));
assert.strictEqual(output.split("\\n").length, 3);

let invalid = validatePoints([
  {x:200,y:200,sequenceIndex:0,pointIndex:0},
  {x:401,y:0,sequenceIndex:0,pointIndex:1}
], false);
assert.strictEqual(invalid.allReachable, false);
assert.strictEqual(invalid.firstFailure.index, 1);
assert.strictEqual(generateLoadPointsPackage(invalid), "");

let split = splitDiscontinuousSubpaths([
  {x:0,y:0},{x:2,y:0},{x:100,y:100},{x:102,y:100}
], 2);
assert.strictEqual(split.length, 2);
assert.strictEqual(split[0].length, 2);
assert.strictEqual(split[1].length, 2);

let records = splitPathDataRecords([
  {type:"M",values:[0,0]}, {type:"L",values:[10,0]},
  {type:"M",values:[20,20]}, {type:"C",values:[21,20,22,20,23,20]}
]);
assert.strictEqual(records.length, 2);
assert.strictEqual(records[1][0].type, "M");
assert.strictEqual(
  serializePathData(records[0]),
  "M 0 0 L 10 0"
);
`;

vm.runInContext(`${script}\n${tests}`, sandbox);
console.log("SVG path planner tests: OK");
