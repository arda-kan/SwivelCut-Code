const assert = require("assert");
const fs = require("fs");
const vm = require("vm");

const html = fs.readFileSync("swivelcut_visualizer.html", "utf8");
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];

const elements = new Map();
function element(id) {
  if (!elements.has(id)) {
    elements.set(id, {
      value: "",
      style: {},
      classList: { add() {}, remove() {} },
      addEventListener() {},
      querySelectorAll() { return []; },
      textContent: "",
      innerHTML: "",
      getBoundingClientRect() { return { width: 800, height: 600 }; },
    });
  }
  return elements.get(id);
}

const context2d = new Proxy(
  {},
  {
    get(target, key) {
      if (!(key in target)) target[key] = () => {};
      return target[key];
    },
    set(target, key, value) {
      target[key] = value;
      return true;
    },
  }
);
element("cv").getContext = () => context2d;

const sandbox = {
  console,
  document: {
    documentElement: {},
    getElementById: element,
  },
  window: {
    devicePixelRatio: 1,
    addEventListener() {},
    getComputedStyle() {
      return { getPropertyValue: () => "#000" };
    },
  },
  ResizeObserver: class {
    observe() {}
  },
  requestAnimationFrame() { return 1; },
  performance: { now: () => 0 },
};
vm.createContext(sandbox);

const tests = `
S.L1=200; S.L2=200;
let straight=forward(0,0);
assert.ok(Math.abs(straight[0]) < 1e-9);
assert.ok(Math.abs(straight[1]-400) < 1e-9);
let elbowStraight=elbowPosition(0);
assert.ok(Math.abs(elbowStraight[0]) < 1e-9);
assert.ok(Math.abs(elbowStraight[1]-200) < 1e-9);
let elbowRight=elbowPosition(Math.PI/2);
assert.ok(Math.abs(elbowRight[0]-200) < 1e-9);
assert.ok(Math.abs(elbowRight[1]) < 1e-9);
let straightIK=inverse(0,400,"up");
assert.ok(Math.abs(straightIK[0]) < 1e-9);
assert.ok(Math.abs(straightIK[1]) < 1e-9);
assert.strictEqual(normalizeAngle(Math.PI), Math.PI);
assert.ok(Math.abs(Math.abs(normalizeAngle(3*Math.PI))-Math.PI) < 1e-9);

S.L1=500; S.L2=100;
assert.strictEqual(lineReachable(600,0,-600,0), false);
assert.strictEqual(lineReachable(600,0,500,0), true);

S.L1=200; S.L2=200; S.g1=6; S.g2=9; S.spr=200; S.micro=32;
S.cur={t1:10*D2R,t2:120*D2R};
$("tx").value="200"; $("ty").value="100";
S.command="xyj1"; S.elbow="up";
let j1Only=commandTarget();
assert.strictEqual(j1Only.ok,true);
assert.ok(Math.abs(j1Only.t2-S.cur.t2)<1e-12);
assert.strictEqual(j1Only.x,200);
assert.strictEqual(j1Only.y,100);
S.command="xyj2";
let j2Only=commandTarget();
assert.strictEqual(j2Only.ok,true);
assert.ok(Math.abs(j2Only.t1-S.cur.t1)<1e-12);

S.cur={t1:0,t2:Math.PI}; S.command="cut";
$("x0").value="0"; $("y0").value="20"; $("x1").value="0"; $("y1").value="400";
let foldedCut=commandTarget();
assert.strictEqual(foldedCut.ok,false);
assert.ok(foldedCut.error.includes("unfold"));
S.cur={t1:-40.945*D2R,t2:171.89*D2R};
let validCut=commandTarget();
assert.strictEqual(validCut.ok,true);
assert.ok(cutSolutions(0,20,0,400,"up").length>2);

S.coupling=0.25;
S.cur={t1:-179.9*D2R,t2:20*D2R};
S.cmd={t1:-179.8*D2R,t2:21*D2R,ok:true,x:0,y:0,error:""};
updateTelemetry();
const expectedJ1=pyRound(S.cmd.t1*stepsPerRad(S.g1))
  -pyRound(S.cur.t1*stepsPerRad(S.g1));
assert.strictEqual($("o_s1").textContent, expectedJ1.toLocaleString());
const expectedJ2=pyRound(
  (S.cmd.t2+S.coupling*S.cmd.t1)*stepsPerRad(S.g2)
)-pyRound(
  (S.cur.t2+S.coupling*S.cur.t1)*stepsPerRad(S.g2)
);
assert.strictEqual($("o_s2").textContent, expectedJ2.toLocaleString());
assert.strictEqual(pyRound(2.5),2);
assert.strictEqual(pyRound(3.5),4);
assert.strictEqual(pyRound(-2.5),-2);
`;

vm.runInContext(`${script}\n${tests}`, vm.createContext({ ...sandbox, assert }));
console.log("visualizer tests: OK");
