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
let straightIK=inverse(0,400,"up");
assert.ok(Math.abs(straightIK[0]) < 1e-9);
assert.ok(Math.abs(straightIK[1]) < 1e-9);
$("tr").value="400"; $("tphi").value="0"; S.coord="polar";
let polarForward=targetXY();
assert.ok(Math.abs(polarForward[0]) < 1e-9);
assert.ok(Math.abs(polarForward[1]-400) < 1e-9);
S.coord="xy";

S.L1=500; S.L2=100;
assert.strictEqual(lineReachable(600,0,-600,0), false);
assert.strictEqual(lineReachable(600,0,500,0), true);

S.g1=6; S.spr=200; S.micro=16;
S.cur.t1=-179.9*D2R;
S.cmd={t1:-179.8*D2R,t2:0,ok:true,x:0,y:0};
S.mode="both";
updateTelemetry();
assert.strictEqual($("o_s1").textContent, "6");
`;

vm.runInContext(`${script}\n${tests}`, vm.createContext({ ...sandbox, assert }));
console.log("visualizer tests: OK");
