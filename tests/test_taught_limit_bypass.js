const assert = require("assert");
const fs = require("fs");

const firmware = fs.readFileSync(
  "SwivelCutArduino/SwivelCutArduino.ino",
  "utf8",
);

assert.match(
  firmware,
  /constexpr bool ALLOW_TAUGHT_PATH_OUTSIDE_SOFTWARE_LIMITS = false;/,
);
assert.match(
  firmware,
  /if \(!allowOutsideLimits && !angleInRange\(j1Deg, j2Deg\)\)/,
);
assert.match(
  firmware,
  /if \(!ALLOW_TAUGHT_PATH_OUTSIDE_SOFTWARE_LIMITS &&\s+\(!j1NearRange \|\| !j2NearRange\)\)/,
);
assert.match(firmware, /TEACH_LIMIT_BYPASS_ACTIVE/);
assert.match(
  firmware,
  /const bool allowOutsideLimits =\s+ALLOW_TAUGHT_PATH_OUTSIDE_SOFTWARE_LIMITS;/,
);

// Ordinary callers still rely on the default false argument.
assert.match(
  firmware,
  /bool moveToAngles\(\s*float j1Deg, float j2Deg, bool report = true,\s*bool allowOutsideLimits = false\)/,
);

console.log("taught limit bypass tests: OK");
