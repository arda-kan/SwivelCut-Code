#include <Arduino.h>
#include <math.h>

// SwivelCut ESP32 + two TB6600 drivers.
// Common-anode wiring:
//   PUL+, DIR+, ENA+ -> 3.3 V
//   PUL-, DIR-, ENA- -> the GPIO pins below

constexpr int J1_PUL_PIN = 25;
constexpr int J1_DIR_PIN = 26;
constexpr int J2_PUL_PIN = 32;
constexpr int J2_DIR_PIN = 33;
constexpr int ENA_PIN = 27;

constexpr int FULL_STEPS_PER_REV = 200;
constexpr int MICROSTEP = 4;  // TB6600 DIP switches must be set to 1/4.
constexpr float J1_GEAR_RATIO = 6.0f;
constexpr float J2_GEAR_RATIO = 9.0f;

constexpr float J1_MIN_DEG = -90.0f;
constexpr float J1_MAX_DEG = 90.0f;
constexpr float J2_MIN_DEG = -180.0f;
constexpr float J2_MAX_DEG = 180.0f;

constexpr bool INVERT_J1 = false;
constexpr bool INVERT_J2 = true;

// The example waveform: 1500 us LOW + 1500 us HIGH.
// One complete pulse period is 3000 us, or about 333 pulses/second.
constexpr unsigned long STEP_HALF_PERIOD_US = 1500;
constexpr unsigned long DIR_SETUP_US = 100;

// With this TB6600 wiring, ENA- HIGH enables and ENA- LOW disables.
constexpr int OUTPUTS_ENABLED = HIGH;
constexpr int OUTPUTS_DISABLED = LOW;
constexpr int STEP_ACTIVE = LOW;
constexpr int STEP_IDLE = HIGH;

constexpr float J1_STEPS_PER_DEG =
    FULL_STEPS_PER_REV * MICROSTEP * J1_GEAR_RATIO / 360.0f;
constexpr float J2_STEPS_PER_DEG =
    FULL_STEPS_PER_REV * MICROSTEP * J2_GEAR_RATIO / 360.0f;

long j1PositionSteps = 0;
long j2PositionSteps = lroundf(180.0f * J2_STEPS_PER_DEG);
bool armed = false;
String inputLine;

void setDirection(int pin, long delta, bool invert) {
  bool forward = delta >= 0;
  if (invert) {
    forward = !forward;
  }

  // DIR- HIGH means the optocoupler is inactive; DIR- LOW means active.
  digitalWrite(pin, forward ? HIGH : LOW);
}

void enableDrivers() {
  digitalWrite(ENA_PIN, OUTPUTS_ENABLED);
  delay(2);
}

void disableDrivers() {
  digitalWrite(ENA_PIN, OUTPUTS_DISABLED);
  armed = false;
}

void pulseSelectedAxes(bool stepJ1, bool stepJ2) {
  if (stepJ1) {
    digitalWrite(J1_PUL_PIN, STEP_ACTIVE);
  }
  if (stepJ2) {
    digitalWrite(J2_PUL_PIN, STEP_ACTIVE);
  }

  delayMicroseconds(STEP_HALF_PERIOD_US);

  if (stepJ1) {
    digitalWrite(J1_PUL_PIN, STEP_IDLE);
  }
  if (stepJ2) {
    digitalWrite(J2_PUL_PIN, STEP_IDLE);
  }

  delayMicroseconds(STEP_HALF_PERIOD_US);
}

void executeSteps(long deltaJ1, long deltaJ2) {
  const long countJ1 = labs(deltaJ1);
  const long countJ2 = labs(deltaJ2);
  const long total = max(countJ1, countJ2);
  if (total == 0) {
    return;
  }

  setDirection(J1_DIR_PIN, deltaJ1, INVERT_J1);
  setDirection(J2_DIR_PIN, deltaJ2, INVERT_J2);
  delayMicroseconds(DIR_SETUP_US);

  // Error accumulators distribute each axis's steps across the same duration.
  long errorJ1 = 0;
  long errorJ2 = 0;
  for (long i = 0; i < total; ++i) {
    errorJ1 += countJ1;
    errorJ2 += countJ2;

    bool stepJ1 = false;
    bool stepJ2 = false;
    if (errorJ1 >= total) {
      errorJ1 -= total;
      stepJ1 = true;
      j1PositionSteps += deltaJ1 >= 0 ? 1 : -1;
    }
    if (errorJ2 >= total) {
      errorJ2 -= total;
      stepJ2 = true;
      j2PositionSteps += deltaJ2 >= 0 ? 1 : -1;
    }

    pulseSelectedAxes(stepJ1, stepJ2);
  }
}

bool angleInRange(float j1Deg, float j2Deg) {
  return j1Deg >= J1_MIN_DEG && j1Deg <= J1_MAX_DEG &&
         j2Deg >= J2_MIN_DEG && j2Deg <= J2_MAX_DEG;
}

void moveToAngles(float j1Deg, float j2Deg) {
  if (!armed) {
    Serial.println("ERROR: type ARM FOLDED first");
    return;
  }
  if (!angleInRange(j1Deg, j2Deg)) {
    Serial.println("ERROR: angle outside software limits");
    return;
  }

  const long targetJ1 = lroundf(j1Deg * J1_STEPS_PER_DEG);
  const long targetJ2 = lroundf(j2Deg * J2_STEPS_PER_DEG);
  executeSteps(targetJ1 - j1PositionSteps, targetJ2 - j2PositionSteps);
  Serial.println("OK");
}

void printPosition() {
  const float j1Deg = j1PositionSteps / J1_STEPS_PER_DEG;
  const float j2Deg = j2PositionSteps / J2_STEPS_PER_DEG;
  Serial.print("POS J1=");
  Serial.print(j1Deg, 2);
  Serial.print(" J2=");
  Serial.println(j2Deg, 2);
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  ARM FOLDED       set current pose to J1=0, J2=180 and enable");
  Serial.println("  DISARM           disable motor outputs");
  Serial.println("  J1 <degrees>     move J1, keep J2 fixed");
  Serial.println("  J2 <degrees>     move J2, keep J1 fixed");
  Serial.println("  ANGLES <j1> <j2> move both joints together");
  Serial.println("  POS              show tracked joint angles");
  Serial.println("  TEST J1 <steps>  raw signed J1 pulse test");
  Serial.println("  TEST J2 <steps>  raw signed J2 pulse test");
}

void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) {
    return;
  }
  command.toUpperCase();

  if (command == "ARM FOLDED") {
    j1PositionSteps = 0;
    j2PositionSteps = lroundf(180.0f * J2_STEPS_PER_DEG);
    enableDrivers();
    armed = true;
    Serial.println("ARMED at J1=0, J2=180");
    return;
  }
  if (command == "DISARM") {
    disableDrivers();
    Serial.println("DISARMED");
    return;
  }
  if (command == "POS") {
    printPosition();
    return;
  }
  if (command == "HELP") {
    printHelp();
    return;
  }

  float first = 0.0f;
  float second = 0.0f;
  if (sscanf(command.c_str(), "ANGLES %f %f", &first, &second) == 2) {
    moveToAngles(first, second);
    return;
  }
  if (sscanf(command.c_str(), "J1 %f", &first) == 1) {
    moveToAngles(first, j2PositionSteps / J2_STEPS_PER_DEG);
    return;
  }
  if (sscanf(command.c_str(), "J2 %f", &first) == 1) {
    moveToAngles(j1PositionSteps / J1_STEPS_PER_DEG, first);
    return;
  }

  char axis[3] = {};
  long rawSteps = 0;
  if (sscanf(command.c_str(), "TEST %2s %ld", axis, &rawSteps) == 2) {
    if (!armed) {
      Serial.println("ERROR: type ARM FOLDED first");
    } else if (strcmp(axis, "J1") == 0) {
      executeSteps(rawSteps, 0);
      Serial.println("OK");
    } else if (strcmp(axis, "J2") == 0) {
      executeSteps(0, rawSteps);
      Serial.println("OK");
    } else {
      Serial.println("ERROR: use TEST J1 or TEST J2");
    }
    return;
  }

  Serial.println("ERROR: unknown command; type HELP");
}

void setup() {
  pinMode(J1_PUL_PIN, OUTPUT);
  pinMode(J1_DIR_PIN, OUTPUT);
  pinMode(J2_PUL_PIN, OUTPUT);
  pinMode(J2_DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);

  digitalWrite(J1_PUL_PIN, STEP_IDLE);
  digitalWrite(J2_PUL_PIN, STEP_IDLE);
  digitalWrite(J1_DIR_PIN, LOW);
  digitalWrite(J2_DIR_PIN, LOW);
  disableDrivers();

  Serial.begin(115200);
  Serial.setTimeout(50);
  Serial.println();
  Serial.println("SwivelCut Arduino controller ready");
  Serial.println("Physically fold the arm, then type ARM FOLDED");
  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\n' || incoming == '\r') {
      if (inputLine.length() > 0) {
        handleCommand(inputLine);
        inputLine = "";
      }
    } else {
      inputLine += incoming;
    }
  }
}
