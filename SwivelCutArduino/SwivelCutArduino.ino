#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// SwivelCut firmware for an ESP32 and two TB6600 stepper drivers.
// The pulse waveform follows the supplied DFRobot example:
// STEP HIGH for 1500 us, then STEP LOW for 1500 us.

constexpr int J1_PUL_PIN = 25;
constexpr int J1_DIR_PIN = 26;
constexpr int J2_PUL_PIN = 32;
constexpr int J2_DIR_PIN = 33;
constexpr int ENA_PIN = 27;
constexpr int BLADE_IN1_PIN = 13;
constexpr int BLADE_IN2_PIN = 14;
constexpr unsigned long BLADE_DRIVE_MS = 500;
constexpr bool BLADE_REVERSE_TO_RETRACT = true;

constexpr int START_STOP_BUTTON_PIN = 5;
constexpr int STABILIZATION_BUTTON_PIN = 22;
constexpr int REPEAT_BUTTON_PIN = 23;
constexpr int HEAD_ID_PIN = 34;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 35;
constexpr unsigned long HEAD_SAMPLE_INTERVAL_MS = 20;
constexpr unsigned long CONTROL_TEST_REPORT_MS = 500;
constexpr int HEAD_STABLE_SAMPLE_COUNT = 5;

constexpr int CUTTING_HEAD_ADC_MIN = 400;
constexpr int CUTTING_HEAD_ADC_MAX = 1125;
constexpr int TRACING_HEAD_ADC_MIN = 1500;
constexpr int TRACING_HEAD_ADC_MAX = 2550;
constexpr int HEAD_DISCONNECTED_ADC_MIN = 3500;
constexpr float PRODUCT_TEACH_HZ = 20.0f;
constexpr float PRODUCT_TEACH_MAX_SECONDS = 60.0f;
constexpr float PRODUCT_SMOOTHING_MS = 150.0f;
constexpr float PRODUCT_MAX_DEVIATION_DEG = 1.0f;
constexpr bool XY_SMOOTHING_IMPLEMENTED = false;

constexpr int FULL_STEPS_PER_REV = 200;
constexpr int MICROSTEP = 4;  // TB6600 DIP switches must also be set to 1/4.
constexpr float J1_GEAR_RATIO = 6.0f;
constexpr float J2_GEAR_RATIO = 9.0f;
constexpr float LINK_1_MM = 200.0f;
constexpr float LINK_2_MM = 200.0f;

constexpr float J1_MIN_DEG = -90.0f;
constexpr float J1_MAX_DEG = 90.0f;
constexpr float J2_MIN_DEG = -180.0f;
constexpr float J2_MAX_DEG = 180.0f;
constexpr bool INVERT_J1 = false;
constexpr bool INVERT_J2 = true;

// Change these three levels only if the driver's input wiring is changed.
constexpr uint8_t STEP_ACTIVE = HIGH;
constexpr uint8_t STEP_IDLE = LOW;
constexpr uint8_t OUTPUTS_ENABLED = HIGH;
constexpr uint8_t OUTPUTS_DISABLED = LOW;
constexpr unsigned long STEP_HALF_PERIOD_US = 1500;
constexpr unsigned long DIR_SETUP_US = 100;

// Encoder branches set this to true. The main branch needs no AS5600 modules.
constexpr bool USE_ENCODERS = true;
constexpr uint8_t AS5600_ADDRESS = 0x36;
constexpr int J1_SDA_PIN = 16;
constexpr int J1_SCL_PIN = 17;
constexpr int J2_SDA_PIN = 18;
constexpr int J2_SCL_PIN = 19;
constexpr int ENCODER_J1_SIGN = -1;
constexpr int ENCODER_J2_SIGN = 1;
constexpr float FEEDBACK_TOLERANCE_DEG = 0.25f;
constexpr float FEEDBACK_MAX_ERROR_DEG = 10.0f;
constexpr int FEEDBACK_MAX_CORRECTIONS = 3;
constexpr int MAX_TEACH_POINTS = USE_ENCODERS ? 3000 : 1;

constexpr float J1_STEPS_PER_DEG =
    FULL_STEPS_PER_REV * MICROSTEP * J1_GEAR_RATIO / 360.0f;
constexpr float J2_STEPS_PER_DEG =
    FULL_STEPS_PER_REV * MICROSTEP * J2_GEAR_RATIO / 360.0f;

struct TeachPoint {
  float seconds;
  float j1Deg;
  float j2Deg;
};

struct ButtonInput {
  int pin;
  int number;
  const char *name;
  int rawState;
  int stableState;
  unsigned long rawChangedMs;
};

enum class HeadType {
  UNKNOWN,
  CUTTING,
  TRACING,
  DISCONNECTED,
};

enum class ProductState {
  IDLE,
  TEACHING,
  CUTTING,
};

class AS5600Tracker {
 public:
  AS5600Tracker(TwoWire &wire, int sign)
      : wire_(wire), sign_(sign), calibrated_(false), lastRaw_(0),
        positionCounts_(0) {}

  bool begin(int sda, int scl) {
    wire_.begin(sda, scl, 400000);
    return probe();
  }

  bool probe() {
    wire_.beginTransmission(AS5600_ADDRESS);
    return wire_.endTransmission() == 0;
  }

  const char *magnetState(bool &found) {
    uint8_t status = 0;
    if (!readRegister(0x0B, &status, 1)) {
      found = false;
      return "UNREADABLE";
    }
    found = true;
    if (status & 0x08) return "TOO STRONG";
    if (status & 0x10) return "TOO WEAK";
    if (status & 0x20) return "OK";
    return "MISSING";
  }

  bool magnetOk() {
    bool found = false;
    return strcmp(magnetState(found), "OK") == 0;
  }

  bool calibrate() {
    uint16_t raw = 0;
    if (!readRaw(raw) || !magnetOk()) {
      return false;
    }
    lastRaw_ = raw;
    positionCounts_ = 0;
    calibrated_ = true;
    return true;
  }

  bool angleDegrees(float &degrees) {
    if (!calibrated_) {
      return false;
    }
    int deltas[5] = {};
    for (int i = 0; i < 5; ++i) {
      uint16_t raw = 0;
      if (!readRaw(raw)) return false;
      int delta = static_cast<int>(raw) - static_cast<int>(lastRaw_);
      if (delta > 2048) delta -= 4096;
      if (delta < -2048) delta += 4096;
      deltas[i] = delta;
    }
    for (int i = 1; i < 5; ++i) {
      const int value = deltas[i];
      int j = i - 1;
      while (j >= 0 && deltas[j] > value) {
        deltas[j + 1] = deltas[j];
        --j;
      }
      deltas[j + 1] = value;
    }
    const int medianDelta = deltas[2];
    positionCounts_ += medianDelta;
    lastRaw_ = static_cast<uint16_t>(
        (static_cast<int>(lastRaw_) + medianDelta + 4096) % 4096);
    degrees = sign_ * positionCounts_ * (360.0f / 4096.0f);
    return true;
  }

  bool rawValue(int16_t &raw) {
    uint16_t unsignedRaw = 0;
    if (!readRaw(unsignedRaw)) return false;
    raw = unsignedRaw > 2047
              ? static_cast<int16_t>(unsignedRaw) - 4096
              : static_cast<int16_t>(unsignedRaw);
    return true;
  }

 private:
  bool readRaw(uint16_t &raw) {
    uint8_t data[2] = {};
    if (!readRegister(0x0C, data, 2)) {
      return false;
    }
    raw = (static_cast<uint16_t>(data[0]) << 8 | data[1]) & 0x0FFF;
    return true;
  }

  bool readRegister(uint8_t reg, uint8_t *data, size_t length) {
    wire_.beginTransmission(AS5600_ADDRESS);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) {
      return false;
    }
    if (wire_.requestFrom(AS5600_ADDRESS, length) != length) {
      return false;
    }
    for (size_t i = 0; i < length; ++i) {
      data[i] = wire_.read();
    }
    return true;
  }

  TwoWire &wire_;
  int sign_;
  bool calibrated_;
  uint16_t lastRaw_;
  long positionCounts_;
};

TwoWire j1Wire(0);
TwoWire j2Wire(1);
AS5600Tracker j1Encoder(j1Wire, ENCODER_J1_SIGN);
AS5600Tracker j2Encoder(j2Wire, ENCODER_J2_SIGN);
TeachPoint taught[MAX_TEACH_POINTS];
TeachPoint rawTaught[MAX_TEACH_POINTS];
TeachPoint teachScratch[MAX_TEACH_POINTS];

long j1PositionSteps = 0;
long j2PositionSteps = lroundf(180.0f * J2_STEPS_PER_DEG);
int taughtCount = 0;
bool taughtJ1Only = false;
bool armed = false;
enum class AxisMode { DUAL, J1_ONLY, J2_ONLY };
AxisMode armMode = AxisMode::DUAL;
bool encoderFault = false;
bool encodersCalibrated = false;
AxisMode encoderMode = AxisMode::DUAL;
bool encoderFeedbackEnabled = true;
bool encoderStreamEnabled = false;
float encoderStreamHz = 10.0f;
unsigned long nextEncoderStreamMs = 0;
String inputLine;

ButtonInput buttons[] = {
    {START_STOP_BUTTON_PIN, 1, "START_STOP", HIGH, HIGH, 0},
    {STABILIZATION_BUTTON_PIN, 2, "STABILIZATION", HIGH, HIGH, 0},
    {REPEAT_BUTTON_PIN, 3, "REPEAT", HIGH, HIGH, 0},
};
constexpr size_t BUTTON_COUNT = sizeof(buttons) / sizeof(buttons[0]);

HeadType stableHeadType = HeadType::UNKNOWN;
HeadType candidateHeadType = HeadType::UNKNOWN;
int candidateHeadSamples = 0;
int latestHeadAdc = 0;
bool headTypeInitialized = false;
unsigned long nextHeadSampleMs = 0;
bool controlTestEnabled = false;
bool stateTestEnabled = false;
unsigned long nextControlTestReportMs = 0;
bool testTeachingActive = false;
bool testStabilizationEnabled = false;
bool testHasLastCut = false;
HeadType testActiveHead = HeadType::UNKNOWN;
ProductState productState = ProductState::IDLE;
bool productReady = false;
bool stabilizationEnabled = false;
bool bladeExtended = false;
bool productCutActive = false;
bool productAbortRequested = false;
bool productCutRequiresHold = false;
bool productHasLastCut = false;
unsigned long productTeachStartedMs = 0;
unsigned long nextProductTeachSampleMs = 0;

float currentJ1Deg() { return j1PositionSteps / J1_STEPS_PER_DEG; }
float currentJ2Deg() { return j2PositionSteps / J2_STEPS_PER_DEG; }

void disableDrivers();
void serviceProductWorkflow();
void handleProductButtonChange(const ButtonInput &button);
void printOperationReport(const char *label);

const char *headTypeName(HeadType type) {
  switch (type) {
    case HeadType::CUTTING:
      return "CUTTER";
    case HeadType::TRACING:
      return "TRACER";
    case HeadType::DISCONNECTED:
      return "DISCONNECTED";
    default:
      return "UNKNOWN";
  }
}

HeadType classifyHeadAdc(int adc) {
  if (adc >= HEAD_DISCONNECTED_ADC_MIN) return HeadType::DISCONNECTED;
  if (adc >= CUTTING_HEAD_ADC_MIN && adc <= CUTTING_HEAD_ADC_MAX) {
    return HeadType::CUTTING;
  }
  if (adc >= TRACING_HEAD_ADC_MIN && adc <= TRACING_HEAD_ADC_MAX) {
    return HeadType::TRACING;
  }
  return HeadType::UNKNOWN;
}

void printButtonEvent(const ButtonInput &button) {
  Serial.print("PIN ");
  Serial.print(button.number);
  Serial.print(button.stableState == LOW ? " PRESSED" : " RELEASED");
  Serial.print(" - ");
  Serial.print(button.name);
  Serial.print(" (GPIO");
  Serial.print(button.pin);
  Serial.println(")");
  if (button.stableState != LOW) return;

  Serial.print("TEST ONLY - ");
  if (button.number == 1) {
    testTeachingActive = !testTeachingActive;
    Serial.println(
        testTeachingActive
            ? "STARTING TEACHING REQUESTED; NO FUNCTION STARTED"
            : "STOPPING TEACHING REQUESTED; NO FUNCTION STARTED");
  } else if (button.number == 2) {
    testStabilizationEnabled = !testStabilizationEnabled;
    Serial.println(
        testStabilizationEnabled
            ? "STABILIZATION ON REQUESTED; NO FUNCTION STARTED"
            : "STABILIZATION OFF REQUESTED; NO FUNCTION STARTED");
  } else if (button.number == 3) {
    Serial.println("REPEAT REQUESTED; NO FUNCTION STARTED");
  }
}

void printStateTestEvent(const ButtonInput &button) {
  if (button.stableState != LOW) return;

  if (button.number == 1) {
    if (testTeachingActive) {
      Serial.println(
          testActiveHead == HeadType::TRACING
              ? "TRACING_STOPPED"
              : "CUTTING_STOPPED");
      if (testActiveHead == HeadType::CUTTING) {
        testHasLastCut = true;
      }
      testTeachingActive = false;
      testActiveHead = HeadType::UNKNOWN;
      return;
    }

    if (!headTypeInitialized ||
        (stableHeadType != HeadType::CUTTING &&
         stableHeadType != HeadType::TRACING)) {
      Serial.println("NOT_DOING_ANYTHING");
      return;
    }

    testTeachingActive = true;
    testActiveHead = stableHeadType;
    if (testActiveHead == HeadType::TRACING) {
      Serial.println("TRACING_STARTED");
    } else if (testStabilizationEnabled) {
      Serial.println("CUTTING_STARTED_WITH_STABILIZATION");
    } else {
      Serial.println("CUTTING_STARTED");
    }
  } else if (button.number == 2) {
    if (testTeachingActive) {
      Serial.println("STABILIZATION_CHANGE_IGNORED_ACTIVE_EVENT");
      return;
    }
    testStabilizationEnabled = !testStabilizationEnabled;
    Serial.println(
        testStabilizationEnabled
            ? "STABILIZATION_ON"
            : "STABILIZATION_OFF");
  } else if (button.number == 3) {
    if (testTeachingActive || !testHasLastCut) {
      Serial.println("NOT_DOING_ANYTHING");
    } else if (testStabilizationEnabled) {
      Serial.println("REPEATING_LAST_CUT_EVENT_WITH_STABILIZATION");
    } else {
      Serial.println("REPEATING_LAST_CUT_EVENT");
    }
  }
}

void printControlStatus() {
  Serial.print("CONTROLS");
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    Serial.print(" B");
    Serial.print(buttons[i].number);
    Serial.print("_");
    Serial.print(buttons[i].name);
    Serial.print("=");
    Serial.print(buttons[i].stableState == LOW ? "PRESSED" : "RELEASED");
  }
  Serial.print(" POWER_SWITCH=HARDWARE_ONLY");
  latestHeadAdc = analogRead(HEAD_ID_PIN);
  const HeadType measuredHead = classifyHeadAdc(latestHeadAdc);
  Serial.print(" HEAD=");
  Serial.print(headTypeName(
      headTypeInitialized ? stableHeadType : measuredHead));
  Serial.print(" ADC=");
  Serial.println(latestHeadAdc);
}

void serviceControlInputs() {
  const unsigned long now = millis();
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    ButtonInput &button = buttons[i];
    const int raw = digitalRead(button.pin);
    if (raw != button.rawState) {
      button.rawState = raw;
      button.rawChangedMs = now;
    }
    if (raw != button.stableState &&
        now - button.rawChangedMs >= BUTTON_DEBOUNCE_MS) {
      button.stableState = raw;
      if (controlTestEnabled) printButtonEvent(button);
      if (stateTestEnabled) printStateTestEvent(button);
      if (!controlTestEnabled && !stateTestEnabled) {
        handleProductButtonChange(button);
      }
    }
  }

  if (static_cast<long>(now - nextHeadSampleMs) < 0) return;
  nextHeadSampleMs = now + HEAD_SAMPLE_INTERVAL_MS;
  latestHeadAdc = analogRead(HEAD_ID_PIN);
  const HeadType measuredHead = classifyHeadAdc(latestHeadAdc);
  if (measuredHead == candidateHeadType) {
    if (candidateHeadSamples < HEAD_STABLE_SAMPLE_COUNT) {
      ++candidateHeadSamples;
    }
  } else {
    candidateHeadType = measuredHead;
    candidateHeadSamples = 1;
  }

  if (candidateHeadSamples >= HEAD_STABLE_SAMPLE_COUNT &&
      (!headTypeInitialized || candidateHeadType != stableHeadType)) {
    stableHeadType = candidateHeadType;
    headTypeInitialized = true;
    Serial.print("HEAD ");
    Serial.print(headTypeName(stableHeadType));
    Serial.print(" ADC=");
    Serial.println(latestHeadAdc);
    if (controlTestEnabled) {
      // The general head-change line above is also useful in wiring test mode.
    } else if (!stateTestEnabled) {
      if (productState == ProductState::TEACHING &&
          stableHeadType != HeadType::TRACING) {
        taughtCount = 0;
        productState = ProductState::IDLE;
        Serial.println(
            "TEACHING_STOPPED_HEAD_CHANGED PATH_DISCARDED");
        printOperationReport("TRACING_STOPPED_HEAD_CHANGED");
      }
      if (productState == ProductState::CUTTING &&
          stableHeadType != HeadType::CUTTING) {
        productAbortRequested = true;
        Serial.println("CUT_ABORT_REQUESTED_HEAD_CHANGED");
      }
    }
  }

  if (controlTestEnabled &&
      static_cast<long>(now - nextControlTestReportMs) >= 0) {
    nextControlTestReportMs = now + CONTROL_TEST_REPORT_MS;
    printControlStatus();
  }
}

void setControlTest(bool enabled) {
  controlTestEnabled = enabled;
  stateTestEnabled = false;
  if (!enabled) {
    Serial.println("CONTROL TEST OFF");
    return;
  }

  disableDrivers();
  digitalWrite(BLADE_IN1_PIN, LOW);
  digitalWrite(BLADE_IN2_PIN, LOW);
  encoderStreamEnabled = false;
  const unsigned long now = millis();
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    buttons[i].rawState = digitalRead(buttons[i].pin);
    buttons[i].stableState = buttons[i].rawState;
    buttons[i].rawChangedMs = now;
  }
  candidateHeadSamples = 0;
  headTypeInitialized = false;
  testTeachingActive = false;
  testStabilizationEnabled = false;
  nextHeadSampleMs = 0;
  nextControlTestReportMs = now;
  Serial.println(
      "CONTROL TEST ON: motors and blade outputs disabled; "
      "press buttons or change head");
  printControlStatus();
}

void setStateTest(bool enabled) {
  stateTestEnabled = enabled;
  controlTestEnabled = false;
  if (!enabled) {
    Serial.println("STATE_TEST_OFF");
    return;
  }

  disableDrivers();
  digitalWrite(BLADE_IN1_PIN, LOW);
  digitalWrite(BLADE_IN2_PIN, LOW);
  encoderStreamEnabled = false;
  const unsigned long now = millis();
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    buttons[i].rawState = digitalRead(buttons[i].pin);
    buttons[i].stableState = buttons[i].rawState;
    buttons[i].rawChangedMs = now;
  }
  candidateHeadSamples = 0;
  headTypeInitialized = false;
  testTeachingActive = false;
  testStabilizationEnabled = false;
  testHasLastCut = false;
  testActiveHead = HeadType::UNKNOWN;
  nextHeadSampleMs = 0;
  Serial.println("NOT_DOING_ANYTHING");
}

void stopBlade() {
  digitalWrite(BLADE_IN1_PIN, LOW);
  digitalWrite(BLADE_IN2_PIN, LOW);
}

void driveBlade(uint8_t in1, uint8_t in2) {
  digitalWrite(BLADE_IN1_PIN, in1);
  digitalWrite(BLADE_IN2_PIN, in2);
  delay(BLADE_DRIVE_MS);
  stopBlade();
}

void extendBlade() {
  Serial.println("BLADE_EXTENDING");
  driveBlade(HIGH, LOW);
  bladeExtended = true;
}

void retractBlade() {
  Serial.println("BLADE_RETRACTING");
  if (BLADE_REVERSE_TO_RETRACT) {
    driveBlade(LOW, HIGH);
  } else {
    driveBlade(HIGH, LOW);
  }
  bladeExtended = false;
}

void enableDrivers() {
  digitalWrite(ENA_PIN, OUTPUTS_ENABLED);
  delay(2);
}

void disableDrivers() {
  digitalWrite(ENA_PIN, OUTPUTS_DISABLED);
  armed = false;
}

void feedbackFault(const char *message) {
  disableDrivers();
  if (bladeExtended) retractBlade();
  encoderFault = true;
  productReady = false;
  Serial.print("FEEDBACK FAULT: ");
  Serial.println(message);
}

void setDirection(int pin, long delta, bool invert) {
  bool forward = delta >= 0;
  if (invert) {
    forward = !forward;
  }
  digitalWrite(pin, forward ? HIGH : LOW);
}

void pulseSelectedAxes(bool stepJ1, bool stepJ2) {
  if (stepJ1) digitalWrite(J1_PUL_PIN, STEP_ACTIVE);
  if (stepJ2) digitalWrite(J2_PUL_PIN, STEP_ACTIVE);
  delayMicroseconds(STEP_HALF_PERIOD_US);
  if (stepJ1) digitalWrite(J1_PUL_PIN, STEP_IDLE);
  if (stepJ2) digitalWrite(J2_PUL_PIN, STEP_IDLE);
  delayMicroseconds(STEP_HALF_PERIOD_US);
}

bool encoderJointAngles(float &j1Deg, float &j2Deg) {
  if (encoderMode == AxisMode::J2_ONLY) {
    j1Deg = currentJ1Deg();
  } else {
    float motor1Deg = 0.0f;
    if (!j1Encoder.angleDegrees(motor1Deg)) return false;
    j1Deg = motor1Deg / J1_GEAR_RATIO;
  }
  if (encoderMode == AxisMode::J1_ONLY) {
    j2Deg = currentJ2Deg();
    return true;
  }
  float motor2Deg = 0.0f;
  if (!j2Encoder.angleDegrees(motor2Deg)) return false;
  j2Deg = 180.0f + motor2Deg / J2_GEAR_RATIO;
  return true;
}

void serviceEncoderStream() {
  if (!encoderStreamEnabled || !encodersCalibrated) return;
  const unsigned long now = millis();
  if (static_cast<long>(now - nextEncoderStreamMs) < 0) return;
  nextEncoderStreamMs =
      now + static_cast<unsigned long>(1000.0f / encoderStreamHz);

  float j1Deg = 0.0f;
  float j2Deg = 0.0f;
  if (!encoderJointAngles(j1Deg, j2Deg)) {
    encoderStreamEnabled = false;
    Serial.println("ENC_STREAM ERROR: read failed; stream stopped");
    return;
  }
  int16_t j1Raw = 0;
  int16_t j2Raw = 0;
  if ((encoderMode != AxisMode::J2_ONLY && !j1Encoder.rawValue(j1Raw)) ||
      (encoderMode != AxisMode::J1_ONLY && !j2Encoder.rawValue(j2Raw))) {
    encoderStreamEnabled = false;
    Serial.println("ENC_STREAM ERROR: raw read failed; stream stopped");
    return;
  }
  Serial.print("ENC_STREAM");
  if (encoderMode != AxisMode::J2_ONLY) {
    Serial.print(" J1=");
    Serial.print(j1Deg, 2);
    Serial.print(" RAW1=");
    Serial.print(j1Raw);
  }
  if (encoderMode != AxisMode::J1_ONLY) {
    Serial.print(" J2=");
    Serial.print(j2Deg, 2);
    Serial.print(" RAW2=");
    Serial.print(j2Raw);
  }
  Serial.println();
}

bool checkFeedback() {
  if (!USE_ENCODERS || !encoderFeedbackEnabled) return true;
  float measuredJ1 = 0.0f;
  float measuredJ2 = 0.0f;
  if (!encoderJointAngles(measuredJ1, measuredJ2)) {
    feedbackFault("AS5600 read failed");
    return false;
  }
  const float errorJ1 = measuredJ1 - currentJ1Deg();
  const float errorJ2 = measuredJ2 - currentJ2Deg();
  if ((armMode != AxisMode::J2_ONLY &&
       fabsf(errorJ1) > FEEDBACK_MAX_ERROR_DEG) ||
      (armMode != AxisMode::J1_ONLY &&
       fabsf(errorJ2) > FEEDBACK_MAX_ERROR_DEG)) {
    disableDrivers();
    encoderFault = true;
    Serial.print("FEEDBACK FAULT:");
    if (armMode != AxisMode::J2_ONLY) {
      Serial.print(" expected J1=");
      Serial.print(currentJ1Deg(), 2);
      Serial.print(" measured J1=");
      Serial.print(measuredJ1, 2);
      Serial.print(" error=");
      Serial.print(errorJ1, 2);
    }
    if (armMode != AxisMode::J1_ONLY) {
      Serial.print("; expected J2=");
      Serial.print(currentJ2Deg(), 2);
      Serial.print(" measured J2=");
      Serial.print(measuredJ2, 2);
      Serial.print(" error=");
      Serial.print(errorJ2, 2);
    }
    Serial.println();
    return false;
  }
  return true;
}

bool executeSteps(long deltaJ1, long deltaJ2) {
  const long countJ1 = labs(deltaJ1);
  const long countJ2 = labs(deltaJ2);
  const long total = max(countJ1, countJ2);
  if (total == 0) return true;

  setDirection(J1_DIR_PIN, deltaJ1, INVERT_J1);
  setDirection(J2_DIR_PIN, deltaJ2, INVERT_J2);
  delayMicroseconds(DIR_SETUP_US);

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
    serviceControlInputs();
    serviceEncoderStream();
    if (productCutActive) {
      if (productCutRequiresHold &&
          digitalRead(START_STOP_BUTTON_PIN) == HIGH) {
        productAbortRequested = true;
      }
      if (classifyHeadAdc(analogRead(HEAD_ID_PIN)) != HeadType::CUTTING) {
        productAbortRequested = true;
      }
      if (productAbortRequested) {
        Serial.println("CUT_ABORTED");
        return false;
      }
    }
    if (USE_ENCODERS && (i & 255) == 255 && !checkFeedback()) return false;
  }
  return checkFeedback();
}

bool angleInRange(float j1Deg, float j2Deg) {
  return j1Deg >= J1_MIN_DEG && j1Deg <= J1_MAX_DEG &&
         j2Deg >= J2_MIN_DEG && j2Deg <= J2_MAX_DEG;
}

bool moveToAngles(float j1Deg, float j2Deg, bool report = true) {
  if (!armed) {
    Serial.println("ERROR: type ARM FOLDED first");
    return false;
  }
  if (armMode == AxisMode::J1_ONLY &&
      fabsf(j2Deg - currentJ2Deg()) > 0.001f) {
    Serial.println("ERROR: ARM J1 mode blocks J2 motion");
    return false;
  }
  if (armMode == AxisMode::J2_ONLY &&
      fabsf(j1Deg - currentJ1Deg()) > 0.001f) {
    Serial.println("ERROR: ARM J2 mode blocks J1 motion");
    return false;
  }
  if (!angleInRange(j1Deg, j2Deg)) {
    Serial.println("ERROR: angle outside software limits");
    return false;
  }

  for (int correction = 0; correction <= FEEDBACK_MAX_CORRECTIONS; ++correction) {
    long targetJ1 = lroundf(j1Deg * J1_STEPS_PER_DEG);
    long targetJ2 = lroundf(j2Deg * J2_STEPS_PER_DEG);
    if (!executeSteps(targetJ1 - j1PositionSteps,
                      targetJ2 - j2PositionSteps)) return false;
    if (!USE_ENCODERS || !encoderFeedbackEnabled) break;

    float measuredJ1 = 0.0f;
    float measuredJ2 = 0.0f;
    if (!encoderJointAngles(measuredJ1, measuredJ2)) {
      feedbackFault("AS5600 read failed");
      return false;
    }
    const float errorJ1 = j1Deg - measuredJ1;
    const float errorJ2 = j2Deg - measuredJ2;
    const bool j1Settled =
        armMode == AxisMode::J2_ONLY ||
        fabsf(errorJ1) <= FEEDBACK_TOLERANCE_DEG;
    const bool j2Settled =
        armMode == AxisMode::J1_ONLY ||
        fabsf(errorJ2) <= FEEDBACK_TOLERANCE_DEG;
    if (j1Settled && j2Settled) break;
    if (correction == FEEDBACK_MAX_CORRECTIONS) {
      feedbackFault("target did not settle");
      return false;
    }
    if (armMode != AxisMode::J2_ONLY) {
      j1PositionSteps = lroundf(measuredJ1 * J1_STEPS_PER_DEG);
    }
    if (armMode != AxisMode::J1_ONLY) {
      j2PositionSteps = lroundf(measuredJ2 * J2_STEPS_PER_DEG);
    }
  }
  if (report) {
    Serial.println("OK");
    printOperationReport("MOVE_COMPLETE");
  }
  return true;
}

void forwardKinematics(float j1Deg, float j2Deg, float &x, float &y) {
  const float t1 = radians(j1Deg);
  const float t2 = radians(j2Deg);
  y = LINK_1_MM * cosf(t1) + LINK_2_MM * cosf(t1 + t2);
  x = LINK_1_MM * sinf(t1) + LINK_2_MM * sinf(t1 + t2);
}

void printOperationReport(const char *label) {
  const float softwareJ1 = currentJ1Deg();
  const float softwareJ2 = currentJ2Deg();
  float x = 0.0f;
  float y = 0.0f;
  forwardKinematics(softwareJ1, softwareJ2, x, y);
  Serial.print("REPORT ");
  Serial.print(label);
  Serial.print(" SW_J1=");
  Serial.print(softwareJ1, 2);
  Serial.print(" SW_J2=");
  Serial.print(softwareJ2, 2);
  Serial.print(" X=");
  Serial.print(x, 1);
  Serial.print(" Y=");
  Serial.print(y, 1);

  if (encodersCalibrated) {
    float encoderJ1 = 0.0f;
    float encoderJ2 = 0.0f;
    int16_t rawJ1 = 0;
    int16_t rawJ2 = 0;
    const bool anglesOk = encoderJointAngles(encoderJ1, encoderJ2);
    const bool raw1Ok =
        encoderMode == AxisMode::J2_ONLY || j1Encoder.rawValue(rawJ1);
    const bool raw2Ok =
        encoderMode == AxisMode::J1_ONLY || j2Encoder.rawValue(rawJ2);
    if (anglesOk) {
      Serial.print(" ENC_J1=");
      Serial.print(encoderJ1, 2);
      Serial.print(" ENC_J2=");
      Serial.print(encoderJ2, 2);
    } else {
      Serial.print(" ENC_ANGLES=READ_ERROR");
    }
    if (raw1Ok && encoderMode != AxisMode::J2_ONLY) {
      Serial.print(" RAW1=");
      Serial.print(rawJ1);
    }
    if (raw2Ok && encoderMode != AxisMode::J1_ONLY) {
      Serial.print(" RAW2=");
      Serial.print(rawJ2);
    }
    if (!raw1Ok || !raw2Ok) Serial.print(" RAW=READ_ERROR");
  } else {
    Serial.print(" ENC=UNCALIBRATED");
  }
  Serial.println();
}

bool inverseKinematics(float x, float y, bool elbowDown,
                       float &j1Deg, float &j2Deg) {
  float c2 = (x * x + y * y - LINK_1_MM * LINK_1_MM -
              LINK_2_MM * LINK_2_MM) / (2.0f * LINK_1_MM * LINK_2_MM);
  if (c2 < -1.00001f || c2 > 1.00001f) return false;
  c2 = constrain(c2, -1.0f, 1.0f);
  float s2 = sqrtf(max(0.0f, 1.0f - c2 * c2));
  if (elbowDown) s2 = -s2;
  const float t2 = atan2f(s2, c2);
  const float t1 = atan2f(x, y) -
                   atan2f(LINK_2_MM * s2, LINK_1_MM + LINK_2_MM * c2);
  j1Deg = degrees(t1);
  j2Deg = degrees(t2);
  return angleInRange(j1Deg, j2Deg);
}

void printPosition() {
  printOperationReport("POSITION");
}

bool moveToXY(float x, float y, bool elbowDown) {
  float j1 = 0.0f;
  float j2 = 0.0f;
  if (!inverseKinematics(x, y, elbowDown, j1, j2)) {
    Serial.println("ERROR: XY point unreachable or outside joint limits");
    return false;
  }
  return moveToAngles(j1, j2, false);
}

bool cutLine(float x0, float y0, float x1, float y1, bool elbowDown) {
  const float distance = hypotf(x1 - x0, y1 - y0);
  const int segments = max(1, static_cast<int>(ceilf(distance / 2.0f)));
  for (int i = 0; i <= segments; ++i) {
    const float fraction = static_cast<float>(i) / segments;
    float j1 = 0.0f;
    float j2 = 0.0f;
    if (!inverseKinematics(x0 + (x1 - x0) * fraction,
                           y0 + (y1 - y0) * fraction,
                           elbowDown, j1, j2)) {
      Serial.println("ERROR: cut crosses unreachable workspace");
      return false;
    }
  }
  for (int i = 0; i <= segments; ++i) {
    const float fraction = static_cast<float>(i) / segments;
    if (!moveToXY(x0 + (x1 - x0) * fraction,
                  y0 + (y1 - y0) * fraction, elbowDown)) return false;
  }
  Serial.println("OK");
  printOperationReport("CUT_LINE_COMPLETE");
  return true;
}

void stabilizeTeachPoints(float smoothingMs, float maxDeviationDeg) {
  if (taughtCount < 3 || smoothingMs <= 0.0f) return;
  memcpy(teachScratch, rawTaught, taughtCount * sizeof(TeachPoint));
  for (int i = 0; i < taughtCount; ++i) {
    float sum1 = 0.0f;
    float sum2 = 0.0f;
    int count = 0;
    for (int j = 0; j < taughtCount; ++j) {
      if (fabsf((teachScratch[j].seconds - teachScratch[i].seconds) * 1000.0f) <=
          smoothingMs) {
        sum1 += teachScratch[j].j1Deg;
        sum2 += teachScratch[j].j2Deg;
        ++count;
      }
    }
    float smooth1 = sum1 / count;
    float smooth2 = sum2 / count;
    if (maxDeviationDeg > 0.0f) {
      smooth1 = constrain(smooth1, teachScratch[i].j1Deg - maxDeviationDeg,
                          teachScratch[i].j1Deg + maxDeviationDeg);
      smooth2 = constrain(smooth2, teachScratch[i].j2Deg - maxDeviationDeg,
                          teachScratch[i].j2Deg + maxDeviationDeg);
    }
    taught[i].j1Deg = smooth1;
    taught[i].j2Deg = smooth2;
  }
}

void stabilizeTeachPointsXY(float smoothingMs, float maxDeviationDeg) {
  if (taughtCount < 3 || smoothingMs <= 0.0f) return;

  for (int i = 0; i < taughtCount; ++i) {
    teachScratch[i].seconds = rawTaught[i].seconds;
    forwardKinematics(rawTaught[i].j1Deg, rawTaught[i].j2Deg,
                      teachScratch[i].j1Deg, teachScratch[i].j2Deg);
  }

  const float maxDeviationMm =
      radians(maxDeviationDeg) * (LINK_1_MM + LINK_2_MM);
  for (int i = 0; i < taughtCount; ++i) {
    float sumX = 0.0f;
    float sumY = 0.0f;
    int count = 0;
    for (int j = 0; j < taughtCount; ++j) {
      if (fabsf((teachScratch[j].seconds - teachScratch[i].seconds) *
                1000.0f) <= smoothingMs) {
        sumX += teachScratch[j].j1Deg;
        sumY += teachScratch[j].j2Deg;
        ++count;
      }
    }

    float smoothX = sumX / count;
    float smoothY = sumY / count;
    if (maxDeviationDeg > 0.0f) {
      smoothX = constrain(smoothX,
                          teachScratch[i].j1Deg - maxDeviationMm,
                          teachScratch[i].j1Deg + maxDeviationMm);
      smoothY = constrain(smoothY,
                          teachScratch[i].j2Deg - maxDeviationMm,
                          teachScratch[i].j2Deg + maxDeviationMm);
    }

    float smoothJ1 = 0.0f;
    float smoothJ2 = 0.0f;
    const bool elbowDown = rawTaught[i].j2Deg - 180.0f < 0.0f;
    if (inverseKinematics(smoothX, smoothY, elbowDown,
                          smoothJ1, smoothJ2)) {
      taught[i].j1Deg = smoothJ1;
      taught[i].j2Deg = smoothJ2;
    } else {
      taught[i].j1Deg = rawTaught[i].j1Deg;
      taught[i].j2Deg = rawTaught[i].j2Deg;
      Serial.print("XY_SMOOTH_FALLBACK point=");
      Serial.println(i);
    }
  }
}

void prepareTaughtPath(float smoothingMs, float maxDeviationDeg) {
  memcpy(taught, rawTaught, taughtCount * sizeof(TeachPoint));
  if (XY_SMOOTHING_IMPLEMENTED) {
    stabilizeTeachPointsXY(smoothingMs, maxDeviationDeg);
  } else {
    stabilizeTeachPoints(smoothingMs, maxDeviationDeg);
  }
  if (taughtCount > 0) {
    taught[0] = rawTaught[0];
    taught[taughtCount - 1] = rawTaught[taughtCount - 1];
  }
}

void recordTeach(float seconds, float hz, bool j1Only,
                 float smoothingMs, float maxDeviationDeg) {
  if (!USE_ENCODERS) {
    Serial.println("ERROR: this branch has no AS5600 teach support");
    return;
  }
  if ((j1Only && armMode != AxisMode::J1_ONLY) ||
      (!j1Only && armMode != AxisMode::DUAL)) {
    Serial.println(
        armMode == AxisMode::J2_ONLY
            ? "ERROR: TEACH is unavailable in ARM J2 mode"
            : "ERROR: use ARM J1 for TEACH J1 or ARM FOLDED for TEACH");
    return;
  }
  if (!armed || seconds <= 0.0f || seconds > 60.0f ||
      hz < 1.0f || hz > 50.0f) {
    Serial.println("ERROR: TEACH needs 0-60 seconds and 1-50 Hz");
    return;
  }
  const int requested = static_cast<int>(ceilf(seconds * hz));
  if (requested > MAX_TEACH_POINTS) {
    Serial.println("ERROR: too many teach points");
    return;
  }

  disableDrivers();
  taughtCount = 0;
  taughtJ1Only = j1Only;
  productHasLastCut = false;
  const unsigned long started = millis();
  const unsigned long interval = static_cast<unsigned long>(1000.0f / hz);
  for (int i = 0; i < requested; ++i) {
    while (millis() - started < static_cast<unsigned long>(i) * interval) {
      serviceControlInputs();
      serviceEncoderStream();
      delay(1);
    }
    float j1 = 0.0f;
    float j2 = 0.0f;
    if (!encoderJointAngles(j1, j2)) {
      feedbackFault("AS5600 read failed while teaching");
      taughtCount = 0;
      return;
    }
    rawTaught[taughtCount++] = {
      (millis() - started) / 1000.0f, j1, j1Only ? 180.0f : j2
    };
  }
  for (int i = 1; i < taughtCount; ++i) {
    const float jumpJ1 =
        fabsf(rawTaught[i].j1Deg - rawTaught[i - 1].j1Deg);
    const float jumpJ2 =
        fabsf(rawTaught[i].j2Deg - rawTaught[i - 1].j2Deg);
    if (jumpJ1 > 5.0f || (!j1Only && jumpJ2 > 5.0f)) {
      Serial.print("TEACH REJECTED: encoder jump at point ");
      Serial.print(i);
      Serial.print(" J1=");
      Serial.print(jumpJ1, 2);
      if (!j1Only) {
        Serial.print(" J2=");
        Serial.print(jumpJ2, 2);
      }
      Serial.println(" deg");
      taughtCount = 0;
      return;
    }
  }
  prepareTaughtPath(smoothingMs, maxDeviationDeg);
  Serial.print("TAUGHT: ");
  Serial.print(taughtCount);
  Serial.print(" points, J1 ");
  Serial.print(taught[0].j1Deg, 2);
  Serial.print(" -> ");
  Serial.print(taught[taughtCount - 1].j1Deg, 2);
  Serial.println("; type PLAY");
  printOperationReport("TEACH_COMPLETE");
}

bool replayTeach(bool operateBlade = false) {
  if (!USE_ENCODERS || taughtCount == 0) {
    Serial.println("ERROR: no taught movement");
    return false;
  }
  armMode = taughtJ1Only ? AxisMode::J1_ONLY : AxisMode::DUAL;
  float measuredJ1 = 0.0f;
  float measuredJ2 = 0.0f;
  if (!encoderJointAngles(measuredJ1, measuredJ2)) {
    feedbackFault("AS5600 read failed before replay");
    return false;
  }
  j1PositionSteps = lroundf(measuredJ1 * J1_STEPS_PER_DEG);
  j2PositionSteps = lroundf(measuredJ2 * J2_STEPS_PER_DEG);
  enableDrivers();
  armed = true;
  Serial.print("PLAY RETURN: J1 ");
  Serial.print(measuredJ1, 2);
  Serial.print(" -> ");
  Serial.println(taught[0].j1Deg, 2);
  if (!moveToAngles(taught[0].j1Deg, taught[0].j2Deg, false)) {
    disableDrivers();
    return false;
  }
  if (operateBlade) extendBlade();
  for (int i = 1; i < taughtCount; ++i) {
    if (!moveToAngles(taught[i].j1Deg, taught[i].j2Deg, false)) {
      Serial.print("PLAY STOPPED AT POINT ");
      Serial.print(i);
      Serial.print("/");
      Serial.println(taughtCount - 1);
      disableDrivers();
      if (bladeExtended) retractBlade();
      if (!operateBlade) printOperationReport("PLAY_STOPPED");
      return false;
    }
  }
  disableDrivers();
  if (bladeExtended) retractBlade();
  Serial.print("PLAYED: ");
  Serial.print(taughtCount);
  Serial.println(" points");
  if (!operateBlade) printOperationReport("PLAY_COMPLETE");
  return true;
}

bool validateRawTeachPath(bool j1Only) {
  for (int i = 1; i < taughtCount; ++i) {
    const float jumpJ1 =
        fabsf(rawTaught[i].j1Deg - rawTaught[i - 1].j1Deg);
    const float jumpJ2 =
        fabsf(rawTaught[i].j2Deg - rawTaught[i - 1].j2Deg);
    if (jumpJ1 > 5.0f || (!j1Only && jumpJ2 > 5.0f)) {
      Serial.print("TEACH_REJECTED_ENCODER_JUMP POINT=");
      Serial.print(i);
      Serial.print(" J1=");
      Serial.print(jumpJ1, 2);
      Serial.print(" J2=");
      Serial.println(jumpJ2, 2);
      taughtCount = 0;
      return false;
    }
  }
  return true;
}

bool sampleProductTeach(unsigned long now) {
  if (taughtCount >= MAX_TEACH_POINTS) {
    Serial.println("TEACHING_STOPPED_MEMORY_LIMIT");
    return false;
  }
  float j1 = 0.0f;
  float j2 = 0.0f;
  if (!encoderJointAngles(j1, j2)) {
    feedbackFault("AS5600 read failed while teaching");
    taughtCount = 0;
    return false;
  }
  rawTaught[taughtCount++] = {
      (now - productTeachStartedMs) / 1000.0f, j1, j2};
  return true;
}

void startProductTeach(unsigned long now) {
  if (!productReady || !encodersCalibrated ||
      encoderMode != AxisMode::DUAL) {
    Serial.println("ERROR_PRODUCT_NOT_READY_USE_ARM_FOLDED");
    return;
  }
  if (!headTypeInitialized || stableHeadType != HeadType::TRACING) {
    Serial.println("ERROR_TRACER_HEAD_REQUIRED");
    return;
  }

  disableDrivers();
  taughtJ1Only = false;
  taughtCount = 0;
  productHasLastCut = false;
  productTeachStartedMs = now;
  nextProductTeachSampleMs = now;
  productState = ProductState::TEACHING;
  if (!sampleProductTeach(now)) {
    productState = ProductState::IDLE;
    return;
  }
  nextProductTeachSampleMs =
      now + static_cast<unsigned long>(1000.0f / PRODUCT_TEACH_HZ);
  Serial.println("TRACING_STARTED_HOLD_BUTTON");
}

void stopProductTeach(unsigned long now) {
  if (productState != ProductState::TEACHING) return;
  if (taughtCount == 0 ||
      rawTaught[taughtCount - 1].seconds <
          (now - productTeachStartedMs) / 1000.0f) {
    sampleProductTeach(now);
  }
  productState = ProductState::IDLE;
  if (taughtCount < 2) {
    taughtCount = 0;
    Serial.println("ERROR_TRACED_PATH_TOO_SHORT");
    printOperationReport("TRACING_STOPPED_ERROR");
    return;
  }
  if (!validateRawTeachPath(false)) {
    printOperationReport("TRACING_REJECTED");
    return;
  }
  prepareTaughtPath(0.0f, 0.0f);

  float j1 = 0.0f;
  float j2 = 0.0f;
  if (!encoderJointAngles(j1, j2)) {
    feedbackFault("AS5600 read failed after teaching");
    taughtCount = 0;
    return;
  }
  j1PositionSteps = lroundf(j1 * J1_STEPS_PER_DEG);
  j2PositionSteps = lroundf(j2 * J2_STEPS_PER_DEG);
  Serial.print("TRACING_STOPPED POINTS=");
  Serial.println(taughtCount);
  printOperationReport("TRACING_COMPLETE");
}

void runProductCut(bool repeat, bool requireStartHeld) {
  if (!productReady || !encodersCalibrated ||
      encoderMode != AxisMode::DUAL) {
    Serial.println("ERROR_PRODUCT_NOT_READY_USE_ARM_FOLDED");
    return;
  }
  if (!headTypeInitialized || stableHeadType != HeadType::CUTTING) {
    Serial.println("ERROR_CUTTER_HEAD_REQUIRED");
    return;
  }
  if (taughtCount < 2) {
    Serial.println("ERROR_NO_TRACED_PATH");
    return;
  }
  if (repeat && !productHasLastCut) {
    Serial.println("ERROR_NO_COMPLETED_CUT_TO_REPEAT");
    return;
  }

  prepareTaughtPath(
      stabilizationEnabled ? PRODUCT_SMOOTHING_MS : 0.0f,
      stabilizationEnabled ? PRODUCT_MAX_DEVIATION_DEG : 0.0f);
  taughtJ1Only = false;
  productState = ProductState::CUTTING;
  productCutActive = true;
  productAbortRequested = false;
  productCutRequiresHold = requireStartHeld;
  Serial.print(repeat ? "REPEATING_LAST_CUT" : "CUTTING_STARTED");
  Serial.println(
      stabilizationEnabled ? "_WITH_STABILIZATION" : "");
  const bool completed = replayTeach(true);
  productCutActive = false;
  productCutRequiresHold = false;
  productState = ProductState::IDLE;
  disableDrivers();
  if (bladeExtended) retractBlade();
  if (completed) {
    productHasLastCut = true;
    Serial.println(repeat ? "REPEAT_COMPLETE" : "CUT_COMPLETE");
  } else {
    Serial.println(repeat ? "REPEAT_STOPPED" : "CUT_STOPPED");
  }
  printOperationReport(
      completed
          ? (repeat ? "REPEAT_COMPLETE" : "CUT_COMPLETE")
          : (repeat ? "REPEAT_STOPPED" : "CUT_STOPPED"));
}

void handleProductButtonChange(const ButtonInput &button) {
  const bool pressed = button.stableState == LOW;
  const unsigned long now = millis();

  if (button.number == 1) {
    if (pressed) {
      if (productState == ProductState::IDLE) {
        if (headTypeInitialized && stableHeadType == HeadType::TRACING) {
          startProductTeach(now);
        } else if (headTypeInitialized &&
                   stableHeadType == HeadType::CUTTING) {
          runProductCut(false, true);
        } else {
          Serial.println("ERROR_RECOGNIZED_HEAD_REQUIRED");
        }
      }
    } else if (productState == ProductState::TEACHING) {
      stopProductTeach(now);
    } else if (productState == ProductState::CUTTING &&
               productCutRequiresHold) {
      productAbortRequested = true;
      Serial.println("CUT_ABORT_REQUESTED_BUTTON_RELEASED");
    }
    return;
  }

  if (!pressed) return;
  if (button.number == 2) {
    if (productState != ProductState::IDLE) {
      Serial.println("STABILIZATION_CHANGE_IGNORED_ACTIVE_OPERATION");
      return;
    }
    stabilizationEnabled = !stabilizationEnabled;
    Serial.println(
        stabilizationEnabled ? "STABILIZATION_ON" : "STABILIZATION_OFF");
  } else if (button.number == 3) {
    if (productState != ProductState::IDLE) {
      Serial.println("REPEAT_IGNORED_ACTIVE_OPERATION");
      return;
    }
    runProductCut(true, false);
  }
}

void serviceProductWorkflow() {
  if (controlTestEnabled || stateTestEnabled) return;
  if (productState != ProductState::TEACHING) return;

  const unsigned long now = millis();
  if (now - productTeachStartedMs >=
      static_cast<unsigned long>(PRODUCT_TEACH_MAX_SECONDS * 1000.0f)) {
    stopProductTeach(now);
    Serial.println("TRACING_STOPPED_MAXIMUM_DURATION");
    return;
  }
  if (static_cast<long>(now - nextProductTeachSampleMs) >= 0) {
    if (!sampleProductTeach(now)) {
      productState = ProductState::IDLE;
      printOperationReport("TRACING_STOPPED_ERROR");
      return;
    }
    nextProductTeachSampleMs +=
        static_cast<unsigned long>(1000.0f / PRODUCT_TEACH_HZ);
  }
}

bool parseElbow(const char *text) {
  return text != nullptr && strcmp(text, "DOWN") == 0;
}

void printHelp() {
  Serial.println("Physical product controls after ARM FOLDED:");
  Serial.println("  Hold Start/Stop + tracer: record; release: stop");
  Serial.println("  Hold Start/Stop + cutter: cut; release: abort");
  Serial.println("  Stabilization: toggle while idle");
  Serial.println("  Repeat: repeat the last completed cut");
  Serial.println("Commands:");
  Serial.println("  ARM FOLDED | ARM J1 | ARM J2 | DISARM");
  Serial.println("  TEST J1 <steps> | TEST J2 <steps>");
  Serial.println("  J1 <deg> | J2 <deg> | ANGLES <j1> <j2>");
  Serial.println("  XY <x> <y> [UP|DOWN]");
  Serial.println("  CUT <x0> <y0> <x1> <y1> [UP|DOWN]");
  Serial.println("  ENC | TEACH [J1] <seconds> [Hz] [smooth_ms] [max_dev]");
  Serial.println("  STREAM ON | STREAM OFF | STREAM RATE <1-50 Hz>");
  Serial.println("  FEEDBACK ON | FEEDBACK OFF | FEEDBACK STATUS");
  Serial.println("  CONTROLS | CONTROL TEST ON/OFF | STATE TEST ON/OFF");
  Serial.println("  PLAY | CLEAR | POS | HELP");
}

void armAtFoldedPose(AxisMode mode) {
  disableDrivers();
  j1PositionSteps = 0;
  j2PositionSteps = lroundf(180.0f * J2_STEPS_PER_DEG);
  encoderFault = false;
  armMode = mode;
  encodersCalibrated = false;
  encoderMode = mode;
  productReady = false;
  productState = ProductState::IDLE;
  if (USE_ENCODERS) {
    const bool j1Ok =
        mode == AxisMode::J2_ONLY || j1Encoder.calibrate();
    const bool j2Ok =
        mode == AxisMode::J1_ONLY || j2Encoder.calibrate();
    if (!j1Ok || !j2Ok) {
      Serial.println("ERROR: encoder or magnet check failed");
      return;
    }
    encodersCalibrated = true;
  } else if (mode != AxisMode::DUAL) {
    Serial.println("ERROR: single-axis ARM requires an encoder branch");
    return;
  }
  enableDrivers();
  armed = true;
  productReady = mode == AxisMode::DUAL;
  if (mode == AxisMode::J1_ONLY) {
    Serial.println("ARMED J1 TEST: only J1 motion is allowed");
  } else if (mode == AxisMode::J2_ONLY) {
    Serial.println("ARMED J2 TEST: J2 homed at 180; only J2 motion is allowed");
  } else {
    Serial.println(
        "ARMED at J1=0, J2=180; physical product buttons enabled");
  }
  printOperationReport("ARM_COMPLETE");
}

void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) return;
  command.toUpperCase();

  if (command == "CONTROL TEST ON") return setControlTest(true);
  if (command == "CONTROL TEST OFF") return setControlTest(false);
  if (command == "STATE TEST ON") return setStateTest(true);
  if (command == "STATE TEST OFF") return setStateTest(false);
  if (command == "CONTROL TEST STATUS") {
    Serial.println(controlTestEnabled ? "CONTROL TEST ON" : "CONTROL TEST OFF");
    return;
  }
  if (command == "CONTROLS") return printControlStatus();
  if (command == "HELP") return printHelp();
  if (controlTestEnabled || stateTestEnabled) {
    Serial.println(
        "ERROR: input test is active; turn the test OFF first");
    return;
  }

  if (command == "ARM FOLDED") return armAtFoldedPose(AxisMode::DUAL);
  if (command == "ARM J1") return armAtFoldedPose(AxisMode::J1_ONLY);
  if (command == "ARM J2") return armAtFoldedPose(AxisMode::J2_ONLY);
  if (command == "DISARM") {
    disableDrivers();
    armMode = AxisMode::DUAL;
    productReady = false;
    productState = ProductState::IDLE;
    if (bladeExtended) retractBlade();
    Serial.println("DISARMED");
    printOperationReport("DISARMED");
    return;
  }
  if (command == "POS") return printPosition();
  if (command == "CLEAR") {
    taughtCount = 0;
    productHasLastCut = false;
    Serial.println("TAUGHT MOVEMENT CLEARED");
    return;
  }
  if (command == "PLAY") {
    replayTeach();
    return;
  }
  if (command == "FEEDBACK ON") {
    encoderFeedbackEnabled = true;
    Serial.println("FEEDBACK ON: correction and position faults enabled");
    return;
  }
  if (command == "FEEDBACK OFF") {
    encoderFeedbackEnabled = false;
    Serial.println("WARNING: FEEDBACK OFF; motor motion is open-loop");
    return;
  }
  if (command == "FEEDBACK STATUS") {
    Serial.println(
        encoderFeedbackEnabled
            ? "FEEDBACK ON: correction and position faults enabled"
            : "FEEDBACK OFF: motor motion is open-loop");
    return;
  }
  if (command == "STREAM ON") {
    if (!encodersCalibrated) {
      Serial.println("ERROR: type ARM FOLDED, ARM J1, or ARM J2 before streaming");
    } else {
      encoderStreamEnabled = true;
      nextEncoderStreamMs = 0;
      Serial.println("ENC_STREAM ON");
    }
    return;
  }
  if (command == "STREAM OFF") {
    encoderStreamEnabled = false;
    Serial.println("ENC_STREAM OFF");
    return;
  }
  float streamRate = 0.0f;
  if (sscanf(command.c_str(), "STREAM RATE %f", &streamRate) == 1) {
    if (streamRate < 1.0f || streamRate > 50.0f) {
      Serial.println("ERROR: STREAM RATE must be 1-50 Hz");
    } else {
      encoderStreamHz = streamRate;
      nextEncoderStreamMs = 0;
      Serial.print("ENC_STREAM RATE ");
      Serial.print(encoderStreamHz, 1);
      Serial.println(" Hz");
    }
    return;
  }
  if (command == "ENC") {
    if (!USE_ENCODERS) {
      Serial.println("ENC unavailable on this branch");
      return;
    }
    bool j1Found = false;
    bool j2Found = false;
    const char *j1Magnet = j1Encoder.magnetState(j1Found);
    const char *j2Magnet = j2Encoder.magnetState(j2Found);
    Serial.print("ENC J1=[found=");
    Serial.print(j1Found ? "YES" : "NO");
    Serial.print(" magnet=");
    Serial.print(j1Magnet);
    Serial.print("] J2=[found=");
    Serial.print(j2Found ? "YES" : "NO");
    Serial.print(" magnet=");
    Serial.print(j2Magnet);
    Serial.print("]");
    if (encodersCalibrated) {
      float j1 = 0.0f;
      float j2 = 0.0f;
      if (encoderJointAngles(j1, j2)) {
        Serial.print(" J1=");
        Serial.print(j1, 2);
        Serial.print(" J2=");
        Serial.print(j2, 2);
      } else {
        Serial.print(" angles=READ_ERROR");
      }
    } else {
      Serial.print(" angles=UNCALIBRATED");
    }
    Serial.println();
    return;
  }

  float a = 0.0f, b = 0.0f, c = 0.0f, d = 0.0f;
  char option[8] = {};
  if (sscanf(command.c_str(), "ANGLES %f %f", &a, &b) == 2)
    return static_cast<void>(moveToAngles(a, b));
  if (sscanf(command.c_str(), "J1 %f", &a) == 1)
    return static_cast<void>(moveToAngles(a, currentJ2Deg()));
  if (sscanf(command.c_str(), "J2 %f", &a) == 1)
    return static_cast<void>(moveToAngles(currentJ1Deg(), a));
  int xyFields = sscanf(command.c_str(), "XY %f %f %7s", &a, &b, option);
  if (xyFields >= 2) {
    if (!armed || armMode != AxisMode::DUAL) {
      Serial.println("ERROR: XY requires ARM FOLDED");
    } else if (moveToXY(a, b, xyFields == 3 && parseElbow(option))) {
      printPosition();
    }
    return;
  }
  int cutFields = sscanf(command.c_str(), "CUT %f %f %f %f %7s",
                         &a, &b, &c, &d, option);
  if (cutFields >= 4) {
    if (!armed || armMode != AxisMode::DUAL) {
      Serial.println("ERROR: CUT requires ARM FOLDED");
    } else {
      cutLine(a, b, c, d, cutFields == 5 && parseElbow(option));
    }
    return;
  }

  char axis[3] = {};
  long rawSteps = 0;
  if (sscanf(command.c_str(), "TEST %2s %ld", axis, &rawSteps) == 2) {
    if (!armed) {
      Serial.println("ERROR: type ARM FOLDED first");
    } else if (strcmp(axis, "J1") == 0 &&
               armMode != AxisMode::J2_ONLY) {
      executeSteps(rawSteps, 0);
      Serial.println("OK");
      printOperationReport("TEST_J1_COMPLETE");
    } else if (strcmp(axis, "J2") == 0 &&
               armMode != AxisMode::J1_ONLY) {
      executeSteps(0, rawSteps);
      Serial.println("OK");
      printOperationReport("TEST_J2_COMPLETE");
    } else {
      Serial.println("ERROR: invalid or blocked test axis");
    }
    return;
  }

  float seconds = 0.0f, hz = 20.0f, smooth = 150.0f, deviation = 1.0f;
  int fields = sscanf(command.c_str(), "TEACH J1 %f %f %f %f",
                      &seconds, &hz, &smooth, &deviation);
  if (fields >= 1) {
    recordTeach(seconds, hz, true, smooth, deviation);
    return;
  }
  fields = sscanf(command.c_str(), "TEACH %f %f %f %f",
                  &seconds, &hz, &smooth, &deviation);
  if (fields >= 1) {
    recordTeach(seconds, hz, false, smooth, deviation);
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
  pinMode(BLADE_IN1_PIN, OUTPUT);
  pinMode(BLADE_IN2_PIN, OUTPUT);
  pinMode(START_STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STABILIZATION_BUTTON_PIN, INPUT_PULLUP);
  pinMode(REPEAT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(HEAD_ID_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(HEAD_ID_PIN, ADC_11db);
  digitalWrite(J1_PUL_PIN, STEP_IDLE);
  digitalWrite(J2_PUL_PIN, STEP_IDLE);
  digitalWrite(J1_DIR_PIN, LOW);
  digitalWrite(J2_DIR_PIN, LOW);
  digitalWrite(BLADE_IN1_PIN, LOW);
  digitalWrite(BLADE_IN2_PIN, LOW);
  disableDrivers();

  Serial.begin(115200);
  Serial.setTimeout(50);
  const unsigned long controlsStartedMs = millis();
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    buttons[i].rawState = digitalRead(buttons[i].pin);
    buttons[i].stableState = buttons[i].rawState;
    buttons[i].rawChangedMs = controlsStartedMs;
  }
  if (USE_ENCODERS) {
    const bool j1Found = j1Encoder.begin(J1_SDA_PIN, J1_SCL_PIN);
    const bool j2Found = j2Encoder.begin(J2_SDA_PIN, J2_SCL_PIN);
    if (!j1Found || !j2Found) {
      Serial.println("WARNING: one or both AS5600 encoders were not found");
    }
  }
  Serial.println();
  Serial.println("SwivelCut Arduino controller ready");
  Serial.println("Fold the arm, then type ARM FOLDED");
  Serial.println("Product buttons become active after ARM FOLDED.");
  Serial.println("Type CONTROL TEST ON to test buttons and head ID.");
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
    } else if (inputLine.length() < 120) {
      inputLine += incoming;
    }
  }
  serviceControlInputs();
  serviceProductWorkflow();
  serviceEncoderStream();
}
