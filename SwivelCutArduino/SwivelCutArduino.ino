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

constexpr int START_STOP_BUTTON_PIN = 21;
constexpr int STABILIZATION_BUTTON_PIN = 22;
constexpr int REPEAT_BUTTON_PIN = 23;
constexpr int HEAD_ID_PIN = 34;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 35;
constexpr unsigned long HEAD_SAMPLE_INTERVAL_MS = 20;
constexpr int HEAD_STABLE_SAMPLE_COUNT = 5;

constexpr int CUTTING_HEAD_ADC_MIN = 400;
constexpr int CUTTING_HEAD_ADC_MAX = 1125;
constexpr int TRACING_HEAD_ADC_MIN = 1500;
constexpr int TRACING_HEAD_ADC_MAX = 2550;
constexpr int HEAD_DISCONNECTED_ADC_MIN = 3500;

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
constexpr bool STABILIZE_TEACH = false;
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

float currentJ1Deg() { return j1PositionSteps / J1_STEPS_PER_DEG; }
float currentJ2Deg() { return j2PositionSteps / J2_STEPS_PER_DEG; }

const char *headTypeName(HeadType type) {
  switch (type) {
    case HeadType::CUTTING:
      return "CUTTING";
    case HeadType::TRACING:
      return "TRACING";
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
  Serial.print("BUTTON ");
  Serial.print(button.number);
  Serial.print(" ");
  Serial.print(button.name);
  Serial.print(" GPIO");
  Serial.print(button.pin);
  Serial.println(button.stableState == LOW ? " PRESSED" : " RELEASED");
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
      printButtonEvent(button);
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
  }
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
  encoderFault = true;
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
  if (report) Serial.println("OK");
  return true;
}

void forwardKinematics(float j1Deg, float j2Deg, float &x, float &y) {
  const float t1 = radians(j1Deg);
  const float t2 = radians(j2Deg);
  y = LINK_1_MM * cosf(t1) + LINK_2_MM * cosf(t1 + t2);
  x = LINK_1_MM * sinf(t1) + LINK_2_MM * sinf(t1 + t2);
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
  float x = 0.0f;
  float y = 0.0f;
  forwardKinematics(currentJ1Deg(), currentJ2Deg(), x, y);
  Serial.print("POS x=");
  Serial.print(x, 1);
  Serial.print(" y=");
  Serial.print(y, 1);
  Serial.print(" J1=");
  Serial.print(currentJ1Deg(), 2);
  Serial.print(" J2=");
  Serial.println(currentJ2Deg(), 2);
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
  return true;
}

void stabilizeTeachPoints(float smoothingMs, float maxDeviationDeg) {
  if (!STABILIZE_TEACH || taughtCount < 3 || smoothingMs <= 0.0f) return;
  memcpy(teachScratch, taught, taughtCount * sizeof(TeachPoint));
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
    taught[taughtCount++] = {
      (millis() - started) / 1000.0f, j1, j1Only ? 180.0f : j2
    };
  }
  for (int i = 1; i < taughtCount; ++i) {
    const float jumpJ1 = fabsf(taught[i].j1Deg - taught[i - 1].j1Deg);
    const float jumpJ2 = fabsf(taught[i].j2Deg - taught[i - 1].j2Deg);
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
  stabilizeTeachPoints(smoothingMs, maxDeviationDeg);
  Serial.print("TAUGHT: ");
  Serial.print(taughtCount);
  Serial.print(" points, J1 ");
  Serial.print(taught[0].j1Deg, 2);
  Serial.print(" -> ");
  Serial.print(taught[taughtCount - 1].j1Deg, 2);
  Serial.println("; type PLAY");
}

void replayTeach() {
  if (!USE_ENCODERS || taughtCount == 0) {
    Serial.println("ERROR: no taught movement");
    return;
  }
  armMode = taughtJ1Only ? AxisMode::J1_ONLY : AxisMode::DUAL;
  float measuredJ1 = 0.0f;
  float measuredJ2 = 0.0f;
  if (!encoderJointAngles(measuredJ1, measuredJ2)) {
    feedbackFault("AS5600 read failed before replay");
    return;
  }
  j1PositionSteps = lroundf(measuredJ1 * J1_STEPS_PER_DEG);
  j2PositionSteps = lroundf(measuredJ2 * J2_STEPS_PER_DEG);
  enableDrivers();
  armed = true;
  Serial.print("PLAY RETURN: J1 ");
  Serial.print(measuredJ1, 2);
  Serial.print(" -> ");
  Serial.println(taught[0].j1Deg, 2);
  if (!moveToAngles(taught[0].j1Deg, taught[0].j2Deg, false)) return;
  for (int i = 1; i < taughtCount; ++i) {
    if (!moveToAngles(taught[i].j1Deg, taught[i].j2Deg, false)) {
      Serial.print("PLAY STOPPED AT POINT ");
      Serial.print(i);
      Serial.print("/");
      Serial.println(taughtCount - 1);
      return;
    }
  }
  Serial.print("PLAYED: ");
  Serial.print(taughtCount);
  Serial.println(" points");
  printPosition();
}

bool parseElbow(const char *text) {
  return text != nullptr && strcmp(text, "DOWN") == 0;
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  ARM FOLDED | ARM J1 | ARM J2 | DISARM");
  Serial.println("  TEST J1 <steps> | TEST J2 <steps>");
  Serial.println("  J1 <deg> | J2 <deg> | ANGLES <j1> <j2>");
  Serial.println("  XY <x> <y> [UP|DOWN]");
  Serial.println("  CUT <x0> <y0> <x1> <y1> [UP|DOWN]");
  Serial.println("  ENC | TEACH [J1] <seconds> [Hz] [smooth_ms] [max_dev]");
  Serial.println("  STREAM ON | STREAM OFF | STREAM RATE <1-50 Hz>");
  Serial.println("  FEEDBACK ON | FEEDBACK OFF | FEEDBACK STATUS");
  Serial.println("  CONTROLS");
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
  if (mode == AxisMode::J1_ONLY) {
    Serial.println("ARMED J1 TEST: only J1 motion is allowed");
  } else if (mode == AxisMode::J2_ONLY) {
    Serial.println("ARMED J2 TEST: J2 homed at 180; only J2 motion is allowed");
  } else {
    Serial.println("ARMED at J1=0, J2=180");
  }
}

void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) return;
  command.toUpperCase();

  if (command == "ARM FOLDED") return armAtFoldedPose(AxisMode::DUAL);
  if (command == "ARM J1") return armAtFoldedPose(AxisMode::J1_ONLY);
  if (command == "ARM J2") return armAtFoldedPose(AxisMode::J2_ONLY);
  if (command == "DISARM") {
    disableDrivers();
    armMode = AxisMode::DUAL;
    Serial.println("DISARMED");
    return;
  }
  if (command == "POS") return printPosition();
  if (command == "CONTROLS") return printControlStatus();
  if (command == "HELP") return printHelp();
  if (command == "CLEAR") {
    taughtCount = 0;
    Serial.println("TAUGHT MOVEMENT CLEARED");
    return;
  }
  if (command == "PLAY") return replayTeach();
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
    } else if (strcmp(axis, "J2") == 0 &&
               armMode != AxisMode::J1_ONLY) {
      executeSteps(0, rawSteps);
      Serial.println("OK");
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
  Serial.println("Type CONTROLS to print buttons and head ID.");
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
  serviceEncoderStream();
}
