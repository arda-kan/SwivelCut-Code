#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// SwivelCut firmware for an ESP32 and two TB6600 stepper drivers.
// Common-anode STEP waveform: active LOW for 1500 us, then idle HIGH for
// 1500 us.

constexpr int J1_PUL_PIN = 25;
constexpr int J1_DIR_PIN = 26;
constexpr int J2_PUL_PIN = 32;
constexpr int J2_DIR_PIN = 33;
constexpr int ENA_PIN = 27;

// Product controls. The latching on/off switch disconnects hardware power and
// is intentionally not connected to an ESP32 logic input.
constexpr int START_STOP_PIN = 18;
constexpr int STABILIZATION_PIN = 19;
constexpr int REPEAT_PIN = 23;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 35;

// Head ID: 3V3 -> 10k pull-up -> GPIO34 -> attachment resistor -> GND.
// Tracing head = 10k, cutting head = 2.2k.
constexpr int HEAD_ID_PIN = 34;
constexpr int TRACING_ADC_MIN = 1500;
constexpr int TRACING_ADC_MAX = 2550;
constexpr int CUTTING_ADC_MIN = 400;
constexpr int CUTTING_ADC_MAX = 1125;
constexpr int DISCONNECTED_ADC_MIN = 3500;
constexpr int HEAD_STABLE_READS = 4;

// Blade motor H-bridge inputs.
constexpr int BLADE_A_PIN = 13;
constexpr int BLADE_B_PIN = 14;
constexpr unsigned long BLADE_DRIVE_MS = 500;
constexpr bool BLADE_REVERSE_TO_RETRACT = true;

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

// Verified common-anode wiring: PUL+, DIR+, and ENA+ connect to 3.3 V, while
// the ESP32 drives PUL-, DIR-, and ENA-. STEP activates LOW and idles HIGH.
// On the tested TB6600, ENA- LOW disables the outputs, so HIGH enables them.
constexpr uint8_t STEP_ACTIVE = LOW;
constexpr uint8_t STEP_IDLE = HIGH;
constexpr uint8_t OUTPUTS_ENABLED = HIGH;
constexpr uint8_t OUTPUTS_DISABLED = LOW;
constexpr unsigned long STEP_HALF_PERIOD_US = 1500;
constexpr unsigned long DIR_SETUP_US = 100;

// Encoder branches set this to true. The main branch needs no AS5600 modules.
constexpr bool USE_ENCODERS = true;
constexpr bool STABILIZE_TEACH = true;
constexpr uint8_t AS5600_ADDRESS = 0x36;
constexpr int J1_SDA_PIN = 16;
constexpr int J1_SCL_PIN = 17;
constexpr int J2_SDA_PIN = 21;
constexpr int J2_SCL_PIN = 22;
constexpr int ENCODER_J1_SIGN = -1;
constexpr int ENCODER_J2_SIGN = 1;
constexpr float FEEDBACK_TOLERANCE_DEG = 0.25f;
constexpr float FEEDBACK_MAX_ERROR_DEG = 10.0f;
constexpr int FEEDBACK_MAX_CORRECTIONS = 3;
constexpr int MAX_TEACH_POINTS = USE_ENCODERS ? 3000 : 1;
constexpr float PRODUCT_TEACH_HZ = 20.0f;
constexpr float PRODUCT_TEACH_MAX_SECONDS = 60.0f;
constexpr float PRODUCT_SMOOTHING_MS = 150.0f;
constexpr float PRODUCT_MAX_DEVIATION_DEG = 1.0f;

constexpr float J1_STEPS_PER_DEG =
    FULL_STEPS_PER_REV * MICROSTEP * J1_GEAR_RATIO / 360.0f;
constexpr float J2_STEPS_PER_DEG =
    FULL_STEPS_PER_REV * MICROSTEP * J2_GEAR_RATIO / 360.0f;

struct TeachPoint {
  float seconds;
  float j1Deg;
  float j2Deg;
};

enum HeadType {
  HEAD_UNKNOWN,
  HEAD_DISCONNECTED,
  HEAD_TRACING,
  HEAD_CUTTING,
};

enum ProductState {
  PRODUCT_IDLE,
  PRODUCT_TEACHING,
  PRODUCT_CUTTING,
};

class DebouncedButton {
 public:
  explicit DebouncedButton(int pin)
      : pin_(pin), rawPressed_(false), pressed_(false), changedAt_(0) {}

  void begin() {
    pinMode(pin_, INPUT_PULLUP);
    rawPressed_ = digitalRead(pin_) == LOW;
    pressed_ = rawPressed_;
    changedAt_ = millis();
  }

  bool update(unsigned long now) {
    const bool rawPressed = digitalRead(pin_) == LOW;
    if (rawPressed != rawPressed_) {
      rawPressed_ = rawPressed;
      changedAt_ = now;
    }
    if (rawPressed != pressed_ && now - changedAt_ >= BUTTON_DEBOUNCE_MS) {
      pressed_ = rawPressed;
      return pressed_;
    }
    return false;
  }

 private:
  int pin_;
  bool rawPressed_;
  bool pressed_;
  unsigned long changedAt_;
};

class AS5600Tracker {
 public:
  AS5600Tracker(TwoWire &wire, int sign)
      : wire_(wire), sign_(sign), calibrated_(false), lastRaw_(0),
        positionCounts_(0) {}

  bool begin(int sda, int scl) {
    wire_.begin(sda, scl, 400000);
    wire_.beginTransmission(AS5600_ADDRESS);
    return wire_.endTransmission() == 0;
  }

  bool magnetOk() {
    uint8_t status = 0;
    return readRegister(0x0B, &status, 1) && (status & 0x20) &&
           !(status & 0x18);
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
DebouncedButton startStopButton(START_STOP_PIN);
DebouncedButton stabilizationButton(STABILIZATION_PIN);
DebouncedButton repeatButton(REPEAT_PIN);

long j1PositionSteps = 0;
long j2PositionSteps = lroundf(180.0f * J2_STEPS_PER_DEG);
int taughtCount = 0;
bool taughtJ1Only = false;
bool armed = false;
bool j1OnlyMode = false;
bool encoderFault = false;
bool encodersCalibrated = false;
bool encodersJ1Only = false;
bool encoderFeedbackEnabled = true;
bool encoderStreamEnabled = false;
float encoderStreamHz = 10.0f;
unsigned long nextEncoderStreamMs = 0;
ProductState productState = PRODUCT_IDLE;
HeadType detectedHead = HEAD_UNKNOWN;
HeadType candidateHead = HEAD_UNKNOWN;
int candidateHeadReads = 0;
int lastHeadAdc = 0;
bool productReady = false;
bool stabilizationEnabled = false;
bool bladeExtended = false;
bool productCutActive = false;
bool cutAbortArmed = false;
int cuttingHeadBadReads = 0;
unsigned long productTeachStartedMs = 0;
unsigned long nextProductTeachSampleMs = 0;
String inputLine;

float currentJ1Deg() { return j1PositionSteps / J1_STEPS_PER_DEG; }
float currentJ2Deg() { return j2PositionSteps / J2_STEPS_PER_DEG; }

const char *headName(HeadType head) {
  switch (head) {
    case HEAD_TRACING:
      return "tracing";
    case HEAD_CUTTING:
      return "cutting";
    case HEAD_DISCONNECTED:
      return "disconnected";
    default:
      return "unknown";
  }
}

HeadType classifyHeadAdc(int value) {
  if (value >= DISCONNECTED_ADC_MIN) return HEAD_DISCONNECTED;
  if (value >= CUTTING_ADC_MIN && value <= CUTTING_ADC_MAX) {
    return HEAD_CUTTING;
  }
  if (value >= TRACING_ADC_MIN && value <= TRACING_ADC_MAX) {
    return HEAD_TRACING;
  }
  return HEAD_UNKNOWN;
}

HeadType sampleHead(bool requireStable = true) {
  lastHeadAdc = analogRead(HEAD_ID_PIN);
  const HeadType candidate = classifyHeadAdc(lastHeadAdc);
  if (!requireStable) return candidate;
  if (candidate == candidateHead) {
    if (candidateHeadReads < HEAD_STABLE_READS) ++candidateHeadReads;
  } else {
    candidateHead = candidate;
    candidateHeadReads = 1;
  }
  if (candidateHeadReads >= HEAD_STABLE_READS && detectedHead != candidate) {
    detectedHead = candidate;
    Serial.print("HEAD: ");
    Serial.println(headName(detectedHead));
  }
  return detectedHead;
}

void stopBlade() {
  digitalWrite(BLADE_A_PIN, LOW);
  digitalWrite(BLADE_B_PIN, LOW);
}

void driveBlade(uint8_t a, uint8_t b) {
  digitalWrite(BLADE_A_PIN, a);
  digitalWrite(BLADE_B_PIN, b);
  delay(BLADE_DRIVE_MS);
  stopBlade();
}

void extendBlade() {
  driveBlade(HIGH, LOW);
  bladeExtended = true;
}

void retractBlade() {
  if (BLADE_REVERSE_TO_RETRACT) {
    driveBlade(LOW, HIGH);
  } else {
    driveBlade(HIGH, LOW);
  }
  bladeExtended = false;
}

void homeBladeAtBoot() {
  if (BLADE_REVERSE_TO_RETRACT) {
    retractBlade();
  } else {
    stopBlade();
    bladeExtended = false;
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
  if (bladeExtended) retractBlade();
  encoderFault = true;
  encodersCalibrated = false;
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

bool productCutAbortRequested() {
  if (!productCutActive) return false;

  if (digitalRead(START_STOP_PIN) == HIGH) {
    cutAbortArmed = true;
  } else if (cutAbortArmed) {
    Serial.println("CUT ABORT: Start/Stop pressed");
    return true;
  }

  if (sampleHead(false) == HEAD_CUTTING) {
    cuttingHeadBadReads = 0;
  } else if (++cuttingHeadBadReads >= 3) {
    Serial.println("CUT ABORT: cutting head removed or unrecognized");
    return true;
  }
  return false;
}

bool encoderJointAngles(float &j1Deg, float &j2Deg) {
  float motor1Deg = 0.0f;
  if (!j1Encoder.angleDegrees(motor1Deg)) return false;
  j1Deg = motor1Deg / J1_GEAR_RATIO;
  if (encodersJ1Only) {
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
  Serial.print("ENC_STREAM J1=");
  Serial.print(j1Deg, 2);
  if (!j1OnlyMode) {
    Serial.print(" J2=");
    Serial.print(j2Deg, 2);
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
  if (fabsf(errorJ1) > FEEDBACK_MAX_ERROR_DEG ||
      (!j1OnlyMode && fabsf(errorJ2) > FEEDBACK_MAX_ERROR_DEG)) {
    disableDrivers();
    encoderFault = true;
    Serial.print("FEEDBACK FAULT: expected J1=");
    Serial.print(currentJ1Deg(), 2);
    Serial.print(" measured J1=");
    Serial.print(measuredJ1, 2);
    Serial.print(" error=");
    Serial.print(errorJ1, 2);
    if (!j1OnlyMode) {
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
    if (productCutAbortRequested()) {
      disableDrivers();
      return false;
    }
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
  if (j1OnlyMode && fabsf(j2Deg - currentJ2Deg()) > 0.001f) {
    Serial.println("ERROR: ARM J1 mode blocks J2 motion");
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
    if (fabsf(errorJ1) <= FEEDBACK_TOLERANCE_DEG &&
        (j1OnlyMode || fabsf(errorJ2) <= FEEDBACK_TOLERANCE_DEG)) break;
    if (correction == FEEDBACK_MAX_CORRECTIONS) {
      feedbackFault("target did not settle");
      return false;
    }
    j1PositionSteps = lroundf(measuredJ1 * J1_STEPS_PER_DEG);
    if (!j1OnlyMode) {
      j2PositionSteps = lroundf(measuredJ2 * J2_STEPS_PER_DEG);
    }
  }
  if (report) {
    Serial.println(
        encoderFeedbackEnabled
            ? "OK: encoder verified target"
            : "PULSES SENT: feedback off; physical movement not verified");
  }
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
  for (int i = 0; i < taughtCount; ++i) {
    float sum1 = 0.0f;
    float sum2 = 0.0f;
    int count = 0;
    for (int j = 0; j < taughtCount; ++j) {
      if (fabsf((rawTaught[j].seconds - rawTaught[i].seconds) * 1000.0f) <=
          smoothingMs) {
        sum1 += rawTaught[j].j1Deg;
        sum2 += rawTaught[j].j2Deg;
        ++count;
      }
    }
    float smooth1 = sum1 / count;
    float smooth2 = sum2 / count;
    if (maxDeviationDeg > 0.0f) {
      smooth1 = constrain(smooth1, rawTaught[i].j1Deg - maxDeviationDeg,
                          rawTaught[i].j1Deg + maxDeviationDeg);
      smooth2 = constrain(smooth2, rawTaught[i].j2Deg - maxDeviationDeg,
                          rawTaught[i].j2Deg + maxDeviationDeg);
    }
    taught[i].j1Deg = smooth1;
    taught[i].j2Deg = smooth2;
  }
  taught[0] = rawTaught[0];
  taught[taughtCount - 1] = rawTaught[taughtCount - 1];
}

void prepareTaughtPath(float smoothingMs, float maxDeviationDeg) {
  memcpy(taught, rawTaught, taughtCount * sizeof(TeachPoint));
  stabilizeTeachPoints(smoothingMs, maxDeviationDeg);
}

void recordTeach(float seconds, float hz, bool j1Only,
                 float smoothingMs, float maxDeviationDeg) {
  if (!USE_ENCODERS) {
    Serial.println("ERROR: this branch has no AS5600 teach support");
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
}

bool replayTeach(bool operateBlade = false) {
  if (!USE_ENCODERS || taughtCount == 0) {
    Serial.println("ERROR: no taught movement");
    return false;
  }
  j1OnlyMode = taughtJ1Only;
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
      return false;
    }
  }
  disableDrivers();
  if (bladeExtended) retractBlade();
  Serial.print("PLAYED: ");
  Serial.print(taughtCount);
  Serial.println(" points");
  printPosition();
  return true;
}

bool validateRawTeachPath(bool j1Only) {
  for (int i = 1; i < taughtCount; ++i) {
    const float jumpJ1 =
        fabsf(rawTaught[i].j1Deg - rawTaught[i - 1].j1Deg);
    const float jumpJ2 =
        fabsf(rawTaught[i].j2Deg - rawTaught[i - 1].j2Deg);
    if (jumpJ1 > 5.0f || (!j1Only && jumpJ2 > 5.0f)) {
      Serial.print("TEACH REJECTED: encoder jump at point ");
      Serial.println(i);
      taughtCount = 0;
      return false;
    }
  }
  return true;
}

bool sampleProductTeach(unsigned long now) {
  if (taughtCount >= MAX_TEACH_POINTS) {
    Serial.println("TEACHING STOPPED: memory limit reached");
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
    (now - productTeachStartedMs) / 1000.0f, j1, j2
  };
  return true;
}

void startProductTeach(unsigned long now) {
  if (!productReady || !encodersCalibrated) {
    Serial.println("ERROR: folded calibration is not ready");
    return;
  }
  if (detectedHead != HEAD_TRACING) {
    Serial.println("ERROR: tracing head is required");
    return;
  }
  disableDrivers();
  j1OnlyMode = false;
  taughtJ1Only = false;
  taughtCount = 0;
  productTeachStartedMs = now;
  nextProductTeachSampleMs = now;
  productState = PRODUCT_TEACHING;
  sampleProductTeach(now);
  nextProductTeachSampleMs =
      now + static_cast<unsigned long>(1000.0f / PRODUCT_TEACH_HZ);
  Serial.println("TEACHING: press Start/Stop to finish");
}

void stopProductTeach(unsigned long now) {
  if (productState != PRODUCT_TEACHING) return;
  if (taughtCount == 0 ||
      rawTaught[taughtCount - 1].seconds <
          (now - productTeachStartedMs) / 1000.0f) {
    sampleProductTeach(now);
  }
  productState = PRODUCT_IDLE;
  if (taughtCount < 2) {
    taughtCount = 0;
    Serial.println("ERROR: taught path is too short");
    return;
  }
  if (!validateRawTeachPath(false)) return;
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
  Serial.print("TAUGHT: ");
  Serial.print(taughtCount);
  Serial.println(" raw points; install cutting head");
}

void runProductCut() {
  if (!productReady || !encodersCalibrated) {
    Serial.println("ERROR: folded calibration is not ready");
    return;
  }
  if (detectedHead != HEAD_CUTTING) {
    Serial.println("ERROR: cutting head is required");
    return;
  }
  if (taughtCount < 2) {
    Serial.println("ERROR: no taught path");
    return;
  }

  prepareTaughtPath(
      stabilizationEnabled ? PRODUCT_SMOOTHING_MS : 0.0f,
      stabilizationEnabled ? PRODUCT_MAX_DEVIATION_DEG : 0.0f);
  taughtJ1Only = false;
  productState = PRODUCT_CUTTING;
  productCutActive = true;
  cutAbortArmed = false;
  cuttingHeadBadReads = 0;
  Serial.print("CUTTING: stabilization ");
  Serial.println(stabilizationEnabled ? "ON" : "OFF");
  const bool completed = replayTeach(true);
  productCutActive = false;
  productState = PRODUCT_IDLE;
  disableDrivers();
  if (bladeExtended) retractBlade();
  Serial.println(completed ? "CUT COMPLETE" : "CUT STOPPED");
}

bool calibrateProductFoldedPose() {
  disableDrivers();
  j1PositionSteps = 0;
  j2PositionSteps = lroundf(180.0f * J2_STEPS_PER_DEG);
  encoderFault = false;
  j1OnlyMode = false;
  encodersJ1Only = false;
  encodersCalibrated = false;
  productReady = false;
  if (!USE_ENCODERS || !j1Encoder.calibrate() || !j2Encoder.calibrate()) {
    Serial.println("ERROR: folded calibration or magnet check failed");
    return false;
  }
  encodersCalibrated = true;
  productReady = true;
  Serial.println("READY: folded pose calibrated; drivers disabled");
  return true;
}

void serviceProductControls() {
  const unsigned long now = millis();
  sampleHead();
  const bool startPressed = startStopButton.update(now);
  const bool stabilizationPressed = stabilizationButton.update(now);
  const bool repeatPressed = repeatButton.update(now);

  if (productState == PRODUCT_TEACHING) {
    if (detectedHead != HEAD_TRACING) {
      taughtCount = 0;
      productState = PRODUCT_IDLE;
      Serial.println("TEACHING STOPPED: tracing head removed; path discarded");
      return;
    }
    if (startPressed) {
      stopProductTeach(now);
      return;
    }
    if (now - productTeachStartedMs >=
        static_cast<unsigned long>(PRODUCT_TEACH_MAX_SECONDS * 1000.0f)) {
      stopProductTeach(now);
      Serial.println("TEACHING STOPPED: maximum duration reached");
      return;
    }
    if (static_cast<long>(now - nextProductTeachSampleMs) >= 0) {
      if (!sampleProductTeach(now)) {
        productState = PRODUCT_IDLE;
        return;
      }
      nextProductTeachSampleMs +=
          static_cast<unsigned long>(1000.0f / PRODUCT_TEACH_HZ);
    }
    return;
  }

  if (productState != PRODUCT_IDLE) return;
  if (stabilizationPressed) {
    stabilizationEnabled = !stabilizationEnabled;
    Serial.print("STABILIZATION ");
    Serial.println(stabilizationEnabled ? "ON" : "OFF");
  }
  if (startPressed) {
    if (detectedHead == HEAD_TRACING) {
      startProductTeach(now);
    } else if (detectedHead == HEAD_CUTTING) {
      runProductCut();
    } else {
      Serial.println("ERROR: connect a recognized head");
    }
  }
  if (repeatPressed) runProductCut();
}

bool parseElbow(const char *text) {
  return text != nullptr && strcmp(text, "DOWN") == 0;
}

void printHelp() {
  Serial.println("Product buttons:");
  Serial.println("  Start/Stop: teach with tracing head; cut/abort with cutting head");
  Serial.println("  Stabilization: toggle path smoothing");
  Serial.println("  Repeat: repeat the last cut");
  Serial.println("Commands:");
  Serial.println("  ARM FOLDED | ARM J1 | DISARM");
  Serial.println("  TEST J1 <steps> | TEST J2 <steps>");
  Serial.println("  J1 <deg> | J2 <deg> | ANGLES <j1> <j2>");
  Serial.println("  XY <x> <y> [UP|DOWN]");
  Serial.println("  CUT <x0> <y0> <x1> <y1> [UP|DOWN]");
  Serial.println("  ENC | TEACH [J1] <seconds> [Hz] [smooth_ms] [max_dev]");
  Serial.println("  STREAM ON | STREAM OFF | STREAM RATE <1-50 Hz>");
  Serial.println("  FEEDBACK ON | FEEDBACK OFF | FEEDBACK STATUS");
  Serial.println("  HEAD | PLAY | CLEAR | POS | HELP");
}

void armAtFoldedPose(bool j1Only) {
  disableDrivers();
  j1PositionSteps = 0;
  j2PositionSteps = lroundf(180.0f * J2_STEPS_PER_DEG);
  encoderFault = false;
  j1OnlyMode = j1Only;
  encodersCalibrated = false;
  encodersJ1Only = j1Only;
  if (!j1Only) productReady = false;
  if (USE_ENCODERS) {
    if (!j1Encoder.calibrate() || (!j1Only && !j2Encoder.calibrate())) {
      Serial.println("ERROR: encoder or magnet check failed");
      return;
    }
    encodersCalibrated = true;
    productReady = !j1Only;
  } else if (j1Only) {
    Serial.println("ERROR: ARM J1 requires an encoder branch");
    return;
  }
  enableDrivers();
  armed = true;
  Serial.println(j1Only ? "ARMED J1 TEST" : "ARMED at J1=0, J2=180");
}

void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) return;
  command.toUpperCase();

  if (command == "ARM FOLDED") return armAtFoldedPose(false);
  if (command == "ARM J1") return armAtFoldedPose(true);
  if (command == "DISARM") {
    disableDrivers();
    j1OnlyMode = false;
    Serial.println("DISARMED");
    return;
  }
  if (command == "POS") return printPosition();
  if (command == "HELP") return printHelp();
  if (command == "HEAD") {
    sampleHead();
    Serial.print("HEAD ");
    Serial.print(headName(detectedHead));
    Serial.print(" ADC=");
    Serial.println(lastHeadAdc);
    return;
  }
  if (command == "CLEAR") {
    taughtCount = 0;
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
      Serial.println("ERROR: type ARM FOLDED or ARM J1 before streaming");
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
    float j1 = 0.0f;
    float j2 = 0.0f;
    if (encoderJointAngles(j1, j2)) {
      Serial.print("ENC J1=");
      Serial.print(j1, 2);
      Serial.print(" J2=");
      Serial.println(j2, 2);
    } else {
      Serial.println("ERROR: encoders are not calibrated");
    }
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
    if (!armed || j1OnlyMode) {
      Serial.println("ERROR: XY requires ARM FOLDED");
    } else if (moveToXY(a, b, xyFields == 3 && parseElbow(option))) {
      printPosition();
    }
    return;
  }
  int cutFields = sscanf(command.c_str(), "CUT %f %f %f %f %7s",
                         &a, &b, &c, &d, option);
  if (cutFields >= 4) {
    if (!armed || j1OnlyMode) {
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
    } else if (strcmp(axis, "J1") == 0) {
      if (executeSteps(rawSteps, 0)) {
        Serial.println(
            encoderFeedbackEnabled
                ? "OK: encoder accepted J1 test"
                : "PULSES SENT: feedback off; physical movement not verified");
      }
    } else if (strcmp(axis, "J2") == 0 && !j1OnlyMode) {
      if (executeSteps(0, rawSteps)) {
        Serial.println(
            encoderFeedbackEnabled
                ? "OK: encoder accepted J2 test"
                : "PULSES SENT: feedback off; physical movement not verified");
      }
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
  pinMode(HEAD_ID_PIN, INPUT);
  pinMode(BLADE_A_PIN, OUTPUT);
  pinMode(BLADE_B_PIN, OUTPUT);
  startStopButton.begin();
  stabilizationButton.begin();
  repeatButton.begin();
  analogReadResolution(12);
  digitalWrite(J1_PUL_PIN, STEP_IDLE);
  digitalWrite(J2_PUL_PIN, STEP_IDLE);
  digitalWrite(J1_DIR_PIN, LOW);
  digitalWrite(J2_DIR_PIN, LOW);
  stopBlade();
  disableDrivers();

  Serial.begin(115200);
  Serial.setTimeout(50);
  if (USE_ENCODERS) {
    const bool j1Found = j1Encoder.begin(J1_SDA_PIN, J1_SCL_PIN);
    const bool j2Found = j2Encoder.begin(J2_SDA_PIN, J2_SCL_PIN);
    if (!j1Found || !j2Found) {
      Serial.println("WARNING: one or both AS5600 encoders were not found");
    } else {
      calibrateProductFoldedPose();
    }
  }
  homeBladeAtBoot();
  Serial.println();
  Serial.println("SwivelCut Arduino controller ready");
  Serial.println("Fold the arm before every power-on or reset");
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
  serviceEncoderStream();
  serviceProductControls();
}
