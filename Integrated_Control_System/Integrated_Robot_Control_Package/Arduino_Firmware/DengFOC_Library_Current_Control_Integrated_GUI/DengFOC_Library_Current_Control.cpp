#include "DengFOC_Library_Current_Control.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {
constexpr float TWO_PI_F = 6.28318530718f;
constexpr float PI_OVER_TWO = 1.57079632679f;
constexpr float PI_OVER_THREE = 1.04719755120f;
constexpr float THREE_PI_OVER_TWO = 4.71238898038f;
constexpr float SQRT3_F = 1.73205080757f;
constexpr float ONE_OVER_SQRT3 = 0.57735026919f;
constexpr float TWO_OVER_SQRT3 = 1.15470053838f;

constexpr int DRIVER_ENABLE_PIN = 12;
constexpr int PWM_BITS = 8;
constexpr int PWM_FREQ = 30000;
constexpr uint8_t POLE_PAIRS = 7;
constexpr float SUPPLY_VOLTAGE = 12.6f;
constexpr float DEFAULT_VOLTAGE_LIMIT = 3.0f;
constexpr float DEFAULT_CURRENT_LIMIT = 3.0f;
constexpr float DEFAULT_VELOCITY_LIMIT = 20.0f;
constexpr float DEFAULT_TORQUE_CONSTANT = 0.0955f;
constexpr uint32_t CONTROL_PERIOD_US = 1000;
constexpr uint32_t TELEMETRY_PERIOD_MS = 50;

constexpr uint16_t FAULT_ESTOP = 1u << 0;
constexpr uint16_t FAULT_WATCHDOG = 1u << 1;
constexpr uint16_t FAULT_DISABLED = 1u << 2;
constexpr uint16_t FAULT_SENSOR = 1u << 3;
constexpr uint16_t FAULT_OVERCURRENT = 1u << 4;

float clampf(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float normalizeAngle(float angle) {
  float result = fmodf(angle, TWO_PI_F);
  return result >= 0.0f ? result : result + TWO_PI_F;
}

bool textEquals(const char *left, const char *right) {
  if (!left || !right) return false;
  while (*left && *right) {
    if (tolower(static_cast<unsigned char>(*left++)) !=
        tolower(static_cast<unsigned char>(*right++))) {
      return false;
    }
  }
  return *left == '\0' && *right == '\0';
}

uint8_t xorChecksum(const char *text) {
  uint8_t checksum = 0;
  while (*text) checksum ^= static_cast<uint8_t>(*text++);
  return checksum;
}

uint8_t parseHexByte(const char *text) {
  return static_cast<uint8_t>(strtoul(text, nullptr, 16) & 0xFFu);
}

class PIDLoop {
public:
  PIDLoop(float kpValue, float kiValue, float kdValue, float rampValue, float limitValue)
      : kp(kpValue), ki(kiValue), kd(kdValue), ramp(rampValue), limit(limitValue) {}

  float update(float error, float dt) {
    if (dt <= 0.0f || dt > 0.5f) dt = 0.001f;
    const float proportional = kp * error;
    integral_ += ki * dt * 0.5f * (error + previousError_);
    integral_ = clampf(integral_, -limit, limit);
    const float derivative = kd * (error - previousError_) / dt;
    float output = clampf(proportional + integral_ + derivative, -limit, limit);
    if (ramp > 0.0f) {
      const float outputRate = (output - previousOutput_) / dt;
      if (outputRate > ramp) output = previousOutput_ + ramp * dt;
      if (outputRate < -ramp) output = previousOutput_ - ramp * dt;
    }
    previousError_ = error;
    previousOutput_ = output;
    return output;
  }

  void configure(float kpValue, float kiValue, float kdValue, float rampValue, float limitValue) {
    kp = kpValue;
    ki = kiValue;
    kd = kdValue;
    ramp = rampValue;
    limit = limitValue;
    reset();
  }

  void reset() {
    integral_ = 0.0f;
    previousError_ = 0.0f;
    previousOutput_ = 0.0f;
  }

  float kp;
  float ki;
  float kd;
  float ramp;
  float limit;

private:
  float integral_ = 0.0f;
  float previousError_ = 0.0f;
  float previousOutput_ = 0.0f;
};

class LowPassFilter {
public:
  explicit LowPassFilter(float timeConstant) : timeConstant_(timeConstant) {}

  float update(float input, float dt) {
    if (!initialized_) {
      initialized_ = true;
      previous_ = input;
      return input;
    }
    if (dt <= 0.0f || dt > 0.5f) dt = 0.001f;
    const float alpha = timeConstant_ / (timeConstant_ + dt);
    previous_ = alpha * previous_ + (1.0f - alpha) * input;
    return previous_;
  }

  void reset(float value = 0.0f) {
    previous_ = value;
    initialized_ = true;
  }

private:
  float timeConstant_;
  float previous_ = 0.0f;
  bool initialized_ = false;
};

class AS5600Sensor {
public:
  bool begin(TwoWire &wire) {
    wire_ = &wire;
    previousUpdateUs_ = micros();
    wire_->beginTransmission(0x36);
    healthy_ = wire_->endTransmission() == 0;
    if (!healthy_) return false;
    update(previousUpdateUs_);
    previousMechanicalAngle_ = mechanicalAngle_;
    return true;
  }

  bool update(uint32_t nowUs) {
    if (!wire_) {
      healthy_ = false;
      return false;
    }
    wire_->beginTransmission(0x36);
    wire_->write(0x0C);
    if (wire_->endTransmission(false) != 0) {
      healthy_ = false;
      return false;
    }
    if (wire_->requestFrom(0x36, static_cast<uint8_t>(2)) != 2) {
      healthy_ = false;
      return false;
    }
    const uint16_t raw = (static_cast<uint16_t>(wire_->read()) << 8) | wire_->read();
    mechanicalAngle_ = (raw & 0x0FFF) * (TWO_PI_F / 4096.0f);
    float delta = mechanicalAngle_ - previousMechanicalAngle_;
    if (delta > PI) {
      rotations_--;
      delta -= TWO_PI_F;
    } else if (delta < -PI) {
      rotations_++;
      delta += TWO_PI_F;
    }
    angle_ = rotations_ * TWO_PI_F + mechanicalAngle_;
    const float dt = (nowUs - previousUpdateUs_) * 1e-6f;
    if (dt > 0.0f) velocity_ = delta / dt;
    previousMechanicalAngle_ = mechanicalAngle_;
    previousUpdateUs_ = nowUs;
    healthy_ = true;
    return true;
  }

  float mechanicalAngle() const { return mechanicalAngle_; }
  float angle() const { return angle_; }
  float velocity() const { return velocity_; }
  bool healthy() const { return healthy_; }

private:
  TwoWire *wire_ = nullptr;
  float mechanicalAngle_ = 0.0f;
  float angle_ = 0.0f;
  float velocity_ = 0.0f;
  float previousMechanicalAngle_ = 0.0f;
  int32_t rotations_ = 0;
  uint32_t previousUpdateUs_ = 0;
  bool healthy_ = false;
};

class DengFOCCurrentSense {
public:
  DengFOCCurrentSense(int pinA, int pinB) : pinA_(pinA), pinB_(pinB) {}

  void begin() {
    pinMode(pinA_, INPUT);
    pinMode(pinB_, INPUT);
    offsetA_ = 0.0f;
    offsetB_ = 0.0f;
    constexpr int samples = 1000;
    for (int i = 0; i < samples; ++i) {
      offsetA_ += readVoltage(pinA_);
      offsetB_ += readVoltage(pinB_);
      delayMicroseconds(500);
    }
    offsetA_ /= samples;
    offsetB_ /= samples;
  }

  void update() {
    // DengFOC V0.6 "DengFOC V4" current gain convention is positive.
    currentA_ = (readVoltage(pinA_) - offsetA_) * VOLTS_TO_AMPS;
    currentB_ = (readVoltage(pinB_) - offsetB_) * VOLTS_TO_AMPS;
  }

  float iq(float electricalAngle) const {
    const float iAlpha = currentA_;
    const float iBeta = ONE_OVER_SQRT3 * currentA_ + TWO_OVER_SQRT3 * currentB_;
    return iBeta * cosf(electricalAngle) - iAlpha * sinf(electricalAngle);
  }

private:
  static constexpr float ADC_VOLTAGE = 3.3f;
  static constexpr float ADC_RESOLUTION = 4095.0f;
  static constexpr float SHUNT_RESISTOR = 0.01f;
  static constexpr float AMP_GAIN = 50.0f;
  static constexpr float VOLTS_TO_AMPS = 1.0f / SHUNT_RESISTOR / AMP_GAIN;

  static float readVoltage(int pin) {
    return analogRead(pin) * (ADC_VOLTAGE / ADC_RESOLUTION);
  }

  int pinA_;
  int pinB_;
  float offsetA_ = 0.0f;
  float offsetB_ = 0.0f;
  float currentA_ = 0.0f;
  float currentB_ = 0.0f;
};

enum ControlMode : uint8_t {
  MODE_TORQUE = 0,
  MODE_VELOCITY = 1,
  MODE_POSITION = 2,
};

enum class TestMode : uint8_t {
  Idle,
  Align,
  TestIq,
  OpenLoop,
  Phase,
};

struct MotorChannel {
  MotorChannel(int pa, int pb, int pc, int ca, int cb)
      : pwmA(pa),
        pwmB(pb),
        pwmC(pc),
        currentSense(ca, cb),
        currentLoop(5.0f, 80.0f, 0.0f, 100000.0f, DEFAULT_VOLTAGE_LIMIT),
        velocityLoop(0.20f, 4.0f, 0.0005f, 1000.0f, DEFAULT_CURRENT_LIMIT),
        angleLoop(8.0f, 0.0f, 0.0f, 100000.0f, DEFAULT_VELOCITY_LIMIT),
        velocityFilter(0.04f),
        currentFilter(0.05f) {}

  int pwmA;
  int pwmB;
  int pwmC;
  int8_t commutationDir = -1;
  int8_t encoderDir = -1;
  int8_t velocityDir = 1;
  int8_t currentSenseDir = 1;
  int8_t actuatorDir = 1;
  AS5600Sensor sensor;
  DengFOCCurrentSense currentSense;
  PIDLoop currentLoop;
  PIDLoop velocityLoop;
  PIDLoop angleLoop;
  LowPassFilter velocityFilter;
  LowPassFilter currentFilter;
  ControlMode mode = MODE_TORQUE;
  bool enabled = false;
  bool sensorReady = false;
  bool aligned = false;
  float zeroElectricalAngle = 0.0f;
  float logicalZeroAngle = 0.0f;
  float commandTarget = 0.0f;
  float torqueTargetNm = 0.0f;
  float torqueMeasuredNm = 0.0f;
  float torqueConstant = DEFAULT_TORQUE_CONSTANT;
  float angle = 0.0f;
  float velocity = 0.0f;
  float iqMeasured = 0.0f;
  float iqTarget = 0.0f;
  float iqTargetDrive = 0.0f;
  float uqCommand = 0.0f;
  float velocityTarget = 0.0f;
  float velocityError = 0.0f;
  float angleError = 0.0f;
};

MotorChannel motor1(32, 33, 25, 39, 36);
MotorChannel motor2(26, 27, 14, 35, 34);
Stream *serialPort = nullptr;

float voltageLimit = DEFAULT_VOLTAGE_LIMIT;
float currentLimit = DEFAULT_CURRENT_LIMIT;
float velocityLimit = DEFAULT_VELOCITY_LIMIT;
uint32_t watchdogMs = 500;
uint32_t lastCommandMs = 0;
uint32_t lastControlUs = 0;
uint32_t lastTelemetryMs = 0;
bool estopLatched = false;
char lineBuffer[240] = {};
uint8_t lineLength = 0;

TestMode testMode = TestMode::Idle;
uint8_t activeMotorIndex = 0;
uint32_t testStartMs = 0;
uint32_t testDurationMs = 0;
float testTarget = 0.0f;
float testVoltage = 0.0f;
float testSpeed = 0.0f;
float openLoopBaseAngle = 0.0f;
char testName[12] = "idle";
char phaseName = 'a';
uint32_t sampleCount = 0;
float sumIq = 0.0f;
float sumUq = 0.0f;
float sumVelocity = 0.0f;
float startAngle = 0.0f;
float endAngle = 0.0f;
bool saturationSeen = false;

MotorChannel &motor(uint8_t index) {
  return index == 0 ? motor1 : motor2;
}

const char *motorName(uint8_t index) {
  return index == 0 ? "m1" : "m2";
}

bool parseMotor(const char *text, uint8_t &index) {
  if (textEquals(text, "m1") || !strcmp(text, "0") || !strcmp(text, "1")) {
    index = 0;
    return true;
  }
  if (textEquals(text, "m2") || !strcmp(text, "2")) {
    index = 1;
    return true;
  }
  return false;
}

const char *modeName(ControlMode mode) {
  switch (mode) {
    case MODE_TORQUE: return "torque";
    case MODE_VELOCITY: return "velocity";
    case MODE_POSITION: return "position";
    default: return "unknown";
  }
}

bool parseMode(const char *text, ControlMode &mode) {
  if (textEquals(text, "torque")) {
    mode = MODE_TORQUE;
    return true;
  }
  if (textEquals(text, "velocity")) {
    mode = MODE_VELOCITY;
    return true;
  }
  if (textEquals(text, "position") || textEquals(text, "angle")) {
    mode = MODE_POSITION;
    return true;
  }
  return false;
}

void sendPacket(const char *payload) {
  if (!serialPort) return;
  char packet[640];
  snprintf(packet, sizeof(packet), "%s*%02X", payload, xorChecksum(payload));
  serialPort->println(packet);
}

void sendAck(const char *text) {
  char payload[80];
  snprintf(payload, sizeof(payload), "OK,%s", text);
  sendPacket(payload);
}

void sendError(const char *text) {
  char payload[100];
  snprintf(payload, sizeof(payload), "ERR,%s", text);
  sendPacket(payload);
}

bool readCheckedPayload(char *line) {
  char *star = strrchr(line, '*');
  if (!star || strlen(star + 1) < 2) return false;
  const uint8_t received = parseHexByte(star + 1);
  *star = '\0';
  return received == xorChecksum(line);
}

void configurePwm(MotorChannel &channel) {
  ledcAttach(channel.pwmA, PWM_FREQ, PWM_BITS);
  ledcAttach(channel.pwmB, PWM_FREQ, PWM_BITS);
  ledcAttach(channel.pwmC, PWM_FREQ, PWM_BITS);
}

void enableDriver() {
  digitalWrite(DRIVER_ENABLE_PIN, HIGH);
}

void disableDriverIfIdle() {
  if (testMode == TestMode::Idle && !motor1.enabled && !motor2.enabled) {
    digitalWrite(DRIVER_ENABLE_PIN, LOW);
  }
}

void setPwm(MotorChannel &channel, float ua, float ub, float uc) {
  ua = clampf(ua, 0.0f, SUPPLY_VOLTAGE);
  ub = clampf(ub, 0.0f, SUPPLY_VOLTAGE);
  uc = clampf(uc, 0.0f, SUPPLY_VOLTAGE);
  ledcWrite(channel.pwmA, static_cast<uint32_t>(255.0f * ua / SUPPLY_VOLTAGE));
  ledcWrite(channel.pwmB, static_cast<uint32_t>(255.0f * ub / SUPPLY_VOLTAGE));
  ledcWrite(channel.pwmC, static_cast<uint32_t>(255.0f * uc / SUPPLY_VOLTAGE));
}

void neutralPwm(MotorChannel &channel) {
  const float half = SUPPLY_VOLTAGE * 0.5f;
  setPwm(channel, half, half, half);
  channel.iqTarget = 0.0f;
  channel.iqTargetDrive = 0.0f;
  channel.uqCommand = 0.0f;
  channel.currentLoop.reset();
}

// Port of DengFOC V0.6 M0_setTorque/M1_setTorque SVPWM.
void setPhaseVoltage(MotorChannel &channel, float uq, float electricalAngle) {
  uq = clampf(uq, -voltageLimit, voltageLimit);
  channel.uqCommand = uq;
  if (uq < 0.0f) electricalAngle += PI;
  const float magnitude = fabsf(uq);
  electricalAngle = normalizeAngle(electricalAngle + PI_OVER_TWO);
  int sector = static_cast<int>(floorf(electricalAngle / PI_OVER_THREE)) + 1;
  sector = constrain(sector, 1, 6);
  const float t1 =
      SQRT3_F * sinf(sector * PI_OVER_THREE - electricalAngle) * magnitude / SUPPLY_VOLTAGE;
  const float t2 =
      SQRT3_F * sinf(electricalAngle - (sector - 1.0f) * PI_OVER_THREE) * magnitude /
      SUPPLY_VOLTAGE;
  const float t0 = 1.0f - t1 - t2;
  float ta = 0.5f;
  float tb = 0.5f;
  float tc = 0.5f;
  switch (sector) {
    case 1:
      ta = t1 + t2 + t0 * 0.5f;
      tb = t2 + t0 * 0.5f;
      tc = t0 * 0.5f;
      break;
    case 2:
      ta = t1 + t0 * 0.5f;
      tb = t1 + t2 + t0 * 0.5f;
      tc = t0 * 0.5f;
      break;
    case 3:
      ta = t0 * 0.5f;
      tb = t1 + t2 + t0 * 0.5f;
      tc = t2 + t0 * 0.5f;
      break;
    case 4:
      ta = t0 * 0.5f;
      tb = t1 + t0 * 0.5f;
      tc = t1 + t2 + t0 * 0.5f;
      break;
    case 5:
      ta = t2 + t0 * 0.5f;
      tb = t0 * 0.5f;
      tc = t1 + t2 + t0 * 0.5f;
      break;
    case 6:
      ta = t1 + t2 + t0 * 0.5f;
      tb = t0 * 0.5f;
      tc = t1 + t0 * 0.5f;
      break;
  }
  setPwm(
      channel,
      clampf(ta, 0.0f, 1.0f) * SUPPLY_VOLTAGE,
      clampf(tb, 0.0f, 1.0f) * SUPPLY_VOLTAGE,
      clampf(tc, 0.0f, 1.0f) * SUPPLY_VOLTAGE);
}

float electricalAngle(const MotorChannel &channel) {
  return normalizeAngle(
      static_cast<float>(channel.commutationDir * POLE_PAIRS) *
          channel.sensor.mechanicalAngle() -
      channel.zeroElectricalAngle);
}

float motorAngle(const MotorChannel &channel) {
  return channel.encoderDir * channel.sensor.angle() - channel.logicalZeroAngle;
}

float motorVelocity(const MotorChannel &channel) {
  return channel.velocityDir * channel.encoderDir * channel.sensor.velocity();
}

float torqueLimitNm(const MotorChannel &channel) {
  return currentLimit * channel.torqueConstant;
}

float torqueToCurrent(const MotorChannel &channel, float torqueNm) {
  if (channel.torqueConstant <= 0.0f) return 0.0f;
  return clampf(torqueNm / channel.torqueConstant, -currentLimit, currentLimit);
}

bool anyMotorEnabled() {
  return motor1.enabled || motor2.enabled;
}

bool sensorsHealthy() {
  return (!motor1.enabled || (motor1.sensorReady && motor1.sensor.healthy())) &&
         (!motor2.enabled || (motor2.sensorReady && motor2.sensor.healthy()));
}

void resetChannel(MotorChannel &channel) {
  channel.enabled = false;
  channel.commandTarget = 0.0f;
  channel.torqueTargetNm = 0.0f;
  channel.torqueMeasuredNm = 0.0f;
  channel.iqTarget = 0.0f;
  channel.iqTargetDrive = 0.0f;
  channel.uqCommand = 0.0f;
  channel.velocityTarget = 0.0f;
  channel.velocityError = 0.0f;
  channel.angleError = 0.0f;
  channel.currentLoop.reset();
  channel.velocityLoop.reset();
  channel.angleLoop.reset();
  neutralPwm(channel);
}

void stopAll(bool latchEstop) {
  resetChannel(motor1);
  resetChannel(motor2);
  testMode = TestMode::Idle;
  strcpy(testName, "idle");
  if (latchEstop) estopLatched = true;
  disableDriverIfIdle();
}

void applySet(MotorChannel &channel, ControlMode mode, float target, bool enabled) {
  channel.mode = mode;
  channel.commandTarget = target;
  channel.enabled = enabled;
  if (mode == MODE_TORQUE) {
    channel.torqueTargetNm = clampf(target, -torqueLimitNm(channel), torqueLimitNm(channel));
  } else {
    channel.torqueTargetNm = 0.0f;
  }
  if (!enabled) resetChannel(channel);
}

float calculateIqTarget(MotorChannel &channel, float dt) {
  if (channel.mode == MODE_TORQUE) {
    channel.torqueTargetNm =
        clampf(channel.commandTarget, -torqueLimitNm(channel), torqueLimitNm(channel));
    return torqueToCurrent(channel, channel.torqueTargetNm);
  }
  if (channel.mode == MODE_VELOCITY) {
    channel.velocityTarget = clampf(channel.commandTarget, -velocityLimit, velocityLimit);
    channel.velocityError = channel.velocityTarget - channel.velocity;
    return channel.velocityLoop.update(channel.velocityError, dt);
  }
  channel.angleError = channel.commandTarget - channel.angle;
  channel.velocityTarget = channel.angleLoop.update(channel.angleError, dt);
  channel.velocityTarget = clampf(channel.velocityTarget, -velocityLimit, velocityLimit);
  channel.velocityError = channel.velocityTarget - channel.velocity;
  return channel.velocityLoop.update(channel.velocityError, dt);
}

void updateNormalChannel(MotorChannel &channel, float dt) {
  if (!channel.enabled || estopLatched || !channel.aligned || !channel.sensorReady ||
      !channel.sensor.healthy()) {
    channel.iqTarget = 0.0f;
    channel.iqTargetDrive = 0.0f;
    setPhaseVoltage(channel, 0.0f, electricalAngle(channel));
    return;
  }
  channel.iqTarget = clampf(calculateIqTarget(channel, dt), -currentLimit, currentLimit);
  channel.iqTargetDrive = channel.actuatorDir * channel.iqTarget;
  channel.uqCommand =
      channel.currentLoop.update(channel.iqTargetDrive - channel.iqMeasured, dt);
  setPhaseVoltage(channel, channel.uqCommand, electricalAngle(channel));
}

void resetTestStats(MotorChannel &channel) {
  sampleCount = 0;
  sumIq = 0.0f;
  sumUq = 0.0f;
  sumVelocity = 0.0f;
  saturationSeen = false;
  startAngle = channel.angle;
  endAngle = channel.angle;
}

bool beginTimedTest(uint8_t index, TestMode mode, const char *name, uint32_t durationMs) {
  if (estopLatched) {
    sendError("estop_latched");
    return false;
  }
  MotorChannel &channel = motor(index);
  if (!channel.sensorReady || !channel.sensor.healthy()) {
    sendError("sensor");
    return false;
  }
  if (mode == TestMode::TestIq && !channel.aligned) {
    sendError("not_aligned");
    return false;
  }
  stopAll(false);
  activeMotorIndex = index;
  testMode = mode;
  strncpy(testName, name, sizeof(testName) - 1);
  testName[sizeof(testName) - 1] = '\0';
  if (mode == TestMode::Align) channel.zeroElectricalAngle = 0.0f;
  testStartMs = millis();
  testDurationMs = constrain(durationMs, 100u, 8000u);
  resetTestStats(channel);
  channel.enabled = true;
  enableDriver();
  return true;
}

void finishTest(const char *reason) {
  MotorChannel &channel = motor(activeMotorIndex);
  endAngle = channel.angle;
  const float avgIq = sampleCount ? sumIq / sampleCount : 0.0f;
  const float avgUq = sampleCount ? sumUq / sampleCount : 0.0f;
  const float avgVelocity = sampleCount ? sumVelocity / sampleCount : 0.0f;
  char payload[240];
  snprintf(
      payload,
      sizeof(payload),
      "DIAG,%s,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%s",
      testName,
      motorName(activeMotorIndex),
      testTarget,
      avgIq,
      avgUq,
      endAngle - startAngle,
      avgVelocity,
      saturationSeen ? 1u : 0u,
      reason);
  sendPacket(payload);
  resetChannel(channel);
  testMode = TestMode::Idle;
  strcpy(testName, "idle");
  disableDriverIfIdle();
}

void updateTest(float dt) {
  if (testMode == TestMode::Idle) return;
  MotorChannel &channel = motor(activeMotorIndex);
  const uint32_t elapsed = millis() - testStartMs;
  if (estopLatched || !channel.sensor.healthy()) {
    finishTest(estopLatched ? "estop" : "sensor");
    return;
  }
  if (elapsed >= testDurationMs) {
    if (testMode == TestMode::Align) {
      channel.zeroElectricalAngle = normalizeAngle(
          static_cast<float>(channel.commutationDir * POLE_PAIRS) *
          channel.sensor.mechanicalAngle());
      channel.aligned = true;
    }
    finishTest("done");
    return;
  }
  if (fabsf(channel.iqMeasured) > currentLimit * 1.5f) {
    finishTest("overcurrent");
    return;
  }
  if (testMode == TestMode::Align) {
    setPhaseVoltage(channel, testVoltage, THREE_PI_OVER_TWO);
  } else if (testMode == TestMode::TestIq) {
    channel.iqTarget = testTarget;
    channel.iqTargetDrive = channel.actuatorDir * channel.iqTarget;
    const float uq =
        channel.currentLoop.update(channel.iqTargetDrive - channel.iqMeasured, dt);
    setPhaseVoltage(channel, uq, electricalAngle(channel));
  } else if (testMode == TestMode::OpenLoop) {
    const float theta = normalizeAngle(openLoopBaseAngle + testSpeed * elapsed * 0.001f);
    channel.iqTarget = 0.0f;
    setPhaseVoltage(channel, testVoltage, theta);
  } else if (testMode == TestMode::Phase) {
    const float half = SUPPLY_VOLTAGE * 0.5f;
    const float voltage = clampf(testVoltage, 0.0f, voltageLimit);
    if (phaseName == 'a') {
      setPwm(channel, half + voltage, half - voltage * 0.5f, half - voltage * 0.5f);
    } else if (phaseName == 'b') {
      setPwm(channel, half - voltage * 0.5f, half + voltage, half - voltage * 0.5f);
    } else {
      setPwm(channel, half - voltage * 0.5f, half - voltage * 0.5f, half + voltage);
    }
    channel.uqCommand = voltage;
  }
  sampleCount++;
  sumIq += channel.iqMeasured;
  sumUq += channel.uqCommand;
  sumVelocity += channel.velocity;
  endAngle = channel.angle;
  if (fabsf(channel.uqCommand) >= voltageLimit * 0.92f) saturationSeen = true;
}

uint16_t faultMask() {
  uint16_t faults = 0;
  if (estopLatched) faults |= FAULT_ESTOP;
  if (anyMotorEnabled() && millis() - lastCommandMs > watchdogMs) faults |= FAULT_WATCHDOG;
  if (!anyMotorEnabled()) faults |= FAULT_DISABLED;
  if (!sensorsHealthy()) faults |= FAULT_SENSOR;
  if ((motor1.enabled && fabsf(motor1.iqMeasured) > currentLimit * 1.25f) ||
      (motor2.enabled && fabsf(motor2.iqMeasured) > currentLimit * 1.25f)) {
    faults |= FAULT_OVERCURRENT;
  }
  return faults;
}

void sendConfig() {
  char payload[260];
  snprintf(
      payload,
      sizeof(payload),
      "CFG,%.5f,%.5f,%.5f,%lu,%.5f,%.5f,%u",
      voltageLimit,
      currentLimit,
      velocityLimit,
      static_cast<unsigned long>(watchdogMs),
      motor1.torqueConstant,
      motor2.torqueConstant,
      faultMask());
  sendPacket(payload);
}

void sendPidConfig() {
  char payload[360];
  snprintf(
      payload,
      sizeof(payload),
      "PID,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
      motor1.currentLoop.kp,
      motor1.currentLoop.ki,
      motor1.currentLoop.kd,
      motor1.currentLoop.ramp,
      motor1.currentLoop.limit,
      motor1.velocityLoop.kp,
      motor1.velocityLoop.ki,
      motor1.velocityLoop.kd,
      motor1.velocityLoop.ramp,
      motor1.velocityLoop.limit,
      motor1.angleLoop.kp,
      motor1.angleLoop.ki,
      motor1.angleLoop.kd,
      motor1.angleLoop.ramp,
      motor1.angleLoop.limit);
  sendPacket(payload);
}

void sendControlTelemetry() {
  char payload[620];
  snprintf(
      payload,
      sizeof(payload),
      "TQC2,%lu,%u,%s,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%s,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%u",
      millis(),
      anyMotorEnabled() ? 1 : 0,
      modeName(motor1.mode),
      motor1.commandTarget,
      motor1.angle,
      electricalAngle(motor1),
      motor1.velocity,
      motor1.torqueMeasuredNm,
      motor1.actuatorDir * motor1.iqMeasured,
      motor1.iqTarget,
      motor1.uqCommand,
      motor1.torqueConstant,
      torqueLimitNm(motor1),
      modeName(motor2.mode),
      motor2.commandTarget,
      motor2.angle,
      electricalAngle(motor2),
      motor2.velocity,
      motor2.torqueMeasuredNm,
      motor2.actuatorDir * motor2.iqMeasured,
      motor2.iqTarget,
      motor2.uqCommand,
      motor2.torqueConstant,
      torqueLimitNm(motor2),
      voltageLimit,
      currentLimit,
      faultMask());
  sendPacket(payload);
}

void sendDiagnosticTelemetry() {
  const MotorChannel &active = motor(activeMotorIndex);
  char payload[340];
  snprintf(
      payload,
      sizeof(payload),
      "FDD,%lu,%s,%s,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%u,%.3f,%.3f",
      static_cast<unsigned long>(millis()),
      testName,
      motorName(activeMotorIndex),
      active.enabled ? 1u : 0u,
      motor1.angle * 180.0f / PI,
      motor1.velocity,
      motor1.iqMeasured,
      motor1.iqTarget,
      motor1.uqCommand,
      motor2.angle * 180.0f / PI,
      motor2.velocity,
      motor2.iqMeasured,
      motor2.iqTarget,
      motor2.uqCommand,
      active.zeroElectricalAngle,
      saturationSeen ? 1u : 0u,
      estopLatched ? 1u : 0u,
      voltageLimit,
      currentLimit);
  sendPacket(payload);
}

void processCommand(char *line) {
  char *command = strtok(line, ",");
  if (!command) return;
  lastCommandMs = millis();

  if (textEquals(command, "PING")) {
    sendPacket("OK,pong");
    return;
  }
  if (textEquals(command, "CFG?")) {
    sendConfig();
    return;
  }
  if (textEquals(command, "PID?")) {
    sendPidConfig();
    return;
  }
  if (textEquals(command, "ESTOP")) {
    stopAll(true);
    sendAck("estop");
    return;
  }
  if (textEquals(command, "CLR")) {
    estopLatched = false;
    stopAll(false);
    sendAck("clear_fault");
    return;
  }
  if (textEquals(command, "DUAL")) {
    sendAck("dual_always_on");
    return;
  }
  if (textEquals(command, "SAFE")) {
    char *voltage = strtok(nullptr, ",");
    char *current = strtok(nullptr, ",");
    char *third = strtok(nullptr, ",");
    char *fourth = strtok(nullptr, ",");
    if (!voltage || !current || !third) {
      sendError("safe_args");
      return;
    }
    voltageLimit = clampf(atof(voltage), 0.1f, SUPPLY_VOLTAGE * 0.9f);
    currentLimit = clampf(atof(current), 0.1f, 12.0f);
    if (fourth) {
      velocityLimit = clampf(atof(third), 0.1f, 300.0f);
      watchdogMs = static_cast<uint32_t>(clampf(atof(fourth), 50.0f, 5000.0f));
    } else {
      watchdogMs = static_cast<uint32_t>(clampf(atof(third), 50.0f, 5000.0f));
    }
    motor1.currentLoop.limit = voltageLimit;
    motor2.currentLoop.limit = voltageLimit;
    motor1.velocityLoop.limit = currentLimit;
    motor2.velocityLoop.limit = currentLimit;
    motor1.angleLoop.limit = velocityLimit;
    motor2.angleLoop.limit = velocityLimit;
    sendAck("safe");
    return;
  }
  if (textEquals(command, "DIR")) {
    char *motorText = strtok(nullptr, ",");
    uint8_t index = 0;
    if (!motorText || !parseMotor(motorText, index)) {
      sendError("motor");
      return;
    }
    char *comm = strtok(nullptr, ",");
    char *encoder = strtok(nullptr, ",");
    char *velocity = strtok(nullptr, ",");
    char *current = strtok(nullptr, ",");
    char *actuator = strtok(nullptr, ",");
    if (!comm || !encoder || !velocity || !current || !actuator) {
      sendError("dir_args");
      return;
    }
    MotorChannel &channel = motor(index);
    channel.commutationDir = atoi(comm) < 0 ? -1 : 1;
    channel.encoderDir = atoi(encoder) < 0 ? -1 : 1;
    channel.velocityDir = atoi(velocity) < 0 ? -1 : 1;
    channel.currentSenseDir = atoi(current) < 0 ? -1 : 1;
    channel.actuatorDir = atoi(actuator) < 0 ? -1 : 1;
    channel.aligned = false;
    sendAck("dir");
    return;
  }
  if (textEquals(command, "ZERO")) {
    char *motorText = strtok(nullptr, ",");
    if (!motorText || textEquals(motorText, "all")) {
      motor1.logicalZeroAngle = motor1.encoderDir * motor1.sensor.angle();
      motor2.logicalZeroAngle = motor2.encoderDir * motor2.sensor.angle();
      sendAck("zero_all");
      return;
    }
    uint8_t index = 0;
    if (!parseMotor(motorText, index)) {
      sendError("motor");
      return;
    }
    MotorChannel &channel = motor(index);
    channel.logicalZeroAngle = channel.encoderDir * channel.sensor.angle();
    sendAck("zero");
    return;
  }
  if (textEquals(command, "ALIGN")) {
    char *motorText = strtok(nullptr, ",");
    char *voltage = strtok(nullptr, ",");
    char *duration = strtok(nullptr, ",");
    uint8_t index = 0;
    if (!motorText || !voltage || !duration || !parseMotor(motorText, index)) {
      sendError("align_args");
      return;
    }
    testVoltage = clampf(fabsf(atof(voltage)), 0.2f, voltageLimit);
    testTarget = testVoltage;
    if (beginTimedTest(index, TestMode::Align, "align", atoi(duration))) sendAck("align");
    return;
  }
  if (textEquals(command, "TESTIQ")) {
    char *motorText = strtok(nullptr, ",");
    char *target = strtok(nullptr, ",");
    char *duration = strtok(nullptr, ",");
    uint8_t index = 0;
    if (!motorText || !target || !duration || !parseMotor(motorText, index)) {
      sendError("testiq_args");
      return;
    }
    testTarget = clampf(atof(target), -currentLimit, currentLimit);
    if (beginTimedTest(index, TestMode::TestIq, "iq", atoi(duration))) sendAck("testiq");
    return;
  }
  if (textEquals(command, "OPEN")) {
    char *motorText = strtok(nullptr, ",");
    char *direction = strtok(nullptr, ",");
    char *voltage = strtok(nullptr, ",");
    char *speed = strtok(nullptr, ",");
    char *duration = strtok(nullptr, ",");
    uint8_t index = 0;
    if (!motorText || !direction || !voltage || !speed || !duration ||
        !parseMotor(motorText, index)) {
      sendError("open_args");
      return;
    }
    const int8_t sign = textEquals(direction, "ccw") || !strcmp(direction, "-") ? -1 : 1;
    testVoltage = clampf(fabsf(atof(voltage)), 0.2f, voltageLimit);
    testSpeed = sign * clampf(fabsf(atof(speed)), 0.05f, 20.0f);
    testTarget = testSpeed;
    if (beginTimedTest(index, TestMode::OpenLoop, "open", atoi(duration))) {
      openLoopBaseAngle = electricalAngle(motor(index));
      sendAck("open");
    }
    return;
  }
  if (textEquals(command, "PHASE")) {
    char *motorText = strtok(nullptr, ",");
    char *phase = strtok(nullptr, ",");
    char *voltage = strtok(nullptr, ",");
    char *duration = strtok(nullptr, ",");
    uint8_t index = 0;
    if (!motorText || !phase || !voltage || !duration || !parseMotor(motorText, index)) {
      sendError("phase_args");
      return;
    }
    phaseName = tolower(static_cast<unsigned char>(phase[0]));
    testVoltage = clampf(fabsf(atof(voltage)), 0.2f, voltageLimit);
    testTarget = testVoltage;
    if (beginTimedTest(index, TestMode::Phase, "phase", atoi(duration))) sendAck("phase");
    return;
  }
  if (textEquals(command, "KT")) {
    char *selector = strtok(nullptr, ",");
    char *value = strtok(nullptr, ",");
    if (!selector || !value) {
      sendError("kt_args");
      return;
    }
    const float kt = clampf(atof(value), 0.001f, 5.0f);
    if (textEquals(selector, "all") || textEquals(selector, "m1")) motor1.torqueConstant = kt;
    if (textEquals(selector, "all") || textEquals(selector, "m2")) motor2.torqueConstant = kt;
    sendAck("kt");
    return;
  }
  if (textEquals(command, "PIDQ")) {
    char *kp = strtok(nullptr, ",");
    char *ki = strtok(nullptr, ",");
    char *kd = strtok(nullptr, ",");
    char *ramp = strtok(nullptr, ",");
    if (!kp || !ki || !kd || !ramp) {
      sendError("pidq_args");
      return;
    }
    motor1.currentLoop.configure(atof(kp), atof(ki), atof(kd), atof(ramp), voltageLimit);
    motor2.currentLoop.configure(atof(kp), atof(ki), atof(kd), atof(ramp), voltageLimit);
    sendAck("pidq");
    return;
  }
  if (textEquals(command, "PIDV")) {
    char *kp = strtok(nullptr, ",");
    char *ki = strtok(nullptr, ",");
    char *kd = strtok(nullptr, ",");
    char *ramp = strtok(nullptr, ",");
    char *limit = strtok(nullptr, ",");
    if (!kp || !ki || !kd || !ramp || !limit) {
      sendError("pidv_args");
      return;
    }
    const float outputLimit = clampf(atof(limit), 0.1f, currentLimit);
    motor1.velocityLoop.configure(atof(kp), atof(ki), atof(kd), atof(ramp), outputLimit);
    motor2.velocityLoop.configure(atof(kp), atof(ki), atof(kd), atof(ramp), outputLimit);
    sendAck("pidv");
    return;
  }
  if (textEquals(command, "PIDP") || textEquals(command, "PIDA")) {
    char *kp = strtok(nullptr, ",");
    char *ki = strtok(nullptr, ",");
    char *kd = strtok(nullptr, ",");
    char *ramp = strtok(nullptr, ",");
    char *limit = strtok(nullptr, ",");
    if (!kp || !ki || !kd || !ramp || !limit) {
      sendError("pidp_args");
      return;
    }
    const float outputLimit = clampf(atof(limit), 0.1f, velocityLimit);
    motor1.angleLoop.configure(atof(kp), atof(ki), atof(kd), atof(ramp), outputLimit);
    motor2.angleLoop.configure(atof(kp), atof(ki), atof(kd), atof(ramp), outputLimit);
    sendAck("pidp");
    return;
  }
  if (textEquals(command, "TQ2")) {
    char *m1Target = strtok(nullptr, ",");
    char *m1Enable = strtok(nullptr, ",");
    char *m2Target = strtok(nullptr, ",");
    char *m2Enable = strtok(nullptr, ",");
    if (!m1Target || !m1Enable || !m2Target || !m2Enable) {
      sendError("tq2_args");
      return;
    }
    stopAll(false);
    applySet(motor1, MODE_TORQUE, atof(m1Target), atoi(m1Enable) != 0);
    applySet(motor2, MODE_TORQUE, atof(m2Target), atoi(m2Enable) != 0);
    if (anyMotorEnabled()) enableDriver();
    sendAck("tq2");
    return;
  }
  if (textEquals(command, "LQR2")) {
    char *m1Target = strtok(nullptr, ",");
    char *m2Target = strtok(nullptr, ",");
    char *enable = strtok(nullptr, ",");
    if (!m1Target || !m2Target || !enable) {
      sendError("lqr2_args");
      return;
    }
    stopAll(false);
    const bool enabled = atoi(enable) != 0;
    applySet(motor1, MODE_TORQUE, atof(m1Target), enabled);
    applySet(motor2, MODE_TORQUE, atof(m2Target), enabled);
    if (enabled) enableDriver();
    sendAck("lqr2");
    return;
  }
  if (textEquals(command, "SET2")) {
    char *m1ModeText = strtok(nullptr, ",");
    char *m1Target = strtok(nullptr, ",");
    char *m1Enable = strtok(nullptr, ",");
    char *m2ModeText = strtok(nullptr, ",");
    char *m2Target = strtok(nullptr, ",");
    char *m2Enable = strtok(nullptr, ",");
    ControlMode m1Mode;
    ControlMode m2Mode;
    if (!m1ModeText || !m1Target || !m1Enable || !m2ModeText || !m2Target || !m2Enable ||
        !parseMode(m1ModeText, m1Mode) || !parseMode(m2ModeText, m2Mode)) {
      sendError("set2_args");
      return;
    }
    stopAll(false);
    applySet(motor1, m1Mode, atof(m1Target), atoi(m1Enable) != 0);
    applySet(motor2, m2Mode, atof(m2Target), atoi(m2Enable) != 0);
    if (anyMotorEnabled()) enableDriver();
    sendAck("set2");
    return;
  }
  sendError("unknown_cmd");
}

void pollSerial() {
  while (serialPort && serialPort->available()) {
    const char c = static_cast<char>(serialPort->read());
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuffer[lineLength] = '\0';
      if (lineLength > 0) {
        if (readCheckedPayload(lineBuffer)) {
          processCommand(lineBuffer);
        } else {
          sendError("bad_checksum");
        }
      }
      lineLength = 0;
      continue;
    }
    if (lineLength < sizeof(lineBuffer) - 1) {
      lineBuffer[lineLength++] = c;
    } else {
      lineLength = 0;
      sendError("line_too_long");
    }
  }
}

void updateMeasurements(float dt, uint32_t nowUs) {
  if (motor1.sensorReady) motor1.sensor.update(nowUs);
  if (motor2.sensorReady) motor2.sensor.update(nowUs);
  motor1.currentSense.update();
  motor2.currentSense.update();
  motor1.angle = motorAngle(motor1);
  motor2.angle = motorAngle(motor2);
  motor1.velocity = motor1.velocityFilter.update(motorVelocity(motor1), dt);
  motor2.velocity = motor2.velocityFilter.update(motorVelocity(motor2), dt);
  motor1.iqMeasured = motor1.currentSenseDir *
                      motor1.currentFilter.update(
                          motor1.currentSense.iq(electricalAngle(motor1)), dt);
  motor2.iqMeasured = motor2.currentSenseDir *
                      motor2.currentFilter.update(
                          motor2.currentSense.iq(electricalAngle(motor2)), dt);
  motor1.torqueMeasuredNm =
      motor1.actuatorDir * motor1.iqMeasured * motor1.torqueConstant;
  motor2.torqueMeasuredNm =
      motor2.actuatorDir * motor2.iqMeasured * motor2.torqueConstant;
}
}  // namespace

void DengFOCLibraryCurrentController::begin(
    Stream &serial,
    TwoWire &motor1Wire,
    TwoWire &motor2Wire) {
  serialPort = &serial;
  pinMode(DRIVER_ENABLE_PIN, OUTPUT);
  digitalWrite(DRIVER_ENABLE_PIN, LOW);
  configurePwm(motor1);
  configurePwm(motor2);
  neutralPwm(motor1);
  neutralPwm(motor2);
  motor1.currentSense.begin();
  motor2.currentSense.begin();
  motor1.sensorReady = motor1.sensor.begin(motor1Wire);
  motor2.sensorReady = motor2.sensor.begin(motor2Wire);
  motor1.logicalZeroAngle = motor1.encoderDir * motor1.sensor.angle();
  motor2.logicalZeroAngle = motor2.encoderDir * motor2.sensor.angle();
  lastCommandMs = millis();
  sendPacket("BOOT,DengFOC_Library_Current_Control_Integrated_GUI");
  sendPacket(motor1.sensorReady ? "OK,m1_sensor" : "ERR,m1_sensor");
  sendPacket(motor2.sensorReady ? "OK,m2_sensor" : "ERR,m2_sensor");
  sendConfig();
  sendPidConfig();
}

void DengFOCLibraryCurrentController::update() {
  pollSerial();
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();
  if (testMode != TestMode::Idle && nowMs - testStartMs > testDurationMs + watchdogMs) {
    finishTest("watchdog");
  }
  if (testMode == TestMode::Idle && anyMotorEnabled() &&
      nowMs - lastCommandMs > watchdogMs) {
    stopAll(false);
  }
  if (nowUs - lastControlUs >= CONTROL_PERIOD_US) {
    const float dt = lastControlUs == 0 ? 0.001f : (nowUs - lastControlUs) * 1e-6f;
    lastControlUs = nowUs;
    updateMeasurements(dt, nowUs);
    const bool overCurrent =
        (motor1.enabled && fabsf(motor1.iqMeasured) > currentLimit * 1.25f) ||
        (motor2.enabled && fabsf(motor2.iqMeasured) > currentLimit * 1.25f);
    if (overCurrent && testMode != TestMode::Idle) {
      estopLatched = true;
      finishTest("overcurrent");
      stopAll(true);
    } else if (overCurrent) {
      stopAll(true);
    } else if (testMode == TestMode::Idle) {
      updateNormalChannel(motor1, dt);
      updateNormalChannel(motor2, dt);
    } else {
      updateTest(dt);
    }
  }
  if (nowMs - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMs;
    sendControlTelemetry();
    sendDiagnosticTelemetry();
  }
}
