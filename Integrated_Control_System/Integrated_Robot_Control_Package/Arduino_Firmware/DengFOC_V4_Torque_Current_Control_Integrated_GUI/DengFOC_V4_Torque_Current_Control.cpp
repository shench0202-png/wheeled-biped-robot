#include "DengFOC_V4_Torque_Current_Control.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {
constexpr float DFOC_TWO_PI = 6.28318530718f;
constexpr float THREE_PI_OVER_TWO = 4.71238898038f;
constexpr float ONE_OVER_SQRT3 = 0.57735026919f;
constexpr float TWO_OVER_SQRT3 = 1.15470053838f;

float clampValue(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}
}

PIDLoop::PIDLoop(float kpValue, float kiValue, float kdValue, float rampValue, float limitValue)
    : kp(kpValue), ki(kiValue), kd(kdValue), ramp(rampValue), limit(limitValue) {}

float PIDLoop::update(float error, float dt) {
  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.001f;
  }

  const float proportional = kp * error;
  integral_ += ki * dt * 0.5f * (error + prevError_);
  integral_ = clampValue(integral_, -limit, limit);
  const float derivative = kd * (error - prevError_) / dt;

  float output = proportional + integral_ + derivative;
  output = clampValue(output, -limit, limit);

  if (ramp > 0.0f) {
    const float outputRate = (output - prevOutput_) / dt;
    if (outputRate > ramp) {
      output = prevOutput_ + ramp * dt;
    } else if (outputRate < -ramp) {
      output = prevOutput_ - ramp * dt;
    }
  }

  prevError_ = error;
  prevOutput_ = output;
  return output;
}

void PIDLoop::configure(float kpValue, float kiValue, float kdValue, float rampValue, float limitValue) {
  kp = kpValue;
  ki = kiValue;
  kd = kdValue;
  ramp = rampValue;
  limit = limitValue;
  reset();
}

void PIDLoop::reset() {
  integral_ = 0.0f;
  prevError_ = 0.0f;
  prevOutput_ = 0.0f;
}

LowPassFilter::LowPassFilter(float timeConstant) : tf_(timeConstant) {}

float LowPassFilter::update(float input, float dt) {
  if (!initialized_) {
    previous_ = input;
    initialized_ = true;
    return input;
  }
  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.001f;
  }
  const float alpha = tf_ / (tf_ + dt);
  previous_ = alpha * previous_ + (1.0f - alpha) * input;
  return previous_;
}

void LowPassFilter::reset(float value) {
  previous_ = value;
  initialized_ = true;
}

bool AS5600Sensor::begin(TwoWire &wire) {
  wire_ = &wire;
  prevUpdateUs_ = micros();
  wire_->beginTransmission(0x36);
  healthy_ = wire_->endTransmission() == 0;
  if (!healthy_) {
    return false;
  }
  update(prevUpdateUs_);
  prevMechanicalAngle_ = mechanicalAngle_;
  return true;
}

bool AS5600Sensor::update(uint32_t nowUs) {
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
  mechanicalAngle_ = (raw & 0x0FFF) * (DFOC_TWO_PI / 4096.0f);

  float delta = mechanicalAngle_ - prevMechanicalAngle_;
  if (delta > PI) {
    rotations_--;
    delta -= DFOC_TWO_PI;
  } else if (delta < -PI) {
    rotations_++;
    delta += DFOC_TWO_PI;
  }

  angle_ = rotations_ * DFOC_TWO_PI + mechanicalAngle_;
  const float dt = (nowUs - prevUpdateUs_) * 1e-6f;
  if (dt > 0.0f) {
    velocity_ = delta / dt;
  }
  prevMechanicalAngle_ = mechanicalAngle_;
  prevUpdateUs_ = nowUs;
  healthy_ = true;
  return true;
}

float AS5600Sensor::mechanicalAngle() const { return mechanicalAngle_; }
float AS5600Sensor::angle() const { return angle_; }
float AS5600Sensor::velocity() const { return velocity_; }
bool AS5600Sensor::healthy() const { return healthy_; }

InlineCurrentSense::InlineCurrentSense(int pinA, int pinB) : pinA_(pinA), pinB_(pinB) {}

void InlineCurrentSense::begin() {
  pinMode(pinA_, INPUT);
  pinMode(pinB_, INPUT);
  calibrate();
}

void InlineCurrentSense::calibrate() {
  constexpr int samples = 1000;
  offsetA_ = 0.0f;
  offsetB_ = 0.0f;
  for (int i = 0; i < samples; ++i) {
    offsetA_ += analogRead(pinA_) * (ADC_VOLTAGE / ADC_RESOLUTION);
    offsetB_ += analogRead(pinB_) * (ADC_VOLTAGE / ADC_RESOLUTION);
    delayMicroseconds(500);
  }
  offsetA_ /= samples;
  offsetB_ /= samples;
}

void InlineCurrentSense::update() {
  const float va = analogRead(pinA_) * (ADC_VOLTAGE / ADC_RESOLUTION);
  const float vb = analogRead(pinB_) * (ADC_VOLTAGE / ADC_RESOLUTION);
  currentA_ = (va - offsetA_) * -VOLTS_TO_AMPS;
  currentB_ = (vb - offsetB_) * -VOLTS_TO_AMPS;
}

float InlineCurrentSense::iq(float electricalAngle) const {
  const float iAlpha = currentA_;
  const float iBeta = ONE_OVER_SQRT3 * currentA_ + TWO_OVER_SQRT3 * currentB_;
  return iBeta * cosf(electricalAngle) - iAlpha * sinf(electricalAngle);
}

DengFOCTorqueCurrentController::MotorChannel::MotorChannel(
    int pwmAValue,
    int pwmBValue,
    int pwmCValue,
    int currentA,
    int currentB,
    int8_t commutationDirValue,
    int8_t encoderDirValue,
    int8_t velocityDirValue,
    int8_t currentSenseDirValue,
    int8_t actuatorDirValue)
    : pwmA(pwmAValue),
      pwmB(pwmBValue),
      pwmC(pwmCValue),
      commutationDir(commutationDirValue),
      encoderDir(encoderDirValue),
      velocityDir(velocityDirValue),
      currentSenseDir(currentSenseDirValue),
      actuatorDir(actuatorDirValue),
      currentSense(currentA, currentB),
      currentLoop(
          DengFOCTorqueCurrentController::DEFAULT_CURRENT_PID_KP,
          DengFOCTorqueCurrentController::DEFAULT_CURRENT_PID_KI,
          DengFOCTorqueCurrentController::DEFAULT_CURRENT_PID_KD,
          DengFOCTorqueCurrentController::DEFAULT_CURRENT_PID_RAMP,
          DengFOCTorqueCurrentController::DEFAULT_VOLTAGE_LIMIT),
      velocityLoop(
          DengFOCTorqueCurrentController::DEFAULT_VELOCITY_PID_KP,
          DengFOCTorqueCurrentController::DEFAULT_VELOCITY_PID_KI,
          DengFOCTorqueCurrentController::DEFAULT_VELOCITY_PID_KD,
          DengFOCTorqueCurrentController::DEFAULT_VELOCITY_PID_RAMP,
          DengFOCTorqueCurrentController::DEFAULT_CURRENT_LIMIT),
      angleLoop(
          DengFOCTorqueCurrentController::DEFAULT_ANGLE_PID_KP,
          DengFOCTorqueCurrentController::DEFAULT_ANGLE_PID_KI,
          DengFOCTorqueCurrentController::DEFAULT_ANGLE_PID_KD,
          DengFOCTorqueCurrentController::DEFAULT_ANGLE_PID_RAMP,
          DengFOCTorqueCurrentController::DEFAULT_VELOCITY_LIMIT),
      velocityFilter(0.04f),
      currentFilter(0.05f) {}

void DengFOCTorqueCurrentController::begin(Stream &serialPort, TwoWire &motor1Wire, TwoWire &motor2Wire) {
  serial_ = &serialPort;
  configurePwm(motor1_);
  configurePwm(motor2_);
  enableDriver();

  motor1_.currentSense.begin();
  motor2_.currentSense.begin();
  motor1_.sensorReady = motor1_.sensor.begin(motor1Wire);
  motor2_.sensorReady = motor2_.sensor.begin(motor2Wire);
  const uint32_t nowMs = millis();
  if (motor1_.sensorReady) {
    startAlignment(motor1_, nowMs);
  } else if (motor2_.sensorReady) {
    startAlignment(motor2_, nowMs);
  }
  if (!alignmentActive()) {
    disableDriver();
  }

  motor1_.logicalZeroAngle = motorAngle(motor1_);
  motor2_.logicalZeroAngle = motorAngle(motor2_);
  lastCommandMs_ = millis();
  sendPacket("BOOT,DengFOC_V4_Torque_Current_Control");
  sendPacket(motor1_.sensorReady ? "OK,m1_sensor" : "ERR,m1_sensor");
  sendPacket(motor2_.sensorReady ? "OK,m2_sensor" : "ERR,m2_sensor");
  sendPacket(motor1_.alignmentInProgress ? "OK,m1_align_pending" : (motor1_.aligned ? "OK,m1_align" : "ERR,m1_align"));
  sendPacket(motor2_.alignmentInProgress
                 ? "OK,m2_align_pending"
                 : (motor2_.aligned
                        ? "OK,m2_align"
                        : (motor2_.sensorReady && motor1_.alignmentInProgress
                               ? "OK,m2_align_wait"
                               : "ERR,m2_align")));
}

void DengFOCTorqueCurrentController::update() {
  pollSerial();

  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  const bool motor1Aligning = motor1_.alignmentInProgress;
  const bool motor2Aligning = motor2_.alignmentInProgress;
  if (motor1Aligning) {
    serviceAlignment(motor1_, nowMs, nowUs);
  } else if (motor2Aligning) {
    serviceAlignment(motor2_, nowMs, nowUs);
  }
  if (motor1Aligning && !motor1_.alignmentInProgress &&
      motor2_.sensorReady && !motor2_.aligned && !motor2_.alignmentInProgress) {
    startAlignment(motor2_, nowMs);
  }
  if (!alignmentActive() && !anyMotorEnabled()) {
    disableDriver();
  }

  if (anyMotorEnabled() && nowMs - lastCommandMs_ > watchdogMs_) {
    stopAll(false);
  }

  if (nowUs - lastControlUs_ >= CONTROL_PERIOD_US) {
    const float dt = lastControlUs_ == 0 ? 0.001f : (nowUs - lastControlUs_) * 1e-6f;
    lastControlUs_ = nowUs;

    if (motor1_.sensorReady) motor1_.sensor.update(nowUs);
    if (motor2_.sensorReady) motor2_.sensor.update(nowUs);
    motor1_.currentSense.update();
    motor2_.currentSense.update();

    motor1_.iqMeasured =
        motor1_.currentSenseDir * motor1_.currentFilter.update(motor1_.currentSense.iq(electricalAngle(motor1_)), dt);
    motor2_.iqMeasured =
        motor2_.currentSenseDir * motor2_.currentFilter.update(motor2_.currentSense.iq(electricalAngle(motor2_)), dt);
    motor1_.iqMeasuredLogical = motor1_.actuatorDir * motor1_.iqMeasured;
    motor2_.iqMeasuredLogical = motor2_.actuatorDir * motor2_.iqMeasured;
    motor1_.torqueMeasuredNm = currentToTorque(motor1_, motor1_.iqMeasuredLogical);
    motor2_.torqueMeasuredNm = currentToTorque(motor2_, motor2_.iqMeasuredLogical);
    motor1_.velocityMeasured = motor1_.velocityFilter.update(motor1_.velocityDir * motorVelocity(motor1_), dt);
    motor2_.velocityMeasured = motor2_.velocityFilter.update(motor2_.velocityDir * motorVelocity(motor2_), dt);

    const bool overCurrent =
        (motor1_.enabled && fabsf(motor1_.iqMeasured) > currentLimit_ * 1.25f) ||
        (motor2_.enabled && fabsf(motor2_.iqMeasured) > currentLimit_ * 1.25f);
    if (overCurrent) {
      estopLatched_ = true;
    }

    if (!alignmentActive()) {
      updateChannel(motor1_, dt, nowUs, overCurrent);
      updateChannel(motor2_, dt, nowUs, overCurrent);
    }
  }

  if (nowMs - lastTelemetryMs_ >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs_ = nowMs;
    sendTelemetry();
  }
}

float DengFOCTorqueCurrentController::clampf(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float DengFOCTorqueCurrentController::normalizeAngle(float angle) {
  float normalized = fmodf(angle, DFOC_TWO_PI);
  return normalized >= 0.0f ? normalized : normalized + DFOC_TWO_PI;
}

uint8_t DengFOCTorqueCurrentController::xorChecksum(const char *text) {
  uint8_t checksum = 0;
  while (*text) {
    checksum ^= static_cast<uint8_t>(*text++);
  }
  return checksum;
}

uint8_t DengFOCTorqueCurrentController::parseHexByte(const char *text) {
  uint8_t value = 0;
  for (uint8_t i = 0; i < 2 && text[i]; ++i) {
    value <<= 4;
    const char c = text[i];
    if (c >= '0' && c <= '9') value |= c - '0';
    else if (c >= 'A' && c <= 'F') value |= c - 'A' + 10;
    else if (c >= 'a' && c <= 'f') value |= c - 'a' + 10;
  }
  return value;
}

const char *DengFOCTorqueCurrentController::modeName(Mode mode) {
  switch (mode) {
    case MODE_TORQUE: return "torque";
    case MODE_VELOCITY: return "velocity";
    case MODE_POSITION: return "position";
    default: return "unknown";
  }
}

bool DengFOCTorqueCurrentController::parseMode(const char *text, Mode &mode) {
  if (!strcmp(text, "torque") || !strcmp(text, "TORQUE")) {
    mode = MODE_TORQUE;
    return true;
  }
  if (!strcmp(text, "velocity") || !strcmp(text, "VELOCITY")) {
    mode = MODE_VELOCITY;
    return true;
  }
  if (!strcmp(text, "position") || !strcmp(text, "POSITION") ||
      !strcmp(text, "angle") || !strcmp(text, "ANGLE")) {
    mode = MODE_POSITION;
    return true;
  }
  return false;
}

bool DengFOCTorqueCurrentController::parseMotorSelector(const char *text, uint8_t &index) {
  if (!strcmp(text, "m1") || !strcmp(text, "M1") || !strcmp(text, "0")) {
    index = 0;
    return true;
  }
  if (!strcmp(text, "m2") || !strcmp(text, "M2") || !strcmp(text, "1")) {
    index = 1;
    return true;
  }
  return false;
}

DengFOCTorqueCurrentController::MotorChannel &DengFOCTorqueCurrentController::motor(uint8_t index) {
  return index == 0 ? motor1_ : motor2_;
}

const DengFOCTorqueCurrentController::MotorChannel &DengFOCTorqueCurrentController::motor(uint8_t index) const {
  return index == 0 ? motor1_ : motor2_;
}

bool DengFOCTorqueCurrentController::anyMotorEnabled() const {
  return motor1_.enabled || motor2_.enabled;
}

bool DengFOCTorqueCurrentController::alignmentActive() const {
  return motor1_.alignmentInProgress || motor2_.alignmentInProgress;
}

bool DengFOCTorqueCurrentController::enabledSensorsHealthy() const {
  const bool m1Healthy =
      !motor1_.enabled || (motor1_.sensorReady && motor1_.aligned && motor1_.sensor.healthy());
  const bool m2Healthy =
      !motor2_.enabled || (motor2_.sensorReady && motor2_.aligned && motor2_.sensor.healthy());
  return m1Healthy && m2Healthy;
}

void DengFOCTorqueCurrentController::configurePwm(MotorChannel &channel) {
  pinMode(DRIVER_ENABLE_PIN, OUTPUT);
  ledcAttach(channel.pwmA, PWM_FREQ, PWM_BITS);
  ledcAttach(channel.pwmB, PWM_FREQ, PWM_BITS);
  ledcAttach(channel.pwmC, PWM_FREQ, PWM_BITS);
  setPwm(channel, SUPPLY_VOLTAGE * 0.5f, SUPPLY_VOLTAGE * 0.5f, SUPPLY_VOLTAGE * 0.5f);
}

void DengFOCTorqueCurrentController::enableDriver() {
  digitalWrite(DRIVER_ENABLE_PIN, HIGH);
}

void DengFOCTorqueCurrentController::disableDriver() {
  digitalWrite(DRIVER_ENABLE_PIN, LOW);
}

bool DengFOCTorqueCurrentController::alignSensor(MotorChannel &channel) {
  if (!channel.sensor.healthy()) {
    return false;
  }
  setPhaseVoltage(channel, 3.0f, THREE_PI_OVER_TWO);
  channel.sensor.update(micros());
  channel.zeroElectricalAngle = electricalAngle(channel);
  setPhaseVoltage(channel, 0.0f, THREE_PI_OVER_TWO);
  return true;
}

void DengFOCTorqueCurrentController::startAlignment(MotorChannel &channel, uint32_t nowMs) {
  channel.aligned = false;
  channel.alignmentInProgress = true;
  channel.alignmentStartMs = nowMs;
  enableDriver();
  setPhaseVoltage(channel, 3.0f, THREE_PI_OVER_TWO);
}

void DengFOCTorqueCurrentController::serviceAlignment(
    MotorChannel &channel,
    uint32_t nowMs,
    uint32_t nowUs) {
  if (!channel.alignmentInProgress) {
    return;
  }

  if (!channel.sensorReady || !channel.sensor.update(nowUs)) {
    channel.alignmentInProgress = false;
    channel.aligned = false;
    setPhaseVoltage(channel, 0.0f, THREE_PI_OVER_TWO);
    sendPacket(&channel == &motor1_ ? "ERR,m1_align" : "ERR,m2_align");
    return;
  }

  if (nowMs - channel.alignmentStartMs < ALIGNMENT_HOLD_MS) {
    setPhaseVoltage(channel, 3.0f, THREE_PI_OVER_TWO);
    return;
  }

  channel.zeroElectricalAngle = electricalAngle(channel);
  channel.aligned = true;
  channel.alignmentInProgress = false;
  setPhaseVoltage(channel, 0.0f, THREE_PI_OVER_TWO);
  sendPacket(&channel == &motor1_ ? "OK,m1_align" : "OK,m2_align");
}

float DengFOCTorqueCurrentController::electricalAngle(const MotorChannel &channel) const {
  return normalizeAngle(
      static_cast<float>(channel.commutationDir * POLE_PAIRS) *
          channel.sensor.mechanicalAngle() -
      channel.zeroElectricalAngle);
}

float DengFOCTorqueCurrentController::motorAngle(const MotorChannel &channel) const {
  return channel.encoderDir * channel.sensor.angle();
}

float DengFOCTorqueCurrentController::motorVelocity(const MotorChannel &channel) const {
  return channel.encoderDir * channel.sensor.velocity();
}

float DengFOCTorqueCurrentController::torqueLimitNm(const MotorChannel &channel) const {
  return currentLimit_ * channel.torqueConstantNmPerAmp;
}

float DengFOCTorqueCurrentController::torqueToCurrent(const MotorChannel &channel, float torqueNm) const {
  if (channel.torqueConstantNmPerAmp <= 0.0f) {
    return 0.0f;
  }
  return clampf(torqueNm / channel.torqueConstantNmPerAmp, -currentLimit_, currentLimit_);
}

float DengFOCTorqueCurrentController::currentToTorque(const MotorChannel &channel, float currentA) const {
  return currentA * channel.torqueConstantNmPerAmp;
}

void DengFOCTorqueCurrentController::setPhaseVoltage(MotorChannel &channel, float uq, float electricalAngleValue) {
  uq = clampf(uq, -voltageLimit_, voltageLimit_);
  const float uAlpha = -uq * sinf(electricalAngleValue);
  const float uBeta = uq * cosf(electricalAngleValue);
  const float ua = uAlpha + SUPPLY_VOLTAGE * 0.5f;
  const float ub = (sqrtf(3.0f) * uBeta - uAlpha) * 0.5f + SUPPLY_VOLTAGE * 0.5f;
  const float uc = (-uAlpha - sqrtf(3.0f) * uBeta) * 0.5f + SUPPLY_VOLTAGE * 0.5f;
  setPwm(channel, ua, ub, uc);
}

void DengFOCTorqueCurrentController::setPwm(MotorChannel &channel, float ua, float ub, float uc) {
  ua = clampf(ua, 0.0f, SUPPLY_VOLTAGE);
  ub = clampf(ub, 0.0f, SUPPLY_VOLTAGE);
  uc = clampf(uc, 0.0f, SUPPLY_VOLTAGE);
  ledcWrite(channel.pwmA, static_cast<uint32_t>(255.0f * ua / SUPPLY_VOLTAGE));
  ledcWrite(channel.pwmB, static_cast<uint32_t>(255.0f * ub / SUPPLY_VOLTAGE));
  ledcWrite(channel.pwmC, static_cast<uint32_t>(255.0f * uc / SUPPLY_VOLTAGE));
}

void DengFOCTorqueCurrentController::neutralPwm(MotorChannel &channel) {
  const float neutral = SUPPLY_VOLTAGE * 0.5f;
  setPwm(channel, neutral, neutral, neutral);
}

void DengFOCTorqueCurrentController::resetLoops(MotorChannel &channel) {
  channel.currentLoop.reset();
  channel.velocityLoop.reset();
  channel.angleLoop.reset();
  channel.currentFilter.reset(0.0f);
  channel.velocityFilter.reset(0.0f);
}

void DengFOCTorqueCurrentController::stopChannel(MotorChannel &channel) {
  channel.enabled = false;
  channel.commandTarget = 0.0f;
  channel.targetValue = 0.0f;
  channel.torqueTargetNm = 0.0f;
  channel.torqueMeasuredNm = 0.0f;
  channel.iqTarget = 0.0f;
  channel.iqTargetDrive = 0.0f;
  channel.uqCommand = 0.0f;
  channel.angleError = 0.0f;
  channel.velocityTarget = 0.0f;
  channel.velocityError = 0.0f;
  resetLoops(channel);
  neutralPwm(channel);
}

void DengFOCTorqueCurrentController::stopAll(bool latchEstop) {
  stopChannel(motor1_);
  stopChannel(motor2_);
  if (latchEstop) {
    estopLatched_ = true;
  }
  disableDriver();
}

void DengFOCTorqueCurrentController::updateChannel(
    MotorChannel &channel,
    float dt,
    uint32_t,
    bool globalOverCurrent) {
  if (!channel.enabled || estopLatched_ || !channel.sensorReady || !channel.aligned ||
      !channel.sensor.healthy() || globalOverCurrent) {
    channel.torqueTargetNm = 0.0f;
    channel.iqTarget = 0.0f;
    channel.iqTargetDrive = 0.0f;
    channel.uqCommand = 0.0f;
    setPhaseVoltage(channel, 0.0f, electricalAngle(channel));
    return;
  }

  if (channel.mode == MODE_TORQUE) {
    channel.angleError = 0.0f;
    channel.velocityTarget = 0.0f;
    channel.velocityError = 0.0f;
    channel.torqueTargetNm = clampf(channel.targetValue, -torqueLimitNm(channel), torqueLimitNm(channel));
    channel.iqTarget = torqueToCurrent(channel, channel.torqueTargetNm);
  } else if (channel.mode == MODE_VELOCITY) {
    channel.torqueTargetNm = 0.0f;
    channel.angleError = 0.0f;
    channel.velocityTarget = clampf(channel.targetValue, -velocityLimit_, velocityLimit_);
    channel.velocityError = channel.velocityTarget - channel.velocityMeasured;
    channel.iqTarget = channel.velocityLoop.update(channel.velocityError, dt);
  } else {
    channel.torqueTargetNm = 0.0f;
    channel.angleError = channel.targetValue - (motorAngle(channel) - channel.logicalZeroAngle);
    channel.velocityTarget = channel.angleLoop.update(channel.angleError, dt);
    channel.velocityTarget = clampf(channel.velocityTarget, -velocityLimit_, velocityLimit_);
    channel.velocityError = channel.velocityTarget - channel.velocityMeasured;
    channel.iqTarget = channel.velocityLoop.update(channel.velocityError, dt);
  }

  channel.iqTarget = clampf(channel.iqTarget, -currentLimit_, currentLimit_);
  channel.iqTargetDrive = channel.actuatorDir * channel.iqTarget;
  channel.uqCommand = channel.currentLoop.update(channel.iqTargetDrive - channel.iqMeasured, dt);
  channel.uqCommand = clampf(channel.uqCommand, -voltageLimit_, voltageLimit_);
  setPhaseVoltage(channel, channel.uqCommand, electricalAngle(channel));
}

void DengFOCTorqueCurrentController::applySet(MotorChannel &channel, Mode requestedMode, float target, bool enabled) {
  channel.mode = requestedMode;
  channel.commandTarget = target;
  if (channel.mode == MODE_TORQUE) {
    channel.targetValue = clampf(channel.commandTarget, -torqueLimitNm(channel), torqueLimitNm(channel));
    channel.torqueTargetNm = channel.targetValue;
  } else if (channel.mode == MODE_VELOCITY) {
    channel.targetValue = clampf(channel.commandTarget, -velocityLimit_, velocityLimit_);
    channel.torqueTargetNm = 0.0f;
  } else {
    channel.targetValue = channel.commandTarget;
    channel.torqueTargetNm = 0.0f;
  }
  channel.enabled = enabled;
  if (!enabled) {
    stopChannel(channel);
  }
}

void DengFOCTorqueCurrentController::applyTorqueSet(MotorChannel &channel, float torqueNm, bool enabled) {
  applySet(channel, MODE_TORQUE, torqueNm, enabled);
}

bool DengFOCTorqueCurrentController::configureTorqueConstant(const char *selector, const char *ktText) {
  if (!selector || !ktText) {
    return false;
  }

  const float kt = clampf(atof(ktText), 0.001f, 5.0f);
  if (!strcmp(selector, "all") || !strcmp(selector, "ALL")) {
    motor1_.torqueConstantNmPerAmp = kt;
    motor2_.torqueConstantNmPerAmp = kt;
    return true;
  }

  uint8_t index = 0;
  if (!parseMotorSelector(selector, index)) {
    return false;
  }
  motor(index).torqueConstantNmPerAmp = kt;
  return true;
}

void DengFOCTorqueCurrentController::configureAllPidQ(float kp, float ki, float kd, float ramp) {
  motor1_.currentLoop.configure(kp, ki, kd, ramp, voltageLimit_);
  motor2_.currentLoop.configure(kp, ki, kd, ramp, voltageLimit_);
}

void DengFOCTorqueCurrentController::configureAllPidV(float kp, float ki, float kd, float ramp, float limit) {
  const float clampedLimit = clampf(limit, 0.1f, currentLimit_);
  motor1_.velocityLoop.configure(kp, ki, kd, ramp, clampedLimit);
  motor2_.velocityLoop.configure(kp, ki, kd, ramp, clampedLimit);
}

void DengFOCTorqueCurrentController::configureAllPidP(float kp, float ki, float kd, float ramp, float limit) {
  const float clampedLimit = clampf(limit, 0.1f, velocityLimit_);
  motor1_.angleLoop.configure(kp, ki, kd, ramp, clampedLimit);
  motor2_.angleLoop.configure(kp, ki, kd, ramp, clampedLimit);
}

void DengFOCTorqueCurrentController::pollSerial() {
  while (serial_ && serial_->available()) {
    const char c = static_cast<char>(serial_->read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      lineBuffer_[lineLength_] = '\0';
      if (lineLength_ > 0) {
        processCommand(lineBuffer_);
      }
      lineLength_ = 0;
      continue;
    }
    if (lineLength_ < sizeof(lineBuffer_) - 1) {
      lineBuffer_[lineLength_++] = c;
    } else {
      lineLength_ = 0;
      sendPacket("ERR,line_too_long");
    }
  }
}

bool DengFOCTorqueCurrentController::readCheckedPayload(char *line) {
  char *star = strchr(line, '*');
  if (!star || strlen(star + 1) < 2) {
    return false;
  }
  *star = '\0';
  return xorChecksum(line) == parseHexByte(star + 1);
}

void DengFOCTorqueCurrentController::processCommand(char *line) {
  if (!readCheckedPayload(line)) {
    sendPacket("ERR,bad_checksum");
    return;
  }

  char *command = strtok(line, ",");
  if (!command) {
    return;
  }
  lastCommandMs_ = millis();

  if (!strcmp(command, "PING")) {
    sendPacket("OK,pong");
    return;
  }
  if (!strcmp(command, "CFG?")) {
    sendConfig();
    return;
  }
  if (!strcmp(command, "PID?")) {
    sendPidConfig();
    return;
  }
  if (!strcmp(command, "ESTOP")) {
    stopAll(true);
    sendPacket("OK,estop");
    return;
  }
  if (!strcmp(command, "CLR")) {
    estopLatched_ = false;
    sendPacket("OK,clear_fault");
    return;
  }
  if (!strcmp(command, "DUAL")) {
    sendPacket("OK,dual_always_on");
    return;
  }
  if (!strcmp(command, "ZERO")) {
    char *selector = strtok(nullptr, ",");
    if (!selector || !strcmp(selector, "all")) {
      motor1_.logicalZeroAngle = motorAngle(motor1_);
      motor2_.logicalZeroAngle = motorAngle(motor2_);
      if (motor1_.mode == MODE_POSITION) {
        motor1_.commandTarget = 0.0f;
        motor1_.targetValue = 0.0f;
      }
      if (motor2_.mode == MODE_POSITION) {
        motor2_.commandTarget = 0.0f;
        motor2_.targetValue = 0.0f;
      }
      sendPacket("OK,zero_all");
      return;
    }
    uint8_t index = 0;
    if (!parseMotorSelector(selector, index)) {
      sendPacket("ERR,zero_args");
      return;
    }
    MotorChannel &channel = motor(index);
    channel.logicalZeroAngle = motorAngle(channel);
    if (channel.mode == MODE_POSITION) {
      channel.commandTarget = 0.0f;
      channel.targetValue = 0.0f;
    }
    sendPacket(index == 0 ? "OK,zero_m1" : "OK,zero_m2");
    return;
  }
  if (!strcmp(command, "KT")) {
    char *selector = strtok(nullptr, ",");
    char *kt = strtok(nullptr, ",");
    if (!configureTorqueConstant(selector, kt)) {
      sendPacket("ERR,kt_args");
      return;
    }
    sendPacket("OK,kt");
    return;
  }
  if (!strcmp(command, "TQ")) {
    char *first = strtok(nullptr, ",");
    if (!first) {
      sendPacket("ERR,tq_args");
      return;
    }

    uint8_t index = 0;
    if (parseMotorSelector(first, index)) {
      char *torqueText = strtok(nullptr, ",");
      char *enableText = strtok(nullptr, ",");
      if (!torqueText || !enableText) {
        sendPacket("ERR,tq_args");
        return;
      }
      applyTorqueSet(motor(index), atof(torqueText), atoi(enableText) != 0);
      if (anyMotorEnabled()) {
        estopLatched_ = false;
        enableDriver();
      } else {
        disableDriver();
      }
      sendPacket(index == 0 ? "OK,tq_m1" : "OK,tq_m2");
      return;
    }

    char *enableText = strtok(nullptr, ",");
    if (!enableText) {
      sendPacket("ERR,tq_args");
      return;
    }
    const float torqueNm = atof(first);
    const bool enabled = atoi(enableText) != 0;
    applyTorqueSet(motor1_, torqueNm, enabled);
    applyTorqueSet(motor2_, torqueNm, enabled);
    if (enabled) {
      estopLatched_ = false;
      enableDriver();
    } else {
      disableDriver();
    }
    sendPacket("OK,tq_all");
    return;
  }
  if (!strcmp(command, "TQ2")) {
    char *m1TorqueText = strtok(nullptr, ",");
    char *m1EnableText = strtok(nullptr, ",");
    char *m2TorqueText = strtok(nullptr, ",");
    char *m2EnableText = strtok(nullptr, ",");
    if (!m1TorqueText || !m1EnableText || !m2TorqueText || !m2EnableText) {
      sendPacket("ERR,tq2_args");
      return;
    }
    applyTorqueSet(motor1_, atof(m1TorqueText), atoi(m1EnableText) != 0);
    applyTorqueSet(motor2_, atof(m2TorqueText), atoi(m2EnableText) != 0);
    if (anyMotorEnabled()) {
      estopLatched_ = false;
      enableDriver();
    } else {
      disableDriver();
    }
    sendPacket("OK,tq2");
    return;
  }
  if (!strcmp(command, "LQR2")) {
    char *m1TorqueText = strtok(nullptr, ",");
    char *m2TorqueText = strtok(nullptr, ",");
    char *enableText = strtok(nullptr, ",");
    if (!m1TorqueText || !m2TorqueText || !enableText) {
      sendPacket("ERR,lqr2_args");
      return;
    }
    const bool enabled = atoi(enableText) != 0;
    applyTorqueSet(motor1_, atof(m1TorqueText), enabled);
    applyTorqueSet(motor2_, atof(m2TorqueText), enabled);
    if (anyMotorEnabled()) {
      estopLatched_ = false;
      enableDriver();
    } else {
      disableDriver();
    }
    sendPacket("OK,lqr2");
    return;
  }
  if (!strcmp(command, "SET")) {
    char *first = strtok(nullptr, ",");
    if (!first) {
      sendPacket("ERR,set_args");
      return;
    }

    uint8_t index = 0;
    if (parseMotorSelector(first, index)) {
      char *modeText = strtok(nullptr, ",");
      char *targetText = strtok(nullptr, ",");
      char *enableText = strtok(nullptr, ",");
      Mode requestedMode;
      if (!modeText || !targetText || !enableText || !parseMode(modeText, requestedMode)) {
        sendPacket("ERR,set_args");
        return;
      }
      applySet(motor(index), requestedMode, atof(targetText), atoi(enableText) != 0);
      if (anyMotorEnabled()) {
        estopLatched_ = false;
        enableDriver();
      } else {
        disableDriver();
      }
      sendPacket(index == 0 ? "OK,set_m1" : "OK,set_m2");
      return;
    }

    char *targetText = strtok(nullptr, ",");
    char *enableText = strtok(nullptr, ",");
    Mode requestedMode;
    if (!targetText || !enableText || !parseMode(first, requestedMode)) {
      sendPacket("ERR,set_args");
      return;
    }
    const float target = atof(targetText);
    const bool enabled = atoi(enableText) != 0;
    applySet(motor1_, requestedMode, target, enabled);
    applySet(motor2_, requestedMode, target, enabled);
    if (enabled) {
      estopLatched_ = false;
      enableDriver();
    } else {
      disableDriver();
    }
    sendPacket("OK,set_all");
    return;
  }
  if (!strcmp(command, "SET2")) {
    char *m1ModeText = strtok(nullptr, ",");
    char *m1TargetText = strtok(nullptr, ",");
    char *m1EnableText = strtok(nullptr, ",");
    char *m2ModeText = strtok(nullptr, ",");
    char *m2TargetText = strtok(nullptr, ",");
    char *m2EnableText = strtok(nullptr, ",");
    Mode m1Mode;
    Mode m2Mode;
    if (!m1ModeText || !m1TargetText || !m1EnableText ||
        !m2ModeText || !m2TargetText || !m2EnableText ||
        !parseMode(m1ModeText, m1Mode) || !parseMode(m2ModeText, m2Mode)) {
      sendPacket("ERR,set2_args");
      return;
    }
    applySet(motor1_, m1Mode, atof(m1TargetText), atoi(m1EnableText) != 0);
    applySet(motor2_, m2Mode, atof(m2TargetText), atoi(m2EnableText) != 0);
    if (anyMotorEnabled()) {
      estopLatched_ = false;
      enableDriver();
    } else {
      disableDriver();
    }
    sendPacket("OK,set2");
    return;
  }
  if (!strcmp(command, "SAFE")) {
    char *voltage = strtok(nullptr, ",");
    char *current = strtok(nullptr, ",");
    char *velocity = strtok(nullptr, ",");
    char *watchdog = strtok(nullptr, ",");
    if (!voltage || !current || !velocity || !watchdog) {
      sendPacket("ERR,safe_args");
      return;
    }
    voltageLimit_ = clampf(atof(voltage), 0.1f, SUPPLY_VOLTAGE * 0.9f);
    currentLimit_ = clampf(atof(current), 0.1f, 12.0f);
    velocityLimit_ = clampf(atof(velocity), 0.1f, 300.0f);
    watchdogMs_ = static_cast<uint32_t>(clampf(atof(watchdog), 50.0f, 5000.0f));
    motor1_.currentLoop.limit = voltageLimit_;
    motor2_.currentLoop.limit = voltageLimit_;
    motor1_.velocityLoop.limit = currentLimit_;
    motor2_.velocityLoop.limit = currentLimit_;
    motor1_.angleLoop.limit = velocityLimit_;
    motor2_.angleLoop.limit = velocityLimit_;
    sendPacket("OK,safe");
    return;
  }
  if (!strcmp(command, "PIDQ")) {
    char *kp = strtok(nullptr, ",");
    char *ki = strtok(nullptr, ",");
    char *kd = strtok(nullptr, ",");
    char *ramp = strtok(nullptr, ",");
    if (!kp || !ki || !kd || !ramp) {
      sendPacket("ERR,pidq_args");
      return;
    }
    configureAllPidQ(atof(kp), atof(ki), atof(kd), atof(ramp));
    sendPacket("OK,pidq");
    return;
  }
  if (!strcmp(command, "PIDV")) {
    char *kp = strtok(nullptr, ",");
    char *ki = strtok(nullptr, ",");
    char *kd = strtok(nullptr, ",");
    char *ramp = strtok(nullptr, ",");
    char *limit = strtok(nullptr, ",");
    if (!kp || !ki || !kd || !ramp || !limit) {
      sendPacket("ERR,pidv_args");
      return;
    }
    configureAllPidV(atof(kp), atof(ki), atof(kd), atof(ramp), atof(limit));
    sendPacket("OK,pidv");
    return;
  }
  if (!strcmp(command, "PIDP") || !strcmp(command, "PIDA")) {
    char *kp = strtok(nullptr, ",");
    char *ki = strtok(nullptr, ",");
    char *kd = strtok(nullptr, ",");
    char *ramp = strtok(nullptr, ",");
    char *limit = strtok(nullptr, ",");
    if (!kp || !ki || !kd || !ramp || !limit) {
      sendPacket("ERR,pidp_args");
      return;
    }
    configureAllPidP(atof(kp), atof(ki), atof(kd), atof(ramp), atof(limit));
    sendPacket("OK,pidp");
    return;
  }

  sendPacket("ERR,unknown_cmd");
}

void DengFOCTorqueCurrentController::sendPacket(const char *payload) {
  if (!serial_) {
    return;
  }
  char packet[600];
  snprintf(packet, sizeof(packet), "%s*%02X", payload, xorChecksum(payload));
  serial_->println(packet);
}

void DengFOCTorqueCurrentController::sendConfig() {
  char payload[260];
  snprintf(
      payload,
      sizeof(payload),
      "CFG,%.5f,%.5f,%.5f,%lu,%.5f,%.5f,%u",
      voltageLimit_,
      currentLimit_,
      velocityLimit_,
      static_cast<unsigned long>(watchdogMs_),
      motor1_.torqueConstantNmPerAmp,
      motor2_.torqueConstantNmPerAmp,
      faultMask());
  sendPacket(payload);
}

void DengFOCTorqueCurrentController::sendPidConfig() {
  char payload[360];
  snprintf(
      payload,
      sizeof(payload),
      "PID,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
      motor1_.currentLoop.kp,
      motor1_.currentLoop.ki,
      motor1_.currentLoop.kd,
      motor1_.currentLoop.ramp,
      motor1_.currentLoop.limit,
      motor1_.velocityLoop.kp,
      motor1_.velocityLoop.ki,
      motor1_.velocityLoop.kd,
      motor1_.velocityLoop.ramp,
      motor1_.velocityLoop.limit,
      motor1_.angleLoop.kp,
      motor1_.angleLoop.ki,
      motor1_.angleLoop.kd,
      motor1_.angleLoop.ramp,
      motor1_.angleLoop.limit);
  sendPacket(payload);
}

void DengFOCTorqueCurrentController::sendTelemetry() {
  char payload[560];
  const uint16_t faults = faultMask();
  snprintf(
      payload,
      sizeof(payload),
      "TQC2,%lu,%u,%s,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%s,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%u",
      millis(),
      anyMotorEnabled() ? 1 : 0,
      modeName(motor1_.mode),
      motor1_.commandTarget,
      motorAngle(motor1_) - motor1_.logicalZeroAngle,
      electricalAngle(motor1_),
      motor1_.velocityMeasured,
      motor1_.torqueMeasuredNm,
      motor1_.iqMeasuredLogical,
      motor1_.iqTarget,
      motor1_.uqCommand,
      motor1_.torqueConstantNmPerAmp,
      torqueLimitNm(motor1_),
      modeName(motor2_.mode),
      motor2_.commandTarget,
      motorAngle(motor2_) - motor2_.logicalZeroAngle,
      electricalAngle(motor2_),
      motor2_.velocityMeasured,
      motor2_.torqueMeasuredNm,
      motor2_.iqMeasuredLogical,
      motor2_.iqTarget,
      motor2_.uqCommand,
      motor2_.torqueConstantNmPerAmp,
      torqueLimitNm(motor2_),
      voltageLimit_,
      currentLimit_,
      faults);
  sendPacket(payload);
}

uint16_t DengFOCTorqueCurrentController::faultMask() const {
  uint16_t faults = 0;
  if (estopLatched_) faults |= FAULT_ESTOP;
  if (anyMotorEnabled() && millis() - lastCommandMs_ > watchdogMs_) faults |= FAULT_WATCHDOG;
  if (!anyMotorEnabled()) faults |= FAULT_DISABLED;
  if (!enabledSensorsHealthy()) faults |= FAULT_SENSOR;
  if ((motor1_.enabled && fabsf(motor1_.iqMeasured) > currentLimit_ * 1.25f) ||
      (motor2_.enabled && fabsf(motor2_.iqMeasured) > currentLimit_ * 1.25f)) {
    faults |= FAULT_OVERCURRENT;
  }
  return faults;
}
