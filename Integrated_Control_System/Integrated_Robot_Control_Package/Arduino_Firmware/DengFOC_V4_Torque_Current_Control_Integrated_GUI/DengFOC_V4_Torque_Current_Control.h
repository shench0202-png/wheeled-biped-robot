#ifndef DENGFOC_V4_TORQUE_CURRENT_CONTROL_H
#define DENGFOC_V4_TORQUE_CURRENT_CONTROL_H

#include <Arduino.h>
#include <Wire.h>

class PIDLoop {
public:
  PIDLoop(float kp, float ki, float kd, float ramp, float limit);
  float update(float error, float dt);
  void configure(float kp, float ki, float kd, float ramp, float limit);
  void reset();

  float kp;
  float ki;
  float kd;
  float ramp;
  float limit;

private:
  float integral_ = 0.0f;
  float prevError_ = 0.0f;
  float prevOutput_ = 0.0f;
};

class LowPassFilter {
public:
  explicit LowPassFilter(float timeConstant);
  float update(float input, float dt);
  void reset(float value = 0.0f);

private:
  float tf_;
  float previous_ = 0.0f;
  bool initialized_ = false;
};

class AS5600Sensor {
public:
  bool begin(TwoWire &wire);
  bool update(uint32_t nowUs);
  float mechanicalAngle() const;
  float angle() const;
  float velocity() const;
  bool healthy() const;

private:
  TwoWire *wire_ = nullptr;
  float mechanicalAngle_ = 0.0f;
  float angle_ = 0.0f;
  float velocity_ = 0.0f;
  float prevMechanicalAngle_ = 0.0f;
  int32_t rotations_ = 0;
  uint32_t prevUpdateUs_ = 0;
  bool healthy_ = false;
};

class InlineCurrentSense {
public:
  InlineCurrentSense(int pinA, int pinB);
  void begin();
  void calibrate();
  void update();
  float iq(float electricalAngle) const;

private:
  static constexpr float ADC_VOLTAGE = 3.3f;
  static constexpr float ADC_RESOLUTION = 4095.0f;
  static constexpr float SHUNT_RESISTOR = 0.01f;
  static constexpr float AMP_GAIN = 50.0f;
  static constexpr float VOLTS_TO_AMPS = 1.0f / SHUNT_RESISTOR / AMP_GAIN;

  int pinA_;
  int pinB_;
  float offsetA_ = 0.0f;
  float offsetB_ = 0.0f;
  float currentA_ = 0.0f;
  float currentB_ = 0.0f;
};

class DengFOCTorqueCurrentController {
public:
  void begin(Stream &serialPort, TwoWire &motor1Wire, TwoWire &motor2Wire);
  void update();

private:
  enum Mode : uint8_t {
    MODE_TORQUE = 0,
    MODE_VELOCITY = 1,
    MODE_POSITION = 2,
  };

  struct MotorChannel {
    MotorChannel(
        int pwmA,
        int pwmB,
        int pwmC,
        int currentA,
        int currentB,
        int8_t commutationDir,
        int8_t encoderDir,
        int8_t velocityDir,
        int8_t currentSenseDir,
        int8_t actuatorDir);

    int pwmA;
    int pwmB;
    int pwmC;
    int8_t commutationDir;
    int8_t encoderDir;
    int8_t velocityDir;
    int8_t currentSenseDir;
    int8_t actuatorDir;

    AS5600Sensor sensor;
    InlineCurrentSense currentSense;
    PIDLoop currentLoop;
    PIDLoop velocityLoop;
    PIDLoop angleLoop;
    LowPassFilter velocityFilter;
    LowPassFilter currentFilter;

    Mode mode = MODE_TORQUE;
    bool enabled = false;
    bool sensorReady = false;
    bool aligned = false;
    bool alignmentInProgress = false;
    uint32_t alignmentStartMs = 0;
    float zeroElectricalAngle = 0.0f;
    float logicalZeroAngle = 0.0f;
    float commandTarget = 0.0f;
    float targetValue = 0.0f;
    float torqueTargetNm = 0.0f;
    float torqueMeasuredNm = 0.0f;
    float torqueConstantNmPerAmp = DEFAULT_TORQUE_CONSTANT_NM_PER_A;
    float iqMeasured = 0.0f;
    float iqMeasuredLogical = 0.0f;
    float velocityMeasured = 0.0f;
    float iqTarget = 0.0f;
    float iqTargetDrive = 0.0f;
    float uqCommand = 0.0f;
    float angleError = 0.0f;
    float velocityTarget = 0.0f;
    float velocityError = 0.0f;
  };

  static constexpr int DRIVER_ENABLE_PIN = 12;
  static constexpr int PWM_BITS = 8;
  static constexpr int PWM_FREQ = 30000;
  static constexpr uint8_t POLE_PAIRS = 7;
  static constexpr float SUPPLY_VOLTAGE = 12.6f;
  static constexpr float DEFAULT_VOLTAGE_LIMIT = 3.0f;
  static constexpr float DEFAULT_CURRENT_LIMIT = 3.0f;
  static constexpr float DEFAULT_VELOCITY_LIMIT = 20.0f;
  static constexpr float DEFAULT_TORQUE_CONSTANT_NM_PER_A = 0.0955f;

  // Tune these preset PID values before flashing. UART PIDQ/PIDV/PIDP
  // commands can still override them at runtime.
  static constexpr float DEFAULT_CURRENT_PID_KP = 5.0f;
  static constexpr float DEFAULT_CURRENT_PID_KI = 80.0f;
  static constexpr float DEFAULT_CURRENT_PID_KD = 0.0f;
  static constexpr float DEFAULT_CURRENT_PID_RAMP = 100000.0f;

  static constexpr float DEFAULT_VELOCITY_PID_KP = 0.20f;
  static constexpr float DEFAULT_VELOCITY_PID_KI = 4.0f;
  static constexpr float DEFAULT_VELOCITY_PID_KD = 0.0005f;
  static constexpr float DEFAULT_VELOCITY_PID_RAMP = 1000.0f;

  static constexpr float DEFAULT_ANGLE_PID_KP = 8.0f;
  static constexpr float DEFAULT_ANGLE_PID_KI = 0.0f;
  static constexpr float DEFAULT_ANGLE_PID_KD = 0.0f;
  static constexpr float DEFAULT_ANGLE_PID_RAMP = 100000.0f;

  static constexpr uint32_t CONTROL_PERIOD_US = 1000;
  static constexpr uint32_t TELEMETRY_PERIOD_MS = 20;
  static constexpr uint32_t ALIGNMENT_HOLD_MS = 700;

  static constexpr uint16_t FAULT_ESTOP = 1u << 0;
  static constexpr uint16_t FAULT_WATCHDOG = 1u << 1;
  static constexpr uint16_t FAULT_DISABLED = 1u << 2;
  static constexpr uint16_t FAULT_SENSOR = 1u << 3;
  static constexpr uint16_t FAULT_OVERCURRENT = 1u << 4;

  Stream *serial_ = nullptr;
  MotorChannel motor1_{32, 33, 25, 39, 36, -1, -1, 1, -1, 1};
  MotorChannel motor2_{26, 27, 14, 35, 34, -1, -1, 1, -1, 1};
  bool estopLatched_ = false;
  float currentLimit_ = DEFAULT_CURRENT_LIMIT;
  float voltageLimit_ = DEFAULT_VOLTAGE_LIMIT;
  float velocityLimit_ = DEFAULT_VELOCITY_LIMIT;
  uint32_t watchdogMs_ = 500;
  uint32_t lastCommandMs_ = 0;
  uint32_t lastTelemetryMs_ = 0;
  uint32_t lastControlUs_ = 0;
  char lineBuffer_[220] = {};
  uint8_t lineLength_ = 0;

  static float clampf(float value, float low, float high);
  static float normalizeAngle(float angle);
  static uint8_t xorChecksum(const char *text);
  static uint8_t parseHexByte(const char *text);
  static const char *modeName(Mode mode);
  static bool parseMode(const char *text, Mode &mode);
  static bool parseMotorSelector(const char *text, uint8_t &index);

  MotorChannel &motor(uint8_t index);
  const MotorChannel &motor(uint8_t index) const;
  bool anyMotorEnabled() const;
  bool alignmentActive() const;
  bool enabledSensorsHealthy() const;
  void configurePwm(MotorChannel &channel);
  void enableDriver();
  void disableDriver();
  bool alignSensor(MotorChannel &channel);
  void startAlignment(MotorChannel &channel, uint32_t nowMs);
  void serviceAlignment(MotorChannel &channel, uint32_t nowMs, uint32_t nowUs);
  float electricalAngle(const MotorChannel &channel) const;
  float motorAngle(const MotorChannel &channel) const;
  float motorVelocity(const MotorChannel &channel) const;
  float torqueLimitNm(const MotorChannel &channel) const;
  float torqueToCurrent(const MotorChannel &channel, float torqueNm) const;
  float currentToTorque(const MotorChannel &channel, float currentA) const;
  void setPhaseVoltage(MotorChannel &channel, float uq, float electricalAngle);
  void neutralPwm(MotorChannel &channel);
  void setPwm(MotorChannel &channel, float ua, float ub, float uc);
  void stopChannel(MotorChannel &channel);
  void stopAll(bool latchEstop);
  void resetLoops(MotorChannel &channel);
  void updateChannel(MotorChannel &channel, float dt, uint32_t nowUs, bool globalOverCurrent);
  void applySet(MotorChannel &channel, Mode mode, float target, bool enabled);
  void applyTorqueSet(MotorChannel &channel, float torqueNm, bool enabled);
  bool configureTorqueConstant(const char *selector, const char *ktText);
  void configureAllPidQ(float kp, float ki, float kd, float ramp);
  void configureAllPidV(float kp, float ki, float kd, float ramp, float limit);
  void configureAllPidP(float kp, float ki, float kd, float ramp, float limit);
  void pollSerial();
  bool readCheckedPayload(char *line);
  void processCommand(char *line);
  void sendPacket(const char *payload);
  void sendConfig();
  void sendPidConfig();
  void sendTelemetry();
  uint16_t faultMask() const;
};

#endif
