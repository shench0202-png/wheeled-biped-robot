#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#ifdef ESP32
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif
#include <math.h>
#include <string.h>
#include <Wire.h>

#include "DataJson.h"
#include "EKF_Quaternion.h"
#include "LQRController.h"
#include "Receive.h"

Servo right_front_Servo;
Servo right_back_Servo;
Servo left_front_Servo;
Servo left_back_Servo;

struct ServoChannel {
  const char *name;
  uint8_t pin;
  Servo *servo;
  int angle;
  bool attached;
};

struct LegAngles {
  float alpha;
  float beta;
};

#ifdef ESP32
constexpr uint8_t RIGHT_FRONT_PIN = 25;
constexpr uint8_t RIGHT_BACK_PIN = 26;
constexpr uint8_t LEFT_FRONT_PIN = 27;
constexpr uint8_t LEFT_BACK_PIN = 14;
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t DEBUG_RX_PIN = 16;
constexpr uint8_t DEBUG_TX_PIN = 17;
#elif defined(ARDUINO_TEENSY41)
constexpr uint8_t RIGHT_FRONT_PIN = 20;
constexpr uint8_t RIGHT_BACK_PIN = 21;
constexpr uint8_t LEFT_FRONT_PIN = 22;
constexpr uint8_t LEFT_BACK_PIN = 23;
constexpr uint8_t I2C_SDA_PIN = 18;
constexpr uint8_t I2C_SCL_PIN = 19;
constexpr uint8_t DEBUG_RX_PIN = 7;   // Serial2 RX on Teensy 4.1
constexpr uint8_t DEBUG_TX_PIN = 8;   // Serial2 TX on Teensy 4.1
#else
constexpr uint8_t RIGHT_FRONT_PIN = 20;
constexpr uint8_t RIGHT_BACK_PIN = 21;
constexpr uint8_t LEFT_FRONT_PIN = 22;
constexpr uint8_t LEFT_BACK_PIN = 23;
#endif

constexpr int DEFAULT_SERVO_ANGLE = 90;
constexpr int MIN_SERVO_ANGLE = 0;
constexpr int MAX_SERVO_ANGLE = 180;
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;

constexpr float L5 = 40.0f;
constexpr float L1 = 60.0f;
constexpr float L2 = 100.0f;
constexpr float L4 = 60.0f;
constexpr float L3 = 100.0f;
constexpr float TARGET_X_MM = L5 / 2.0f;
constexpr float MIN_TARGET_X_MM = 1.0f;
constexpr float MAX_TARGET_X_MM = L5 - 1.0f;
constexpr float DEFAULT_HEIGHT_MM = 70.0f;
constexpr float MIN_HEIGHT_MM = 70.0f;
constexpr float MAX_HEIGHT_MM = 155.0f;
constexpr uint32_t MOVEMENT_DURATION_MS = 500;

float LEFT_FRONT_OFFSET = 90.0f;
float LEFT_BACK_OFFSET = 90.0f;
float RIGHT_FRONT_OFFSET = 90.0f;
float RIGHT_BACK_OFFSET = 90.0f;

int LEFT_FRONT_DIR = 1;
int LEFT_BACK_DIR = -1;
int RIGHT_FRONT_DIR = -1;
int RIGHT_BACK_DIR = 1;

ServoChannel channels[] = {
    {"RF", RIGHT_FRONT_PIN, &right_front_Servo, DEFAULT_SERVO_ANGLE, false},
    {"RB", RIGHT_BACK_PIN, &right_back_Servo, DEFAULT_SERVO_ANGLE, false},
    {"LF", LEFT_FRONT_PIN, &left_front_Servo, DEFAULT_SERVO_ANGLE, false},
    {"LB", LEFT_BACK_PIN, &left_back_Servo, DEFAULT_SERVO_ANGLE, false},
};

constexpr uint8_t CHANNEL_COUNT = sizeof(channels) / sizeof(channels[0]);

EKF_Quaternion ekf;
Receive imu;
LQRController lqr;

float left_h_goal = DEFAULT_HEIGHT_MM;
float right_h_goal = DEFAULT_HEIGHT_MM;
float left_h = DEFAULT_HEIGHT_MM;
float right_h = DEFAULT_HEIGHT_MM;
float left_h_start = DEFAULT_HEIGHT_MM;
float right_h_start = DEFAULT_HEIGHT_MM;
float wheel_x_goal = TARGET_X_MM;
float wheel_x = TARGET_X_MM;
float wheel_x_start = TARGET_X_MM;
LegAngles leftAngles = {0.0f, 0.0f};
LegAngles rightAngles = {0.0f, 0.0f};
bool lastIkOk = false;
bool heightMoveActive = false;
uint32_t heightMoveStartMs = 0;
bool servo_enabled = false;
bool lqr_enabled = false;
float target_pitch_deg = 0.0f;
bool lqr_has_wheel_state = false;
float lqr_position_m = 0.0f;
float lqr_velocity_mps = 0.0f;
float lqr_target_position_m = 0.0f;
float lqr_target_velocity_mps = 0.0f;
float lqr_torque_cmd = 0.0f;
float latest_rates[3] = {0.0f, 0.0f, 0.0f};

const uint32_t GUI_UART_BAUD = 115200;
const uint32_t DEBUG_UART_BAUD = 115200;
DataJson receiver(Serial, GUI_UART_BAUD);
SensorData myData;

const int filtered_mode = 3;
const unsigned long imu_interval = 10;   // 100 Hz
const unsigned long gui_interval = 10;   // 100 Hz over direct 115200-baud UART

const uint8_t PACKET_HEADER_1 = 0xAA;
const uint8_t PACKET_HEADER_2 = 0x55;
const uint8_t PACKET_TYPE_TELEMETRY = 0x01;
const uint8_t PACKET_TYPE_IMU_STATUS = 0x02;
const uint8_t PACKET_TYPE_LQR = 0x03;
const uint8_t TELEMETRY_PAYLOAD_LEN = 14;  // 3 float32 + motion flag + mag interference flag
const uint8_t IMU_STATUS_PAYLOAD_LEN = 1;
const uint8_t LQR_PAYLOAD_LEN = 41;        // 10 float32 + enabled flag


unsigned long last_imu_time = 0;
unsigned long last_gui_time = 0;
unsigned long last_imu_status_time = 0;
unsigned long last_lqr_debug_time = 0;

int still_count = 0;
float yaw_offset = 0.0f;
float yaw_sum = 0.0f;
int yaw_offset_count = 0;
bool yaw_offset_set = false;
bool collecting_yaw_offset = false;

float latest_angles[3] = {0.0f, 0.0f, 0.0f};
String latest_state = "MOVING";

void servo_setup();
void servo_control();
void attachChannel(uint8_t index);
void writeChannelAngle(uint8_t index, float angle);
void attachAll();
void setTargetPose(float left_height_mm, float right_height_mm, float wheel_x_mm);
bool solveAlpha(float x, float y, float &alpha1, float &alpha2);
bool solveBeta(float x, float y, float &beta1, float &beta2);
bool inverseKinematics(float x, float y, float &alpha, float &beta);
void setLegServo(float left_alpha, float left_beta, float right_alpha, float right_beta);
float radToDeg(float rad);
float normalizeDegrees360(float angle_deg);
float normalizeDegrees180(float angle_deg);
float clampFloat(float value, float low, float high);
void update_lqr_control(unsigned long now);
bool data_get();
void update_imu(float dt);
void send_data_to_gui();
void sendTelemetryPacket(Print &out);
void sendLqrPacket(Print &out);
void sendImuStatusPacket(Print &out, bool connected);
void writePacket(Print &out, uint8_t type, const uint8_t *payload, uint8_t length);
uint8_t calculateChecksum(uint8_t type, uint8_t length, const uint8_t *payload);
void wrapYaw(float &yaw);

void setup() {
  Serial.begin(GUI_UART_BAUD);
  #ifdef ESP32
  Serial2.begin(DEBUG_UART_BAUD, SERIAL_8N1, DEBUG_RX_PIN, DEBUG_TX_PIN);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  #elif defined(ARDUINO_TEENSY41)
  Serial2.begin(DEBUG_UART_BAUD);
  Wire.begin();
  #else
  Serial2.begin(DEBUG_UART_BAUD);
  Wire.begin();
  #endif

  Wire.setClock(400000);

  if (!imu.begin()) {
    Serial2.println("IMU init failed. System halted.");
    unsigned long last_imu_status_time = 0;
    while (1) {
      unsigned long now = millis();
      if (now - last_imu_status_time >= 1000) {
        last_imu_status_time = now;
        sendImuStatusPacket(Serial, false);
      }
      yield();
    }
  }
  sendImuStatusPacket(Serial, true);

  imu.Offset();
  ekf.init(true);

  servo_setup();
  setTargetPose(DEFAULT_HEIGHT_MM, DEFAULT_HEIGHT_MM, TARGET_X_MM);
  servo_control();
  receiver.begin();
  lqr.setTorqueLimit(2.0f);
  lqr.setOutputScale(1.0f);

  last_imu_time = millis();
  last_gui_time = millis();
  last_imu_status_time = millis();

}

void loop() {
  unsigned long now = millis();

  if (now - last_imu_time >= imu_interval) {
    float dt = (now - last_imu_time) / 1000.0f;
    last_imu_time = now;

    update_imu(dt);

    if (data_get()) {
      servo_enabled = myData.servo_enabled;
      if (fabsf(myData.left_height - left_h_goal) > 0.01f ||
          fabsf(myData.right_height - right_h_goal) > 0.01f ||
          fabsf(myData.wheel_x_mm - wheel_x_goal) > 0.01f) {
        setTargetPose(myData.left_height, myData.right_height, myData.wheel_x_mm);
      }
      target_pitch_deg = myData.target_pitch_deg;
      lqr_enabled = myData.lqr_enabled;
      lqr_has_wheel_state = myData.has_lqr_state;
      lqr_position_m = myData.lqr_position_m;
      lqr_velocity_mps = myData.lqr_velocity_mps;
      lqr_target_position_m = myData.lqr_target_position_m;
      lqr_target_velocity_mps = myData.lqr_target_velocity_mps;
      lqr.setTorqueLimit(myData.lqr_torque_limit_nm);
      lqr.setOutputScale(myData.lqr_output_scale);
      if (myData.has_lqr_gain) {
        LQRGain gui_gain = {myData.lqr_k1, myData.lqr_k2, myData.lqr_k3, myData.lqr_k4};
        lqr.setGainOverride(gui_gain);
      } else {
        lqr.clearGainOverride();
      }
    }

    lqr.setEnabled(lqr_enabled);
    if (servo_enabled) {
      servo_control();
    }

    if (lqr_enabled) {
      update_lqr_control(now);
    } else {
      lqr.reset();
      lqr_torque_cmd = 0.0f;
    }
  }

  if (now - last_gui_time >= gui_interval) {
    last_gui_time = now;
    send_data_to_gui();
  }

  if (now - last_imu_status_time >= 1000) {
    last_imu_status_time = now;
    sendImuStatusPacket(Serial, true);
  }

}

void update_imu(float dt) {
  imu.DataRead(dt);

  float rate[3], acc[3], mag[3];
  imu.Receive_get(rate, acc, filtered_mode);
  latest_rates[0] = rate[0];
  latest_rates[1] = rate[1];
  latest_rates[2] = rate[2];
  imu.getCalibratedMag(mag, filtered_mode);
  ekf.update(rate, acc, mag, dt);

  ekf.getEuler(latest_angles);
  latest_state = ekf.getMotionState();

  if (latest_state == "STILL") {
    still_count++;

    if (still_count % 500 == 0 && still_count > 0) {
      ekf.QR_update(dt);
      ekf.getEuler(latest_angles);

      if (!yaw_offset_set && !collecting_yaw_offset) {
        collecting_yaw_offset = true;
        yaw_offset_count = 0;
        yaw_sum = 0.0f;
        Serial2.println("QR update done, collecting yaw offset");
      }
    }
  } else {
    still_count = 0;
  }

  if (collecting_yaw_offset) {
    yaw_sum += latest_angles[2];
    yaw_offset_count++;

    if (yaw_offset_count >= 50) {
      yaw_offset = yaw_sum / 50.0f;
      yaw_offset_set = true;
      collecting_yaw_offset = false;
      Serial2.print("Yaw offset set: ");
      Serial2.println(yaw_offset, 4);
    }
  }

  if (yaw_offset_set) {
    latest_angles[2] -= yaw_offset;
    wrapYaw(latest_angles[2]);
  }
}

void servo_setup() {
  attachAll();
}

void servo_control() {
  if (heightMoveActive) {
    float progress = static_cast<float>(millis() - heightMoveStartMs) /
                     static_cast<float>(MOVEMENT_DURATION_MS);
    if (progress >= 1.0f) {
      progress = 1.0f;
      heightMoveActive = false;
    }

    left_h = left_h_start + progress * (left_h_goal - left_h_start);
    right_h = right_h_start + progress * (right_h_goal - right_h_start);
    wheel_x = wheel_x_start + progress * (wheel_x_goal - wheel_x_start);
  } else {
    left_h = left_h_goal;
    right_h = right_h_goal;
    wheel_x = wheel_x_goal;
  }

  float leftAlpha;
  float leftBeta;
  float rightAlpha;
  float rightBeta;
  const bool left_ok = inverseKinematics(wheel_x, -left_h, leftAlpha, leftBeta);
  const bool right_ok = inverseKinematics(wheel_x, -right_h, rightAlpha, rightBeta);
  lastIkOk = left_ok && right_ok;

  if (!lastIkOk) {
    return;
  }

  leftAngles.alpha = 180.0f - radToDeg(leftAlpha);
  leftAngles.beta = radToDeg(leftBeta);
  rightAngles.alpha = 180.0f - radToDeg(rightAlpha);
  rightAngles.beta = radToDeg(rightBeta);

  setLegServo(leftAlpha, leftBeta, rightAlpha, rightBeta);
}

void attachChannel(uint8_t index) {
  if (index >= CHANNEL_COUNT || channels[index].attached) {
    return;
  }
  #ifdef ESP32
  channels[index].servo->setPeriodHertz(50);
  channels[index].servo->attach(channels[index].pin, SERVO_MIN_US, SERVO_MAX_US);
  #else
  channels[index].servo->attach(channels[index].pin);
  #endif
  channels[index].attached = true;
  channels[index].servo->write(channels[index].angle);
}

void writeChannelAngle(uint8_t index, float angle) {
  if (index >= CHANNEL_COUNT) {
    return;
  }
  int nextAngle = constrain(static_cast<int>(roundf(angle)), MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  channels[index].angle = nextAngle;
  if (channels[index].attached) {
    channels[index].servo->write(channels[index].angle);
  }
}

void attachAll() {
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    attachChannel(i);
  }
}

void setTargetPose(float left_height_mm, float right_height_mm, float wheel_x_mm) {
  left_h_start = left_h;
  right_h_start = right_h;
  wheel_x_start = wheel_x;
  left_h_goal = clampFloat(left_height_mm, MIN_HEIGHT_MM, MAX_HEIGHT_MM);
  right_h_goal = clampFloat(right_height_mm, MIN_HEIGHT_MM, MAX_HEIGHT_MM);
  wheel_x_goal = clampFloat(wheel_x_mm, MIN_TARGET_X_MM, MAX_TARGET_X_MM);
  heightMoveStartMs = millis();
  heightMoveActive = true;
}

bool solveAlpha(float x, float y, float &alpha1, float &alpha2) {
  float a = 2.0f * x * L1;
  float b = 2.0f * y * L1;
  float c = x * x + y * y + L1 * L1 - L2 * L2;
  float disc = a * a + b * b - c * c;

  if (disc < 0.0f) {
    return false;
  }

  float sqrtDisc = sqrt(disc);
  alpha1 = 2.0f * atan2(b + sqrtDisc, a + c);
  alpha2 = 2.0f * atan2(b - sqrtDisc, a + c);
  return true;
}

bool solveBeta(float x, float y, float &beta1, float &beta2) {
  float d = 2.0f * (x - L5) * L4;
  float e = 2.0f * y * L4;
  float f = (x - L5) * (x - L5) + y * y + L4 * L4 - L3 * L3;
  float disc = d * d + e * e - f * f;

  if (disc < 0.0f) {
    return false;
  }

  float sqrtDisc = sqrt(disc);
  beta1 = 2.0f * atan2(e + sqrtDisc, d + f);
  beta2 = 2.0f * atan2(e - sqrtDisc, d + f);
  return true;
}

bool inverseKinematics(float x, float y, float &alpha, float &beta) {
  float alpha1;
  float alpha2;
  float beta1;
  float beta2;

  if (!solveAlpha(x, y, alpha1, alpha2)) {
    return false;
  }
  if (!solveBeta(x, y, beta1, beta2)) {
    return false;
  }

  float alphaRawDeg = normalizeDegrees360(radToDeg(alpha2));
  float alphaLegDeg = alphaRawDeg > 180.0f ? 360.0f - alphaRawDeg : alphaRawDeg;
  float betaRawDeg = normalizeDegrees180(radToDeg(beta1));

  alpha = (180.0f - alphaLegDeg) * PI / 180.0f;
  beta = -betaRawDeg * PI / 180.0f;
  return true;
}

void setLegServo(float left_alpha, float left_beta, float right_alpha, float right_beta) {
  float leftAlphaDeg = radToDeg(left_alpha);
  float leftBetaDeg = radToDeg(left_beta);
  float rightAlphaDeg = radToDeg(right_alpha);
  float rightBetaDeg = radToDeg(right_beta);

  writeChannelAngle(0, RIGHT_FRONT_OFFSET + RIGHT_FRONT_DIR * rightAlphaDeg);
  writeChannelAngle(1, RIGHT_BACK_OFFSET + RIGHT_BACK_DIR * rightBetaDeg);
  writeChannelAngle(2, LEFT_FRONT_OFFSET + LEFT_FRONT_DIR * leftAlphaDeg);
  writeChannelAngle(3, LEFT_BACK_OFFSET + LEFT_BACK_DIR * leftBetaDeg);
}

float radToDeg(float rad) {
  return rad * 180.0f / PI;
}

float normalizeDegrees360(float angle_deg) {
  while (angle_deg < 0.0f) {
    angle_deg += 360.0f;
  }
  while (angle_deg >= 360.0f) {
    angle_deg -= 360.0f;
  }
  return angle_deg;
}

float normalizeDegrees180(float angle_deg) {
  angle_deg = normalizeDegrees360(angle_deg);
  if (angle_deg > 180.0f) {
    angle_deg -= 360.0f;
  }
  return angle_deg;
}

float clampFloat(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

void update_lqr_control(unsigned long now) {
  const float leg_height_mm = servo_enabled
                                  ? 0.5f * (left_h + right_h)
                                  : 0.5f * (left_h_goal + right_h_goal);
  const float pitch_deg = latest_angles[1];
  const float pitch_rate_dps = latest_rates[1];
  if (lqr_has_wheel_state) {
    LQRState state = {
        lqr_position_m,
        lqr_velocity_mps,
        pitch_deg * PI / 180.0f,
        pitch_rate_dps * PI / 180.0f,
    };
    LQRState target = {
        lqr_target_position_m,
        lqr_target_velocity_mps,
        target_pitch_deg * PI / 180.0f,
        0.0f,
    };
    lqr_torque_cmd = lqr.computeTorque(state, target, leg_height_mm);
  } else {
    lqr_torque_cmd = lqr.computeBalanceTorque(pitch_deg,
                                              pitch_rate_dps,
                                              target_pitch_deg,
                                              leg_height_mm);
  }

  if (now - last_lqr_debug_time >= 100) {
    last_lqr_debug_time = now;
    LQRGain gain = lqr.getGains();
    Serial2.print("LQR h=");
    Serial2.print(lqr.getLastLegHeightMm(), 2);
    Serial2.print(" K=[");
    Serial2.print(gain.k1, 6); Serial2.print(", ");
    Serial2.print(gain.k2, 6); Serial2.print(", ");
    Serial2.print(gain.k3, 6); Serial2.print(", ");
    Serial2.print(gain.k4, 6); Serial2.print("] torque=");
    Serial2.print(lqr_torque_cmd, 6);
    Serial2.print(" state=");
    Serial2.println(lqr_has_wheel_state ? "wheel" : "pitch");
  }
}

bool data_get() {
  myData = receiver.checkAndReceive();
  if (myData.isValid &&
      (myData.id == "esp32" || myData.id == "teensy41" || myData.id == "servo")) {
    return true;
  }

  return false;
}

void send_data_to_gui() {
  sendTelemetryPacket(Serial);
  sendLqrPacket(Serial);
}

void sendTelemetryPacket(Print &out) {
  float roll = latest_angles[0];
  float pitch = latest_angles[1];
  float yaw = latest_angles[2];
  uint8_t moving = (latest_state == "STILL") ? 0 : 1;
  uint8_t mag_interference = ekf.isMagInterfered() ? 1 : 0;

  uint8_t payload[TELEMETRY_PAYLOAD_LEN];
  memcpy(payload + 0, &roll, sizeof(float));
  memcpy(payload + 4, &pitch, sizeof(float));
  memcpy(payload + 8, &yaw, sizeof(float));
  payload[12] = moving;
  payload[13] = mag_interference;

  writePacket(out, PACKET_TYPE_TELEMETRY, payload, TELEMETRY_PAYLOAD_LEN);
}

void sendLqrPacket(Print &out) {
  const float pitch_deg = latest_angles[1];
  const float pitch_rate_dps = latest_rates[1];
  const float leg_height_mm = servo_enabled
                                  ? 0.5f * (left_h + right_h)
                                  : 0.5f * (left_h_goal + right_h_goal);
  const float raw_torque_nm = lqr.getLastRawTorque();
  const float limited_torque_nm = lqr.getLastTorque();
  const LQRGain gain = lqr.getGains();

  uint8_t payload[LQR_PAYLOAD_LEN];
  memcpy(payload + 0, &pitch_deg, sizeof(float));
  memcpy(payload + 4, &pitch_rate_dps, sizeof(float));
  memcpy(payload + 8, &target_pitch_deg, sizeof(float));
  memcpy(payload + 12, &leg_height_mm, sizeof(float));
  payload[16] = lqr_enabled ? 1 : 0;
  memcpy(payload + 17, &raw_torque_nm, sizeof(float));
  memcpy(payload + 21, &limited_torque_nm, sizeof(float));
  memcpy(payload + 25, &gain.k1, sizeof(float));
  memcpy(payload + 29, &gain.k2, sizeof(float));
  memcpy(payload + 33, &gain.k3, sizeof(float));
  memcpy(payload + 37, &gain.k4, sizeof(float));

  writePacket(out, PACKET_TYPE_LQR, payload, LQR_PAYLOAD_LEN);
}

void sendImuStatusPacket(Print &out, bool connected) {
  uint8_t payload[IMU_STATUS_PAYLOAD_LEN] = {static_cast<uint8_t>(connected ? 1 : 0)};
  writePacket(out, PACKET_TYPE_IMU_STATUS, payload, IMU_STATUS_PAYLOAD_LEN);
}

void writePacket(Print &out, uint8_t type, const uint8_t *payload, uint8_t length) {
  out.write(PACKET_HEADER_1);
  out.write(PACKET_HEADER_2);
  out.write(type);
  out.write(length);
  out.write(payload, length);
  out.write(calculateChecksum(type, length, payload));
}

uint8_t calculateChecksum(uint8_t type, uint8_t length, const uint8_t *payload) {
  uint8_t checksum = type ^ length;
  for (uint8_t i = 0; i < length; i++) {
    checksum ^= payload[i];
  }
  return checksum;
}

void wrapYaw(float &yaw) {
  if (yaw > 180.0f) {
    yaw -= 360.0f;
  }
  if (yaw < -180.0f) {
    yaw += 360.0f;
  }
}
