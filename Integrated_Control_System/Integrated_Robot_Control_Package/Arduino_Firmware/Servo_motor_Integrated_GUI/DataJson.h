#ifndef DATAJSON_H
#define DATAJSON_H

#include <Arduino.h>
#include <ArduinoJson.h>

struct SensorData {
  String id;
  float right_height;
  float left_height;
  float wheel_x_mm;
  float target_pitch_deg;
  bool servo_enabled;
  bool lqr_enabled;
  float lqr_torque_limit_nm;
  float lqr_output_scale;
  bool has_lqr_gain;
  float lqr_k1;
  float lqr_k2;
  float lqr_k3;
  float lqr_k4;
  bool has_lqr_state;
  float lqr_position_m;
  float lqr_velocity_mps;
  float lqr_target_position_m;
  float lqr_target_velocity_mps;
  bool isValid;
};

class DataJson {
public:
  DataJson(Stream &serialPort, uint32_t baudRate);
  void begin();
  SensorData checkAndReceive();

private:
  Stream &_serial;
  uint32_t _baudRate;
  char _lineBuffer[512];
  uint16_t _lineLength = 0;
  SensorData parseLine(const char *line);
};

#endif
