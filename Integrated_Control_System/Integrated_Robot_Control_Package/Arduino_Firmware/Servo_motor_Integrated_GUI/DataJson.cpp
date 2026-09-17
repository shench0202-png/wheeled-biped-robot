#include "DataJson.h"

DataJson::DataJson(Stream &serialPort, uint32_t baudRate)
    : _serial(serialPort), _baudRate(baudRate) {}

void DataJson::begin() {}

SensorData DataJson::checkAndReceive() {
  SensorData data;
  data.isValid = false;

  while (_serial.available()) {
    const char incoming = static_cast<char>(_serial.read());
    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      _lineBuffer[_lineLength] = '\0';
      _lineLength = 0;
      if (_lineBuffer[0] != '\0') {
        return parseLine(_lineBuffer);
      }
      continue;
    }

    if (_lineLength < sizeof(_lineBuffer) - 1) {
      _lineBuffer[_lineLength++] = incoming;
    } else {
      _lineLength = 0;
      Serial2.println("JSON line too long");
    }
  }

  return data;
}

SensorData DataJson::parseLine(const char *line) {
  SensorData data;
  data.isValid = false;
  data.id = "";
  data.right_height = 60.0f;
  data.left_height = 60.0f;
  data.wheel_x_mm = 20.0f;
  data.target_pitch_deg = 0.0f;
  data.servo_enabled = false;
  data.lqr_enabled = false;
  data.lqr_torque_limit_nm = 2.0f;
  data.lqr_output_scale = 1.0f;
  data.has_lqr_gain = false;
  data.lqr_k1 = 0.0f;
  data.lqr_k2 = 0.0f;
  data.lqr_k3 = 0.0f;
  data.lqr_k4 = 0.0f;
  data.has_lqr_state = false;
  data.lqr_position_m = 0.0f;
  data.lqr_velocity_mps = 0.0f;
  data.lqr_target_position_m = 0.0f;
  data.lqr_target_velocity_mps = 0.0f;

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, line);

  if (error) {
    Serial2.print("JSON parse error: ");
    Serial2.println(error.c_str());
    return data;
  }

  data.id = doc["id"] | "";
  data.right_height = doc["leg"]["R"] | 60.0f;
  data.left_height = doc["leg"]["L"] | data.right_height;
  data.wheel_x_mm = doc["leg"]["X"] | 20.0f;
  if (doc["target_pitch_deg"].is<float>() || doc["target_pitch_deg"].is<int>()) {
    data.target_pitch_deg = doc["target_pitch_deg"].as<float>();
  } else if (doc["target_x"].is<float>() || doc["target_x"].is<int>()) {
    data.target_pitch_deg = doc["target_x"].as<float>();
  }
  data.servo_enabled = doc["enable"] | false;
  data.lqr_enabled = doc["lqr"]["enable"] | data.servo_enabled;
  data.lqr_torque_limit_nm = doc["lqr"]["torque_limit_nm"] | 2.0f;
  data.lqr_output_scale = doc["lqr"]["output_scale"] | 1.0f;
  JsonObjectConst state = doc["lqr"]["state"].as<JsonObjectConst>();
  if (!state.isNull()) {
    data.has_lqr_state = true;
    data.lqr_position_m = state["position_m"] | 0.0f;
    data.lqr_velocity_mps = state["velocity_mps"] | 0.0f;
    data.lqr_target_position_m = state["target_position_m"] | 0.0f;
    data.lqr_target_velocity_mps = state["target_velocity_mps"] | 0.0f;
  }
  JsonArrayConst kArray = doc["lqr"]["K"].as<JsonArrayConst>();
  if (!kArray.isNull() && kArray.size() >= 4) {
    data.has_lqr_gain = true;
    data.lqr_k1 = kArray[0] | 0.0f;
    data.lqr_k2 = kArray[1] | 0.0f;
    data.lqr_k3 = kArray[2] | 0.0f;
    data.lqr_k4 = kArray[3] | 0.0f;
  } else {
    JsonObjectConst gain = doc["lqr"]["gain"].as<JsonObjectConst>();
    if (gain.isNull()) {
      data.isValid = true;
      return data;
    }
    data.has_lqr_gain = true;
    data.lqr_k1 = gain["k1"] | 0.0f;
    data.lqr_k2 = gain["k2"] | 0.0f;
    data.lqr_k3 = gain["k3"] | 0.0f;
    data.lqr_k4 = gain["k4"] | 0.0f;
  }
  data.isValid = true;
  return data;
}
