#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <stdint.h>

struct LQRState {
  float position_m;
  float velocity_mps;
  float pitch_rad;
  float pitch_rate_radps;
};

struct LQRGain {
  float k1;
  float k2;
  float k3;
  float k4;
};

class LQRController {
public:
  struct Segment {
    float min_mm;
    float max_mm;
    float slope;
    float intercept;
  };

  LQRController();

  void setEnabled(bool enabled);
  bool isEnabled() const;

  void setTorqueLimit(float torque_limit);
  void setOutputScale(float output_scale);
  void setGainOverride(const LQRGain &gain);
  void clearGainOverride();
  void reset();

  LQRGain updateGains(float leg_height_mm);
  LQRGain getGains() const;

  float computeTorque(const LQRState &state,
                      const LQRState &target,
                      float leg_height_mm);

  float computeBalanceTorque(float pitch_deg,
                             float pitch_rate_dps,
                             float target_pitch_deg,
                             float leg_height_mm);

  float getLastRawTorque() const;
  float getLastTorque() const;
  float getLastLegHeightMm() const;

private:
  static constexpr float MIN_LEG_HEIGHT_MM = 60.0f;
  static constexpr float MAX_LEG_HEIGHT_MM = 156.0f;
  static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;

  static float clampFloat(float value, float low, float high);
  static float evalPiecewise(const Segment segments[4], float leg_height_mm);

  bool enabled_;
  float torque_limit_;
  float output_scale_;
  float last_raw_torque_;
  float last_torque_;
  float last_leg_height_mm_;
  bool gain_override_enabled_;
  LQRGain gain_override_;
  LQRGain gain_;
};

#endif
