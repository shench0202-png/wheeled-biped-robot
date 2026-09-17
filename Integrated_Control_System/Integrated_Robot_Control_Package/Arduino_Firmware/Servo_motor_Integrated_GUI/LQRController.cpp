#include "LQRController.h"
#include <math.h>

namespace {
const LQRController::Segment K1_SEGMENTS[4] = {
    {60.0f, 84.0f, 0.0000000000f, -0.4472135955f},
    {84.0f, 108.0f, 0.0000000000f, -0.4472135955f},
    {108.0f, 132.0f, -0.0000000000f, -0.4472135955f},
    {132.0f, 156.0f, 0.0000000000f, -0.4472135955f},
};

const LQRController::Segment K2_SEGMENTS[4] = {
    {60.0f, 84.0f, -0.0000970615f, -0.6710113658f},
    {84.0f, 108.0f, -0.0000981736f, -0.6709179520f},
    {108.0f, 132.0f, -0.0000992334f, -0.6708034926f},
    {132.0f, 156.0f, -0.0001002395f, -0.6706706904f},
};

const LQRController::Segment K3_SEGMENTS[4] = {
    {60.0f, 84.0f, -0.0014435459f, -2.9791489749f},
    {84.0f, 108.0f, -0.0014651305f, -2.9773358734f},
    {108.0f, 132.0f, -0.0014861034f, -2.9750707937f},
    {132.0f, 156.0f, -0.0015064336f, -2.9723872133f},
};

const LQRController::Segment K4_SEGMENTS[4] = {
    {60.0f, 84.0f, -0.0007481298f, -0.4455678693f},
    {84.0f, 108.0f, -0.0007637917f, -0.4442522679f},
    {108.0f, 132.0f, -0.0007794139f, -0.4425650674f},
    {132.0f, 156.0f, -0.0007949742f, -0.4405111098f},
};
}

LQRController::LQRController()
    : enabled_(false),
      torque_limit_(2.0f),
      output_scale_(1.0f),
      last_raw_torque_(0.0f),
      last_torque_(0.0f),
      last_leg_height_mm_(MIN_LEG_HEIGHT_MM),
      gain_override_enabled_(false),
      gain_override_{-0.4472135955f, -0.6768350558f, -3.0657617289f, -0.4904556573f},
      gain_{-0.4472135955f, -0.6768350558f, -3.0657617289f, -0.4904556573f} {
}

void LQRController::setEnabled(bool enabled) {
  enabled_ = enabled;
  if (!enabled_) {
    reset();
  }
}

bool LQRController::isEnabled() const {
  return enabled_;
}

void LQRController::setTorqueLimit(float torque_limit) {
  torque_limit_ = fabsf(torque_limit);
}

void LQRController::setOutputScale(float output_scale) {
  output_scale_ = output_scale;
}

void LQRController::setGainOverride(const LQRGain &gain) {
  gain_override_ = gain;
  gain_override_enabled_ = true;
  gain_ = gain_override_;
}

void LQRController::clearGainOverride() {
  gain_override_enabled_ = false;
}

void LQRController::reset() {
  last_raw_torque_ = 0.0f;
  last_torque_ = 0.0f;
}

LQRGain LQRController::updateGains(float leg_height_mm) {
  last_leg_height_mm_ = clampFloat(leg_height_mm, MIN_LEG_HEIGHT_MM, MAX_LEG_HEIGHT_MM);

  if (gain_override_enabled_) {
    gain_ = gain_override_;
    return gain_;
  }

  gain_.k1 = evalPiecewise(K1_SEGMENTS, last_leg_height_mm_);
  gain_.k2 = evalPiecewise(K2_SEGMENTS, last_leg_height_mm_);
  gain_.k3 = evalPiecewise(K3_SEGMENTS, last_leg_height_mm_);
  gain_.k4 = evalPiecewise(K4_SEGMENTS, last_leg_height_mm_);

  return gain_;
}

LQRGain LQRController::getGains() const {
  return gain_;
}

float LQRController::computeTorque(const LQRState &state,
                                   const LQRState &target,
                                   float leg_height_mm) {
  updateGains(leg_height_mm);

  const float x_err = state.position_m - target.position_m;
  const float v_err = state.velocity_mps - target.velocity_mps;
  const float theta_err = state.pitch_rad - target.pitch_rad;
  const float theta_rate_err = state.pitch_rate_radps - target.pitch_rate_radps;

  last_raw_torque_ = -(gain_.k1 * x_err +
                      gain_.k2 * v_err +
                      gain_.k3 * theta_err +
                      gain_.k4 * theta_rate_err) * output_scale_;

  if (!enabled_) {
    last_torque_ = 0.0f;
    return last_torque_;
  }

  last_torque_ = clampFloat(last_raw_torque_, -torque_limit_, torque_limit_);
  return last_torque_;
}

float LQRController::computeBalanceTorque(float pitch_deg,
                                          float pitch_rate_dps,
                                          float target_pitch_deg,
                                          float leg_height_mm) {
  LQRState state = {
      0.0f,
      0.0f,
      pitch_deg * DEG_TO_RAD_F,
      pitch_rate_dps * DEG_TO_RAD_F,
  };
  LQRState target = {
      0.0f,
      0.0f,
      target_pitch_deg * DEG_TO_RAD_F,
      0.0f,
  };

  return computeTorque(state, target, leg_height_mm);
}

float LQRController::getLastRawTorque() const {
  return last_raw_torque_;
}

float LQRController::getLastTorque() const {
  return last_torque_;
}

float LQRController::getLastLegHeightMm() const {
  return last_leg_height_mm_;
}

float LQRController::clampFloat(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

float LQRController::evalPiecewise(const Segment segments[4], float leg_height_mm) {
  const float clamped_height = clampFloat(leg_height_mm, MIN_LEG_HEIGHT_MM, MAX_LEG_HEIGHT_MM);

  for (uint8_t i = 0; i < 4; i++) {
    if (clamped_height <= segments[i].max_mm || i == 3) {
      return segments[i].slope * clamped_height + segments[i].intercept;
    }
  }

  return segments[3].slope * clamped_height + segments[3].intercept;
}
