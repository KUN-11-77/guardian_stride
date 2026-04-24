// gs_exo_control/src/admittance_controller.cpp
// 导纳控制律实现

#include "admittance_controller.hpp"
#include <cmath>
#include <algorithm>

AdmittanceController::AdmittanceController(const AdmittanceParams& params)
  : params_(params) {}

std::pair<float, float> AdmittanceController::compute(
    const std::string& gait_phase,
    float p_plan_x,   float p_plan_y,
    float p_curr_x,   float p_curr_y,
    float dp_curr_x,  float dp_curr_y) {

  float dx = p_plan_x - p_curr_x;
  float dy = p_plan_y - p_curr_y;
  float dist = std::sqrt(dx * dx + dy * dy);

  float dp_mag = std::sqrt(dp_curr_x * dp_curr_x + dp_curr_y * dp_curr_y);

  float kp, kd;
  if (gait_phase == "swing") {
    kp = params_.kp_guide;
    kd = params_.kd_guide;
  } else {
    kp = params_.kp_damp;
    kd = params_.kd_damp;
  }

  float left_torque = kp * dist - kd * dp_mag;
  float right_torque = kp * dist - kd * dp_mag;

  left_torque = clamp_torque(left_torque);
  right_torque = clamp_torque(right_torque);

  return {left_torque, right_torque};
}

float AdmittanceController::clamp_torque(float torque) const {
  return std::clamp(torque, -params_.max_torque_nm, params_.max_torque_nm);
}
