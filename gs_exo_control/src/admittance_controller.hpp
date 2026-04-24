// gs_exo_control/src/admittance_controller.hpp
// 导纳控制律：τ = Kp*(P_plan - P_current) - Kd*dP_current
// 步态相位切换：摆动相→牵引，支撑相→被动阻尼
// 运行频率：1kHz（P-Core 绑定，SCHED_FIFO rt_priority=90）

#pragma once

#include <string>
#include <utility>

struct AdmittanceParams {
  float kp_guide  = 2.5f;   // 引导增益（牵引）
  float kd_guide  = 0.8f;   // 阻尼增益
  float kp_damp   = 0.0f;   // 支撑相：不主动干预
  float kd_damp   = 1.5f;   // 支撑相：被动阻尼
  float max_torque_nm = 3.0f; // 最大输出力矩（软件限幅，VESC 还有硬件限幅）
};

class AdmittanceController {
public:
  explicit AdmittanceController(const AdmittanceParams& params);

  // 每 1ms 调用一次
  // gait_phase: "stance" | "swing"
  // p_plan: Nav2 期望位置（m）
  // p_current: VIO 当前位置（m）
  // dp_current: 当前速度（m/s）
  // returns: {left_Nm, right_Nm}
  std::pair<float, float> compute(
    const std::string& gait_phase,
    float p_plan_x,   float p_plan_y,
    float p_curr_x,   float p_curr_y,
    float dp_curr_x,  float dp_curr_y
  );

private:
  AdmittanceParams params_;
  float clamp_torque(float torque) const;
};
