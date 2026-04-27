// gs_safety/src/fsm_states.hpp
// M5 安全反射层 FSM 状态定义（纯规则，无 AI）

#pragma once

#include <string>
#include <cstdint>

enum class FSMState : uint8_t {
  NORMAL         = 0,
  SLOW_DOWN      = 1,
  VIRTUAL_WALL   = 2,
  FALL_PROTECT   = 3,
  HIGH_DAMP_SAFE = 4,
};

inline std::string fsm_state_to_string(FSMState s) {
  switch (s) {
    case FSMState::NORMAL:         return "NORMAL";
    case FSMState::SLOW_DOWN:      return "SLOW_DOWN";
    case FSMState::VIRTUAL_WALL:   return "VIRTUAL_WALL";
    case FSMState::FALL_PROTECT:   return "FALL_PROTECT";
    case FSMState::HIGH_DAMP_SAFE: return "HIGH_DAMP_SAFE";
    default:                       return "UNKNOWN";
  }
}

// FSM 转换条件
struct SafetyThresholds {
  float tof_slow_down_m   = 2.0f;
  float tof_virtual_wall_m = 0.5f;
  float imu_fall_pitch_deg = 30.0f;
  float imu_fall_roll_deg  = 30.0f;
  float heartbeat_timeout_ms = 200.0f;
};

// 引导力矩缩放
inline float get_torque_scale(FSMState state) {
  switch (state) {
    case FSMState::NORMAL:         return 1.0f;
    case FSMState::SLOW_DOWN:      return 0.5f;
    case FSMState::VIRTUAL_WALL:   return 0.0f;
    case FSMState::FALL_PROTECT:   return 0.0f;
    case FSMState::HIGH_DAMP_SAFE: return 0.0f;
    default:                       return 0.0f;
  }
}
