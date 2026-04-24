#include "gait_detector.hpp"

#include <cmath>
#include <algorithm>

GaitDetector::GaitDetector(float threshold_accel_z, float threshold_gyro_y)
    : threshold_accel_z_(threshold_accel_z)
    , threshold_gyro_y_(threshold_gyro_y)
    , stance_count_(0)
    , swing_count_(0)
    , last_phase_("stance")
{
}

std::string GaitDetector::detectPhase(float accel_z, float gyro_y, float gyro_z) {
    // 步态检测逻辑：
    // 支撑相 (stance): 脚触地，Z轴加速度接近重力，Y轴角速度接近0
    // 摆动相 (swing): 脚离地，Z轴加速度变化，Y轴角速度有明显过零

    bool is_stance = std::abs(accel_z - 9.81) < threshold_accel_z_ &&
                     std::abs(gyro_y) < threshold_gyro_y_;

    std::string current_phase;

    if (is_stance) {
        current_phase = "stance";
        stance_count_++;
        swing_count_ = 0;

        // 连续stance超过阈值才确认
        if (stance_count_ > STANCE_MIN_COUNT) {
            if (last_phase_ != "stance") {
                phase_transitions_++;
            }
            last_phase_ = "stance";
            return "stance";
        }
    } else {
        current_phase = "swing";
        swing_count_++;
        stance_count_ = 0;

        // 连续swing超过阈值才确认
        if (swing_count_ > SWING_MIN_COUNT) {
            if (last_phase_ != "swing") {
                phase_transitions_++;
            }
            last_phase_ = "swing";
            return "swing";
        }
    }

    return last_phase_;
}

float GaitDetector::calculateStepLength(float accel_z, float time_delta) {
    // 简化的步长估算（积分加速度两次）
    // 仅用于步态分析，不用于精确定位
    static float last_accel_z = 0.0f;
    static float last_vel_z = 0.0f;

    float accel_change = accel_z - last_accel_z;
    last_accel_z = accel_z;

    // 积分得到速度和位移
    last_vel_z += accel_change * time_delta;
    float disp_z = last_vel_z * time_delta;

    last_accel_z = accel_z;

    return std::abs(disp_z);
}

void GaitDetector::reset() {
    stance_count_ = 0;
    swing_count_ = 0;
    last_phase_ = "stance";
    phase_transitions_ = 0;
}
