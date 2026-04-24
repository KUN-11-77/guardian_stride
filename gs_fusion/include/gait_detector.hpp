#pragma once

#include <string>
#include <algorithm>

class GaitDetector {
public:
    GaitDetector(float threshold_accel_z = 1.5f,
                 float threshold_gyro_y = 0.3f);

    std::string detectPhase(float accel_z, float gyro_y, float gyro_z);
    float calculateStepLength(float accel_z, float time_delta);
    void reset();

    float getConfidence() const {
        int total = stance_count_ + swing_count_;
        if (total == 0) return 0.0f;
        float ratio = static_cast<float>(std::max(stance_count_, swing_count_)) / total;
        return ratio;
    }

    int getPhaseTransitions() const { return phase_transitions_; }

private:
    static constexpr int STANCE_MIN_COUNT = 3;
    static constexpr int SWING_MIN_COUNT = 3;

    float threshold_accel_z_;
    float threshold_gyro_y_;
    int stance_count_;
    int swing_count_;
    std::string last_phase_;
    int phase_transitions_;
};
