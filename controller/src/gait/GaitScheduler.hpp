#pragma once

#include <array>
#include <cmath>

#include "../config/RobotSpec.hpp"

// Đồng hồ tạo pha di chuyển (gait clock)
class GaitScheduler {
public:
    // Chu kỳ lấy từ tuning (khoá gait_period_s). Không gọi Configure -> giữ nguyên
    // spec::kGaitPeriodS như trước khi khoá này tồn tại. Tuning::Validate() đã chặn
    // giá trị <= 0 nên ở đây chỉ cần bỏ qua đầu vào vô lý.
    void Configure(float period_s) {
        if (std::isfinite(period_s) && period_s > 0.0f) period_ = period_s;
    }

    float period_s() const { return period_; }
    float time_s() const { return time_; }

    void Reset() { time_ = 0.0f; }

    void Update(float dt) {
        time_ += dt;
        if (time_ > 3600.0f) time_ = std::fmod(time_, period_);
    }

    // Contract 09/17 owns a 50 Hz phase clock. It freezes in STAND and starts
    // a new cycle at zero on the policy step that exits STAND.
    void UpdateStandRecovery(bool previous_stand, bool current_stand, float dt) {
        if (previous_stand && !current_stand) {
            Reset();
        } else if (!current_stand) {
            Update(dt);
        }
    }

    // Trả về pha obs (sin/cos) dựa trên chuẩn hóa lệnh
    std::array<float, 2> PhaseObs(float cmd_norm) const {
        if (cmd_norm < spec::kCmdGateNorm) return {0.0f, 0.0f};
        return PhaseObsRaw();
    }

    // Unlike legacy command gating, 09/17 zeros phase from the actual FSM mode.
    std::array<float, 2> PhaseObsStandRecovery(bool current_stand) const {
        if (current_stand) return {0.0f, 0.0f};
        return PhaseObsRaw();
    }

private:
    std::array<float, 2> PhaseObsRaw() const {
        float ratio = std::fmod(time_, period_) / period_;
        return {std::sin(2.0f * static_cast<float>(M_PI) * ratio),
                std::cos(2.0f * static_cast<float>(M_PI) * ratio)};
    }

    float time_ = 0.0f;
    float period_ = spec::kGaitPeriodS;
};
