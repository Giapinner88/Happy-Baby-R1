#ifndef SIT_CONTROLLER_HPP
#define SIT_CONTROLLER_HPP

#include "runtime/PolicyRunner.hpp"
#include "runtime/R1Config.hpp"
#include "runtime/Tuning.hpp"
#include <array>
#include <string>

// SitController: Mô phỏng chính xác RunSafeShutdown() của high_level_2.
// 4 pha:
//   0 - THU CHÂN : nội suy từ pose hiện tại -> pose đứng thẳng + mở rộng chân đế
//   1 - HẠ NGƯỜI : gập hông+gối, đổ thân về trước, vươn tay (giữ trọng tâm trên bàn chân)
//   2 - GIAO LỰC : mông đã chạm ghế, thu tay về, dựng thân lại
//   3 - GIỮ VÔ HẠN
//
// Bàn chân bám mặt đất qua IMU projected_gravity (sit_ankle_gravity_gain).
class SitController : public PolicyRunner {
public:
    SitController() = default;
    ~SitController() override = default;

    void Init(const std::string&, Ort::Env&, const Ort::SessionOptions&) override {}

    std::vector<float> ComputeObservation(
        const LowState_& robot_state,
        const SportModeState_& sport_state,
        float, float, float, float, const std::array<float, 2>&) override { return {}; }

    // Tính target_q — cần IMU để bù cổ chân
    std::array<float, R1Config::NUM_JOINTS> ComputeTargetQWithIMU(
        const LowState_& current_state);

    std::array<float, R1Config::NUM_JOINTS> ComputeTargetQ(
        const std::vector<float>&) override;   // fallback không dùng IMU

    void Reset(const LowState_& current_state) override;
    bool IsFinished() const override { return false; }
    int  GetInputSize() const override { return 0; }

    void Tick(float dt = 0.02f);

    float kp() const;
    float kd() const;

private:
    static float Ease(float s);   // Smoothstep cubic

    float timer_ = 0.0f;
    int   phase_ = 0;

    std::array<float, R1Config::NUM_JOINTS> start_q_{};
    std::array<float, R1Config::NUM_JOINTS> default_q_{};   // tư thế đứng của flat policy
    std::array<float, R1Config::NUM_JOINTS> last_imu_q_{};

    // Lưu IMU lần cuối để fallback
    float last_pitch_ = 0.0f;
    float last_roll_  = 0.0f;
    int   log_tick_   = 0;
};

#endif
