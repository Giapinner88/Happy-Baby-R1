#include "SitController.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>

// ─── Chỉ số khớp POLICY — đồng bộ với spec::kPolicyToSdk trong high_level_2 ─────
// 0=L_hip_pitch, 1=L_hip_roll, 2=L_hip_yaw, 3=L_knee, 4=L_ank_pitch, 5=L_ank_roll
// 6=R_hip_pitch, 7=R_hip_roll, 8=R_hip_yaw, 9=R_knee, 10=R_ank_pitch, 11=R_ank_roll
// 12=waist_roll, 13=waist_yaw
// 14=L_sho_pitch, 15=L_sho_roll, 16=L_sho_yaw, 17=L_elbow, 18=L_wrist
// 19=R_sho_pitch, 20=R_sho_roll, 21=R_sho_yaw, 22=R_elbow, 23=R_wrist
//
// kDefaultJointPos (từ RobotSpec.hpp / high_level_2):
//   L: pitch=-0.1, roll=0, yaw=0, knee=0.3, ank_pitch=-0.2, ank_roll=0
//   R: pitch=-0.1, roll=0, yaw=0, knee=0.3, ank_pitch=-0.2, ank_roll=0
//
// R1 hip_roll convention (xác nhận từ high_level_2/Application.cpp sit code):
//   gather[1] += spread  →  L_hip_roll DƯƠNG = abduc (xòe trái)
//   gather[7] -= spread  →  R_hip_roll ÂM    = abduc (xòe phải)

static constexpr int L_HIP_P = 0;
static constexpr int L_HIP_R = 1;
static constexpr int L_KNEE  = 3;
static constexpr int L_ANK_P = 4;
static constexpr int L_ANK_R = 5;
static constexpr int R_HIP_P = 6;
static constexpr int R_HIP_R = 7;
static constexpr int R_KNEE  = 9;
static constexpr int R_ANK_P = 10;
static constexpr int R_ANK_R = 11;
static constexpr int L_SHO_P = 14;
static constexpr int L_ELBOW = 17;
static constexpr int R_SHO_P = 19;
static constexpr int R_ELBOW = 22;

// Giới hạn pitch cổ chân R1: [-50°, +33°]
static constexpr float kAnkPMin = -0.873f;
static constexpr float kAnkPMax =  0.576f;

float SitController::Ease(float s) {
    s = std::clamp(s, 0.0f, 1.0f);
    return s * s * (3.0f - 2.0f * s);
}

void SitController::Reset(const LowState_& current_state) {
    timer_    = 0.0f;
    phase_    = 0;
    log_tick_ = 0;

    // Đọc pose hiện tại
    for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
        int idl_idx = R1Config::PolicyToIdl(i);
        start_q_[i] = current_state.motor_state()[idl_idx].q();
    }
    default_q_  = R1Config::DEFAULT_JOINT_POS;
    last_imu_q_ = start_q_;

    std::cout << "[SitController] Reset — 4 pha bat dau.\n";
    printf("  start_q: L_hip_pitch=%.3f  L_knee=%.3f  L_ank_p=%.3f  L_hip_r=%.3f\n",
           start_q_[L_HIP_P], start_q_[L_KNEE], start_q_[L_ANK_P], start_q_[L_HIP_R]);
    printf("  default_q: L_hip_pitch=%.3f  L_knee=%.3f  L_ank_p=%.3f  L_hip_r=%.3f\n",
           default_q_[L_HIP_P], default_q_[L_KNEE], default_q_[L_ANK_P], default_q_[L_HIP_R]);
}

void SitController::Tick(float dt) { timer_ += dt; }

std::array<float, R1Config::NUM_JOINTS>
SitController::ComputeTargetQWithIMU(const LowState_& cs) {
    const auto& t = Tuning::Get();
    const float D2R = static_cast<float>(M_PI) / 180.0f;

    float Tg = std::max(0.1f, t.sit_gather_time_s);
    float Td = std::max(0.1f, t.sit_descent_time_s);
    float Ts = std::max(0.1f, t.sit_settle_time_s);

    float spread = std::max(0.0f, t.sit_spread);
    float hip_t  = -std::fabs(t.sit_hip_deg)  * D2R;
    float knee_t = std::clamp(t.sit_knee_deg  * D2R, 0.3f, 2.42f);
    float lean_d = std::max(0.0f, t.sit_lean_deg)        * D2R;
    float lean_s = std::max(0.0f, t.sit_seated_lean_deg) * D2R;

    // ── "Thu chân": default + MỞ RỘNG chân đế ─────────────────────────────
    // Giống hệt high_level_2: gather[1] += spread; gather[7] -= spread;
    std::array<float, R1Config::NUM_JOINTS> gather = default_q_;
    gather[L_HIP_R] = default_q_[L_HIP_R] + spread;  // L xòe trái (dương = abduc)
    gather[R_HIP_R] = default_q_[R_HIP_R] - spread;  // R xòe phải (âm = abduc)

    std::array<float, R1Config::NUM_JOINTS> target = gather;
    float lean = 0.0f;

    if (phase_ == 0) {
        // ── Pha 0: THU CHÂN ──────────────────────────────────────────────────
        float e = Ease(timer_ / Tg);
        for (int i = 0; i < R1Config::NUM_JOINTS; ++i)
            target[i] = start_q_[i] + e * (gather[i] - start_q_[i]);
        if (timer_ >= Tg) { phase_ = 1; timer_ = 0.0f; }

    } else {
        // Pha 1/2/3: hip/knee base lấy từ gather (đã spread), rồi override pitch/knee
        float hip  = hip_t;
        float knee = knee_t;
        float arm  = default_q_[L_SHO_P];
        float elb  = default_q_[L_ELBOW];

        if (phase_ == 1) {
            // ── Pha 1: HẠ NGƯỜI ───────────────────────────────────────────
            float e = Ease(timer_ / Td);
            hip  = default_q_[L_HIP_P] + e * (hip_t  - default_q_[L_HIP_P]);
            knee = default_q_[L_KNEE]   + e * (knee_t - default_q_[L_KNEE]);
            lean = e * lean_d;
            arm  = default_q_[L_SHO_P] + e * (t.sit_arm_forward - default_q_[L_SHO_P]);
            elb  = default_q_[L_ELBOW] + e * (t.sit_arm_elbow   - default_q_[L_ELBOW]);
            if (timer_ >= Td) { phase_ = 2; timer_ = 0.0f; }

        } else if (phase_ == 2) {
            // ── Pha 2: GIAO LỰC ───────────────────────────────────────────
            float e = Ease(timer_ / Ts);
            lean = lean_d + e * (lean_s - lean_d);
            arm  = t.sit_arm_forward + e * (default_q_[L_SHO_P] - t.sit_arm_forward);
            elb  = t.sit_arm_elbow   + e * (default_q_[L_ELBOW] - t.sit_arm_elbow);
            if (timer_ >= Ts) { phase_ = 3; timer_ = 0.0f; }

        } else {
            // ── Pha 3: GIỮ VÔ HẠN ────────────────────────────────────────
            lean = lean_s;
        }

        // Gán hip pitch và gối (cả hai bên đối xứng)
        target[L_HIP_P] = target[R_HIP_P] = hip;
        target[L_KNEE]  = target[R_KNEE]  = knee;
        target[L_SHO_P] = target[R_SHO_P] = arm;
        target[L_ELBOW] = target[R_ELBOW] = elb;
        // hip_roll, hip_yaw giữ nguyên từ gather (đã spread)
    }

    // ── projected_gravity — công thức đồng bộ StateEstimator.hpp ──────────
    // state_.projected_gravity = {2*(qw*qy - qx*qz), -2*(qy*qz + qw*qx), 2*(qx²+qy²)-1}
    float qw = cs.imu_state().quaternion()[0];
    float qx = cs.imu_state().quaternion()[1];
    float qy = cs.imu_state().quaternion()[2];
    float qz = cs.imu_state().quaternion()[3];

    float gx_imu =  2.0f * (qw*qy - qx*qz);
    float gy_imu = -2.0f * (qy*qz + qw*qx);
    float gz_imu =  2.0f * (qx*qx + qy*qy) - 1.0f;

    // pitch_meas = atan2(g.x(), -g.z())  — giống Application.cpp line 577
    // roll_meas  = atan2(-g.y(), -g.z()) — giống Application.cpp line 578
    float pitch_meas = std::atan2(gx_imu, -gz_imu);
    float roll_meas  = std::atan2(-gy_imu, -gz_imu);

    float kg     = std::clamp(t.sit_ankle_gravity_gain, 0.0f, 1.0f);
    float corr_p = std::clamp(kg * (pitch_meas - lean), -0.17f, 0.17f);
    float corr_r = std::clamp(kg * roll_meas,            -0.17f, 0.17f);

    // ── Cổ chân pitch/roll — giống hệt Application.cpp line 583-586 ───────
    // target[4]  = -(target[0] + target[3]) - lean - corr_p
    // target[10] = -(target[6] + target[9]) - lean - corr_p
    // target[5]  = -target[1] - corr_r
    // target[11] = -target[7] - corr_r
    target[L_ANK_P] = std::clamp(-(target[L_HIP_P] + target[L_KNEE]) - lean - corr_p, kAnkPMin, kAnkPMax);
    target[R_ANK_P] = std::clamp(-(target[R_HIP_P] + target[R_KNEE]) - lean - corr_p, kAnkPMin, kAnkPMax);
    target[L_ANK_R] = -target[L_HIP_R] - corr_r;
    target[R_ANK_R] = -target[R_HIP_R] - corr_r;

    // Log 1 Hz (50 tick @ 50Hz)
    if (++log_tick_ >= 50) {
        log_tick_ = 0;
        const char* pha[] = {"THU CHAN", "HA NGUOI", "GIAO LUC", "GIU"};
        printf("  [sit] %s t=%.1fs lean=%.0f°(max%.0f°) pitch_imu=%.0f° ank_p=%.3f corr_p=%.3f\n",
               pha[std::clamp(phase_, 0, 3)], timer_,
               lean*(180.0f/M_PI), lean_d*(180.0f/M_PI),
               pitch_meas*(180.0f/M_PI),
               target[L_ANK_P], corr_p);
        fflush(stdout);
    }

    last_imu_q_ = target;
    return target;
}

std::array<float, R1Config::NUM_JOINTS>
SitController::ComputeTargetQ(const std::vector<float>&) {
    return last_imu_q_;
}

float SitController::kp() const { return Tuning::Get().sit_kp_leg; }
float SitController::kd() const { return Tuning::Get().sit_kd; }
