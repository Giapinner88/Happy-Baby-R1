// Scripted (non-policy) states: stand up/lock, returning, sit, get up, lie down.
// Application is split across src/app/Application*.cpp; see Application.hpp.
#include "Application.hpp"
#include "ApplicationInternal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

using unitree_hg::msg::dds_::LowState_;
using unitree_hg::msg::dds_::LowCmd_;
using namespace app_internal;

void Application::RunStandUp() {
    const auto& default_q = locomotion_->default_q();
    std::array<float, spec::kNumJoints> target;

    // Pha cảnh báo khi chuyển sang stand lock.
    // Đường từ đi bộ đã chờ xong dưới policy nên stand_warn_timer_ vào đây đã đầy.
    if (stand_warn_timer_ < tuning_.stand_lock_warn_s) {
        stand_warn_timer_ += spec::kLoopDt;
        sender_.Send(stand_start_q_, stand_gains_kp_, stand_gains_kd_,
                     tuning_.stand_rate_limit, estimator_.state().mode_machine);
        return;
    }

    stand_timer_ += spec::kLoopDt;
    float progress = std::min(1.0f, stand_timer_ / tuning_.stand_up_time_s);

    for (int i = 0; i < spec::kNumJoints; ++i)
        target[i] = stand_start_q_[i] + progress * (default_q[i] - stand_start_q_[i]);

    sender_.Send(target, stand_gains_kp_, stand_gains_kd_,
                 tuning_.stand_rate_limit, estimator_.state().mode_machine);

    if (progress >= 1.0f) {
        std::cout << "[Application] STAND LOCK active.\n";
        state_ = AppState::kStandLock;
        sit_block_timer_ = std::max(0.0f, tuning_.stand_lock_sit_block_s);
    }
}

void Application::RunStandLock() {
    std::array<float, spec::kNumJoints> target = locomotion_->default_q();
    target[1] += tuning_.stand_lock_spread;
    target[7] -= tuning_.stand_lock_spread;
    sender_.Send(target, stand_gains_kp_, stand_gains_kd_,
                 tuning_.lock_rate_limit, estimator_.state().mode_machine);
}

void Application::RunReturning() {
    const RobotState& rs = estimator_.state();

    int worst_j = -1;
    float max_dq = MaxJointSpeed(rs, worst_j);
    std::vector<std::string> reasons;
    if (!CheckMotionPose()) return;
    if (!tuning_.state_motion_guard && fall_detector_.Check(rs.projected_gravity, rs.gyro, max_dq, worst_j, reasons)) {
        std::cout << "\n[Application] Fall detected while returning to default pose!\n";
        for (const auto& r : reasons) std::cout << "  - " << r << "\n";
        EnterIdle("Fall detected - triggering damping");
        return;
    }

    auto mit = mimics_.find(pending_dance_key_);
    if (mit == mimics_.end()) {
        std::cout << "[Application] Dance " << pending_dance_key_
                  << " disappeared - back to LOCOMOTION.\n";
        ActivatePolicy(locomotion_);
        state_ = AppState::kLocomotion;
        return;
    }

    // Đưa robot về tư thế đứng tĩnh ổn định, để soft-start xử lý chuyển sang tư thế mở màn.
    // Cổ chân dùng return_kp_ankle (mềm hơn stand_kp_leg) cho đỡ khựng khi vừa rời
    // locomotion; các khớp còn lại vẫn stand gains. Xem return_gains_kp_ ở Application.hpp.
    const auto& target = locomotion_->default_q();
    sender_.Send(target, return_gains_kp_, stand_gains_kd_,
                 tuning_.return_rate_limit, rs.mode_machine);

    float max_err = 0.0f;
    int   worst_err_j = -1;
    for (int i = 0; i < spec::kNumJoints; ++i) {
        float e = std::abs(rs.q[i] - target[i]);
        if (e > max_err) { max_err = e; worst_err_j = i; }
    }
    const bool return_pose_ready =
        max_err <= tuning_.return_pos_tol &&
        (!tuning_.mimic_transition_v2 ||
         max_dq <= tuning_.mimic_transition_exit_dq_tol);
    if (!return_pose_ready) {
        // Trần thời gian: khớp kẹt hoặc lệch calib thì max_err không bao giờ xuống dưới
        // tol. Giữ PD cứng trên sàn không tự cân bằng (MuJoCo với gain này đổ sau
        // ~0.6 s, simulate_cpp/experiments/hb_stability_20261024), nên hết giờ thì trả
        // về policy đi bộ như L2+Lên, không về STAND LOCK.
        return_timer_ += spec::kLoopDt;
        if (return_timer_ >= std::max(0.5f, tuning_.return_timeout_s)) {
            std::cout << "\n[Application] RETURNING quá " << tuning_.return_timeout_s
                      << "s mà khớp " << worst_err_j << " vẫn lệch " << max_err
                      << " rad (tol " << tuning_.return_pos_tol << ", max dq "
                      << max_dq << ", dq tol " << tuning_.mimic_transition_exit_dq_tol
                      << ").\n";
            CancelReturningToLocomotion("RETURNING timeout");
        }
        return;
    }

    std::cout << "[Application] Default pose reached (max err " << max_err
              << " rad) - MIMIC " << pending_dance_key_ << " soft-start ("
              << tuning_.mimic_warmup_s << "s).\n";
    const float dance_trim_deg = tuning_.DanceTrimDeg(pending_dance_key_);
    estimator_.SetPitchTrimDeg(dance_trim_deg);
    std::cout << "[Application] Dance " << pending_dance_key_
              << " IMU pitch trim: " << dance_trim_deg << " deg.\n";
    ActivatePolicy(mit->second.get());
    std::cout << "[Application] Dance " << pending_dance_key_
              << " runtime entry frame=" << mit->second->StartFrame()
              << (mit->second->TransitionV2Enabled() ? " (transition_v2)" : " (legacy)")
              << "\n";
    music_started_ = false;   // nhạc chỉ bật khi soft-start xong
    dance_stopping_ = false;  // Reset() đã xoá cờ cooldown trong controller
    state_ = AppState::kMimic;
}

void Application::RunSafeShutdown() {
    const RobotState& rs = estimator_.state();
    const auto& dq = locomotion_->default_q();
    const float D2R = static_cast<float>(M_PI) / 180.0f;

    // Trần cổ chân: xem spec::kAnklePitch*/kAnkleRollLimit (có nguồn gốc ở đó).
    constexpr float kAnkPMin = spec::kAnklePitchMin, kAnkPMax = spec::kAnklePitchMax;

    // "desc" là tư thế hạ tự cân bằng. "rest" là tư thế ngồi cuối khi đã có ghế đỡ.
    float spread_d = std::max(0.0f, tuning_.sit_spread);          // dạng chân lúc hạ (nhỏ, an toàn)
    float hip_d    = -std::fabs(tuning_.sit_hip_deg)  * D2R;      // hông lúc hạ (sâu, tự cân bằng)
    float knee_d   = std::clamp(tuning_.sit_knee_deg  * D2R, 0.3f, 2.42f);
    float spread_r = std::max(0.0f, tuning_.sit_rest_spread);     // dạng chân CUỐI (đo thực)
    float yaw_r_v  = tuning_.sit_rest_hip_yaw;                    // xoay mũi chân CUỐI (đo thực)
    float hip_r    = -std::fabs(tuning_.sit_rest_hip_deg) * D2R;  // hông CUỐI (đo thực, nông hơn)
    float knee_r   = std::clamp(tuning_.sit_rest_knee_deg * D2R, 0.3f, 2.42f);
    float lean_d   = std::max(0.0f, tuning_.sit_lean_deg) * D2R;         // đổ người khi hạ
    float lean_s   = std::max(0.0f, tuning_.sit_seated_lean_deg) * D2R;  // sau khi ngồi hẳn

    sit_timer_ += spec::kLoopDt;
    float Tg = std::max(0.1f, tuning_.sit_gather_time_s);
    float Td = std::max(0.1f, tuning_.sit_descent_time_s);
    float Ts = std::max(0.1f, tuning_.sit_settle_time_s);

    std::array<float, spec::kNumJoints> target = dq;
    float lean = 0.0f;

    auto Ease = [](float s) { s = std::clamp(s, 0.0f, 1.0f); return s * s * (3.0f - 2.0f * s); };

    if (sit_phase_ == 0) {
        // Thu chân về tư thế đứng thẳng chuẩn bị ngồi.
        float e = Ease(sit_timer_ / Tg);
        for (int i = 0; i < spec::kNumJoints; ++i)
            target[i] = sit_start_q_[i] + e * (dq[i] - sit_start_q_[i]);
        if (sit_timer_ >= Tg) { sit_phase_ = 1; sit_timer_ = 0.0f; }
    } else {
        float hip, knee, roll_l, roll_r, yaw_l, yaw_r, arm, elb;

        if (sit_phase_ == 1) {
            // Hạ người có cân bằng (chưa chắc có ghế đỡ): Gập hông, gối, vươn tay.
            float e = Ease(sit_timer_ / Td);
            hip  = dq[0]  + e * (hip_d  - dq[0]);
            knee = dq[3]  + e * (knee_d - dq[3]);
            roll_l = e * (-spread_d); roll_r = e * spread_d;
            yaw_l = 0.0f; yaw_r = 0.0f;
            lean = e * lean_d;
            arm  = dq[14] + e * (tuning_.sit_arm_forward - dq[14]);
            elb  = dq[17] + e * (tuning_.sit_arm_elbow   - dq[17]);
            if (sit_timer_ >= Td) {
                // Có ghế (L2+Trái): sang pha 2, nới sang tư thế CUỐI (ghế đỡ rồi, an toàn).
                // KHÔNG ghế (tự động, không ai đỡ): GIỮ NGUYÊN tư thế "desc" tự cân bằng —
                // KHÔNG relax sang rest (rest chỉ an toàn khi có ghế đỡ trọng lượng).
                sit_phase_ = sit_has_chair_ ? 2 : 3;
                sit_timer_ = 0.0f;
            }
        } else if (sit_phase_ == 2) {
            // Chuyển sang tư thế ngồi cuối khi ghế đã đỡ trọng lượng.
            float e = Ease(sit_timer_ / Ts);
            hip  = hip_d  + e * (hip_r  - hip_d);
            knee = knee_d + e * (knee_r - knee_d);
            roll_l = -spread_d + e * (-spread_r - (-spread_d));
            roll_r =  spread_d + e * ( spread_r -  spread_d);
            yaw_l  = e * (-yaw_r_v); yaw_r = e * yaw_r_v;
            lean = lean_d + e * (lean_s - lean_d);
            arm  = tuning_.sit_arm_forward + e * (tuning_.sit_seated_arm_pitch - tuning_.sit_arm_forward);
            elb  = tuning_.sit_arm_elbow   + e * (tuning_.sit_seated_arm_elbow - tuning_.sit_arm_elbow);
            if (sit_timer_ >= Ts) { sit_phase_ = 3; sit_timer_ = 0.0f; }
        } else {
            // Pha 3: Giữ tư thế ngồi (rest nếu có ghế, desc nếu tự cân bằng không ghế).
            if (sit_has_chair_) {
                hip = hip_r; knee = knee_r;
                roll_l = -spread_r; roll_r = spread_r;
                yaw_l = -yaw_r_v; yaw_r = yaw_r_v;
                lean = lean_s;
                arm = tuning_.sit_seated_arm_pitch; elb = tuning_.sit_seated_arm_elbow;
            } else {
                hip = hip_d; knee = knee_d;
                roll_l = -spread_d; roll_r = spread_d;
                yaw_l = 0.0f; yaw_r = 0.0f;
                lean = lean_d;
                arm = tuning_.sit_arm_forward; elb = tuning_.sit_arm_elbow;
            }
        }

        target[0] = target[6] = hip;
        target[3] = target[9] = knee;
        target[1] = roll_l; target[7] = roll_r;
        target[2] = yaw_l;  target[8] = yaw_r;
        target[14] = target[19] = arm;
        target[17] = target[22] = elb;
    }

    // Bù góc cổ chân để giữ lòng bàn chân phẳng với mặt đất.
    const Eigen::Vector3f& g = rs.projected_gravity;
    float pitch_meas = std::atan2(g.x(), -g.z());
    float roll_meas  = std::atan2(-g.y(), -g.z());
    float kg = std::clamp(tuning_.sit_ankle_gravity_gain, 0.0f, 1.0f);
    float corr_p = std::clamp(kg * (pitch_meas - lean), -0.17f, 0.17f);
    float corr_r = std::clamp(kg * roll_meas, -0.17f, 0.17f);

    target[4]  = std::clamp(-(target[0] + target[3]) - lean - corr_p, kAnkPMin, kAnkPMax);
    target[10] = std::clamp(-(target[6] + target[9]) - lean - corr_p, kAnkPMin, kAnkPMax);
    // Cổng chân roll bù hip roll để lòng bàn chân phẳng; clamp tránh vượt tầm khi dạng rộng.
    // Ở pha GIỮ, tư thế ngồi đứng yên vô hạn (sit_release_after=false), nên trần
    // này là giá trị cổ chân bị ép giữ liên tục chứ không phải một đỉnh thoáng
    // qua. sit_rest_spread=0.45 đẩy target lên 0.45..0.62 -> luôn chạm trần.
    target[5]  = std::clamp(-target[1] - corr_r, -spec::kAnkleRollLimit, spec::kAnkleRollLimit);
    target[11] = std::clamp(-target[7] - corr_r, -spec::kAnkleRollLimit, spec::kAnkleRollLimit);

    sender_.Send(target, sit_gains_kp_, sit_gains_kd_,
                 tuning_.sit_rate_limit, rs.mode_machine);

    if (tick_ % 250 == 0) {
        const char* pha = sit_phase_ == 0 ? "THU CHAN"
                        : sit_phase_ == 1 ? "HA NGUOI (do than + vuon tay)"
                        : sit_phase_ == 2 ? "NGOI HAN (giao luc cho ghe)" : "GIU";
        printf("  [sit] %s  t=%.1fs  than_do=%.0f/%.0f deg  hong_nghieng_thuc=%.0f deg\n",
               pha, sit_timer_, lean / D2R, lean_d / D2R, pitch_meas / D2R);
        fflush(stdout);
    }

    if (sit_phase_ == 3 && sit_timer_ >= std::max(0.0f, tuning_.sit_hold_s)) {
        if (tuning_.sit_release_after) {
            std::cout << "[Application] Sit completed - releasing joints.\n";
            EnterIdle("Sit completed - damping");
        } else {
            sit_phase_ = 4;  // giữ vô hạn; không quay lại nhánh này nữa
            std::cout << "[Application] Sit completed - holding pose.\n";
        }
    }
}

// Bù cổ chân theo IMU dựa trên quỹ đạo ghi được để giữ lòng bàn chân bám sàn.
void Application::ApplyAnkleFlat(std::array<float, spec::kNumJoints>& target,
                                 const JointTrajectory& motion, float t_ref) {
    float kg = tuning_.getup_ankle_gravity_gain;
    if (kg <= 0.0f || !motion.has_torso()) return;
    constexpr float kAnkPMin = spec::kAnklePitchMin, kAnkPMax = spec::kAnklePitchMax;
    constexpr float kAnkRLim = spec::kAnkleRollLimit, kCorrLim = 0.20f;
    const Eigen::Vector3f& gm = estimator_.state().projected_gravity;
    Eigen::Vector3f gr = motion.RefGravityAt(t_ref);
    float pitch_m = std::atan2(gm.x(), -gm.z());
    float pitch_r = std::atan2(gr.x(), -gr.z());
    float roll_m  = std::atan2(-gm.y(), -gm.z());
    float roll_r  = std::atan2(-gr.y(), -gr.z());
    float corr_p = std::clamp(kg * (pitch_m - pitch_r), -kCorrLim, kCorrLim);
    float corr_r = std::clamp(kg * (roll_m - roll_r), -kCorrLim, kCorrLim);
    target[4]  = std::clamp(target[4]  - corr_p, kAnkPMin, kAnkPMax);
    target[10] = std::clamp(target[10] - corr_p, kAnkPMin, kAnkPMax);
    target[5]  = std::clamp(target[5]  - corr_r, -kAnkRLim, kAnkRLim);
    target[11] = std::clamp(target[11] - corr_r, -kAnkRLim, kAnkRLim);
}

void Application::RunGetUp() {
    if (!getup_motion_) { EnterIdle("GET UP: thiếu getup.npz giữa chừng"); return; }
    auto Ease = [](float s) { s = std::clamp(s, 0.0f, 1.0f); return s * s * (3.0f - 2.0f * s); };

    std::array<float, spec::kNumJoints> target;
    auto f0 = getup_motion_->PoseAt(0.0f);
    float t_ref = 0.0f;   // vị trí tham chiếu để bù cổ chân (0 khi blend/giữ)

    if (traj_phase_ == 0) {
        // Bước 1: Blend từ tư thế hiện tại về tư thế nằm chuẩn (frame 0).
        float blend_t = std::max(0.05f, tuning_.getup_blend_time_s);
        float e = Ease(traj_timer_ / blend_t);
        for (int i = 0; i < spec::kNumJoints; ++i)
            target[i] = traj_start_q_[i] + e * (f0[i] - traj_start_q_[i]);
        traj_timer_ += spec::kLoopDt;
        if (traj_timer_ >= blend_t) {
            traj_phase_ = 1;   // sang GIỮ, chờ L2+X
            std::cout << "[Application] Đã vào tư thế nằm-chuẩn - GIỮ. Bấm L2+X để đứng dậy.\n";
        }
    } else if (traj_phase_ == 1) {
        // Giữ tư thế nằm chuẩn chờ L2+X.
        target = f0;
    } else {
        // Bước 2: Phát getup.npz rồi chuyển sang Locomotion.
        float speed = std::max(0.05f, tuning_.getup_speed);
        t_ref = traj_timer_ * speed;
        target = getup_motion_->PoseAt(t_ref);
        traj_timer_ += spec::kLoopDt;
        if (t_ref >= getup_motion_->duration_s()) {
            std::cout << "[Application] ĐỨNG DẬY hoàn tất -> LOCOMOTION (policy tự cân bằng).\n";
            ApplyAnkleFlat(target, *getup_motion_, getup_motion_->duration_s());
            sender_.Send(target, getup_gains_kp_, getup_gains_kd_,
                         tuning_.getup_rate_limit, estimator_.state().mode_machine);
            lying_ = false;
            getup_liedown_block_timer_ = std::max(0.0f, tuning_.getup_liedown_block_s);
            ActivatePolicy(locomotion_);
            state_ = AppState::kLocomotion;
            music_.Say(tuning_.voice_locomotion, tuning_.voice_volume, tuning_.voice_speaker_id);
            return;
        }
    }
    ApplyAnkleFlat(target, *getup_motion_, t_ref);
    sender_.Send(target, getup_gains_kp_, getup_gains_kd_,
                 tuning_.getup_rate_limit, estimator_.state().mode_machine);
}

// Phát liedown.npz; hết clip tự chuyển sang DAMPING và đặt lying_.
void Application::RunLieDown() {
    if (!liedown_motion_) { EnterIdle("LIE DOWN: thiếu liedown.npz giữa chừng"); return; }
    auto Ease = [](float s) { s = std::clamp(s, 0.0f, 1.0f); return s * s * (3.0f - 2.0f * s); };

    std::array<float, spec::kNumJoints> target;
    float t_ref = 0.0f;
    if (traj_phase_ == 0) {
        float blend_t = std::max(0.05f, tuning_.liedown_blend_time_s);
        float e = Ease(traj_timer_ / blend_t);
        auto f0 = liedown_motion_->PoseAt(0.0f);
        for (int i = 0; i < spec::kNumJoints; ++i)
            target[i] = traj_start_q_[i] + e * (f0[i] - traj_start_q_[i]);
        traj_timer_ += spec::kLoopDt;
        if (traj_timer_ >= blend_t) { traj_phase_ = 1; traj_timer_ = 0.0f; }
    } else {
        float speed = std::max(0.05f, tuning_.liedown_speed);
        t_ref = traj_timer_ * speed;
        target = liedown_motion_->PoseAt(t_ref);
        traj_timer_ += spec::kLoopDt;
        if (t_ref >= liedown_motion_->duration_s()) {
            // Nằm xong: chuyển sang DAMPING và đặt lying_.
            std::cout << "[Application] NẰM XUỐNG hoàn tất -> DAMPING (nằm nghỉ). "
                         "L2+Lên để chuẩn bị đứng dậy.\n";
            ApplyAnkleFlat(target, *liedown_motion_, liedown_motion_->duration_s());
            sender_.Send(target, getup_gains_kp_, getup_gains_kd_,
                         tuning_.liedown_rate_limit, estimator_.state().mode_machine);
            music_.Stop();
            state_ = AppState::kIdle;
            lying_ = true;
            announcing_ = false;
            pending_action_ = PendingAction::kNone;
            input_.ZeroVelocity();
            input_lost_ = false;
            input_lost_count_ = 0;
            sender_.Damping(estimator_.state().mode_machine);
            return;
        }
    }
    ApplyAnkleFlat(target, *liedown_motion_, t_ref);
    sender_.Send(target, getup_gains_kp_, getup_gains_kd_,
                 tuning_.liedown_rate_limit, estimator_.state().mode_machine);
}
