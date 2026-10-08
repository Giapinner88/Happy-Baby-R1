#pragma once
// ============================================================================
// ArmGesturePlayer — sinh động tác TAY và track ĐẦU tùy chọn cho locomotion.
// Tay được dùng bởi đường Legacy 83-D: BlendInto() overlay góc tay vào
// target_q[14..23]. Việc che
//     q_rel/dq của tay trong obs KHÔNG do player, mà do LocomotionController làm
//     qua ctx.arm_mask_keep (player không còn tự MaskObs ở deploy).
//
// ĐÃ HẾT LÀ TWIN: bản sim ở sim/unitree_mujoco/simulate_cpp/src/motion/
//   ArmGesturePlayer.hpp đã lược bớt pending queue, per-slot blend/retract,
//   RetractSafety và head. Sửa file này KHÔNG tự đồng bộ sang sim.
//
// Quy ước khớp:
//   - policy có 24 khớp; TAY = policy idx 14..23 (5 trái 14..18, 5 phải 19..23).
//   - frame gesture = 10 góc tay (rad), local idx 0..9 tương ứng policy 14..23.
//   - head_pos tùy chọn = {pitch, yaw} (rad), thứ tự IDL legacy 29,30; nằm ngoài policy và được caller
//     gửi riêng qua HeadTarget.
//   - obs: q_rel tại 11+i, dq tại 35+i, last_action tại 59+i (i = policy joint).
//
// Việc player làm:
//   1) BlendInto():      ghi đè + trộn mượt góc tay vào target_q[14..23].
//   2) LeanPitchProxy(): tín hiệu feedforward cho bù thăng bằng (caller nhân hệ số).
//
// BẤT BIẾN (graceful degradation): khi weight==0, BlendInto là NO-OP.
// ============================================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class ArmGesturePlayer {
public:
    static constexpr int kNumJoints = 24;
    static constexpr int kArmBegin  = 14;   // policy idx khớp tay đầu tiên
    static constexpr int kNumArm    = 10;   // 5 trái + 5 phải
    static constexpr int kNumHead   = 2;    // yaw, pitch; ngoài policy 24-D

    struct Gesture {
        std::vector<std::array<float, kNumArm>> frames;
        std::vector<std::array<float, kNumHead>> head_frames;
        float fps  = 50.0f;
        bool  loop = false;   // true: lặp lại; false: chạy tới cuối rồi GIỮ frame cuối
        // Thời gian trộn lên / thu về RIÊNG cho gesture này (giây). <=0 = kế thừa
        // mặc định toàn cục (Init). Dùng để chỉnh từng PHÍM: vd battay thu chậm
        // để bớt lung lay, còn vaytay thu nhanh hơn — không đụng file npz.
        float blend_in_s = 0.0f;
        float retract_s  = 0.0f;
    };

    // Giới hạn động học của target_q trong pha thu/handover. Đây là thông số
    // runtime của overlay, không phải metadata của policy: đổi artifact ONNX
    // không tự đổi tốc độ thu tay của robot.
    struct ReturnLimits {
        float min_duration_s = 2.5f;
        float max_velocity_rad_s = 0.60f;
        float max_acceleration_rad_s2 = 2.0f;
    };

    // default_arm = 10 góc tay mặc định = default_q[14..23].
    // safety_retract_s: thời gian thu tay NHANH cho pha AN TOÀN (guard nghiêng / rời
    // loco / teleop chiếm quyền) — BỎ QUA retract per-slot "thẩm mỹ".
    // safety_max_vel_rad_s: trần vận tốc cho pha đó (<=0 = tắt, giữ hành vi cũ). Cần vì
    // safety_retract_s là MỘT con số cố định trong khi quãng đường thu chênh nhau ~10 lần
    // giữa các gesture: 0.4s hợp cho clip vươn 0.5 rad, nhưng clip vươn 2.2 rad thì thành
    // cú giật 8+ rad/s ĐÚNG LÚC robot đang nghiêng — tự bơm mô-men phản lực vào thân.
    void Init(const std::array<float, kNumArm>& default_arm,
              float blend_in_s = 0.4f, float retract_s = 1.0f,
              float safety_retract_s = 0.4f,
              float safety_max_vel_rad_s = 0.0f) {
        default_arm_      = default_arm;
        cur_arm_          = default_arm;
        pose_arm_         = default_arm;
        default_head_.fill(0.0f);
        pose_head_        = default_head_;
        def_blend_in_s_   = blend_in_s > 1e-3f ? blend_in_s : 1e-3f;
        def_retract_s_    = retract_s  > 1e-3f ? retract_s  : 1e-3f;
        safety_retract_s_ = safety_retract_s > 1e-3f ? safety_retract_s : 1e-3f;
        safety_max_vel_   = safety_max_vel_rad_s;
        blend_in_s_       = def_blend_in_s_;
        retract_s_        = def_retract_s_;
        state_            = State::kIdle;
        command_velocity_.fill(0.0f);
        last_command_valid_ = false;
        return_paused_ = false;
        return_limits_enabled_ = false;
    }

    // Explicit opt-in để các caller legacy giữ nguyên hành vi hiện hữu. H4 gọi
    // hàm này lúc InitGestures(), do đó thu tay và handover đều chịu bound v/a.
    void SetReturnLimits(const ReturnLimits& limits) {
        return_limits_.min_duration_s = std::max(1e-3f, limits.min_duration_s);
        return_limits_.max_velocity_rad_s = std::max(1e-3f, limits.max_velocity_rad_s);
        return_limits_.max_acceleration_rad_s2 =
            std::max(1e-3f, limits.max_acceleration_rad_s2);
        return_limits_enabled_ = true;
    }

    // Khi IMU đang bị kích thích, giữ nguyên target thu hiện tại; quỹ đạo chỉ
    // tiếp tục khi thân yên lại, không phát một target step mới.
    void SetReturnPaused(bool paused) { return_paused_ = paused; }

    // frames/head_frames: đơn vị rad; head_frames rỗng nghĩa là gesture không điều
    // khiển đầu. loop=true lặp, false giữ frame cuối. blend_in_s/retract_s <=0
    // -> dùng mặc định toàn cục (override riêng theo phím).
    void AddGesture(const std::string& name,
                    std::vector<std::array<float, kNumArm>> frames,
                    float fps, bool loop,
                    float blend_in_s = 0.0f, float retract_s = 0.0f,
                    std::vector<std::array<float, kNumHead>> head_frames = {}) {
        if (frames.empty()) return;
        if (!head_frames.empty() && head_frames.size() != frames.size())
            head_frames.clear();  // malformed optional data degrades to arm-only.
        gestures_[name] = Gesture{std::move(frames), std::move(head_frames),
                                  fps > 1e-3f ? fps : 50.0f, loop, blend_in_s, retract_s};
    }

    bool Has(const std::string& name) const { return gestures_.count(name) > 0; }

    bool Loops(const std::string& name) const {
        const auto it = gestures_.find(name);
        return it != gestures_.end() && it->second.loop;
    }

    // A voice-owned command must be bounded. One-shot clips finish according
    // to the actual registered frames/fps (including the configured slot speed);
    // looping assets are rejected by the voice owner.
    bool ActiveClipFinished() const {
        const auto it = gestures_.find(active_);
        if (it == gestures_.end() || it->second.loop || !playing_) return false;
        const float duration = static_cast<float>(it->second.frames.size() - 1) /
                               std::max(it->second.fps, 1e-3f);
        return play_time_ >= duration;
    }

    // Bấm lần 1: chơi. Bấm lại đúng gesture đang chạy: thu tay về.
    // Đổi sang gesture khác: xếp hàng, thu hoàn toàn về policy/default rồi mới chạy.
    void Trigger(const std::string& name) {
        if (!gestures_.count(name)) return;
        if (active_ == name && pending_.empty() && !retracting_
            && state_ != State::kRetractRequested && state_ != State::kReturning
            && state_ != State::kHandover) {
            Retract();
            return;
        }
        if (!active_.empty() || weight_ > 0.0f) {
            pending_ = name;       // yêu cầu mới nhất thắng
            RequestReturn();
            playing_ = false;      // giữ pose hiện tại trong lúc thu về
            return;
        }
        Start(name);
    }

    // Thu tay về "THẨM MỸ" (người chủ động double-click tắt): giữ thời gian retract
    // riêng của gesture (per-slot) -> tắt êm. Huỷ mọi gesture đang chờ.
    void Retract() {
        pending_.clear();
        if (!active_.empty()) {
            RequestReturn();
            playing_ = false;
        }
    }

    // Thu tay về "AN TOÀN" khi guard/mode yêu cầu. H4 dùng quỹ đạo return/handover
    // runtime có bound v/a; safety_retract_s_ và safety_max_vel_ chỉ thuộc nhánh legacy.
    // Chốt MỘT lần ở nhánh legacy vì hàm này có thể được gọi mỗi tick suốt lúc guard xấu.
    void RetractSafety() {
        if (return_limits_enabled_) {
            // Bound runtime mới nghiêm hơn đường safety cũ (0.4s/4rad/s); giữ
            // trajectory đang chạy để không có cú giật lúc robot đã nghiêng.
            Retract();
            return;
        }
        if (!safety_latched_ && (!active_.empty() || weight_ > 0.0f)) {
            safety_latched_ = true;
            retract_s_ = safety_retract_s_;
            if (safety_max_vel_ > 1e-3f) {
                // Quãng đường còn phải đi = phần đã vươn khỏi default, nhân smoothstep
                // của weight hiện tại. Đỉnh vận tốc của smoothstep = quãng * 1.5 / t.
                const float w = weight_ * weight_ * (3.0f - 2.0f * weight_);
                float dist = 0.0f;
                for (int j = 0; j < kNumArm; ++j)
                    dist = std::max(dist, std::fabs(pose_arm_[j] - default_arm_[j]));
                dist *= w;
                retract_s_ = std::max(retract_s_, dist * 1.5f / safety_max_vel_);
            }
        }
        Retract();
    }

    // App state ngoài locomotion (stand transition, sit, idle, mimic...) có
    // target tay riêng và không gọi BlendInto. Xóa quyền sở hữu gesture ở đây
    // để pose cũ không bị phát lại khi policy locomotion chạy lần tiếp theo.
    void Cancel() {
        active_.clear();
        pending_.clear();
        weight_ = 0.0f;
        play_time_ = 0.0f;
        playing_ = false;
        retracting_ = false;
        safety_latched_ = false;
        state_ = State::kIdle;
        return_elapsed_s_ = 0.0f;
        return_duration_s_ = 0.0f;
        return_paused_ = false;
        command_velocity_.fill(0.0f);
        pose_arm_ = default_arm_;
        pose_head_ = default_head_;
        cur_arm_ = default_arm_;
    }

    const std::string& PendingName() const { return pending_; }
    bool Idle() const {
        return (!return_limits_enabled_ || state_ == State::kIdle)
            && active_.empty() && pending_.empty() && weight_ <= 0.0f;
    }

private:
    enum class State { kIdle, kPlaying, kRetractRequested, kReturning, kHandover };

    void RequestReturn() {
        if (return_limits_enabled_) {
            // Idempotent: callers such as movement/fall guards may request
            // safety retract every control tick. Never restart a return already
            // in progress or a handover waiting for measured stability.
            if (state_ == State::kPlaying) state_ = State::kRetractRequested;
        } else {
            retracting_ = true;
        }
    }

    static float QuinticEase(float u) {
        u = std::clamp(u, 0.0f, 1.0f);
        return u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
    }

    static std::array<float, kNumArm> PolicyArm(
            const std::array<float, kNumJoints>& target_q) {
        std::array<float, kNumArm> out{};
        for (int j = 0; j < kNumArm; ++j) out[j] = target_q[kArmBegin + j];
        return out;
    }

    static std::array<float, kNumArm> QuinticBoundary(
            const std::array<float, kNumArm>& q0,
            const std::array<float, kNumArm>& dq0,
            const std::array<float, kNumArm>& q1, float duration_s, float elapsed_s) {
        const float t = std::clamp(elapsed_s, 0.0f, duration_s);
        std::array<float, kNumArm> out{};
        for (int j = 0; j < kNumArm; ++j) {
            const float delta = q1[j] - q0[j];
            const float a3 = (10.0f * delta - 6.0f * dq0[j] * duration_s)
                / (duration_s * duration_s * duration_s);
            const float a4 = (-15.0f * delta + 8.0f * dq0[j] * duration_s)
                / (duration_s * duration_s * duration_s * duration_s);
            const float a5 = (6.0f * delta - 3.0f * dq0[j] * duration_s)
                / (duration_s * duration_s * duration_s * duration_s * duration_s);
            out[j] = q0[j] + dq0[j] * t + a3 * t * t * t + a4 * t * t * t * t
                + a5 * t * t * t * t * t;
        }
        return out;
    }

    float ReturnPeakVelocity(float duration_s) const {
        float peak = 0.0f;
        for (int sample = 0; sample <= 100; ++sample) {
            const float t = duration_s * static_cast<float>(sample) / 100.0f;
            for (int j = 0; j < kNumArm; ++j) {
                const float delta = return_goal_arm_[j] - return_start_arm_[j];
                const float a3 = (10.0f * delta - 6.0f * return_start_velocity_[j] * duration_s)
                    / (duration_s * duration_s * duration_s);
                const float a4 = (-15.0f * delta + 8.0f * return_start_velocity_[j] * duration_s)
                    / (duration_s * duration_s * duration_s * duration_s);
                const float a5 = (6.0f * delta - 3.0f * return_start_velocity_[j] * duration_s)
                    / (duration_s * duration_s * duration_s * duration_s * duration_s);
                const float velocity = return_start_velocity_[j] + 3.0f * a3 * t * t
                    + 4.0f * a4 * t * t * t + 5.0f * a5 * t * t * t * t;
                peak = std::max(peak, std::fabs(velocity));
            }
        }
        return peak;
    }

    float ReturnPeakAcceleration(float duration_s) const {
        float peak = 0.0f;
        for (int sample = 0; sample <= 100; ++sample) {
            const float t = duration_s * static_cast<float>(sample) / 100.0f;
            for (int j = 0; j < kNumArm; ++j) {
                const float delta = return_goal_arm_[j] - return_start_arm_[j];
                const float a3 = (10.0f * delta - 6.0f * return_start_velocity_[j] * duration_s)
                    / (duration_s * duration_s * duration_s);
                const float a4 = (-15.0f * delta + 8.0f * return_start_velocity_[j] * duration_s)
                    / (duration_s * duration_s * duration_s * duration_s);
                const float a5 = (6.0f * delta - 3.0f * return_start_velocity_[j] * duration_s)
                    / (duration_s * duration_s * duration_s * duration_s * duration_s);
                const float acceleration = 6.0f * a3 * t + 12.0f * a4 * t * t
                    + 20.0f * a5 * t * t * t;
                peak = std::max(peak, std::fabs(acceleration));
            }
        }
        return peak;
    }

    void BeginReturn(const std::array<float, kNumArm>& policy_arm) {
        return_start_arm_ = cur_arm_;
        // A sampled gesture can have a high instantaneous command velocity.
        // Carrying it into the return polynomial defeats the configured cap:
        // extending duration cannot lower the polynomial's initial velocity.
        for (int j = 0; j < kNumArm; ++j) {
            return_start_velocity_[j] = std::clamp(
                command_velocity_[j], -return_limits_.max_velocity_rad_s,
                return_limits_.max_velocity_rad_s);
        }
        return_goal_arm_ = policy_arm;
        return_duration_s_ = return_limits_.min_duration_s;
        for (int attempt = 0; attempt < 24; ++attempt) {
            if (ReturnPeakVelocity(return_duration_s_) <= return_limits_.max_velocity_rad_s
                && ReturnPeakAcceleration(return_duration_s_)
                    <= return_limits_.max_acceleration_rad_s2) break;
            return_duration_s_ *= 1.25f;
        }
        return_elapsed_s_ = 0.0f;
        state_ = State::kReturning;
    }

    std::array<float, kNumArm> FollowPolicyTarget(
            const std::array<float, kNumArm>& policy_arm) {
        std::array<float, kNumArm> out{};
        const float max_velocity = return_limits_.max_velocity_rad_s;
        const float max_delta_velocity = return_limits_.max_acceleration_rad_s2 * last_dt_;
        for (int j = 0; j < kNumArm; ++j) {
            const float desired_velocity = std::clamp(
                (policy_arm[j] - cur_arm_[j]) / last_dt_, -max_velocity, max_velocity);
            float velocity = command_velocity_[j] + std::clamp(
                desired_velocity - command_velocity_[j], -max_delta_velocity, max_delta_velocity);
            float command = cur_arm_[j] + velocity * last_dt_;
            if ((policy_arm[j] - cur_arm_[j]) * (policy_arm[j] - command) <= 0.0f) {
                command = policy_arm[j];
                velocity = 0.0f;
            }
            out[j] = command;
            command_velocity_[j] = velocity;
        }
        return out;
    }

    void CommitCommand(const std::array<float, kNumArm>& command) {
        if (last_command_valid_) {
            for (int j = 0; j < kNumArm; ++j)
                command_velocity_[j] = (command[j] - cur_arm_[j]) / last_dt_;
        }
        cur_arm_ = command;
        last_command_valid_ = true;
    }

    void Start(const std::string& name) {
        active_     = name;
        play_time_  = 0.0f;
        retracting_ = false;
        state_      = State::kPlaying;
        playing_    = true;
        safety_latched_ = false;
        return_paused_ = false;
        // Áp thời gian blend/retract riêng của gesture (nếu có), nếu không dùng mặc định.
        const Gesture& g = gestures_.at(name);
        blend_in_s_ = g.blend_in_s > 1e-3f ? g.blend_in_s : def_blend_in_s_;
        retract_s_  = g.retract_s  > 1e-3f ? g.retract_s  : def_retract_s_;
    }

public:
    // Gọi MỖI tick điều khiển (dt giây). Cập nhật weight + tư thế tay/đầu hiện tại.
    void Update(float dt) {
        last_dt_ = std::max(dt, 1e-4f);
        if (return_limits_enabled_) {
            if (state_ == State::kPlaying) {
                weight_ = std::min(1.0f, weight_ + dt / blend_in_s_);
            } else if (state_ == State::kReturning) {
                if (!return_paused_) return_elapsed_s_ += dt;
                const float u = std::clamp(return_elapsed_s_ / return_duration_s_, 0.0f, 1.0f);
                weight_ = 1.0f - QuinticEase(u);
            }
            if (state_ == State::kIdle) {
                pose_arm_ = default_arm_;
                pose_head_ = default_head_;
                return;
            }
            if (state_ == State::kPlaying && playing_) play_time_ += dt;
            if (!active_.empty()) {
                pose_arm_ = SampleActive();
                pose_head_ = SampleActiveHead();
            }
            return;
        }
        if (!active_.empty() && !retracting_) {
            weight_ = std::min(1.0f, weight_ + dt / blend_in_s_);
        } else {
            weight_ = std::max(0.0f, weight_ - dt / retract_s_);
            if (weight_ <= 0.0f) {
                active_.clear();
                retracting_ = false;
                playing_ = false;
                safety_latched_ = false;
                pose_arm_ = default_arm_;
                pose_head_ = default_head_;
                if (!pending_.empty()) {
                    std::string queued = pending_;
                    pending_.clear();
                    Start(queued);
                }
            }
        }
        if (active_.empty()) {
            pose_arm_ = default_arm_;
            pose_head_ = default_head_;
            return;
        }
        if (playing_) play_time_ += dt;
        pose_arm_ = SampleActive();
        pose_head_ = SampleActiveHead();
    }

    // Ghi đè target_q[14..23] = trộn(policy_arm, gesture_pose) theo weight. NO-OP khi weight==0.
    // LƯU Ý: gọi trên bản sao target_q thô của policy mỗi tick (tránh trộn chồng lặp lại).
    void BlendInto(std::array<float, kNumJoints>& target_q) {
        if (return_limits_enabled_) {
            if (state_ == State::kIdle) {
                CommitCommand(PolicyArm(target_q));
                return;
            }
            const auto policy_arm = PolicyArm(target_q);
            if (state_ == State::kRetractRequested) BeginReturn(policy_arm);

            std::array<float, kNumArm> command{};
            if (state_ == State::kReturning) {
                const float elapsed = std::min(return_elapsed_s_, return_duration_s_);
                command = QuinticBoundary(return_start_arm_, return_start_velocity_,
                                          return_goal_arm_, return_duration_s_, elapsed);
                if (return_elapsed_s_ >= return_duration_s_) {
                    command = return_goal_arm_;
                    state_ = State::kHandover;
                    command_velocity_.fill(0.0f);
                }
            } else if (state_ == State::kHandover) {
                command = FollowPolicyTarget(policy_arm);
            } else {
                const float w = QuinticEase(weight_);
                for (int j = 0; j < kNumArm; ++j)
                    command[j] = (1.0f - w) * policy_arm[j] + w * pose_arm_[j];
            }
            CommitCommand(command);
            for (int j = 0; j < kNumArm; ++j) target_q[kArmBegin + j] = command[j];
            return;
        }
        if (weight_ <= 0.0f) {
            for (int j = 0; j < kNumArm; ++j) cur_arm_[j] = target_q[kArmBegin + j];
            return;
        }
        // Smoothstep (ease-in-out): vận tốc tay = 0 ở đầu/cuối blend -> hết giật lúc thu
        // tay về (bớt lung lay) và lúc giơ lên. weight thô vẫn tuyến tính (guard/timing).
        const float w = weight_ * weight_ * (3.0f - 2.0f * weight_);
        for (int j = 0; j < kNumArm; ++j) {
            float blended = (1.0f - w) * target_q[kArmBegin + j] + w * pose_arm_[j];
            target_q[kArmBegin + j] = blended;
            cur_arm_[j] = blended;   // tay THỰC đang lệnh -> dùng cho mask/balance
        }
    }

    bool  Active() const { return return_limits_enabled_ ? state_ != State::kIdle : weight_ > 0.0f; }
    float Weight() const { return weight_; }
    const std::string& ActiveName() const { return active_; }
    const std::array<float, kNumArm>& CurrentArm() const { return cur_arm_; }
    bool HandoverPending() const { return return_limits_enabled_ && state_ == State::kHandover; }

    bool HandoverMatches(const std::array<float, kNumJoints>& policy_target,
                         float position_tolerance,
                         float command_velocity_tolerance) const {
        if (!HandoverPending()) return false;
        const float pos_tol = std::max(0.0f, position_tolerance);
        const float vel_tol = std::max(0.0f, command_velocity_tolerance);
        for (int j = 0; j < kNumArm; ++j) {
            if (std::fabs(cur_arm_[j] - policy_target[kArmBegin + j]) > pos_tol
                || std::fabs(command_velocity_[j]) > vel_tol) return false;
        }
        return true;
    }

    void ReleaseHandover() {
        if (!HandoverPending()) return;
        active_.clear();
        playing_ = false;
        retracting_ = false;
        weight_ = 0.0f;
        state_ = State::kIdle;
        return_paused_ = false;
        if (!pending_.empty()) {
            std::string queued = pending_;
            pending_.clear();
            Start(queued);
        }
    }

    // Đầu dùng cùng smoothstep/blend/retract với tay. Caller phải clamp và
    // rate-limit trước khi gửi LowCmd; nếu gesture không có head_pos thì đây là 0.
    std::array<float, kNumHead> ReferenceHead() const {
        std::array<float, kNumHead> out;
        const float w = weight_ * weight_ * (3.0f - 2.0f * weight_);
        for (int j = 0; j < kNumHead; ++j)
            out[j] = default_head_[j] + w * (pose_head_[j] - default_head_[j]);
        return out;
    }

    // True cả trong pha retract để đầu luôn quay về 0 mượt thay vì bị thả đột ngột.
    bool HeadActive() const {
        if (weight_ <= 0.0f || active_.empty()) return false;
        const auto it = gestures_.find(active_);
        return it != gestures_.end() && !it->second.head_frames.empty();
    }

    // Proxy độ nghiêng pitch do tay (feedforward bù thăng bằng): tổng lệch của
    // khớp VAI-PITCH (local idx 0 = tay trái, idx 5 = tay phải) so với default.
    // weight đã nằm trong cur_arm_ (đã blend) -> tự scale theo mức override.
    // Caller nhân hệ số k (đo trong sim); sim/deploy tự chọn dấu.
    float LeanPitchProxy() const {
        return (cur_arm_[0] - default_arm_[0]) + (cur_arm_[5] - default_arm_[5]);
    }

    // ------------------------------------------------------------------
    // Gesture mẫu (KHÔNG cần npz) — để test đường ống overlay+mask ngay trong sim.
    // Dao động NHẸ tay phải quanh default (±biên rad) hình sin, tay trái giữ default.
    // Đủ để thấy overlay hoạt động + kiểm tra locomotion có lắc không, mà an toàn.
    // ------------------------------------------------------------------
    static std::vector<std::array<float, kNumArm>> MakeDemoWave(
            const std::array<float, kNumArm>& default_arm,
            float amp = 0.35f, int num_frames = 60) {
        std::vector<std::array<float, kNumArm>> frames;
        frames.reserve(num_frames);
        for (int f = 0; f < num_frames; ++f) {
            float ph = 2.0f * static_cast<float>(M_PI) * f / num_frames;
            std::array<float, kNumArm> fr = default_arm;
            // Tay phải: nâng vai (local5) + vẫy khuỷu (local8) theo sin.
            fr[5] = default_arm[5] - amp * (0.5f + 0.5f * std::sin(ph));   // vai pitch
            fr[8] = default_arm[8] + amp * std::sin(2.0f * ph);           // khuỷu vẫy
            frames.push_back(fr);
        }
        return frames;
    }

private:
    std::array<float, kNumArm> SampleActive() const {
        const Gesture& g = gestures_.at(active_);
        const int n = static_cast<int>(g.frames.size());
        if (n == 1) return g.frames[0];
        float fidx = play_time_ * g.fps;
        int i0, i1;
        if (g.loop) {
            fidx = std::fmod(fidx, static_cast<float>(n));
            if (fidx < 0.0f) fidx += n;
            i0 = static_cast<int>(std::floor(fidx));
            i1 = (i0 + 1) % n;
        } else {
            if (fidx > n - 1) fidx = static_cast<float>(n - 1);   // kẹp cuối = giữ tư thế
            i0 = static_cast<int>(std::floor(fidx));
            i1 = std::min(i0 + 1, n - 1);
        }
        const float a = fidx - i0;
        std::array<float, kNumArm> out;
        for (int j = 0; j < kNumArm; ++j)
            out[j] = (1.0f - a) * g.frames[i0][j] + a * g.frames[i1][j];
        return out;
    }

    std::array<float, kNumHead> SampleActiveHead() const {
        const Gesture& g = gestures_.at(active_);
        if (g.head_frames.empty()) return default_head_;
        const int n = static_cast<int>(g.head_frames.size());
        if (n == 1) return g.head_frames[0];
        float fidx = play_time_ * g.fps;
        int i0, i1;
        if (g.loop) {
            fidx = std::fmod(fidx, static_cast<float>(n));
            if (fidx < 0.0f) fidx += n;
            i0 = static_cast<int>(std::floor(fidx));
            i1 = (i0 + 1) % n;
        } else {
            if (fidx > n - 1) fidx = static_cast<float>(n - 1);
            i0 = static_cast<int>(std::floor(fidx));
            i1 = std::min(i0 + 1, n - 1);
        }
        const float a = fidx - i0;
        std::array<float, kNumHead> out;
        for (int j = 0; j < kNumHead; ++j)
            out[j] = (1.0f - a) * g.head_frames[i0][j] + a * g.head_frames[i1][j];
        return out;
    }

    std::unordered_map<std::string, Gesture> gestures_;
    std::array<float, kNumArm> default_arm_{};
    std::array<float, kNumArm> cur_arm_{};
    std::array<float, kNumArm> pose_arm_{};
    std::array<float, kNumArm> command_velocity_{};
    std::array<float, kNumArm> return_start_arm_{};
    std::array<float, kNumArm> return_start_velocity_{};
    std::array<float, kNumArm> return_goal_arm_{};
    std::array<float, kNumHead> default_head_{};
    std::array<float, kNumHead> pose_head_{};
    std::string active_;
    std::string pending_;
    float weight_        = 0.0f;
    float play_time_     = 0.0f;
    float blend_in_s_    = 0.4f;   // đang hiệu lực cho gesture hiện tại
    float retract_s_     = 1.0f;
    float def_blend_in_s_ = 0.4f;  // mặc định toàn cục (fallback khi slot không override)
    float def_retract_s_  = 1.0f;
    float safety_retract_s_ = 0.4f; // thu NHANH cố định cho pha an toàn (RetractSafety)
    float safety_max_vel_   = 0.0f; // trần vận tốc pha an toàn (<=0 = tắt)
    float last_dt_ = 0.002f;
    float return_duration_s_ = 0.0f;
    float return_elapsed_s_ = 0.0f;
    ReturnLimits return_limits_{};
    State state_ = State::kIdle;
    bool return_limits_enabled_ = false;
    bool last_command_valid_ = false;
    bool return_paused_ = false;
    bool  playing_       = false;
    bool  retracting_    = false;
    bool  safety_latched_ = false;  // đã chốt retract_s_ cho pha an toàn lần này
};
