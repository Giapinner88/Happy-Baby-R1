#pragma once
// ============================================================================
// ArmGesturePlayer — overlay động tác TAY chồng lên output của policy locomotion.
//
// TWIN FILE: bản sao y hệt nằm ở HB/high_level_2/src/motion/ArmGesturePlayer.hpp
//            Sửa một bên thì đồng bộ bên kia (thuần std, không phụ thuộc codebase).
//
// Quy ước khớp (khớp cả sim R1Config lẫn hl2 spec):
//   - policy có 24 khớp; TAY = policy idx 14..23 (5 trái 14..18, 5 phải 19..23).
//   - frame gesture = 10 góc tay (rad), local idx 0..9 tương ứng policy 14..23.
//   - obs 83-D: q_rel tại 11+i, dq tại 35+i, last_action tại 59+i (i = policy joint).
//
// Ba việc player làm (khớp 3 trụ trong docs/PLAN_upperbody_overlay_teleop.md):
//   1) BlendInto(): ghi đè + trộn mượt góc tay vào target_q[14..23].
//   2) MaskObs():   che q_rel/dq của tay trong observation (giữ last_action).
//   3) LeanPitchProxy(): tín hiệu feedforward cho bù thăng bằng (caller nhân hệ số).
//
// BẤT BIẾN (graceful degradation): khi weight==0, BlendInto/MaskObs là NO-OP,
// obs & target_q giữ nguyên -> locomotion chạy y hệt khi CHƯA có gesture nào.
// ============================================================================

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

    struct Gesture {
        std::vector<std::array<float, kNumArm>> frames;
        float fps  = 50.0f;
        bool  loop = false;   // true: lặp lại; false: chạy tới cuối rồi GIỮ frame cuối
    };

    // default_arm = 10 góc tay mặc định = default_q[14..23].
    void Init(const std::array<float, kNumArm>& default_arm,
              float blend_in_s = 0.4f, float retract_s = 1.0f) {
        default_arm_ = default_arm;
        cur_arm_     = default_arm;
        pose_arm_    = default_arm;
        blend_in_s_  = blend_in_s > 1e-3f ? blend_in_s : 1e-3f;
        retract_s_   = retract_s  > 1e-3f ? retract_s  : 1e-3f;
    }

    void AddGesture(const std::string& name,
                    std::vector<std::array<float, kNumArm>> frames,
                    float fps, bool loop) {
        if (frames.empty()) return;
        gestures_[name] = Gesture{std::move(frames), fps > 1e-3f ? fps : 50.0f, loop};
    }

    bool Has(const std::string& name) const { return gestures_.count(name) > 0; }

    // Bấm lần 1: chơi. Bấm lại đúng gesture đang chạy: thu tay về.
    void Trigger(const std::string& name) {
        if (!gestures_.count(name)) return;
        if (active_ == name && !retracting_) { retracting_ = true; return; }
        active_     = name;
        play_time_  = 0.0f;
        retracting_ = false;
        playing_    = true;
    }

    // Ép thu tay về (dùng cho safety: rời locomotion / balance guard / ngã).
    void Retract() { if (!active_.empty()) retracting_ = true; }

    // Gọi MỖI tick điều khiển (dt giây). Cập nhật weight + tư thế tay hiện tại.
    void Update(float dt) {
        if (!active_.empty() && !retracting_) {
            weight_ = std::min(1.0f, weight_ + dt / blend_in_s_);
        } else {
            weight_ = std::max(0.0f, weight_ - dt / retract_s_);
            if (weight_ <= 0.0f) { active_.clear(); retracting_ = false; playing_ = false; }
        }
        if (active_.empty()) { pose_arm_ = default_arm_; return; }
        if (playing_) play_time_ += dt;
        pose_arm_ = SampleActive();
    }

    // Ghi đè target_q[14..23] = trộn(policy_arm, gesture_pose) theo weight. NO-OP khi weight==0.
    // LƯU Ý: gọi trên bản sao target_q thô của policy mỗi tick (tránh trộn chồng lặp lại).
    void BlendInto(std::array<float, kNumJoints>& target_q) {
        if (weight_ <= 0.0f) {
            for (int j = 0; j < kNumArm; ++j) cur_arm_[j] = target_q[kArmBegin + j];
            return;
        }
        // Smoothstep (ease-in-out): vận tốc tay = 0 ở đầu/cuối blend -> hết giật lúc thu
        // tay về (bớt lung lay) và lúc giơ lên. Đồng bộ với high_level_2 twin.
        const float w = weight_ * weight_ * (3.0f - 2.0f * weight_);
        for (int j = 0; j < kNumArm; ++j) {
            float blended = (1.0f - w) * target_q[kArmBegin + j] + w * pose_arm_[j];
            target_q[kArmBegin + j] = blended;
            cur_arm_[j] = blended;   // tay THỰC đang lệnh -> dùng cho mask/balance
        }
    }

    // Che q_rel & dq của tay trong obs (trộn về 0 theo weight, GIỮ last_action). NO-OP khi weight==0.
    void MaskObs(std::vector<float>& obs) const {
        if (weight_ <= 0.0f) return;
        const float w = weight_ * weight_ * (3.0f - 2.0f * weight_);  // smoothstep, khớp BlendInto
        const float keep = 1.0f - w;
        for (int j = 0; j < kNumArm; ++j) {
            const int i = kArmBegin + j;
            obs[11 + i] *= keep;   // q_rel
            obs[35 + i] *= keep;   // dq
        }
    }

    bool  Active() const { return weight_ > 0.0f; }
    float Weight() const { return weight_; }
    const std::string& ActiveName() const { return active_; }
    const std::array<float, kNumArm>& CurrentArm() const { return cur_arm_; }

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

    std::unordered_map<std::string, Gesture> gestures_;
    std::array<float, kNumArm> default_arm_{};
    std::array<float, kNumArm> cur_arm_{};
    std::array<float, kNumArm> pose_arm_{};
    std::string active_;
    float weight_     = 0.0f;
    float play_time_  = 0.0f;
    float blend_in_s_ = 0.4f;
    float retract_s_  = 1.0f;
    bool  playing_    = false;
    bool  retracting_ = false;
};
