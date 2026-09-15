#pragma once
// ============================================================================
// ArmGesturePlayer — sinh động tác TAY và track ĐẦU tùy chọn cho locomotion.
// Tay có hai đường tiêu thụ:
//   - Legacy 83-D: BlendInto() overlay góc tay vào target_q[14..23]. Việc che
//     q_rel/dq của tay trong obs KHÔNG do player, mà do LocomotionController làm
//     qua ctx.arm_mask_keep (player không còn tự MaskObs ở deploy).
//   - Unified 105-D: ReferenceArm() -> arm_q_ref/dq_ref trong obs, policy tự bù.
//
// ĐÃ HẾT LÀ TWIN: bản sim ở unitree_mujoco/simulate_cpp/src/core/ArmGesturePlayer.hpp
//   đã LỆCH (sim thiếu pending queue, per-slot blend/retract, RetractSafety,
//   ReferenceArm và vẫn tự gọi MaskObs). Sửa file này KHÔNG tự đồng bộ sang sim.
//
// Quy ước khớp:
//   - policy có 24 khớp; TAY = policy idx 14..23 (5 trái 14..18, 5 phải 19..23).
//   - frame gesture = 10 góc tay (rad), local idx 0..9 tương ứng policy 14..23.
//   - head_pos tùy chọn = {yaw, pitch} (rad), nằm ngoài policy và được caller
//     gửi riêng qua HeadTarget.
//   - obs: q_rel tại 11+i, dq tại 35+i, last_action tại 59+i (i = policy joint).
//
// Việc player làm:
//   1) BlendInto():      (legacy) ghi đè + trộn mượt góc tay vào target_q[14..23].
//   2) ReferenceArm():   (unified) tư thế tay tham chiếu default->pose (smoothstep).
//   3) LeanPitchProxy(): tín hiệu feedforward cho bù thăng bằng (caller nhân hệ số).
//
// BẤT BIẾN (graceful degradation): khi weight==0, BlendInto là NO-OP và
// ReferenceArm trả về default -> locomotion chạy y hệt khi CHƯA có gesture nào.
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

    // default_arm = 10 góc tay mặc định = default_q[14..23].
    // safety_retract_s: thời gian thu tay NHANH cho pha AN TOÀN (guard nghiêng / rời
    // loco / teleop chiếm quyền) — luôn dùng số này, BỎ QUA retract per-slot "thẩm mỹ".
    void Init(const std::array<float, kNumArm>& default_arm,
              float blend_in_s = 0.4f, float retract_s = 1.0f,
              float safety_retract_s = 0.4f) {
        default_arm_      = default_arm;
        cur_arm_          = default_arm;
        pose_arm_         = default_arm;
        default_head_.fill(0.0f);
        pose_head_        = default_head_;
        def_blend_in_s_   = blend_in_s > 1e-3f ? blend_in_s : 1e-3f;
        def_retract_s_    = retract_s  > 1e-3f ? retract_s  : 1e-3f;
        safety_retract_s_ = safety_retract_s > 1e-3f ? safety_retract_s : 1e-3f;
        blend_in_s_       = def_blend_in_s_;
        retract_s_        = def_retract_s_;
    }

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

    // Bấm lần 1: chơi. Bấm lại đúng gesture đang chạy: thu tay về.
    // Đổi sang gesture khác: xếp hàng, thu hoàn toàn về policy/default rồi mới chạy.
    void Trigger(const std::string& name) {
        if (!gestures_.count(name)) return;
        if (active_ == name && pending_.empty() && !retracting_) {
            Retract();
            return;
        }
        if (!active_.empty() || weight_ > 0.0f) {
            pending_ = name;       // yêu cầu mới nhất thắng
            retracting_ = true;
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
            retracting_ = true;
            playing_ = false;
        }
    }

    // Thu tay về "AN TOÀN" (guard nghiêng / rời loco / teleop / ngã): ép thời gian
    // NHANH cố định, bỏ qua retract per-slot để kéo tay vào giúp giữ thăng bằng.
    void RetractSafety() {
        if (!active_.empty() || weight_ > 0.0f)
            retract_s_ = safety_retract_s_;
        Retract();
    }

    const std::string& PendingName() const { return pending_; }
    bool Idle() const { return active_.empty() && pending_.empty() && weight_ <= 0.0f; }

private:
    void Start(const std::string& name) {
        active_     = name;
        play_time_  = 0.0f;
        retracting_ = false;
        playing_    = true;
        // Áp thời gian blend/retract riêng của gesture (nếu có), nếu không dùng mặc định.
        const Gesture& g = gestures_.at(name);
        blend_in_s_ = g.blend_in_s > 1e-3f ? g.blend_in_s : def_blend_in_s_;
        retract_s_  = g.retract_s  > 1e-3f ? g.retract_s  : def_retract_s_;
    }

public:
    // Gọi MỖI tick điều khiển (dt giây). Cập nhật weight + tư thế tay/đầu hiện tại.
    void Update(float dt) {
        if (!active_.empty() && !retracting_) {
            weight_ = std::min(1.0f, weight_ + dt / blend_in_s_);
        } else {
            weight_ = std::max(0.0f, weight_ - dt / retract_s_);
            if (weight_ <= 0.0f) {
                active_.clear();
                retracting_ = false;
                playing_ = false;
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

    bool  Active() const { return weight_ > 0.0f; }
    float Weight() const { return weight_; }
    const std::string& ActiveName() const { return active_; }
    const std::array<float, kNumArm>& CurrentArm() const { return cur_arm_; }

    // Reference tuyệt đối của gesture (default -> pose theo cùng smoothstep của
    // BlendInto). Unified policy đọc giá trị này trong observation; nó KHÔNG gọi
    // BlendInto nên vẫn có quyền tự bù bằng chân/hông/waist.
    std::array<float, kNumArm> ReferenceArm() const {
        std::array<float, kNumArm> out;
        const float w = weight_ * weight_ * (3.0f - 2.0f * weight_);
        for (int j = 0; j < kNumArm; ++j)
            out[j] = default_arm_[j] + w * (pose_arm_[j] - default_arm_[j]);
        return out;
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
    bool  playing_       = false;
    bool  retracting_    = false;
};
