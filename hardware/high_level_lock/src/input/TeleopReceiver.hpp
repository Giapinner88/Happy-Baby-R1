#pragma once
// ============================================================================
// TeleopReceiver — nhận stream teleop (tay + đầu) qua UDP, làm nguồn override
// thân trên thứ 2 (song song với ArmGesturePlayer). Cùng "hình dáng" API với
// phần overlay tay (Update/Active/Weight/BlendInto/LeanPitchProxy; MaskObs chỉ
// dùng ở đường sim) để Application/sim chọn nguồn theo ưu tiên rồi gọi y hệt.
//
// TWIN FILE: bản y hệt ở unitree_mujoco/simulate_cpp/src/core/TeleopReceiver.hpp
//            (sim để ở core/, deploy để ở input/). Sửa một bên -> sync bên kia.
//
// Gói UDP (little-endian, đóng gói chặt, 60 byte) — xem scripts/teleop_send_test.py:
//   uint32 magic('UTL1'=0x314C5455), uint32 seq,
//   uint8 enable, uint8 arm_valid, uint8 head_valid, uint8 pad,
//   float arm_q[10]  (policy idx 14..23, rad),
//   float head_yaw, float head_pitch (rad).
//
// AN TOÀN: mất gói > timeout HOẶC enable=0 -> weight ramp về 0 (thu tay về policy),
//          head_valid=false. weight==0 -> BlendInto/MaskObs là NO-OP.
// ============================================================================

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

class TeleopReceiver {
public:
    static constexpr uint32_t kMagic  = 0x314C5455u;   // "UTL1"
    static constexpr int kNumJoints   = 24;
    static constexpr int kArmBegin    = 14;
    static constexpr int kNumArm      = 10;

#pragma pack(push, 1)
    struct Packet {
        uint32_t magic;
        uint32_t seq;
        uint8_t  enable;
        uint8_t  arm_valid;
        uint8_t  head_valid;
        uint8_t  pad;
        float    arm_q[kNumArm];
        float    head_yaw;
        float    head_pitch;
    };
#pragma pack(pop)

    ~TeleopReceiver() { Stop(); }

    // enabled=false -> không mở socket (teleop tắt hoàn toàn). head_*_max = clamp (rad).
    void Init(const std::array<float, kNumArm>& default_arm, bool enabled, int port,
              float timeout_ms, float blend_in_s, float retract_s, float smooth_hz,
              float head_yaw_max, float head_pitch_max) {
        default_arm_    = default_arm;
        cur_arm_        = default_arm;
        tgt_arm_        = default_arm;
        enabled_        = enabled;
        port_           = port;
        timeout_ms_     = timeout_ms > 1.0f ? timeout_ms : 1.0f;
        blend_in_s_     = blend_in_s > 1e-3f ? blend_in_s : 1e-3f;
        retract_s_      = retract_s  > 1e-3f ? retract_s  : 1e-3f;
        smooth_hz_      = smooth_hz  > 1e-3f ? smooth_hz  : 1e-3f;
        head_yaw_max_   = std::fabs(head_yaw_max);
        head_pitch_max_ = std::fabs(head_pitch_max);
        if (enabled_) Start();
    }

    void Start() {
        if (running_.load()) return;
        fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
        if (fd_ < 0) { std::perror("[Teleop] socket"); return; }
        int one = 1;
        setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        // Sidecar chạy ngay trên robot qua SSH. Chỉ loopback được phép gửi
        // target; không mở cổng motor-target ra LAN.
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(static_cast<uint16_t>(port_));
        if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            std::perror("[Teleop] bind");
            close(fd_); fd_ = -1; return;
        }
        running_.store(true);
        thread_ = std::thread(&TeleopReceiver::ThreadFn, this);
    }

    void Stop() {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
        if (fd_ >= 0) { close(fd_); fd_ = -1; }
    }

    // Gọi MỖI tick điều khiển (dt giây). Ramp weight + làm mượt tới hạn tay/đầu.
    // armed=false (nút toggle tắt) -> coi như không tươi -> weight ramp về 0 (nhả quyền).
    void Update(float dt, bool armed = true) {
        Packet p;
        int64_t recv_ns;
        bool have;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            p = latest_;
            recv_ns = last_recv_ns_;
            have = have_packet_;
        }
        int64_t now_ns = NowNs();
        bool fresh = armed && have && p.enable &&
                     (now_ns - recv_ns) < static_cast<int64_t>(timeout_ms_ * 1e6f);

        // Chỉ log ở cạnh chuyển trạng thái để biết owner mất target vì gói
        // STOP tường minh, watchdog UDP hay R3 không còn sẵn sàng. Trước đây cả
        // ba đều chỉ hiện ra như tay rơi về hold/zero torque ở tầng Application.
        if (fresh && !stream_fresh_) {
            const float age_ms = have ? static_cast<float>(now_ns - recv_ns) / 1e6f : -1.0f;
            std::cout << "\n[Teleop] stream ACTIVE seq=" << p.seq
                      << " age_ms=" << age_ms << "\n";
        } else if (!fresh && stream_fresh_) {
            const float age_ms = have ? static_cast<float>(now_ns - recv_ns) / 1e6f : -1.0f;
            const char* reason = !armed ? "operator_not_ready"
                               : !have ? "no_packet"
                               : !p.enable ? "explicit_stop"
                               : "udp_watchdog";
            std::cout << "\n[Teleop] stream INACTIVE reason=" << reason
                      << " seq=" << p.seq << " age_ms=" << age_ms << "\n";
        }
        stream_fresh_ = fresh;

        // Weight ramp.
        if (fresh) weight_ = std::min(1.0f, weight_ + dt / blend_in_s_);
        else       weight_ = std::max(0.0f, weight_ - dt / retract_s_);

        // Mục tiêu tay/đầu (chỉ cập nhật khi có gói tươi hợp lệ).
        if (fresh && p.arm_valid) {
            for (int j = 0; j < kNumArm; ++j) tgt_arm_[j] = p.arm_q[j];
        } else if (weight_ <= 0.0f) {
            tgt_arm_ = default_arm_;   // đã nhả hẳn -> về default
        }
        head_valid_ = fresh && p.head_valid;
        if (head_valid_) {
            head_yaw_   = Clamp(p.head_yaw,   -head_yaw_max_,   head_yaw_max_);
            head_pitch_ = Clamp(p.head_pitch, -head_pitch_max_, head_pitch_max_);
        }

        // Làm mượt tới hạn (critically-damped 1 cực) target -> cur.
        float alpha = 1.0f - std::exp(-dt * 2.0f * static_cast<float>(M_PI) * smooth_hz_);
        for (int j = 0; j < kNumArm; ++j)
            cur_arm_[j] += alpha * (tgt_arm_[j] - cur_arm_[j]);
    }

    bool  Active() const { return weight_ > 0.0f; }
    float Weight() const { return weight_; }

    // Ghi đè target_q[14..23] = trộn(policy_arm, cur_arm) theo weight. NO-OP khi weight==0.
    void BlendInto(std::array<float, kNumJoints>& target_q) {
        if (weight_ <= 0.0f) return;
        for (int j = 0; j < kNumArm; ++j) {
            target_q[kArmBegin + j] =
                (1.0f - weight_) * target_q[kArmBegin + j] + weight_ * cur_arm_[j];
        }
    }

    // Che q_rel & dq của tay (trộn về 0 theo weight, GIỮ last_action). NO-OP khi weight==0.
    // LƯU Ý: ở DEPLOY hàm này KHÔNG được gọi (mask do LocomotionController làm qua
    // ctx.arm_mask_keep). Giữ lại để KHỚP với sim twin (simulate_cpp) đang gọi trực tiếp.
    void MaskObs(std::vector<float>& obs) const {
        if (weight_ <= 0.0f) return;
        const float keep = 1.0f - weight_;
        for (int j = 0; j < kNumArm; ++j) {
            const int i = kArmBegin + j;
            obs[11 + i] *= keep;
            obs[35 + i] *= keep;
        }
    }

    float LeanPitchProxy() const {
        float lean = (cur_arm_[0] - default_arm_[0]) + (cur_arm_[5] - default_arm_[5]);
        return lean * weight_;
    }

    bool  HeadValid() const { return head_valid_ && weight_ > 0.0f; }
    float HeadYaw() const   { return head_yaw_; }
    float HeadPitch() const { return head_pitch_; }

    const std::array<float, kNumArm>& ArmTarget() const { return cur_arm_; }

private:
    static int64_t NowNs() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    static float Clamp(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    void ThreadFn() {
        Packet buf;
        while (running_.load()) {
            ssize_t n = recv(fd_, &buf, sizeof(buf), 0);
            if (n == static_cast<ssize_t>(sizeof(buf)) && buf.magic == kMagic && Valid(buf)) {
                std::lock_guard<std::mutex> lk(mtx_);
                // Bỏ gói cũ (seq lùi) — chống đảo thứ tự.
                if (!have_packet_ || SeqNewer(buf.seq, latest_.seq)) {
                    latest_ = buf;
                    last_recv_ns_ = NowNs();
                    have_packet_ = true;
                }
            } else if (n < 0) {
                // Non-blocking: không có dữ liệu -> ngủ ngắn.
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
    }
    static bool SeqNewer(uint32_t a, uint32_t b) {
        return static_cast<int32_t>(a - b) > 0;   // so sánh vòng
    }
    bool Valid(const Packet& p) const {
        for (float q : p.arm_q)
            if (!std::isfinite(q) || std::fabs(q) > 3.5f) return false;
        return std::isfinite(p.head_yaw) && std::isfinite(p.head_pitch) &&
               std::fabs(p.head_yaw) <= head_yaw_max_ &&
               std::fabs(p.head_pitch) <= head_pitch_max_;
    }

    std::array<float, kNumArm> default_arm_{};
    std::array<float, kNumArm> cur_arm_{};
    std::array<float, kNumArm> tgt_arm_{};
    float weight_ = 0.0f;
    bool  head_valid_ = false;
    float head_yaw_ = 0.0f, head_pitch_ = 0.0f;

    bool  enabled_ = false;
    int   port_ = 0;
    float timeout_ms_ = 300.0f, blend_in_s_ = 0.4f, retract_s_ = 0.5f, smooth_hz_ = 8.0f;
    float head_yaw_max_ = 1.0f, head_pitch_max_ = 0.6f;

    int fd_ = -1;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::mutex mtx_;
    Packet latest_{};
    int64_t last_recv_ns_ = 0;
    bool have_packet_ = false;
    bool stream_fresh_ = false;  // Update() thread only; log transition edges.
};
