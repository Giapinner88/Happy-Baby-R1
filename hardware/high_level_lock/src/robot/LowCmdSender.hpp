#pragma once
/**
 * LowCmdSender.hpp — Lớp DUY NHẤT viết LowCmd_ xuống DDS.
 *
 * - Rate limiter trên lệnh (dùng lệnh trước làm gốc).
 * - Rate limit đặt cao khi chạy policy để chặn spike đột ngột.
 * - Đầu (idl 29/30) giữ 0 rad với gain nhẹ.
 * - CRC32 bắt buộc cho robot thật.
 * - Damping() = Kp/Kd/tau về 0 toàn bộ 35 motor (xả lực khẩn cấp).
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include "../config/RobotSpec.hpp"
#include "../config/Tuning.hpp"

// Mục tiêu điều khiển đầu (teleop hoặc gesture). valid=false -> giữ đầu ở 0.
// max_rate_rad_s<=0 kế thừa rate-limit của policy; gesture dùng trần riêng thấp hơn.
struct HeadTarget {
    bool  valid = false;
    float yaw   = 0.0f;
    float pitch = 0.0f;
    float max_rate_rad_s = 0.0f;
};

// Lớp gửi lệnh LowCmd_ xuống robot qua kênh DDS rt/lowcmd
class LowCmdSender {
public:
    void Init(const Tuning& tuning) {
        tuning_ = &tuning;
        pub_ = std::make_unique<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>("rt/lowcmd");
        pub_->InitChannel();
        last_cmd_q_.fill(0.0f);
    }

    // Đồng bộ lệnh trước đó với vị trí hiện tại của robot
    void SyncToState(const unitree_hg::msg::dds_::LowState_& low) {
        for (int i = 0; i < spec::kNumMotorsIdl; ++i)
            last_cmd_q_[i] = low.motor_state()[i].q();
    }

    // Gửi vị trí khớp mục tiêu kèm gains và giới hạn tốc độ (rate limit)
    void Send(const std::array<float, spec::kNumJoints>& target_q,
              const std::array<float, spec::kNumJoints>& kp,
              const std::array<float, spec::kNumJoints>& kd,
              float max_rate, uint8_t mode_machine,
              const HeadTarget& head = {}) {
        float max_step = max_rate * spec::kLoopDt;
        unitree_hg::msg::dds_::LowCmd_ cmd{};
        cmd.mode_machine() = mode_machine;

        for (int i = 0; i < spec::kNumMotorsIdl; ++i) {
            cmd.motor_cmd()[i].mode() = 1;
            cmd.motor_cmd()[i].tau() = 0;
            cmd.motor_cmd()[i].q() = 0;
            cmd.motor_cmd()[i].dq() = 0;
            cmd.motor_cmd()[i].kp() = 0;
            cmd.motor_cmd()[i].kd() = 0;
        }

        for (int i = 0; i < spec::kNumJoints; ++i) {
            int idl = spec::MotorIdl(i);
            float clamped = Slew(last_cmd_q_[idl], target_q[i], max_step);
            last_cmd_q_[idl] = clamped;
            cmd.motor_cmd()[idl].q() = clamped;
            cmd.motor_cmd()[idl].kp() = kp[i];
            cmd.motor_cmd()[idl].kd() = kd[i];
        }

        // Đặt vị trí đầu: teleop/gesture nếu valid, ngược lại giữ 0. Gesture có
        // rate-limit riêng để một frame NPZ lỗi không thể quật đầu đột ngột.
        float hy = head.valid ? head.yaw   : 0.0f;
        float hp = head.valid ? head.pitch : 0.0f;
        const float head_step = head.max_rate_rad_s > 0.0f
            ? head.max_rate_rad_s * spec::kLoopDt : max_step;
        SetHead(cmd, spec::kHeadYawIdl,   hy, tuning_->head_yaw_kp,   tuning_->head_yaw_kd,   head_step);
        SetHead(cmd, spec::kHeadPitchIdl, hp, tuning_->head_pitch_kp, tuning_->head_pitch_kd, head_step);

        cmd.crc() = Crc32(reinterpret_cast<uint32_t*>(&cmd),
                          (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
        RecordCrc(cmd.crc());
        pub_->Write(cmd);
    }

    // Damping (mode=0): Xả lực động cơ, firmware giữ Kd nội bộ nhỏ tránh tự quay. Cần mode_machine từ LowState.
    void Damping(uint8_t mode_machine) {
        unitree_hg::msg::dds_::LowCmd_ cmd{};
        cmd.mode_machine() = mode_machine;
        for (int i = 0; i < spec::kNumMotorsIdl; ++i) {
            cmd.motor_cmd()[i].mode() = 0;
            cmd.motor_cmd()[i].tau() = 0;
            cmd.motor_cmd()[i].q() = 0;
            cmd.motor_cmd()[i].dq() = 0;
            cmd.motor_cmd()[i].kp() = 0;
            cmd.motor_cmd()[i].kd() = 0;
        }
        cmd.crc() = Crc32(reinterpret_cast<uint32_t*>(&cmd),
                          (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
        RecordCrc(cmd.crc());
        pub_->Write(cmd);
    }

    // Zero Torque (mode=1): Vô hiệu hóa lực hoàn toàn (limp). Chỉ dùng khi đã nằm/được đỡ. Cần mode_machine từ LowState.
    void ZeroTorque(uint8_t mode_machine) {
        unitree_hg::msg::dds_::LowCmd_ cmd{};
        cmd.mode_machine() = mode_machine;
        for (int i = 0; i < spec::kNumMotorsIdl; ++i) {
            cmd.motor_cmd()[i].mode() = 1;   // ← mode=1 thay vì 0, bypass kd nội bộ firmware
            cmd.motor_cmd()[i].tau()  = 0;
            cmd.motor_cmd()[i].q()    = 0;
            cmd.motor_cmd()[i].dq()   = 0;
            cmd.motor_cmd()[i].kp()   = 0;
            cmd.motor_cmd()[i].kd()   = 0;
        }
        cmd.crc() = Crc32(reinterpret_cast<uint32_t*>(&cmd),
                          (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
        RecordCrc(cmd.crc());
        pub_->Write(cmd);
    }

    // Chốt tư thế để khoá các khớp ngoài teleop. Phải gọi ngay trước khi
    // teleop chuyển sang active: chốt sớm hơn thì robot có thể đã bị xê dịch
    // trong lúc chờ, và khoá về một tư thế cũ là một cú giật chứ không phải giữ.
    void LatchNonTeleopHold(const unitree_hg::msg::dds_::LowState_& low) {
        for (int idl : kNonTeleopIdl) hold_q_[static_cast<size_t>(idl)] = low.motor_state()[idl].q();
        hold_latched_ = true;
    }

    void ClearNonTeleopHold() { hold_latched_ = false; }

    bool NonTeleopHoldLatched() const { return hold_latched_; }

    // ZERO TORQUE hỗn hợp: chỉ 10 khớp tay và head target hợp lệ có PD. Vẫn là
    // cùng một publisher/CRC ring nên không thể tranh rt/lowcmd với high-level.
    //
    // `lock_kp > 0` giữ chân và eo tại tư thế đã chốt thay vì để chúng limp.
    // Bản gốc luôn để limp; khoá cứng là hành vi riêng của bản cô lập này và
    // chỉ có nghĩa khi robot đang treo trên giá. Không chốt được tư thế thì
    // KHÔNG khoá: thà giữ nguyên hành vi cũ còn hơn khoá về một số không rõ
    // nguồn gốc.
    void SendUpperBodyZeroTorque(const std::array<float, spec::kNumJoints>& target_q,
                                 float arm_kp, float arm_kd, float max_rate,
                                 uint8_t mode_machine, const HeadTarget& head,
                                 float lock_kp = 0.0f, float lock_kd = 0.0f,
                                 float lock_max_rate = 0.0f) {
        unitree_hg::msg::dds_::LowCmd_ cmd{};
        cmd.mode_machine() = mode_machine;
        for (int i = 0; i < spec::kNumMotorsIdl; ++i) {
            cmd.motor_cmd()[i].mode() = 1;
            cmd.motor_cmd()[i].tau() = 0.0f;
            cmd.motor_cmd()[i].q() = 0.0f;
            cmd.motor_cmd()[i].dq() = 0.0f;
            cmd.motor_cmd()[i].kp() = 0.0f;
            cmd.motor_cmd()[i].kd() = 0.0f;
        }
        if (lock_kp > 0.0f && hold_latched_) {
            const float lock_step = (lock_max_rate > 0.0f ? lock_max_rate : max_rate) * spec::kLoopDt;
            for (int idl : kNonTeleopIdl) {
                const size_t slot = static_cast<size_t>(idl);
                const float q = Slew(last_cmd_q_[slot], hold_q_[slot], lock_step);
                last_cmd_q_[slot] = q;
                cmd.motor_cmd()[idl].q() = q;
                cmd.motor_cmd()[idl].kp() = lock_kp;
                cmd.motor_cmd()[idl].kd() = lock_kd;
            }
        }
        const float max_step = max_rate * spec::kLoopDt;
        for (int j = 0; j < spec::kNumArmJoints; ++j) {
            const int policy = spec::kArmBeginPolicyIdx + j;
            const int idl = spec::MotorIdl(policy);
            const float q = Slew(last_cmd_q_[idl], target_q[policy], max_step);
            last_cmd_q_[idl] = q;
            cmd.motor_cmd()[idl].q() = q;
            cmd.motor_cmd()[idl].kp() = arm_kp;
            cmd.motor_cmd()[idl].kd() = arm_kd;
        }
        if (head.valid) {
            const float head_step = head.max_rate_rad_s * spec::kLoopDt;
            SetHead(cmd, spec::kHeadYawIdl, head.yaw, tuning_->head_yaw_kp,
                    tuning_->head_yaw_kd, head_step);
            SetHead(cmd, spec::kHeadPitchIdl, head.pitch, tuning_->head_pitch_kp,
                    tuning_->head_pitch_kd, head_step);
        }
        cmd.crc() = Crc32(reinterpret_cast<uint32_t*>(&cmd),
                          (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
        RecordCrc(cmd.crc());
        pub_->Write(cmd);
    }

    float last_cmd_q_policy(int policy_idx) const {
        return last_cmd_q_[static_cast<size_t>(spec::MotorIdl(policy_idx))];
    }

    // Kiểm tra CRC của gói rt/lowcmd nhận về có khớp với gói vừa gửi để phân biệt với built-in.
    bool IsOurCrc(uint32_t crc) const {
        for (const auto& c : sent_crc_)
            if (c.load(std::memory_order_acquire) == crc) return true;
        return false;
    }

    // Số gói run_r1 đã gửi (phục vụ chẩn đoán self-echo).
    uint64_t SentCount() const { return sent_count_.load(std::memory_order_relaxed); }

private:
    // Chân trái 0-5, chân phải 6-11, eo roll/yaw 12-13. Đây là toàn bộ khớp
    // policy KHÔNG thuộc teleop; tay là 15-19/22-26 và đầu là 29/30. Các slot
    // IDL còn lại không nằm trong kSdkToIdl nên bản này không đụng tới: khoá
    // một motor không biết là gì thì tệ hơn để nó thụ động.
    static constexpr std::array<int, 14> kNonTeleopIdl = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
    };
    std::array<float, spec::kNumMotorsIdl> hold_q_{};
    bool hold_latched_ = false;

    void RecordCrc(uint32_t crc) {
        size_t h = crc_head_.fetch_add(1, std::memory_order_relaxed) % kCrcRing;
        sent_crc_[h].store(crc, std::memory_order_release);
        sent_count_.fetch_add(1, std::memory_order_relaxed);
    }

    static float Slew(float prev, float target, float max_step) {
        float delta = std::clamp(target - prev, -max_step, max_step);
        return prev + delta;
    }

    void SetHead(unitree_hg::msg::dds_::LowCmd_& cmd, int idl, float target,
                 float kp, float kd, float max_step) {
        float clamped = Slew(last_cmd_q_[idl], target, max_step);
        last_cmd_q_[idl] = clamped;
        cmd.motor_cmd()[idl].mode() = 1;
        cmd.motor_cmd()[idl].q() = clamped;
        cmd.motor_cmd()[idl].kp() = kp;
        cmd.motor_cmd()[idl].kd() = kd;
    }

    // Tính CRC32 checksum theo tiêu chuẩn của Unitree
    static uint32_t Crc32(uint32_t* ptr, uint32_t len) {
        uint32_t crc = 0xFFFFFFFF;
        const uint32_t poly = 0x04c11db7;
        for (uint32_t i = 0; i < len; ++i) {
            uint32_t xbit = 1u << 31;
            uint32_t data = ptr[i];
            for (uint32_t b = 0; b < 32; ++b) {
                if (crc & 0x80000000) {
                    crc <<= 1;
                    crc ^= poly;
                } else {
                    crc <<= 1;
                }
                if (data & xbit) crc ^= poly;
                xbit >>= 1;
            }
        }
        return crc;
    }

    const Tuning* tuning_ = nullptr;
    std::unique_ptr<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>> pub_;
    std::array<float, spec::kNumMotorsIdl> last_cmd_q_{};

    // Ring buffer lưu CRC gói gửi đi.
    static constexpr size_t kCrcRing = 32;
    std::array<std::atomic<uint32_t>, kCrcRing> sent_crc_{};
    std::atomic<size_t> crc_head_{0};
    std::atomic<uint64_t> sent_count_{0};   // chẩn đoán self-echo
};
