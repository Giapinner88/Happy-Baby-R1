#pragma once
/**
 * LowCmdSender.hpp — Lớp DUY NHẤT viết LowCmd_ xuống DDS.
 *
 * - Cổng an toàn cuối: mọi số vào packet motor phải hữu hạn và nằm trong cổng.
 * - Rate limiter trên lệnh (dùng lệnh trước làm gốc).
 * - Rate limit đặt cao khi chạy policy để chặn spike đột ngột.
 * - Đầu (idl 29/30) giữ 0 rad với gain nhẹ.
 * - CRC32 bắt buộc cho robot thật.
 * - Damping() = Kp/Kd/tau về 0 toàn bộ 35 motor (xả lực khẩn cấp).
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include "../config/RobotSpec.hpp"
#include "../config/Tuning.hpp"

// Các khớp policy không thuộc teleop tay/đầu. Chỉ tập này được phép giữ tại
// encoder chốt trong pilot Mode Z; những slot IDL không có mapping vẫn để limp.
namespace teleop_hold {

inline constexpr std::array<int, 14> kNonTeleopIdl = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
};

constexpr bool DisjointFromTeleop() {
    for (int idl : kNonTeleopIdl) {
        if (idl == spec::kHeadYawIdl || idl == spec::kHeadPitchIdl) return false;
        for (int j = 0; j < spec::kNumArmJoints; ++j) {
            if (idl == spec::MotorIdl(spec::kArmBeginPolicyIdx + j)) return false;
        }
    }
    return true;
}

static_assert(DisjointFromTeleop(),
              "a held joint is also driven by teleop");

}  // namespace teleop_hold

// Mục tiêu điều khiển đầu (teleop hoặc gesture). valid=false -> giữ đầu ở 0.
// max_rate_rad_s<=0 kế thừa rate-limit của policy; gesture dùng trần riêng thấp hơn.
struct HeadTarget {
    bool  valid = false;
    float yaw   = 0.0f;
    float pitch = 0.0f;
    float max_rate_rad_s = 0.0f;
};

struct LowCmdSenderTestAccess;

// Lớp gửi lệnh LowCmd_ xuống robot qua kênh DDS rt/lowcmd
class LowCmdSender {
public:
    void Init(const Tuning& tuning) {
        tuning_ = &tuning;
        pub_ = std::make_unique<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>("rt/lowcmd");
        pub_->InitChannel();
        last_cmd_q_.fill(0.0f);
    }

    // Đồng bộ lệnh trước đó với vị trí hiện tại của robot. Encoder không hữu hạn
    // KHÔNG được chép vào last_cmd_q_: mảng đó là gốc của mọi Slew sau này, nhiễm
    // một lần là khớp đó hỏng vĩnh viễn cho tới khi restart.
    void SyncToState(const unitree_hg::msg::dds_::LowState_& low) {
        for (int i = 0; i < spec::kNumMotorsIdl; ++i) {
            const float q = low.motor_state()[i].q();
            if (std::isfinite(q)) last_cmd_q_[i] = q;
            else RaiseFault(kFaultEncoder, i);   // giữ nguyên lệnh cũ của khớp đó
        }
    }

    // Gửi vị trí khớp mục tiêu kèm gains và giới hạn tốc độ (rate limit)
    void Send(const std::array<float, spec::kNumJoints>& target_q,
              const std::array<float, spec::kNumJoints>& kp,
              const std::array<float, spec::kNumJoints>& kd,
              float max_rate, uint8_t mode_machine,
              const HeadTarget& head = {}) {
        unitree_hg::msg::dds_::LowCmd_ cmd{};
        BuildSend(cmd, target_q, kp, kd, max_rate, mode_machine, head);
        RecordCrc(cmd.crc());
        WritePacket(cmd);
    }


    // Mode Z: giữ chân, waist và mọi slot IDL khác ở zero torque; chỉ cấp PD
    // cho 10 khớp tay hợp lệ và (tuỳ chọn) 2 khớp đầu. Đây vẫn là cùng một
    // publisher rt/lowcmd — không tạo motor owner thứ hai.
    void SendUpperBodyZeroTorque(
              const std::array<float, spec::kNumJoints>& target_q,
              bool arm_valid, float arm_kp, float arm_kd,
              float max_rate, uint8_t mode_machine,
              const HeadTarget& head = {},
              float hold_kp = 0.0f, float hold_kd = 0.0f,
              float hold_max_rate = 0.0f) {
        unitree_hg::msg::dds_::LowCmd_ cmd{};
        BuildUpperBodyZeroTorque(cmd, target_q, arm_valid, arm_kp, arm_kd,
                                 max_rate, mode_machine, head,
                                 hold_kp, hold_kd, hold_max_rate);
        RecordCrc(cmd.crc());
        WritePacket(cmd);
    }

    // Chốt đúng encoder ở frame teleop bắt đầu active. Encoder không hữu hạn
    // làm riêng khớp đó rơi về zero torque, không biến thành target vị trí.
    void LatchNonTeleopHold(const unitree_hg::msg::dds_::LowState_& low) {
        for (int idl : teleop_hold::kNonTeleopIdl) {
            const size_t slot = static_cast<size_t>(idl);
            const float q = low.motor_state()[idl].q();
            hold_ok_[slot] = std::isfinite(q);
            hold_q_[slot] = hold_ok_[slot] ? q : 0.0f;
        }
        hold_latched_ = true;
    }

    void ClearNonTeleopHold() {
        hold_latched_ = false;
        hold_ok_.fill(false);
    }

    bool NonTeleopHoldLatched() const { return hold_latched_; }

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
        WritePacket(cmd);
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
        WritePacket(cmd);
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

    // Application đặt mỗi tick TRƯỚC mọi lệnh gửi. false = LowState đã cũ hơn
    // watchdog DDS.
    //
    // Đây là bất biến cuối cùng, không phải bản sao của watchdog trạng thái:
    // watchdog chỉ phủ các state có tên trong danh sách của nó, còn Mode Z lại
    // tự gate đường PD của mình bên trong RunZeroTorqueTeleop(). Đặt điều kiện
    // ở lớp DUY NHẤT ghi rt/lowcmd thì mọi đường — kể cả đường thêm sau này —
    // đều không thể cấp dòng cho motor khi cảm biến đã chết.
    void SetSensorFresh(bool fresh) { sensor_fresh_ = fresh; }
    bool sensor_fresh() const { return sensor_fresh_; }

    // ─── Cổng an toàn cuối ────────────────────────────────────────────────
    // Hai mức phản ứng, cố ý khác nhau:
    //  - KHÔNG HỮU HẠN (NaN/Inf): không có cách nào diễn giải đúng -> giữ lệnh
    //    cũ cho frame này và bật fault; Application đọc fault và về IDLE
    //    (damping). Trước đây std::clamp(NaN, lo, hi) trả thẳng NaN (cả hai so
    //    sánh với NaN đều false), NaN vào packet motor VÀ vào last_cmd_q_, nên
    //    khớp đó hỏng vĩnh viễn kể cả sau khi nguồn lỗi đã hết.
    //  - VƯỢT CỔNG: clamp + đếm, KHÔNG đổi trạng thái. Cổng rộng hơn tầm cơ khí
    //    nhiều lần (spec::kGuardSlack) nên chạm được nghĩa là lệnh đã là rác;
    //    nhưng damping giữa lúc robot đang dồn trọng lượng lên một chân còn
    //    nguy hơn một giá trị bị cắt.
    bool HasFault() const {
        return fault_bits_.load(std::memory_order_acquire) != kFaultNone;
    }
    uint64_t ClampCount() const { return clamp_count_.load(std::memory_order_relaxed); }
    void ClearFault() {
        fault_bits_.store(kFaultNone, std::memory_order_release);
        fault_joint_.store(-1, std::memory_order_relaxed);
    }
    std::string FaultText() const {
        const uint32_t bits = fault_bits_.load(std::memory_order_acquire);
        std::string out = "lenh motor khong huu han (";
        if (bits & kFaultTarget)  out += "target ";
        if (bits & kFaultGain)    out += "gain ";
        if (bits & kFaultRate)    out += "rate-limit ";
        if (bits & kFaultEncoder) out += "encoder ";
        const int j = fault_joint_.load(std::memory_order_relaxed);
        out += "| khop " + (j >= 0 ? std::to_string(j) : std::string("dau/khong ro")) + ")";
        return out;
    }

private:
    friend struct LowCmdSenderTestAccess;
#ifdef HB_TEST_PACKET_SINK
    // Compiled only in Application regression tests: never initialize DDS.
    void (*test_packet_sink_)(const unitree_hg::msg::dds_::LowCmd_&) = nullptr;
#endif
    void WritePacket(const unitree_hg::msg::dds_::LowCmd_& cmd) {
#ifdef HB_TEST_PACKET_SINK
        if (test_packet_sink_) { test_packet_sink_(cmd); return; }
#endif
        pub_->Write(cmd);
    }

    enum : uint32_t {
        kFaultNone    = 0,
        kFaultTarget  = 1u << 0,
        kFaultGain    = 1u << 1,
        kFaultRate    = 1u << 2,
        kFaultEncoder = 1u << 3,
    };
    static constexpr uint64_t kClampLogEvery = 500;   // 1 s ở 500 Hz

    void RaiseFault(uint32_t bit, int policy_idx) {
        fault_bits_.fetch_or(bit, std::memory_order_release);
        int none = -1;   // giữ lại khớp hỏng ĐẦU TIÊN, không để khớp sau ghi đè
        fault_joint_.compare_exchange_strong(none, policy_idx,
                                             std::memory_order_relaxed);
    }

    void NoteClamp(int policy_idx, float value) {
        const uint64_t n = clamp_count_.fetch_add(1, std::memory_order_relaxed);
        if (n % kClampLogEvery == 0) {
            std::cerr << "[LowCmdSender] target ";
            if (policy_idx >= 0) std::cerr << "khop " << policy_idx;
            else                 std::cerr << "dau";
            std::cerr << " = " << value << " rad ngoai cong an toan -> clamp (lan "
                      << n + 1 << ")\n";
        }
    }

    // Target không hữu hạn -> giữ nguyên lệnh cũ của khớp đó. Không có giá trị
    // nào an toàn hơn: về 0 là một bước nhảy tư thế, giữ nguyên thì rate-limit
    // và fault phía Application lo phần còn lại.
    float SafeTarget(int policy_idx, float target, float prev) {
        if (!std::isfinite(target)) {
            RaiseFault(kFaultTarget, policy_idx);
            return std::isfinite(prev) ? prev : 0.0f;
        }
        const float lo = spec::kJointGuardMin[static_cast<size_t>(policy_idx)];
        const float hi = spec::kJointGuardMax[static_cast<size_t>(policy_idx)];
        if (target < lo || target > hi) {
            NoteClamp(policy_idx, target);
            return std::clamp(target, lo, hi);
        }
        return target;
    }

    float SafeHeadTarget(int idl, float target, float prev) {
        const float lo = idl == spec::kHeadYawIdl ? spec::kHeadYawMin : spec::kHeadPitchMin;
        const float hi = idl == spec::kHeadYawIdl ? spec::kHeadYawMax : spec::kHeadPitchMax;
        if (!std::isfinite(target)) {
            RaiseFault(kFaultTarget, -1);
            return std::isfinite(prev) ? prev : 0.0f;
        }
        if (target < lo || target > hi) {
            NoteClamp(-1, target);
            return std::clamp(target, lo, hi);
        }
        return target;
    }

    // Gain không hữu hạn -> 0 (nhả khớp đó) + fault. Kp=NaN là dòng không xác
    // định trong firmware; một frame nhả rồi damping vẫn hơn.
    float SafeGain(float gain, float ceiling, int policy_idx) {
        // Cảm biến chết -> không PD, bất kể caller yêu cầu gì.
        if (!sensor_fresh_) return 0.0f;
        if (!std::isfinite(gain)) {
            RaiseFault(kFaultGain, policy_idx);
            return 0.0f;
        }
        return std::clamp(gain, 0.0f, ceiling);
    }

    // Rate không hữu hạn là MẤT HẲN rate-limit chứ không phải giới hạn rộng:
    // std::clamp(delta, NaN, NaN) trả delta nguyên vẹn, tức lệnh nhảy thẳng tới
    // target trong đúng một frame. Fail-closed = bước 0 (đứng yên).
    float SafeStep(float max_rate) {
        if (!std::isfinite(max_rate)) {
            RaiseFault(kFaultRate, -1);
            return 0.0f;
        }
        return std::clamp(max_rate, 0.0f, spec::kMaxCmdRate) * spec::kLoopDt;
    }

    void BuildSend(unitree_hg::msg::dds_::LowCmd_& cmd,
                   const std::array<float, spec::kNumJoints>& target_q,
                   const std::array<float, spec::kNumJoints>& kp,
                   const std::array<float, spec::kNumJoints>& kd,
                   float max_rate, uint8_t mode_machine,
                   const HeadTarget& head) {
        const float max_step = SafeStep(max_rate);
        cmd = unitree_hg::msg::dds_::LowCmd_{};
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
            const int idl = spec::MotorIdl(i);
            const float target = SafeTarget(i, target_q[i], last_cmd_q_[idl]);
            const float clamped = Slew(last_cmd_q_[idl], target, max_step);
            last_cmd_q_[idl] = clamped;
            cmd.motor_cmd()[idl].q() = clamped;
            cmd.motor_cmd()[idl].kp() = SafeGain(kp[i], spec::kMaxCmdKp, i);
            cmd.motor_cmd()[idl].kd() = SafeGain(kd[i], spec::kMaxCmdKd, i);
        }

        // Đặt vị trí đầu: teleop/gesture nếu valid, ngược lại giữ 0. Gesture có
        // rate-limit riêng để một frame NPZ lỗi không thể quật đầu đột ngột.
        float hy = head.valid ? head.yaw   : 0.0f;
        float hp = head.valid ? head.pitch : 0.0f;
        const float head_step = head.max_rate_rad_s > 0.0f
            ? SafeStep(head.max_rate_rad_s) : max_step;
        SetHead(cmd, spec::kHeadYawIdl,   hy, tuning_->head_yaw_kp,   tuning_->head_yaw_kd,   head_step);
        SetHead(cmd, spec::kHeadPitchIdl, hp, tuning_->head_pitch_kp, tuning_->head_pitch_kd, head_step);

        cmd.crc() = Crc32(reinterpret_cast<uint32_t*>(&cmd),
                          (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
    }

    void BuildUpperBodyZeroTorque(
              unitree_hg::msg::dds_::LowCmd_& cmd,
              const std::array<float, spec::kNumJoints>& target_q,
              bool arm_valid, float arm_kp, float arm_kd,
              float max_rate, uint8_t mode_machine,
              const HeadTarget& head,
              float hold_kp = 0.0f, float hold_kd = 0.0f,
              float hold_max_rate = 0.0f) {
        cmd = unitree_hg::msg::dds_::LowCmd_{};
        cmd.mode_machine() = mode_machine;
        for (int i = 0; i < spec::kNumMotorsIdl; ++i) {
            cmd.motor_cmd()[i].mode() = 1;
            cmd.motor_cmd()[i].tau() = 0.0f;
            cmd.motor_cmd()[i].q() = 0.0f;
            cmd.motor_cmd()[i].dq() = 0.0f;
            cmd.motor_cmd()[i].kp() = 0.0f;
            cmd.motor_cmd()[i].kd() = 0.0f;
        }

        if (hold_latched_ && hold_kp > 0.0f) {
            const float hold_rate = hold_max_rate > 0.0f
                ? hold_max_rate : max_rate;
            const float hold_step = SafeStep(hold_rate);
            const float safe_hold_kp = SafeGain(hold_kp, spec::kMaxCmdKp, -1);
            const float safe_hold_kd = SafeGain(hold_kd, spec::kMaxCmdKd, -1);
            for (int idl : teleop_hold::kNonTeleopIdl) {
                const size_t slot = static_cast<size_t>(idl);
                if (!hold_ok_[slot]) continue;
                const float q = Slew(last_cmd_q_[slot], hold_q_[slot], hold_step);
                last_cmd_q_[slot] = q;
                cmd.motor_cmd()[idl].q() = q;
                cmd.motor_cmd()[idl].kp() = safe_hold_kp;
                cmd.motor_cmd()[idl].kd() = safe_hold_kd;
            }
        }

        const float max_step = SafeStep(max_rate);
        if (arm_valid) {
            for (int i = spec::kArmBeginPolicyIdx;
                 i < spec::kArmBeginPolicyIdx + spec::kNumArmJoints; ++i) {
                const int idl = spec::MotorIdl(i);
                const float target = SafeTarget(i, target_q[i], last_cmd_q_[idl]);
                const float clamped = Slew(last_cmd_q_[idl], target, max_step);
                last_cmd_q_[idl] = clamped;
                cmd.motor_cmd()[idl].q() = clamped;
                cmd.motor_cmd()[idl].kp() = SafeGain(arm_kp, spec::kMaxCmdKp, i);
                cmd.motor_cmd()[idl].kd() = SafeGain(arm_kd, spec::kMaxCmdKd, i);
            }
        }

        // Head invalid phải zero torque ngay; không dùng hành vi "giữ 0 rad"
        // của Send() thường vì Mode Z yêu cầu toàn bộ kênh không hợp lệ phải limp.
        if (head.valid) {
            const float head_step = head.max_rate_rad_s > 0.0f
                ? SafeStep(head.max_rate_rad_s) : max_step;
            SetHead(cmd, spec::kHeadYawIdl, head.yaw,
                    tuning_->head_yaw_kp, tuning_->head_yaw_kd, head_step);
            SetHead(cmd, spec::kHeadPitchIdl, head.pitch,
                    tuning_->head_pitch_kp, tuning_->head_pitch_kd, head_step);
        }
        cmd.crc() = Crc32(reinterpret_cast<uint32_t*>(&cmd),
                          (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
    }

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
        const float safe_target = SafeHeadTarget(idl, target, last_cmd_q_[idl]);
        float clamped = Slew(last_cmd_q_[idl], safe_target, max_step);
        last_cmd_q_[idl] = clamped;
        cmd.motor_cmd()[idl].mode() = 1;
        cmd.motor_cmd()[idl].q() = clamped;
        cmd.motor_cmd()[idl].kp() = SafeGain(kp, spec::kMaxCmdKp, -1);
        cmd.motor_cmd()[idl].kd() = SafeGain(kd, spec::kMaxCmdKd, -1);
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
    // Cờ cổng an toàn — DDS callback không đụng tới, nhưng Application đọc ở
    // luồng chính nên vẫn để atomic cho đúng chuẩn.
    std::atomic<uint32_t> fault_bits_{kFaultNone};
    std::atomic<int> fault_joint_{-1};
    std::atomic<uint64_t> clamp_count_{0};
    bool sensor_fresh_ = true;

    std::array<float, spec::kNumMotorsIdl> hold_q_{};
    std::array<bool, spec::kNumMotorsIdl> hold_ok_{};
    bool hold_latched_ = false;

    // Ring buffer lưu CRC gói gửi đi.
    static constexpr size_t kCrcRing = 32;
    std::array<std::atomic<uint32_t>, kCrcRing> sent_crc_{};
    std::atomic<size_t> crc_head_{0};
    std::atomic<uint64_t> sent_count_{0};   // chẩn đoán self-echo
};
