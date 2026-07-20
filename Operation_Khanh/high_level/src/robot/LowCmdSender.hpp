#pragma once
/**
 * LowCmdSender.hpp — Lớp gửi lệnh LowCmd_ xuống DDS.
 */

#include <algorithm>
#include <array>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include "../config/RobotSpec.hpp"
#include "../config/Tuning.hpp"

class LowCmdSender {
public:
    void Init(const Tuning& tuning) {
        tuning_ = &tuning;
        pub_ = std::make_unique<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>("rt/lowcmd");
        pub_->InitChannel();
        last_cmd_q_.fill(0.0f);
    }

    // Đồng bộ trạng thái trước khi thực hiện hành động mới
    void SyncToState(const unitree_hg::msg::dds_::LowState_& low) {
        for (int i = 0; i < spec::kNumMotorsIdl; ++i)
            last_cmd_q_[i] = low.motor_state()[i].q();
    }

    // Gửi lệnh q tới các khớp
    void Send(const std::array<float, spec::kNumJoints>& target_q,
              const std::array<float, spec::kNumJoints>& kp,
              const std::array<float, spec::kNumJoints>& kd,
              float max_rate, uint8_t mode_machine) {
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

        // Đầu: giữ 0 rad, gain nhẹ
        SetHead(cmd, spec::kHeadYawIdl, tuning_->head_yaw_kp, tuning_->head_yaw_kd, max_step);
        SetHead(cmd, spec::kHeadPitchIdl, tuning_->head_pitch_kp, tuning_->head_pitch_kd, max_step);

        cmd.crc() = Crc32(reinterpret_cast<uint32_t*>(&cmd),
                          (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
        pub_->Write(cmd);
    }

    // Xả toàn bộ lực (chế độ Damping)
    void Damping() {
        unitree_hg::msg::dds_::LowCmd_ cmd{};
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
        pub_->Write(cmd);
    }

    float last_cmd_q_policy(int policy_idx) const {
        return last_cmd_q_[static_cast<size_t>(spec::MotorIdl(policy_idx))];
    }

private:
    static float Slew(float prev, float target, float max_step) {
        float delta = std::clamp(target - prev, -max_step, max_step);
        return prev + delta;
    }

    void SetHead(unitree_hg::msg::dds_::LowCmd_& cmd, int idl, float kp, float kd,
                 float max_step) {
        float clamped = Slew(last_cmd_q_[idl], 0.0f, max_step);
        last_cmd_q_[idl] = clamped;
        cmd.motor_cmd()[idl].mode() = 1;
        cmd.motor_cmd()[idl].q() = clamped;
        cmd.motor_cmd()[idl].kp() = kp;
        cmd.motor_cmd()[idl].kd() = kd;
    }

    // Tính CRC32 cho gói tin
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
};
