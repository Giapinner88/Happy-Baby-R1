#pragma once

#include <array>
#include <cmath>
#include <string>

#include <eigen3/Eigen/Dense>
#include <unitree/idl/hg/LowState_.hpp>

#include "../config/RobotSpec.hpp"
#include "../config/Tuning.hpp"
#include "../util/Filters.hpp"

// Trạng thái robot đã chuẩn hóa
struct RobotState {
    std::array<float, spec::kNumJoints> q{};       // Thứ tự khớp của policy
    std::array<float, spec::kNumJoints> dq{};      // Vận tốc khớp đã lọc
    std::array<float, spec::kNumJoints> dq_raw{};

    Eigen::Quaternionf quat{1, 0, 0, 0};           // IMU pelvis (wxyz)
    Eigen::Vector3f gyro{0, 0, 0};                 // Vận tốc góc đã lọc
    Eigen::Vector3f gyro_raw{0, 0, 0};
    Eigen::Vector3f accel_raw{0, 0, 0};
    Eigen::Vector3f projected_gravity{0, 0, -1};

    uint8_t mode_machine = 0;

    float waist_roll() const { return q[spec::kWaistRollPolicyIdx]; }
    float waist_yaw()  const { return q[spec::kWaistYawPolicyIdx]; }
};

// Đọc LowState_ thô và chuẩn hóa thành RobotState cho các module khác
class StateEstimator {
public:
    void Configure(const Tuning& tuning, float dt) {
        gyro_lpf_.Configure(tuning.imu_gyro_lpf_hz, dt);
        dq_lpf_.Configure(tuning.joint_vel_lpf_hz, dt);
        dq_filter_enabled_ = tuning.joint_vel_lpf_hz > 0.0f;

        trim_affects_locomotion_ = tuning.imu_pitch_trim_affects_locomotion;
        trim_affects_dance_      = tuning.imu_pitch_trim_affects_dance;
        SetPitchTrimDeg(tuning.LocomotionTrimDeg());
    }

    // Application đổi trim ngay trước khi chạy từng Mimic. Áp lại lên state hiện tại
    // để Reset() của policy nhận đúng quaternion, không phải chờ tick DDS kế tiếp.
    void SetPitchTrimDeg(float trim_deg) {
        const float trim_rad = trim_deg * static_cast<float>(M_PI) / 180.0f;
        pitch_offset_quat_ = Eigen::Quaternionf(
            Eigen::AngleAxisf(trim_rad, Eigen::Vector3f::UnitY()));
        ApplyPitchTrim();
    }

    void Reset() {
        gyro_lpf_.Reset();
        dq_lpf_.Reset();
        fault_run_ = 0;
        sticky_fault_bits_ = 0;
        sticky_fault_index_ = -1;
    }

    // Cửa vào DUY NHẤT của dữ liệu robot -> LowState hỏng phải dừng ở đây.
    // Trả về false nếu gói này không dùng được; Application coi đó là fault và
    // về IDLE. Giá trị hỏng KHÔNG được chép vào state_ (giữ mẫu hợp lệ cuối).
    //
    // Vì sao cần: trước đây q/dq/quaternion được chép thẳng. Một kênh IMU chết
    // trả quaternion (0,0,0,0) -> công thức dưới cho projected_gravity =
    // (0,0,-1), tức "đứng thẳng hoàn hảo", và FallDetector mù hoàn toàn mà
    // không cần một NaN nào. Quaternion cũng chưa từng được chuẩn hoá dù mọi
    // công thức bên dưới giả định nó là đơn vị.
    bool Update(const unitree_hg::msg::dds_::LowState_& low) {
        fault_bits_ = 0;
        fault_index_ = -1;

        // Map khớp từ SDK sang thứ tự policy
        for (int i = 0; i < spec::kNumJoints; ++i) {
            int idl = spec::MotorIdl(i);
            const float q  = low.motor_state()[idl].q();
            const float dq = low.motor_state()[idl].dq();
            if (!std::isfinite(q) || !std::isfinite(dq)) {
                Fault(kBadJoint, i);
                continue;                      // giữ mẫu hợp lệ cuối của khớp này
            }
            state_.q[i]      = q;
            state_.dq_raw[i] = dq;
        }
        if (dq_filter_enabled_) {
            dq_lpf_.Update(state_.dq_raw, state_.dq);
        } else {
            state_.dq = state_.dq_raw;
        }

        // Lọc thông số IMU
        const auto& imu = low.imu_state();
        Eigen::Quaternionf quat_raw(imu.quaternion()[0], imu.quaternion()[1],
                                    imu.quaternion()[2], imu.quaternion()[3]);
        const float quat_norm = quat_raw.norm();
        // Chuẩn: norm phải ~1. Ngoài [0.5, 2.0] là kênh hỏng chứ không phải sai
        // số — kể cả khi vẫn hữu hạn (ví dụ toàn 0 = IMU chết).
        if (!std::isfinite(quat_norm) || quat_norm < 0.5f || quat_norm > 2.0f) {
            Fault(kBadQuat, -1);
        } else {
            raw_quat_ = quat_raw.normalized();
            ApplyPitchTrim();
        }

        std::array<float, 3> gyro_in = {imu.gyroscope()[0], imu.gyroscope()[1],
                                        imu.gyroscope()[2]};
        if (std::isfinite(gyro_in[0]) && std::isfinite(gyro_in[1]) &&
            std::isfinite(gyro_in[2])) {
            state_.gyro_raw = Eigen::Vector3f(gyro_in[0], gyro_in[1], gyro_in[2]);
            std::array<float, 3> gyro_out;
            gyro_lpf_.Update(gyro_in, gyro_out);
            state_.gyro = Eigen::Vector3f(gyro_out[0], gyro_out[1], gyro_out[2]);
        } else {
            Fault(kBadGyro, -1);
        }

        const Eigen::Vector3f accel(imu.accelerometer()[0],
                                    imu.accelerometer()[1],
                                    imu.accelerometer()[2]);
        if (accel.allFinite()) state_.accel_raw = accel;
        else Fault(kBadAccel, -1);

        // Tính trọng lực chiếu theo tọa độ robot từ quaternion đã bù pitch.
        state_.mode_machine = low.mode_machine();

        // Debounce: một gói dị thường lẻ KHÔNG được kéo robot về damping giữa lúc
        // đang đi. An toàn để bỏ qua vì dữ liệu hỏng đã bị chặn ở trên rồi —
        // state_ vẫn giữ mẫu hợp lệ cuối, không có gì lọt vào obs hay history.
        // Hỏng thật (IMU chết, cáp đứt) thì liên tục nên vẫn chốt sau kFaultTicks
        // gói = 6 ms ở 500 Hz.
        if (fault_bits_ == 0) {
            fault_run_ = 0;
            sticky_fault_bits_ = 0;
            sticky_fault_index_ = -1;
        } else {
            ++fault_run_;
            sticky_fault_bits_ |= fault_bits_;
            if (sticky_fault_index_ < 0) sticky_fault_index_ = fault_index_;
        }
        return fault_run_ < kFaultTicks;
    }

    bool valid() const { return fault_run_ < kFaultTicks; }
    bool sample_valid() const { return fault_bits_ == 0; }

    // Số gói bị bỏ vì hỏng nhưng chưa đủ liên tiếp để dừng (chẩn đoán cáp/nhiễu).
    uint64_t repaired_count() const { return repaired_count_; }

    std::string FaultText() const {
        const uint32_t bits = sticky_fault_bits_ ? sticky_fault_bits_ : fault_bits_;
        std::string out = "LowState khong hop le (";
        if (bits & kBadJoint)
            out += "khop " + std::to_string(sticky_fault_index_) + " ";
        if (bits & kBadQuat)  out += "quaternion IMU ";
        if (bits & kBadGyro)  out += "gyro ";
        if (bits & kBadAccel) out += "accel ";
        out += "| " + std::to_string(fault_run_) + " goi lien tiep)";
        return out;
    }

    const RobotState& state() const { return state_; }

private:
    enum : uint32_t {
        kBadJoint = 1u << 0,
        kBadQuat  = 1u << 1,
        kBadGyro  = 1u << 2,
        kBadAccel = 1u << 3,
    };
    // Số gói hỏng LIÊN TIẾP trước khi coi là mất cảm biến. 3 gói = 6 ms ở 500 Hz.
    static constexpr int kFaultTicks = 3;

    void Fault(uint32_t bit, int index) {
        if (fault_bits_ == 0) ++repaired_count_;   // gói này đã phải vá
        fault_bits_ |= bit;
        if (fault_index_ < 0) fault_index_ = index;
    }

    void ApplyPitchTrim() {
        // Quaternion đã bù pitch — dùng có chọn lọc cho Mimic và locomotion/fall-detector.
        const Eigen::Quaternionf quat_trimmed = raw_quat_ * pitch_offset_quat_;
        state_.quat = trim_affects_dance_ ? quat_trimmed : raw_quat_;
        const Eigen::Quaternionf& q_grav =
            trim_affects_locomotion_ ? quat_trimmed : raw_quat_;
        const float qw = q_grav.w(), qx = q_grav.x();
        const float qy = q_grav.y(), qz = q_grav.z();
        state_.projected_gravity = Eigen::Vector3f(
            2.0f * (qw * qy - qx * qz),
            -2.0f * (qy * qz + qw * qx),
            2.0f * (qx * qx + qy * qy) - 1.0f);
    }

    uint32_t fault_bits_ = 0;
    int fault_index_ = -1;
    uint32_t sticky_fault_bits_ = 0;   // gộp qua cả chuỗi hỏng, cho FaultText()
    int sticky_fault_index_ = -1;
    int fault_run_ = 0;                // số gói hỏng liên tiếp
    uint64_t repaired_count_ = 0;

    RobotState state_;
    LowPassVec<3> gyro_lpf_;
    LowPassVec<spec::kNumJoints> dq_lpf_;
    bool dq_filter_enabled_ = false;
    bool trim_affects_locomotion_ = true;
    bool trim_affects_dance_ = false;
    Eigen::Quaternionf raw_quat_{1, 0, 0, 0};
    Eigen::Quaternionf pitch_offset_quat_{1, 0, 0, 0};
};
