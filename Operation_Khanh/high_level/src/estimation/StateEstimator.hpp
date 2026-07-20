#pragma once
/**
 * StateEstimator.hpp — Lớp DUY NHẤT đọc LowState_ thô.
 */

#include <array>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <unitree/idl/hg/LowState_.hpp>

#include "../config/RobotSpec.hpp"
#include "../config/Tuning.hpp"
#include "../util/Filters.hpp"

// Trạng thái robot đã chuẩn hóa
struct RobotState {
    std::array<float, spec::kNumJoints> q{};
    std::array<float, spec::kNumJoints> dq{};
    std::array<float, spec::kNumJoints> dq_raw{};

    Eigen::Quaternionf quat{1, 0, 0, 0};
    Eigen::Vector3f gyro{0, 0, 0};
    Eigen::Vector3f gyro_raw{0, 0, 0};
    Eigen::Vector3f accel_raw{0, 0, 0};
    Eigen::Vector3f projected_gravity{0, 0, -1};

    uint8_t mode_machine = 0;

    float waist_roll() const { return q[spec::kWaistRollPolicyIdx]; }
    float waist_yaw()  const { return q[spec::kWaistYawPolicyIdx]; }
};

class StateEstimator {
public:
    void Configure(const Tuning& tuning, float dt) {
        gyro_lpf_.Configure(tuning.imu_gyro_lpf_hz, dt);
        dq_lpf_.Configure(tuning.joint_vel_lpf_hz, dt);
        dq_filter_enabled_ = tuning.joint_vel_lpf_hz > 0.0f;

        float trim_rad = tuning.imu_pitch_trim_deg * static_cast<float>(M_PI) / 180.0f;
        pitch_offset_quat_ = Eigen::Quaternionf(
            Eigen::AngleAxisf(trim_rad, Eigen::Vector3f::UnitY()));
    }

    void Reset() {
        gyro_lpf_.Reset();
        dq_lpf_.Reset();
    }

    void Update(const unitree_hg::msg::dds_::LowState_& low) {
        for (int i = 0; i < spec::kNumJoints; ++i) {
            int idl = spec::MotorIdl(i);
            state_.q[i]      = low.motor_state()[idl].q();
            state_.dq_raw[i] = low.motor_state()[idl].dq();
        }
        if (dq_filter_enabled_) {
            dq_lpf_.Update(state_.dq_raw, state_.dq);
        } else {
            state_.dq = state_.dq_raw;
        }

        const auto& imu = low.imu_state();
        state_.quat = Eigen::Quaternionf(imu.quaternion()[0], imu.quaternion()[1],
                                         imu.quaternion()[2], imu.quaternion()[3]);
        std::array<float, 3> gyro_in = {imu.gyroscope()[0], imu.gyroscope()[1],
                                        imu.gyroscope()[2]};
        state_.gyro_raw = Eigen::Vector3f(gyro_in[0], gyro_in[1], gyro_in[2]);
        std::array<float, 3> gyro_out;
        gyro_lpf_.Update(gyro_in, gyro_out);
        state_.gyro = Eigen::Vector3f(gyro_out[0], gyro_out[1], gyro_out[2]);

        state_.accel_raw = Eigen::Vector3f(imu.accelerometer()[0],
                                           imu.accelerometer()[1],
                                           imu.accelerometer()[2]);

        Eigen::Quaternionf q_grav = state_.quat * pitch_offset_quat_;
        float qw = q_grav.w(), qx = q_grav.x();
        float qy = q_grav.y(), qz = q_grav.z();
        state_.projected_gravity = Eigen::Vector3f(
            2.0f * (qw * qy - qx * qz),
            -2.0f * (qy * qz + qw * qx),
            2.0f * (qx * qx + qy * qy) - 1.0f);

        state_.mode_machine = low.mode_machine();
    }

    const RobotState& state() const { return state_; }

private:
    RobotState state_;
    LowPassVec<3> gyro_lpf_;
    LowPassVec<spec::kNumJoints> dq_lpf_;
    bool dq_filter_enabled_ = false;
    Eigen::Quaternionf pitch_offset_quat_{1, 0, 0, 0};
};
