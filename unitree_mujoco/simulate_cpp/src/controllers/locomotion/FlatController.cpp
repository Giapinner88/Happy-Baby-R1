#include "FlatController.hpp"
#include <cmath>

FlatController::FlatController() {
    last_action_.fill(0.0f);
}

void FlatController::Init(const std::string& model_path, Ort::Env& env, const Ort::SessionOptions& session_options) {
    OpenModel(model_path, env, session_options);
    ValidateSingleModelContract(GetInputSize());
    default_q_ = R1Config::DEFAULT_JOINT_POS;
    action_scale_ = R1Config::ACTION_SCALE;
    joint_stiffness_ = R1Config::KP_ARRAY;
    joint_damping_ = R1Config::KD_ARRAY;
    LoadMetadata();
    
    std::cout << "[FlatController] Đã nạp thành công mô hình: " << model_path << std::endl;
}

std::vector<float> FlatController::ComputeObservation(
    const LowState_& robot_state,
    const SportModeState_& sport_state,
    float target_vx, float target_vy, float target_yaw,
    float gait_time, const std::array<float, 2>& gait_phase)
{
    return BuildBaseObservation83(robot_state, sport_state,
                                  target_vx, target_vy, target_yaw,
                                  gait_time, gait_phase);
}

std::vector<float> FlatController::BuildBaseObservation83(
    const LowState_& robot_state,
    const SportModeState_& sport_state,
    float target_vx, float target_vy, float target_yaw,
    float gait_time, const std::array<float, 2>& gait_phase) const
{
    std::vector<float> obs(kObsSize, 0.0f);

    // 1. Gyroscope (3)
    obs[0] = robot_state.imu_state().gyroscope()[0];
    obs[1] = robot_state.imu_state().gyroscope()[1];
    obs[2] = robot_state.imu_state().gyroscope()[2];

    // 2. Gravity Vector (3)
    float qw = robot_state.imu_state().quaternion()[0];
    float qx = robot_state.imu_state().quaternion()[1];
    float qy = robot_state.imu_state().quaternion()[2];
    float qz = robot_state.imu_state().quaternion()[3];

    float gx = 2.0f * (qw * qy - qx * qz);
    float gy = -2.0f * (qy * qz + qw * qx);
    float gz = 2.0f * (qx * qx + qy * qy) - 1.0f;
    obs[3] = gx;
    obs[4] = gy;
    obs[5] = gz;

    // 3. Command (3)
    obs[6] = target_vx;
    obs[7] = target_vy;
    obs[8] = target_yaw;

    // 4. Gait phase (2)
    obs[9] = gait_phase[0];
    obs[10] = gait_phase[1];

    // 5. Joint positions and velocities (48)
    for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
        int idl_idx = R1Config::PolicyToIdl(i);

        float q_real = robot_state.motor_state()[idl_idx].q();
        float dq_real = robot_state.motor_state()[idl_idx].dq();

        obs[11 + i] = q_real - default_q_[i];
        obs[35 + i] = dq_real;
    }

    // 6. Last action (24)
    for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
        obs[59 + i] = last_action_[i];
    }

    return obs;
}

std::array<float, R1Config::NUM_JOINTS> FlatController::ComputeTargetQ(const std::vector<float>& action) {
    std::array<float, R1Config::NUM_JOINTS> target_q;
    for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
        last_action_[i] = action[i];
        target_q[i] = default_q_[i] + action[i] * action_scale_[i];
    }
    return target_q;
}

void FlatController::Reset(const LowState_& current_state) {
    last_action_.fill(0.0f);
}
