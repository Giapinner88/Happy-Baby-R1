#include "RoughController.hpp"
#include "simulator/RaycastGeomFilter.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>

RoughController::RoughController() : mj_model_(nullptr), mj_data_(nullptr) {
    last_action_.fill(0.0f);
    InitGrid();
}

RoughController::RoughController(const std::string& scene_xml_path) : RoughController() {
    LoadScene(scene_xml_path);
}

bool RoughController::LoadScene(const std::string& scene_xml_path) {
    if (mj_data_) {
        mj_deleteData(mj_data_);
        mj_data_ = nullptr;
    }
    if (mj_model_) {
        mj_deleteModel(mj_model_);
        mj_model_ = nullptr;
    }
    pelvis_body_id_ = -1;

    // Load MuJoCo model for raycasting
    char error[1000] = "Could not load XML model";
    mj_model_ = mj_loadXML(scene_xml_path.c_str(), 0, error, 1000);
    if (!mj_model_) {
        std::cerr << "[LỖI] Không thể nạp MuJoCo XML từ " << scene_xml_path << ": " << error << std::endl;
        return false;
    } else {
        pelvis_body_id_ = mj_name2id(mj_model_, mjOBJ_BODY, "pelvis");
        if (pelvis_body_id_ >= 0) {
            const int excluded = RaycastGeomFilter::ExcludeBodySubtree(
                mj_model_, pelvis_body_id_);
            std::cout << "[RoughController] Height scan đã loại " << excluded
                      << " geom thuộc robot." << std::endl;
        }
        mj_data_ = mj_makeData(mj_model_);
        if (!mj_data_ || pelvis_body_id_ < 0) {
            std::cerr << "[LỖI] Scene Rough thiếu body pelvis dùng cho height scan." << std::endl;
            if (mj_data_) {
                mj_deleteData(mj_data_);
                mj_data_ = nullptr;
            }
            mj_deleteModel(mj_model_);
            mj_model_ = nullptr;
            pelvis_body_id_ = -1;
            return false;
        }
        std::cout << "[RoughController] Đã nạp thành công MuJoCo Raycasting từ: " << scene_xml_path << std::endl;
    }
    return mj_data_ != nullptr;
}

RoughController::~RoughController() {
    if (mj_data_) mj_deleteData(mj_data_);
    if (mj_model_) mj_deleteModel(mj_model_);
}

void RoughController::InitGrid() {
    float x_range[] = {-0.8f, 0.8f};
    float y_range[] = {-0.5f, 0.5f};
    float x_step = 0.1f;
    float y_step = 0.1f;
    // torch.meshgrid(x, y, indexing="xy").flatten(): x thay đổi nhanh nhất.
    for (float y = y_range[0]; y <= y_range[1] + 1e-5; y += y_step) {
        for (float x = x_range[0]; x <= x_range[1] + 1e-5; x += x_step) {
            local_grid_.push_back({x, y});
        }
    }
}

float RoughController::cast_ray_to_ground(const mjtNum* ray_start, const mjtNum* ray_dir) {
    if (!mj_model_ || !mj_data_ || pelvis_body_id_ < 0) return -1.0f;

    // Khớp RayCastSensorCfg lúc train: geom groups 0/1/2 và max_distance=5 m.
    // Toàn bộ geom robot đã được chuyển sang group 5 lúc LoadScene.
    const mjtByte geom_group[6] = {1, 1, 1, 0, 0, 0};
    int geom_id = -1;
    const mjtNum distance = mj_ray(
        mj_model_, mj_data_, ray_start, ray_dir,
        geom_group, 1, pelvis_body_id_, &geom_id);
    if (distance < 0.0 || distance > 5.0) return -1.0f;
    return static_cast<float>(distance);
}

void RoughController::Init(const std::string& model_path, Ort::Env& env, const Ort::SessionOptions& session_options) {
    OpenModel(model_path, env, session_options);
    ValidateSingleModelContract(GetInputSize());
    default_q_ = R1Config::DEFAULT_JOINT_POS;
    action_scale_ = R1Config::ACTION_SCALE;
    joint_stiffness_ = R1Config::KP_ARRAY;
    joint_damping_ = R1Config::KD_ARRAY;
    LoadMetadata();
    ValidateStringListMetadata("observation_names", {
        "base_ang_vel", "projected_gravity", "command", "phase",
        "joint_pos", "joint_vel", "actions", "height_scan",
    });
    std::cout << "[RoughController] Đã nạp thành công mô hình: " << model_path << std::endl;
}

std::vector<float> RoughController::ComputeObservation(
    const LowState_& robot_state,
    const SportModeState_& sport_state,
    float target_vx, float target_vy, float target_yaw,
    float gait_time, const std::array<float, 2>& gait_phase) 
{
    std::vector<float> obs(GetInputSize(), 0.0f);

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

    // 7. Height Scan via MuJoCo Raycasting (187)
    float p_pelvis_x = sport_state.position()[0];
    float p_pelvis_y = sport_state.position()[1];
    float p_pelvis_z = sport_state.position()[2];
    
    // Euler angles from quaternion for yaw
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    if (mj_model_ && mj_data_) {
        // Sync state to MuJoCo
        mj_data_->qpos[0] = p_pelvis_x;
        mj_data_->qpos[1] = p_pelvis_y;
        mj_data_->qpos[2] = p_pelvis_z;
        mj_data_->qpos[3] = qw;
        mj_data_->qpos[4] = qx;
        mj_data_->qpos[5] = qy;
        mj_data_->qpos[6] = qz;

        // Sync joint positions to prevent raycasting self-collision
        for (int i = 0; i < 24; ++i) {
            int idl_idx = R1Config::PolicyToIdl(i);
            mj_data_->qpos[7 + i] = robot_state.motor_state()[idl_idx].q();
        }
        
        mj_forward(mj_model_, mj_data_);

        mjtNum ray_dir[3] = {0.0, 0.0, -1.0};
        for (int i = 0; i < 187; ++i) {
            float dx = local_grid_[i].first;
            float dy = local_grid_[i].second;

            float dx_world = dx * std::cos(yaw) - dy * std::sin(yaw);
            float dy_world = dx * std::sin(yaw) + dy * std::cos(yaw);

            // Training phát tia từ đúng frame pelvis; yaw chỉ xoay offset/direction.
            mjtNum ray_start[3] = {p_pelvis_x + dx_world, p_pelvis_y + dy_world, p_pelvis_z};
            
            float dist = cast_ray_to_ground(ray_start, ray_dir);

            float relative_height = 5.0f;
            if (dist >= 0) {
                float ground_z = ray_start[2] - dist;
                relative_height = p_pelvis_z - ground_z;
            }
            relative_height = std::max(-5.0f, std::min(5.0f, relative_height));
            obs[83 + i] = relative_height * 0.2f;
        }
    } else {
        // Fallback if no XML loaded
        float relative_height_scaled = p_pelvis_z * 0.2f;
        for (int i = 0; i < 187; ++i) {
            obs[83 + i] = relative_height_scaled;
        }
    }

    return obs;
}

std::array<float, R1Config::NUM_JOINTS> RoughController::ComputeTargetQ(const std::vector<float>& action) {
    std::array<float, R1Config::NUM_JOINTS> target_q;
    for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
        last_action_[i] = action[i];
        target_q[i] = default_q_[i] + action[i] * action_scale_[i];
    }
    return target_q;
}

void RoughController::Reset(const LowState_& current_state) {
    last_action_.fill(0.0f);
}
