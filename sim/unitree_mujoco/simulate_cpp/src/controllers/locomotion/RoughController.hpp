#ifndef ROUGH_CONTROLLER_HPP
#define ROUGH_CONTROLLER_HPP

#include "runtime/PolicyRunner.hpp"
#include "runtime/R1Config.hpp"
#include <mujoco/mujoco.h>

class RoughController : public PolicyRunner {
public:
    static constexpr int kObsSize = 270;

    RoughController();
    explicit RoughController(const std::string& scene_xml_path);
    ~RoughController() override;

    bool LoadScene(const std::string& scene_xml_path);

    void Init(const std::string& model_path, Ort::Env& env, const Ort::SessionOptions& session_options) override;

    std::vector<float> ComputeObservation(
        const LowState_& robot_state,
        const SportModeState_& sport_state,
        float target_vx, float target_vy, float target_yaw,
        float gait_time, const std::array<float, 2>& gait_phase
    ) override;

    std::array<float, R1Config::NUM_JOINTS> ComputeTargetQ(const std::vector<float>& action) override;

    void Reset(const LowState_& current_state) override;

    int GetInputSize() const override { return kObsSize; }
    bool SceneLoaded() const { return mj_model_ != nullptr && mj_data_ != nullptr; }

private:
    std::array<float, R1Config::NUM_JOINTS> last_action_;

    // MuJoCo context for raycasting
    mjModel* mj_model_;
    mjData*  mj_data_;
    int      pelvis_body_id_ = -1;
    std::vector<std::pair<float, float>> local_grid_;

    void InitGrid();
    float cast_ray_to_ground(const mjtNum* ray_start, const mjtNum* ray_dir);
};

#endif
