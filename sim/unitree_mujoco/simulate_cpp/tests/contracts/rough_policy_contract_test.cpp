#include <onnxruntime_cxx_api.h>

#include <array>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::vector<std::string> Split(const std::string& value) {
    std::stringstream stream(value);
    std::string token;
    std::vector<std::string> result;
    while (std::getline(stream, token, ',')) result.push_back(token);
    return result;
}

std::string MetadataValue(Ort::Session& session, const char* key) {
    Ort::AllocatorWithDefaultOptions allocator;
    Ort::ModelMetadata metadata = session.GetModelMetadata();
    Ort::AllocatedStringPtr value =
        metadata.LookupCustomMetadataMapAllocated(key, allocator);
    if (!value) throw std::runtime_error(std::string("missing metadata: ") + key);
    return value.get();
}

std::vector<float> FloatMetadata(Ort::Session& session, const char* key) {
    std::vector<float> result;
    for (const auto& token : Split(MetadataValue(session, key))) {
        const float value = std::stof(token);
        if (!std::isfinite(value)) throw std::runtime_error(std::string(key) + " is non-finite");
        result.push_back(value);
    }
    if (result.size() != 24) {
        throw std::runtime_error(std::string(key) + " must contain 24 values");
    }
    return result;
}

void Require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: rough_policy_contract_test POLICY.onnx\n";
        return 2;
    }

    try {
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "rough_policy_contract_test");
        Ort::SessionOptions options;
        Ort::Session session(env, argv[1], options);

        Require(session.GetInputCount() == 1, "policy must have exactly one input");
        Require(session.GetOutputCount() == 1, "policy must have exactly one output");
        const auto input_shape = session.GetInputTypeInfo(0)
                                     .GetTensorTypeAndShapeInfo().GetShape();
        const auto output_shape = session.GetOutputTypeInfo(0)
                                      .GetTensorTypeAndShapeInfo().GetShape();
        Require(input_shape.size() == 2 && input_shape[1] == 270,
                "Rough policy input must be [batch,270]");
        Require(output_shape.size() == 2 && output_shape[1] == 24,
                "Rough policy output must be [batch,24]");

        const std::vector<std::string> expected_joints = {
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "waist_roll_joint", "waist_yaw_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
        };
        Require(Split(MetadataValue(session, "joint_names")) == expected_joints,
                "joint_names order does not match canonical R1 policy order");

        const std::vector<std::string> expected_observations = {
            "base_ang_vel", "projected_gravity", "command", "phase",
            "joint_pos", "joint_vel", "actions", "height_scan",
        };
        Require(Split(MetadataValue(session, "observation_names")) == expected_observations,
                "observation_names order does not match the 270-D Rough builder");

        (void)FloatMetadata(session, "action_scale");
        (void)FloatMetadata(session, "default_joint_pos");
        const auto stiffness = FloatMetadata(session, "joint_stiffness");
        const auto damping = FloatMetadata(session, "joint_damping");
        for (int i = 0; i < 24; ++i) {
            Require(stiffness[i] > 0.0f, "joint_stiffness must be positive");
            Require(damping[i] >= 0.0f, "joint_damping must be non-negative");
        }

        std::cout << "rough_policy_contract_test: PASS\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "rough_policy_contract_test: FAIL: " << e.what() << '\n';
        return 1;
    }
}
