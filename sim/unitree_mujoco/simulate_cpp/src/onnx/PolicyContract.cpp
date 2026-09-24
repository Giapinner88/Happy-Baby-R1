#include "onnx/PolicyContract.hpp"

#include "onnx/OnnxModel.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace r1::policy {

namespace {

template <std::size_t Size>
void LoadFloatArray(
    const OnnxModel& model,
    const char* key,
    std::array<float, Size>& destination) {
    const std::string value = model.Metadata(key);
    if (value.empty()) return;

    std::array<float, Size> parsed{};
    std::stringstream stream(value);
    std::string token;
    std::size_t count = 0;
    while (std::getline(stream, token, ',')) {
        if (count >= Size) {
            throw std::runtime_error(std::string(key) + " has too many values");
        }
        std::size_t consumed = 0;
        const float number = std::stof(token, &consumed);
        if (!std::isfinite(number)
            || token.find_first_not_of(" \t\r\n", consumed) != std::string::npos) {
            throw std::runtime_error(std::string(key) + " contains an invalid value");
        }
        parsed[count++] = number;
    }
    if (count != Size) {
        throw std::runtime_error(
            std::string(key) + " must contain exactly " + std::to_string(Size)
            + " values");
    }
    destination = parsed;
}

std::vector<std::string> SplitCsv(const std::string& value) {
    std::vector<std::string> result;
    std::stringstream stream(value);
    std::string token;
    while (std::getline(stream, token, ',')) result.push_back(token);
    return result;
}

}  // namespace

PolicyContract LoadPolicyContract(
    const OnnxModel& model,
    PolicyContract fallback,
    const std::vector<std::string>& expected_joint_names) {
    LoadFloatArray(model, "action_scale", fallback.action_scale);
    LoadFloatArray(model, "default_joint_pos", fallback.default_position);
    LoadFloatArray(model, "joint_stiffness", fallback.stiffness);
    LoadFloatArray(model, "joint_damping", fallback.damping);

    const std::string joint_names = model.Metadata("joint_names");
    if (!joint_names.empty() && SplitCsv(joint_names) != expected_joint_names) {
        throw std::runtime_error("joint_names metadata does not match R1 policy order");
    }
    return fallback;
}

}  // namespace r1::policy
