#include "OnnxPolicy.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>

void OnnxPolicy::Load(const std::string& model_path, Ort::Env& env,
                      const Ort::SessionOptions& options, int expected_obs_size) {
    path_ = model_path;
    session_ = std::make_unique<Ort::Session>(env, model_path.c_str(), options);

    // Get input dimensions and validate obs size
    Ort::TypeInfo type_info = session_->GetInputTypeInfo(0);
    auto input_info = type_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> dims = input_info.GetShape();
    obs_size_ = (dims.size() >= 2) ? static_cast<int>(dims[1]) : -1;
    if (obs_size_ != expected_obs_size) {
        throw std::runtime_error("Model " + model_path + " has input size " +
                                 std::to_string(obs_size_) + ", expected " +
                                 std::to_string(expected_obs_size));
    }

    Ort::TypeInfo output_type = session_->GetOutputTypeInfo(0);
    const std::vector<int64_t> output_dims =
        output_type.GetTensorTypeAndShapeInfo().GetShape();
    const int output_size =
        output_dims.empty() ? -1 : static_cast<int>(output_dims.back());
    if (output_size != spec::kNumJoints) {
        throw std::runtime_error("Model " + model_path + " has output size " +
                                 std::to_string(output_size) + ", expected " +
                                 std::to_string(spec::kNumJoints));
    }

    input_name_ = session_->GetInputNameAllocated(0, allocator_).get();
    output_name_ = session_->GetOutputNameAllocated(0, allocator_).get();

    // Check if model contains metadata
    try {
        Ort::ModelMetadata md = session_->GetModelMetadata();
        auto keys = md.GetCustomMetadataMapKeysAllocated(allocator_);
        has_metadata_ = !keys.empty();
    } catch (...) {
        has_metadata_ = false;
    }

    std::cout << "[OnnxPolicy] Loaded " << model_path << " (obs=" << obs_size_
              << ", metadata=" << (has_metadata_ ? "yes" : "NO") << ")\n";
}

std::vector<float> OnnxPolicy::Infer(std::vector<float>& obs) {
    std::vector<int64_t> shape = {1, static_cast<int64_t>(obs.size())};
    auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input = Ort::Value::CreateTensor<float>(mem, obs.data(), obs.size(),
                                                       shape.data(), shape.size());
    const char* in_names[] = {input_name_.c_str()};
    const char* out_names[] = {output_name_.c_str()};
    auto output = session_->Run(Ort::RunOptions{nullptr}, in_names, &input, 1, out_names, 1);

    const float* data = output[0].GetTensorData<float>();
    return std::vector<float>(data, data + spec::kNumJoints);
}

bool OnnxPolicy::ReadMetadataArray(const char* key,
                                   std::array<float, spec::kNumJoints>& out) {
    try {
        Ort::ModelMetadata md = session_->GetModelMetadata();
        Ort::AllocatedStringPtr ptr = md.LookupCustomMetadataMapAllocated(key, allocator_);
        if (!ptr) return false;

        std::array<float, spec::kNumJoints> tmp;
        std::stringstream ss(ptr.get());
        std::string token;
        int i = 0;
        while (std::getline(ss, token, ',') && i < spec::kNumJoints) {
            tmp[i] = std::stof(token);
            ++i;
        }
        if (i != spec::kNumJoints) {
            std::cerr << "[OnnxPolicy] Metadata '" << key << "' has only " << i << "/"
                      << spec::kNumJoints << " values -> keeping fallback.\n";
            return false;
        }
        out = tmp;
        return true;
    } catch (...) {
        return false;
    }
}
