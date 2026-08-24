#include "OnnxPolicy.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {

std::string ShapeString(const std::vector<int64_t>& dims) {
    std::ostringstream out;
    out << "[";
    for (size_t i = 0; i < dims.size(); ++i) {
        if (i != 0) out << ", ";
        out << dims[i];
    }
    out << "]";
    return out.str();
}

}  // namespace

void OnnxPolicy::Load(const std::string& model_path, Ort::Env& env,
                      const Ort::SessionOptions& options, int expected_obs_size) {
    path_ = model_path;
    session_ = std::make_unique<Ort::Session>(env, model_path.c_str(), options);

    // Infer() chỉ truyền một tensor observation. Kiểm tra trước khi arm controller
    // để model có state/input phụ (vd. time_step) không thể nổ giữa lúc robot chạy.
    const size_t input_count = session_->GetInputCount();
    std::ostringstream input_contract;
    for (size_t i = 0; i < input_count; ++i) {
        if (i != 0) input_contract << ", ";
        auto name = session_->GetInputNameAllocated(i, allocator_);
        const auto shape = session_->GetInputTypeInfo(i)
                               .GetTensorTypeAndShapeInfo().GetShape();
        input_contract << name.get() << ShapeString(shape);
    }
    if (input_count != 1) {
        throw std::runtime_error(
            "ONNX input contract mismatch for " + model_path + ": model accepts " +
            std::to_string(input_count) + " inputs (" + input_contract.str() +
            "), but run_r1 supports exactly one observation input");
    }

    // Get input dimensions and validate obs size
    Ort::TypeInfo type_info = session_->GetInputTypeInfo(0);
    auto input_info = type_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> dims = input_info.GetShape();
    if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        throw std::runtime_error("Model " + model_path + " input '" +
                                 input_contract.str() + "' is not float32");
    }
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
    if (!session_) throw std::runtime_error("ONNX policy was not initialized");
    if (static_cast<int>(obs.size()) != obs_size_) {
        throw std::runtime_error("ONNX observation size changed at runtime for " + path_ +
                                 ": got " + std::to_string(obs.size()) + ", expected " +
                                 std::to_string(obs_size_));
    }
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
        while (std::getline(ss, token, ',')) {
            if (i >= spec::kNumJoints) {
                std::cerr << "[OnnxPolicy] Metadata '" << key << "' has more than "
                          << spec::kNumJoints << " values -> rejecting.\n";
                return false;
            }
            tmp[i] = std::stof(token);
            if (!std::isfinite(tmp[i])) {
                std::cerr << "[OnnxPolicy] Metadata '" << key
                          << "' contains a non-finite value -> rejecting.\n";
                return false;
            }
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

bool OnnxPolicy::ReadMetadataString(const char* key, std::string& out) {
    try {
        Ort::ModelMetadata md = session_->GetModelMetadata();
        Ort::AllocatedStringPtr ptr = md.LookupCustomMetadataMapAllocated(key, allocator_);
        if (!ptr) return false;
        out = ptr.get();
        return true;
    } catch (...) {
        return false;
    }
}
