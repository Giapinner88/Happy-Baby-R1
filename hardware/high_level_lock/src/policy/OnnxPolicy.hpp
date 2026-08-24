#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "../config/RobotSpec.hpp"

// Lớp wrapper cho ONNX Runtime Session để load model và thực thi inference
class OnnxPolicy {
public:
    void Load(const std::string& model_path, Ort::Env& env,
              const Ort::SessionOptions& options, int expected_obs_size);

    std::vector<float> Infer(std::vector<float>& obs);

    // Đọc một mảng metadata từ model (ví dụ: action scale, default position)
    bool ReadMetadataArray(const char* key, std::array<float, spec::kNumJoints>& out);
    // Đọc metadata scalar/string. Dùng để khóa deployment contract, không suy đoán
    // model chỉ bằng tên file hoặc kích thước tensor.
    bool ReadMetadataString(const char* key, std::string& out);

    bool HasMetadata() const { return has_metadata_; }
    int obs_size() const { return obs_size_; }
    const std::string& path() const { return path_; }

private:
    std::unique_ptr<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;
    std::string input_name_, output_name_;
    std::string path_;
    int obs_size_ = 0;
    bool has_metadata_ = false;
};
