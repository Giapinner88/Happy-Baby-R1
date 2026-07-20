#pragma once
/**
 * OnnxPolicy.hpp — Lớp nạp mô hình và chạy inference ONNX.
 */

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "../config/RobotSpec.hpp"

class OnnxPolicy {
public:
    void Load(const std::string& model_path, Ort::Env& env,
              const Ort::SessionOptions& options, int expected_obs_size);

    std::vector<float> Infer(std::vector<float>& obs);

    bool ReadMetadataArray(const char* key, std::array<float, spec::kNumJoints>& out);

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
