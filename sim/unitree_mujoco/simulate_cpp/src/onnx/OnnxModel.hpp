#pragma once

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace r1::policy {

// RAII wrapper around an ONNX Runtime session. It owns tensor names and never
// exposes the session pointer to controllers or the application loop.
class OnnxModel {
public:
    OnnxModel(
        Ort::Env& environment,
        const std::filesystem::path& path,
        const Ort::SessionOptions& options);

    std::size_t InputCount() const { return input_names_.size(); }
    std::size_t OutputCount() const { return output_names_.size(); }
    const std::vector<int64_t>& InputShape(std::size_t index) const;
    const std::vector<int64_t>& OutputShape(std::size_t index) const;
    std::size_t InputSize(std::size_t index) const;
    std::size_t OutputSize(std::size_t index) const;

    std::string Metadata(const std::string& key) const;

    std::vector<std::vector<float>> Run(
        const std::vector<std::vector<float>>& inputs) const;
    std::vector<float> RunSingle(const std::vector<float>& input) const;

private:
    static std::size_t TensorSize(const std::vector<int64_t>& shape);
    void LoadTensorContract(bool input);

    std::unique_ptr<Ort::Session> session_;
    mutable Ort::AllocatorWithDefaultOptions allocator_;
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<std::vector<int64_t>> input_shapes_;
    std::vector<std::vector<int64_t>> output_shapes_;
};

}  // namespace r1::policy
