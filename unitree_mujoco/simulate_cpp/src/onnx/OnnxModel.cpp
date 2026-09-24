#include "onnx/OnnxModel.hpp"

#include <cmath>
#include <stdexcept>

namespace r1::policy {

namespace {

std::vector<int64_t> ConcreteShape(const std::vector<int64_t>& declared) {
    std::vector<int64_t> shape = declared;
    for (auto& dimension : shape) {
        if (dimension <= 0) dimension = 1;
    }
    return shape;
}

}  // namespace

OnnxModel::OnnxModel(
    Ort::Env& environment,
    const std::filesystem::path& path,
    const Ort::SessionOptions& options)
    : session_(std::make_unique<Ort::Session>(
          environment, path.string().c_str(), options)) {
    LoadTensorContract(true);
    LoadTensorContract(false);
}

const std::vector<int64_t>& OnnxModel::InputShape(std::size_t index) const {
    return input_shapes_.at(index);
}

const std::vector<int64_t>& OnnxModel::OutputShape(std::size_t index) const {
    return output_shapes_.at(index);
}

std::size_t OnnxModel::InputSize(std::size_t index) const {
    return TensorSize(input_shapes_.at(index));
}

std::size_t OnnxModel::OutputSize(std::size_t index) const {
    return TensorSize(output_shapes_.at(index));
}

std::string OnnxModel::Metadata(const std::string& key) const {
    auto metadata = session_->GetModelMetadata();
    auto value = metadata.LookupCustomMetadataMapAllocated(key.c_str(), allocator_);
    return value ? std::string(value.get()) : std::string();
}

std::vector<std::vector<float>> OnnxModel::Run(
    const std::vector<std::vector<float>>& inputs) const {
    if (inputs.size() != input_names_.size()) {
        throw std::runtime_error("ONNX input count mismatch");
    }

    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<Ort::Value> tensors;
    tensors.reserve(inputs.size());
    for (std::size_t index = 0; index < inputs.size(); ++index) {
        if (inputs[index].size() != InputSize(index)) {
            throw std::runtime_error(
                "ONNX input size mismatch for " + input_names_[index]);
        }
        for (const float value : inputs[index]) {
            if (!std::isfinite(value))
                throw std::runtime_error(
                    "ONNX input contains non-finite data for " + input_names_[index]);
        }
        auto shape = ConcreteShape(input_shapes_[index]);
        tensors.push_back(Ort::Value::CreateTensor<float>(
            memory,
            const_cast<float*>(inputs[index].data()),
            inputs[index].size(),
            shape.data(),
            shape.size()));
    }

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
    input_names.reserve(input_names_.size());
    output_names.reserve(output_names_.size());
    for (const auto& name : input_names_) input_names.push_back(name.c_str());
    for (const auto& name : output_names_) output_names.push_back(name.c_str());

    auto values = session_->Run(
        Ort::RunOptions{nullptr},
        input_names.data(), tensors.data(), tensors.size(),
        output_names.data(), output_names.size());

    std::vector<std::vector<float>> outputs;
    outputs.reserve(values.size());
    for (auto& value : values) {
        const auto shape = value.GetTensorTypeAndShapeInfo().GetShape();
        const std::size_t size = TensorSize(shape);
        const float* data = value.GetTensorData<float>();
        outputs.emplace_back(data, data + size);
        for (const float output : outputs.back()) {
            if (!std::isfinite(output))
                throw std::runtime_error("ONNX output contains non-finite data");
        }
    }
    return outputs;
}

std::vector<float> OnnxModel::RunSingle(const std::vector<float>& input) const {
    if (InputCount() != 1 || OutputCount() != 1) {
        throw std::runtime_error("single-policy ONNX must have exactly one input and output");
    }
    return Run({input}).front();
}

std::size_t OnnxModel::TensorSize(const std::vector<int64_t>& shape) {
    std::size_t size = 1;
    for (const auto dimension : shape) {
        size *= static_cast<std::size_t>(dimension > 0 ? dimension : 1);
    }
    return size;
}

void OnnxModel::LoadTensorContract(bool input) {
    const std::size_t count = input ? session_->GetInputCount()
                                    : session_->GetOutputCount();
    for (std::size_t index = 0; index < count; ++index) {
        auto type = input ? session_->GetInputTypeInfo(index)
                          : session_->GetOutputTypeInfo(index);
        auto tensor_info = type.GetTensorTypeAndShapeInfo();
        if (tensor_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            throw std::runtime_error(
                std::string("ONNX ") + (input ? "input" : "output")
                + " must use float32 tensors");
        }
        auto shape = tensor_info.GetShape();
        auto name = input ? session_->GetInputNameAllocated(index, allocator_)
                          : session_->GetOutputNameAllocated(index, allocator_);
        if (input) {
            input_names_.emplace_back(name.get());
            input_shapes_.push_back(std::move(shape));
        } else {
            output_names_.emplace_back(name.get());
            output_shapes_.push_back(std::move(shape));
        }
    }
}

}  // namespace r1::policy
