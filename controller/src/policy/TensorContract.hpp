#pragma once
#include <stdexcept>
#include <string>
#include <vector>
#include <onnxruntime_cxx_api.h>

inline void RequirePolicyTensor(ONNXTensorElementDataType type,
                                const std::vector<int64_t>& shape,
                                int features, bool allow_dynamic_batch,
                                const std::string& label) {
    if (type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || shape.size() != 2 ||
        (shape[0] != 1 && !(allow_dynamic_batch && shape[0] == -1)) ||
        shape[1] != features) {
        throw std::runtime_error(label + " must be float32 [1," +
                                 std::to_string(features) + "] (dynamic batch allowed only at load)");
    }
}
