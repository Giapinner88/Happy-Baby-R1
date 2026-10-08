#include <cstdlib>
#include "policy/TensorContract.hpp"

int main() {
    RequirePolicyTensor(ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, {1,24}, 24, false, "output");
    RequirePolicyTensor(ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, {-1,83}, 83, true, "input");
    auto reject = [](auto type, std::vector<int64_t> shape, bool dynamic=false) {
        try { RequirePolicyTensor(type, shape, 24, dynamic, "test"); }
        catch (const std::exception&) { return; }
        std::abort();
    };
    reject(ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE, {1,24});
    reject(ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32, {1,24});
    for (const auto& shape : std::vector<std::vector<int64_t>>{{24},{2,24},{1,2,24},{0,24},{1,23},{-1,24},{}})
        reject(ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, shape);
}
