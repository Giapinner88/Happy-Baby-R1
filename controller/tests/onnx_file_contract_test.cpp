#include <cmath>
#include <iostream>
#include "policy/OnnxPolicy.hpp"

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    const bool expect_accept = std::string(argv[2]) == "accept";
    try {
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "onnx_file_contract_test");
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        OnnxPolicy policy;
        policy.Load(argv[1], env, options, 83);
        std::vector<float> obs(83,0);
        const auto action=policy.Infer(obs);
        if(action.size()!=24) return 3;
        for(float v:action) if(!std::isfinite(v)) return 3;
    } catch (const std::exception& e) {
        std::cout << e.what() << '\n';
        return expect_accept ? 1 : 0;
    }
    return expect_accept ? 0 : 1;
}
