#ifndef ARMA_RUNTIME_HPP
#define ARMA_RUNTIME_HPP

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

#include "controllers/locomotion/BaseObservationHistory.hpp"
#include "onnx/OnnxModel.hpp"
#include "onnx/PolicyContract.hpp"

namespace r1::arma {

// Runtime-only side of A-RMA. The privileged encoder/critic never enter this
// class: deploy runs adaptation(history[K,83]) then one actor attached to a
// declared Flat base-policy view (O83, H4, H5, or H4+gait).
class ArmaRuntime {
public:
    ArmaRuntime(const std::filesystem::path& package_directory,
                Ort::Env& environment,
                const Ort::SessionOptions& options);

    std::vector<float> Adapt(const std::vector<float>& base_observation);
    std::vector<float> RunActor(const std::vector<float>& actor_observation) const;
    void Reset();

    const r1::policy::PolicyContract& Contract() const { return contract_; }
    const std::string& BundleContract() const { return bundle_contract_; }
    const std::string& BasePolicyContract() const { return base_policy_contract_; }
    const std::string& ActorView() const { return actor_view_; }
    int ActorInputSize() const { return actor_input_dim_; }
    int ActorBaseDim() const { return actor_base_dim_; }
    int ActorHistorySteps() const { return actor_history_steps_; }
    int HistorySteps() const { return history_steps_; }
    int LatentSize() const { return latent_dim_; }
    int ObservationSize() const { return observation_dim_; }
    bool GaitConditioned() const { return actor_view_ == "flat_plus_gait_h4"; }
    int LatentAgeSteps() const { return latent_age_steps_; }
    int MaxLatentAgeSteps() const { return max_latent_age_steps_; }
    bool SafetyHold() const { return safety_hold_; }
    const std::vector<float>& Latent() const { return latent_; }

private:
    static bool AllFinite(const std::vector<float>& values);
    void ValidateConfig(const YAML::Node& config);
    void ValidateMetadata() const;

    std::filesystem::path package_directory_;
    std::unique_ptr<r1::policy::OnnxModel> adapter_;
    std::unique_ptr<r1::policy::OnnxModel> actor_;
    r1::policy::PolicyContract contract_{};
    r1::history::BaseObservationHistory history_;
    std::string bundle_contract_;
    std::string adapter_contract_;
    std::string base_policy_contract_;
    std::string actor_view_;
    int observation_dim_ = 83;
    int action_dim_ = 24;
    int latent_dim_ = 8;
    int history_steps_ = 50;
    int actor_base_dim_ = 83;
    int actor_history_steps_ = 1;
    int actor_input_dim_ = 91;
    int adapter_decimation_ = 1;
    int latent_age_steps_ = 0;
    int max_latent_age_steps_ = 1;
    int step_ = 0;
    bool safety_hold_ = false;
    std::vector<float> latent_;
};

}  // namespace r1::arma

#endif
