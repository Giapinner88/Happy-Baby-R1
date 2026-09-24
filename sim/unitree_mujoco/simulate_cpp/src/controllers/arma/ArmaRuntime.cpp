#include "controllers/arma/ArmaRuntime.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "runtime/R1Config.hpp"
#include "runtime/R1PolicyContract.hpp"

namespace r1::arma {

namespace {

std::string RequiredString(const YAML::Node& node, const char* key) {
    const auto value = node[key];
    if (!value || !value.IsScalar())
        throw std::runtime_error(std::string("A-RMA config missing ") + key);
    const auto result = value.as<std::string>();
    if (result.empty())
        throw std::runtime_error(std::string("A-RMA config has empty ") + key);
    return result;
}

template <std::size_t Size>
void LoadConfigFloatArray(const YAML::Node& node, const char* key,
                          std::array<float, Size>& destination) {
    const auto value = node[key];
    if (!value) return;
    if (!value.IsSequence() || value.size() != Size) {
        throw std::runtime_error(std::string("A-RMA config ") + key
                                 + " must contain " + std::to_string(Size)
                                 + " values");
    }
    std::array<float, Size> parsed{};
    for (std::size_t index = 0; index < Size; ++index) {
        parsed[index] = value[index].as<float>();
        if (!std::isfinite(parsed[index])) {
            throw std::runtime_error(std::string("A-RMA config ") + key
                                     + " contains a non-finite value");
        }
    }
    destination = parsed;
}

int ExpectedActorBaseDim(const std::string& view) {
    if (view == "base83") return r1::history::kBaseDim;
    if (view == "flat_plus_h4") return r1::history::kH4ActorDim;
    if (view == "flat_plus_h5") return r1::history::kH5ActorDim;
    if (view == "flat_plus_gait_h4") return r1::history::kH4ActorDim + 3;
    throw std::runtime_error("unsupported A-RMA actor_view: " + view);
}

int ExpectedActorHistorySteps(const std::string& view) {
    if (view == "base83") return 1;
    if (view == "flat_plus_h4" || view == "flat_plus_gait_h4") {
        return r1::history::kH4Steps;
    }
    if (view == "flat_plus_h5") return r1::history::kH5Steps;
    throw std::runtime_error("unsupported A-RMA actor_view: " + view);
}

std::string ExpectedBasePolicyContract(const std::string& view) {
    if (view == "base83") return "flat_base83_v1";
    if (view == "flat_plus_h4") return r1::history::kFlatPlusContract;
    if (view == "flat_plus_h5") return r1::history::kFlatPlusH5Contract;
    if (view == "flat_plus_gait_h4") return "flat_plus_gait_h4_v1";
    throw std::runtime_error("unsupported A-RMA actor_view: " + view);
}

std::string ExpectedActorInputSchema(const std::string& view) {
    if (view == "base83") return "base83_plus_latent_v1";
    if (view == "flat_plus_h4") return "flat_plus_h4_plus_latent_v1";
    if (view == "flat_plus_h5") return "flat_plus_h5_plus_latent_v1";
    if (view == "flat_plus_gait_h4") {
        return "flat_plus_gait_h4_plus_latent_v1";
    }
    throw std::runtime_error("unsupported A-RMA actor_view: " + view);
}

constexpr const char* kObservationTerms =
    "base_ang_vel,projected_gravity,command,phase,joint_pos,joint_vel,actions";

}  // namespace

ArmaRuntime::ArmaRuntime(const std::filesystem::path& package_directory,
                         Ort::Env& environment,
                         const Ort::SessionOptions& options)
    : package_directory_(std::filesystem::absolute(package_directory)),
      history_(history_steps_) {
    const auto deploy_path = package_directory_ / "params/deploy.yaml";
    if (!std::filesystem::is_regular_file(deploy_path))
        throw std::runtime_error("A-RMA deploy.yaml not found: " + deploy_path.string());
    const auto deploy = YAML::LoadFile(deploy_path.string());
    const auto config = deploy["arma"];
    if (!config || !config.IsMap())
        throw std::runtime_error("deploy.yaml is missing arma registry");
    ValidateConfig(config);

    const auto adapter_path = package_directory_ / RequiredString(config, "adapter_model");
    const auto actor_path = package_directory_ / RequiredString(config, "actor_model");
    if (!std::filesystem::is_regular_file(adapter_path)
        || !std::filesystem::is_regular_file(actor_path)) {
        throw std::runtime_error("A-RMA adapter or actor ONNX is missing");
    }
    adapter_ = std::make_unique<r1::policy::OnnxModel>(environment, adapter_path, options);
    actor_ = std::make_unique<r1::policy::OnnxModel>(environment, actor_path, options);

    if (adapter_->InputCount() != 1 || adapter_->OutputCount() != 1
        || adapter_->InputSize(0) != static_cast<std::size_t>(history_steps_ * observation_dim_)
        || adapter_->OutputSize(0) != static_cast<std::size_t>(latent_dim_)) {
        throw std::runtime_error("A-RMA adapter tensor contract is invalid");
    }
    if (actor_->InputCount() != 1 || actor_->OutputCount() != 1
        || actor_->InputSize(0) != static_cast<std::size_t>(actor_input_dim_)
        || actor_->OutputSize(0) != static_cast<std::size_t>(action_dim_)) {
        throw std::runtime_error("A-RMA actor tensor contract is invalid");
    }

    r1::policy::PolicyContract fallback{
        R1Config::DEFAULT_JOINT_POS,
        R1Config::ACTION_SCALE,
        R1Config::KP_ARRAY,
        R1Config::KD_ARRAY};
    // ARMA's current exporter intentionally carries only component metadata.
    // Its train-time action/PD contract is therefore captured in deploy.yaml;
    // do not silently fall back to the older global R1 constants when the
    // bundle declares a different common-PD setup.
    LoadConfigFloatArray(config, "default_joint_pos", fallback.default_position);
    LoadConfigFloatArray(config, "action_scale", fallback.action_scale);
    LoadConfigFloatArray(config, "stiffness", fallback.stiffness);
    LoadConfigFloatArray(config, "damping", fallback.damping);
    contract_ = r1::policy::LoadPolicyContract(
        *actor_, std::move(fallback), r1::contract::JointNames());
    ValidateMetadata();
    latent_.assign(static_cast<std::size_t>(latent_dim_), 0.0f);
    Reset();
}

std::vector<float> ArmaRuntime::Adapt(const std::vector<float>& base_observation) {
    if (static_cast<int>(base_observation.size()) != observation_dim_)
        throw std::runtime_error("A-RMA base observation must be 83-D");
    if (!AllFinite(base_observation)) {
        safety_hold_ = true;
        return latent_;
    }
    history_.Append(base_observation);
    ++step_;
    ++latent_age_steps_;
    if (step_ % adapter_decimation_ == 0) {
        std::vector<float> history;
        history_.PackTimeMajor(history);
        try {
            const auto output = adapter_->RunSingle(history);
            if (output.size() != latent_.size() || !AllFinite(output)) {
                safety_hold_ = true;
                return latent_;
            }
            latent_ = output;
            latent_age_steps_ = 0;
            safety_hold_ = false;
        } catch (...) {
            safety_hold_ = true;
            return latent_;
        }
    }
    if (latent_age_steps_ > max_latent_age_steps_) safety_hold_ = true;
    return latent_;
}

std::vector<float> ArmaRuntime::RunActor(
    const std::vector<float>& actor_observation) const {
    if (static_cast<int>(actor_observation.size()) != actor_input_dim_)
        throw std::runtime_error("A-RMA actor observation dimension mismatch");
    if (!AllFinite(actor_observation))
        throw std::runtime_error("A-RMA actor observation is non-finite");
    const auto output = actor_->RunSingle(actor_observation);
    if (output.size() != static_cast<std::size_t>(action_dim_) || !AllFinite(output))
        throw std::runtime_error("A-RMA actor produced an invalid action");
    return output;
}

void ArmaRuntime::Reset() {
    history_.Reset();
    std::fill(latent_.begin(), latent_.end(), 0.0f);
    latent_age_steps_ = 0;
    step_ = 0;
    safety_hold_ = false;
}

bool ArmaRuntime::AllFinite(const std::vector<float>& values) {
    return std::all_of(values.begin(), values.end(), [](float value) {
        return std::isfinite(value);
    });
}

void ArmaRuntime::ValidateConfig(const YAML::Node& config) {
    const int version = config["config_version"].as<int>(1);
    observation_dim_ = config["observation_dim"].as<int>(83);
    action_dim_ = config["action_dim"].as<int>(24);
    latent_dim_ = config["latent_dim"].as<int>(8);
    history_steps_ = config["history_steps"].as<int>(50);
    actor_input_dim_ = config["actor_input_dim"].as<int>(observation_dim_ + latent_dim_);
    adapter_decimation_ = config["adapter_decimation"].as<int>(1);
    max_latent_age_steps_ = config["max_latent_age_steps"].as<int>(adapter_decimation_);
    bundle_contract_ = RequiredString(config, "bundle_contract");
    adapter_contract_ = config["adapter_contract"].as<std::string>(
        "flat_plus_arma_adapter_k50_z8_v1");
    base_policy_contract_ = RequiredString(config, "base_policy_contract");
    actor_view_ = RequiredString(config, "actor_view");
    actor_base_dim_ = config["actor_base_dim"].as<int>(
        ExpectedActorBaseDim(actor_view_));
    actor_history_steps_ = config["actor_history_steps"].as<int>(
        ExpectedActorHistorySteps(actor_view_));
    const int expected_actor_dim = actor_base_dim_ + latent_dim_;
    if (version != 1 || observation_dim_ != 83 || action_dim_ != R1Config::NUM_JOINTS
        || latent_dim_ <= 0 || history_steps_ <= 0
        || base_policy_contract_ != ExpectedBasePolicyContract(actor_view_)
        || actor_base_dim_ != ExpectedActorBaseDim(actor_view_)
        || actor_history_steps_ != ExpectedActorHistorySteps(actor_view_)
        || actor_input_dim_ != expected_actor_dim
        || adapter_decimation_ <= 0
        || max_latent_age_steps_ < adapter_decimation_) {
        throw std::runtime_error("invalid A-RMA dimensions or timing configuration");
    }
    history_ = r1::history::BaseObservationHistory(history_steps_);
}

void ArmaRuntime::ValidateMetadata() const {
    const auto require = [&](const r1::policy::OnnxModel& model,
                             const char* key, const std::string& expected) {
        if (model.Metadata(key) != expected)
            throw std::runtime_error(std::string("A-RMA metadata mismatch: ") + key);
    };
    // The 2026-09-08 training exporter has a deliberately component-local
    // schema: the actor declares its ARMA contract and prefix, while the
    // adapter declares the [K,83] history. Older staging bundles put the
    // bundle/role fields on both ONNX files. Keep both readers so adding this
    // result cannot invalidate an already staged legacy bundle.
    const bool exported_v1 = !actor_->Metadata("component").empty()
        || !adapter_->Metadata("component").empty();
    if (exported_v1) {
        require(*actor_, "component", "actor");
        require(*actor_, "component_contract", bundle_contract_);
        require(*actor_, "policy_contract", bundle_contract_);
        require(*actor_, "base_observation_schema", r1::history::kBaseSchema);
        require(*actor_, "base_observation_dim", std::to_string(observation_dim_));
        require(*actor_, "actor_input_dim", std::to_string(actor_input_dim_));
        require(*actor_, "actor_prefix_dim", std::to_string(actor_base_dim_));
        require(*actor_, "actor_prefix_schema", r1::history::kBaseSchema);
        require(*actor_, "latent_dim", std::to_string(latent_dim_));
        require(*actor_, "latent_source_contract", adapter_contract_);
        require(*actor_, "action_dim", std::to_string(action_dim_));

        require(*adapter_, "component", "adaptation");
        require(*adapter_, "component_contract", adapter_contract_);
        require(*adapter_, "base_observation_schema", r1::history::kBaseSchema);
        require(*adapter_, "base_observation_dim", std::to_string(observation_dim_));
        require(*adapter_, "adapter_input_schema",
                "flat_plus_base83_sequence_k50_v1");
        require(*adapter_, "adapter_history_k", std::to_string(history_steps_));
        require(*adapter_, "adapter_history_order",
                "time_major_oldest_to_newest");
        require(*adapter_, "adapter_history_padding",
                "repeat_first_with_valid_steps");
        require(*adapter_, "latent_dim", std::to_string(latent_dim_));
    } else {
        require(*actor_, "bundle_contract", bundle_contract_);
        require(*actor_, "component_role", "actor");
        require(*actor_, "policy_contract", base_policy_contract_);
        require(*actor_, "base_policy_contract", base_policy_contract_);
        require(*actor_, "actor_view", actor_view_);
        require(*actor_, "actor_base_dim", std::to_string(actor_base_dim_));
        require(*actor_, "actor_history_steps", std::to_string(actor_history_steps_));
        require(*actor_, "actor_input_dim", std::to_string(actor_input_dim_));
        require(*actor_, "latent_dim", std::to_string(latent_dim_));
        require(*actor_, "base_observation_schema", r1::history::kBaseSchema);
        require(*actor_, "base_observation_dim", std::to_string(observation_dim_));
        require(*actor_, "actor_input_schema", ExpectedActorInputSchema(actor_view_));
        if (actor_view_ != "base83") {
            require(*actor_, "actor_history_order", r1::history::kH4Order);
            require(*actor_, "actor_history_padding", r1::history::kH4Padding);
            require(*actor_, "observation_terms", kObservationTerms);
        }
        if (actor_view_ == "flat_plus_gait_h4") {
            require(*actor_, "gait_mode_schema", "stand_walk_w2s_onehot3_v1");
            require(*actor_, "gait_mode_dim", "3");
            require(*actor_, "gait_mode_order", "stand,walk,w2s");
            require(*actor_, "gait_mode_temporal_semantics", "current_only");
            require(*actor_, "gait_normalization", "none_raw_onehot");
            require(*actor_, "normalizer_contract", "prefix_only_then_raw_gait_v1");
        }
        require(*adapter_, "bundle_contract", bundle_contract_);
        require(*adapter_, "component_role", "adaptation");
        require(*adapter_, "base_policy_contract", base_policy_contract_);
        require(*adapter_, "actor_view", actor_view_);
        require(*adapter_, "observation_dim", std::to_string(observation_dim_));
        require(*adapter_, "history_steps", std::to_string(history_steps_));
        require(*adapter_, "latent_dim", std::to_string(latent_dim_));
        require(*adapter_, "base_observation_schema", r1::history::kBaseSchema);
        require(*adapter_, "history_order", "time_major_oldest_to_newest");
    }
    const auto require_nonempty = [&](const r1::policy::OnnxModel& model,
                                      const char* key) {
        if (model.Metadata(key).empty()) {
            throw std::runtime_error(std::string("A-RMA metadata missing: ") + key);
        }
    };
    if (!exported_v1) {
        require_nonempty(*actor_, "component_sha256");
        require_nonempty(*actor_, "companion_component_sha256");
        require_nonempty(*adapter_, "component_sha256");
        require_nonempty(*adapter_, "companion_component_sha256");
        if (actor_->Metadata("companion_component_sha256") !=
            adapter_->Metadata("component_sha256")) {
            throw std::runtime_error("A-RMA actor/adapter provenance mismatch");
        }
        if (adapter_->Metadata("companion_component_sha256") !=
            actor_->Metadata("component_sha256")) {
            throw std::runtime_error("A-RMA adapter/actor provenance mismatch");
        }
    }
}

}  // namespace r1::arma
