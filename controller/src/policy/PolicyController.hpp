#pragma once

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../config/RobotSpec.hpp"
#include "../config/Tuning.hpp"
#include "../estimation/StateEstimator.hpp"
#include "OnnxPolicy.hpp"

#include "ControlContext.hpp"

// Lớp cơ sở cho các bộ điều khiển dựa trên chính sách (Policy Controller)
class PolicyController {
public:
    virtual ~PolicyController() = default;

    void Init(const std::string& model_path, Ort::Env& env,
              const Ort::SessionOptions& options, const Tuning& tuning) {
        policy_.Load(model_path, env, options, ObsSize());

        const std::string expected_contract = ExpectedPolicyContract();
        if (!expected_contract.empty()) {
            std::string actual_contract, actual_obs, actual_action, actual_layout;
            const bool valid_contract = policy_.ReadMetadataString("policy_contract", actual_contract) &&
                                        actual_contract == expected_contract;
            const bool valid_dims = policy_.ReadMetadataString("observation_dim", actual_obs) &&
                                    policy_.ReadMetadataString("action_dim", actual_action) &&
                                    actual_obs == std::to_string(ObsSize()) &&
                                    actual_action == std::to_string(spec::kNumJoints);
            const std::string expected_layout = ExpectedObservationLayout();
            const bool valid_layout = expected_layout.empty() ||
                                      (policy_.ReadMetadataString("observation_layout", actual_layout) &&
                                       actual_layout == expected_layout);
            if (!valid_contract || !valid_dims || !valid_layout) {
                throw std::runtime_error(
                    "ONNX contract mismatch: expected '" + expected_contract +
                    "' with observation_dim=" + std::to_string(ObsSize()) +
                    " and action_dim=" + std::to_string(spec::kNumJoints) +
                    ", refusing to run " + model_path);
            }
        }

        // Khoá phụ theo từng contract (ví dụ layout history). Chạy cả khi
        // ExpectedPolicyContract() rỗng để không có đường vòng nào bỏ qua.
        ValidateExtraMetadata(model_path);

        default_q_    = spec::kDefaultJointPos;
        action_scale_ = spec::kActionScale;
        kp_           = spec::kKpTrain;
        kd_           = spec::kKdTrain;

        // Ưu tiên nạp thông số từ ONNX metadata.
        int missing = 0;
        missing += !LoadOrFallback("default_joint_pos", default_q_);
        missing += !LoadOrFallback("action_scale", action_scale_);
        missing += !LoadOrFallback("joint_stiffness", kp_);
        missing += !LoadOrFallback("joint_damping", kd_);

        // Bắt buộc có metadata để tránh sai lệch gain khi deploy (gây mất thăng bằng).
        if (missing > 0) {
            std::cerr << "\n"
                      << "==========================================================\n"
                      << " [LỖI] " << model_path << "\n"
                      << " ONNX thiếu " << missing << "/4 khoá metadata bắt buộc.\n"
                      << " Deploy KHÔNG được phép đoán gains -> từ chối chạy policy này.\n"
                      << "\n"
                      << " Cách sửa: export lại từ máy train (runner đã được vá để gắn\n"
                      << " metadata vào policy.onnx), hoặc backfill metadata vào file.\n"
                      << "==========================================================\n\n";
            throw std::runtime_error("ONNX thiếu metadata: " + model_path);
        }

        for (int i = 0; i < spec::kNumJoints; ++i) {
            kp_[i] *= tuning.policy_kp_scale;
            kd_[i] *= tuning.policy_kd_scale;
        }

        std::cout << "[" << Name() << "] Kp[hip]=" << kp_[0] << " Kp[ankle]=" << kp_[4]
                  << " Kp[tay]=" << kp_[14] << " Kd=" << kd_[0] << "\n";
    }

    virtual void Reset(const RobotState& /*state*/) {
        last_action_.fill(0.0f);
        infer_fault_ = false;
    }

    // Thực hiện 1 bước inference (chạy ở 50Hz)
    const std::array<float, spec::kNumJoints>& Step(const ControlContext& ctx) {
        std::vector<float> obs(ObsSize(), 0.0f);
        BuildObservation(ctx, obs);
        std::vector<float> action = policy_.Infer(obs);

        // Kiểm ngay tại NGUỒN. Cổng ở LowCmdSender vẫn chặn được, nhưng nó chỉ
        // thấy "target khớp N" — không biết model nào phân kỳ, mà một máy có thể
        // đang chạy locomotion + nhiều mimic. Ở đây còn giữ được target trước đó
        // thay vì để một action rác nhiễm cả last_action_ (và qua đó nhiễm luôn
        // history buffer của các contract có history).
        for (int i = 0; i < spec::kNumJoints; ++i) {
            if (!std::isfinite(action[i])) {
                if (!infer_fault_) {
                    std::cerr << "[" << Name() << "] output ONNX khong huu han o khop "
                              << i << " -> giu target truoc do, dung an toan.\n";
                }
                infer_fault_ = true;
                return target_q_;
            }
        }

        for (int i = 0; i < spec::kNumJoints; ++i) {
            last_action_[i] = action[i];
            target_q_[i] = default_q_[i] + action[i] * action_scale_[i];
        }
        return target_q_;
    }

    // Policy đã trả NaN/Inf ít nhất một lần kể từ Reset(). Application đọc cờ
    // này và dừng an toàn.
    bool infer_fault() const { return infer_fault_; }

    virtual bool IsFinished() const { return false; }
    virtual int ObsSize() const = 0;
    virtual std::string Name() const = 0;
    // Contract này có chịu được overlay tay kiểu legacy không (ghi đè
    // target_q[14..23] + che obs của tay theo offset cố định của frame 83-D)?
    // Mặc định FALSE để một controller mới quên khai báo thì mất tính năng chứ
    // không phá thăng bằng.
    virtual bool SupportsArmOverlay() const { return false; }
    // History contracts chỉ có thể nhận gesture khi runtime mask q/dq tay trên
    // frame 83-D trước khi frame đó được ghi vào history. Tách capability này
    // khỏi SupportsArmOverlay(); teleop H4 có capability riêng bên dưới.
    virtual bool SupportsHistoryGestureOverlay() const { return false; }
    // Separate from gestures and legacy 83-D. Never infer teleop compatibility
    // from an observation length or a shared history layout.
    virtual bool SupportsH4TeleopOverlay() const { return false; }

    const std::array<float, spec::kNumJoints>& default_q() const { return default_q_; }
    const std::array<float, spec::kNumJoints>& kp() const { return kp_; }
    const std::array<float, spec::kNumJoints>& kd() const { return kd_; }
    const std::array<float, spec::kNumJoints>& last_action() const { return last_action_; }

protected:
    virtual void BuildObservation(const ControlContext& ctx, std::vector<float>& obs) = 0;
    virtual std::string ExpectedPolicyContract() const { return ""; }
    virtual std::string ExpectedObservationLayout() const { return ""; }

    // Kiểm tra thêm metadata riêng của contract. Mặc định no-op.
    virtual void ValidateExtraMetadata(const std::string& /*model_path*/) {}

    // Metadata BẮT BUỘC phải có và phải khớp; thiếu cũng là sai.
    void RequireMetadata(const std::string& model_path, const char* key,
                         const std::string& expected) {
        std::string actual;
        const bool present = policy_.ReadMetadataString(key, actual);
        if (!present || actual != expected) {
            throw std::runtime_error(
                std::string("ONNX metadata mismatch: ") + key + " expected '" +
                expected + "', model has '" +
                (present ? actual : std::string("<missing>")) + "' in " + model_path);
        }
    }

    void RequireMetadataFloat(const std::string& model_path, const char* key,
                              float expected, float tolerance = 1.0e-6f) {
        std::string actual;
        if (!policy_.ReadMetadataString(key, actual)) {
            throw std::runtime_error(std::string("ONNX metadata missing: ") + key +
                                     " in " + model_path);
        }
        try {
            const float value = std::stof(actual);
            if (!std::isfinite(value) || std::fabs(value - expected) > tolerance)
                throw std::runtime_error("value mismatch");
        } catch (...) {
            throw std::runtime_error(
                std::string("ONNX metadata mismatch: ") + key + " expected '" +
                std::to_string(expected) + "', model has '" + actual + "' in " +
                model_path);
        }
    }

    // Đọc thông số mảng từ ONNX metadata hoặc trả về false.
    bool LoadOrFallback(const char* key, std::array<float, spec::kNumJoints>& out) {
        if (policy_.ReadMetadataArray(key, out)) {
            std::cout << "[" << Name() << "] " << key << ": Loaded from ONNX metadata.\n";
            return true;
        }
        std::cout << "[" << Name() << "] " << key << ": KHÔNG có trong metadata.\n";
        return false;
    }

    OnnxPolicy policy_;
    std::array<float, spec::kNumJoints> default_q_{};
    std::array<float, spec::kNumJoints> action_scale_{};
    std::array<float, spec::kNumJoints> kp_{};
    std::array<float, spec::kNumJoints> kd_{};
    std::array<float, spec::kNumJoints> last_action_{};
    std::array<float, spec::kNumJoints> target_q_{};
    bool infer_fault_ = false;
};
