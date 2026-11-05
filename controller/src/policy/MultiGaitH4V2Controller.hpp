#pragma once

// ============================================================================
// r1_armfree0917_multigait_h4_v2 — ĐĂNG KÝ, CHƯA CHẠY ĐƯỢC.
//
// Contract này KHÔNG phải một biến thể của flat_plus_gait_h4_v1. Ba điểm phá
// hợp đồng, đo trực tiếp trên export:
//
//   obs      [1, 355]  = 332 chuẩn hoá (4 x 83 term-major, y hệt H4) + 23
//                        "conditioning" thô, nối sau, KHÔNG chuẩn hoá.
//   actions  [1, 25]   = 24 target khớp + 1 lệnh nhịp (index 24).
//   FSM      8 mode    = STAND,WALK,RUN,S2W,W2S,W2R,R2W,TURN (HB: 3).
//
// Nguồn train (đọc read-only qua ssh train, 2026-11-06):
//   experiments/armfree0917_multigait_run25_v2/
//     contract.py:14-20   ACTOR_PREFIX=332, RAW_DIM=23, ACTION_DIM=25, RAW_LAYOUT
//     actions.py:12-17    action[:, :24] -> khớp; action[:, 24] -> begin_action
//     timing.py:18-44     Clock.advance: chu kỳ do action[24] đặt, slew log 1.5/s
//     machine.py:47-119   FSM 8 mode + substate BRAKE/GATHER/SETTLE
//     command.py:143-148  conditioning() 23-D theo đúng RAW_LAYOUT
//     observations.py     phase = sin/cos clock (0 khi hold); actions = raw[:24]
//
// Vì sao controller này TỪ CHỐI thay vì chạy: xem
// plans/PHASE_E_V2_CONTRACT_NOTES.md. Tóm tắt: export tự khai
// `deployable=False` ở iteration 8400/20000, nó không mang một khoá hợp đồng
// quan sát nào của HB, và HB chưa có chủ sở hữu đồng hồ nhịp do policy điều
// khiển (action 24) lẫn FSM 8 mode + cổng dáng đứng theo FK chân.
//
// CẢNH BÁO CHO NGƯỜI SỬA SAU:
//   ActionSize() ở đây trả 25, lớn hơn spec::kNumJoints = 24. Điều đó CHỈ an
//   toàn vì ValidateExtraMetadata() luôn ném, nên PolicyController::Init()
//   không bao giờ chạy tới nhánh đọc action_scale (ghi 25 float vào mảng 24)
//   và PolicyController::Step() không bao giờ chạy (ghi last_action_[24]).
//   Bỏ lệnh ném ở dưới mà không tách luôn hai chỗ đó ra = tràn bộ đệm.
// ============================================================================

#include <string>
#include <vector>

#include "../config/RobotSpec.hpp"
#include "BaseObservationHistory.hpp"
#include "PolicyController.hpp"

class MultiGaitH4V2Controller final : public PolicyController {
public:
    // 332 = 4 x 83 term-major oldest->newest, trùng bit với flat_plus_h4_v1
    // (history_contract.py: BASE_TERM_ORDER / HISTORY_ORDER). Chỉ ngữ nghĩa
    // hai term đổi: `phase` lấy từ Clock do policy điều khiển, `actions` là
    // action thô 24 đầu của bước trước.
    static constexpr int kNormalizedPrefixDim = r1::history::kH4ActorDim;  // 332
    static constexpr int kConditioningDim = 23;
    static constexpr int kObsSize = kNormalizedPrefixDim + kConditioningDim;  // 355
    static constexpr int kActionDim = spec::kNumJoints + 1;   // 25
    static constexpr int kTimingActionIndex = spec::kNumJoints;  // 24

    std::string Name() const override { return "MultiGaitH4V2"; }

    int ObsSize() const override { return kObsSize; }
    int ActionSize() const override { return kActionDim; }

protected:
    // Export không có khoá `policy_contract`; nó dùng `contract`. Để rỗng ở
    // đây để cổng chung của PolicyController không ném một thông báo sai chỗ,
    // rồi tự kiểm trong ValidateExtraMetadata.
    std::string ExpectedPolicyContract() const override { return ""; }

    void ValidateExtraMetadata(const std::string& model_path) override {
        // Bước 1: chốt danh tính artifact bằng đúng những khoá export CÓ. Sai
        // một khoá nào ở đây nghĩa là file này không phải contract V2, và
        // thông báo phải nói "metadata mismatch" chứ không phải lời từ chối
        // thiết kế bên dưới.
        RequireMetadata(model_path, "contract", "r1_armfree0917_multigait_h4_v2");
        RequireMetadata(model_path, "actor_dim", std::to_string(kObsSize));
        RequireMetadata(model_path, "action_dim", std::to_string(kActionDim));
        RequireMetadata(model_path, "normalization_prefix",
                        std::to_string(kNormalizedPrefixDim));
        RequireMetadata(model_path, "timing_action_index",
                        std::to_string(kTimingActionIndex));
        RequireMetadata(model_path, "mode_names",
                        "STAND,WALK,RUN,S2W,W2S,W2R,R2W,TURN");
        RequireMetadata(model_path, "phase_clock", "continuous_latched_action_step");
        RequireMetadata(model_path, "phase_zero_mask", "hold_in_STAND_or_W2S");
        RequireMetadata(model_path, "scheduled_support_semantics",
                        "cyclic_schedule_or_two_foot_hold_not_measured_contact");
        RequireMetadataFloat(model_path, "policy_dt", spec::kPolicyDt);
        RequireMetadata(model_path, "joint_names",
                        "left_hip_pitch_joint,left_hip_roll_joint,left_hip_yaw_joint,"
                        "left_knee_joint,left_ankle_pitch_joint,left_ankle_roll_joint,"
                        "right_hip_pitch_joint,right_hip_roll_joint,right_hip_yaw_joint,"
                        "right_knee_joint,right_ankle_pitch_joint,right_ankle_roll_joint,"
                        "waist_roll_joint,waist_yaw_joint,left_shoulder_pitch_joint,"
                        "left_shoulder_roll_joint,left_shoulder_yaw_joint,left_elbow_joint,"
                        "left_wrist_roll_joint,right_shoulder_pitch_joint,"
                        "right_shoulder_roll_joint,right_shoulder_yaw_joint,"
                        "right_elbow_joint,right_wrist_roll_joint");

        // Bước 2: từ chối. KHÔNG có nhánh nào thoát khỏi hàm này mà không ném.
        std::string deployable, iteration, revision;
        policy_.ReadMetadataString("deployable", deployable);
        policy_.ReadMetadataString("iteration", iteration);
        policy_.ReadMetadataString("revision", revision);
        throw std::runtime_error(
            "MultiGaitH4V2 refuses to run " + model_path +
            "\n  contract r1_armfree0917_multigait_h4_v2 da dang ky nhung CHUA "
            "trien khai duoc tren HB. Bon quyet dinh con bo ngo (chi tiet: "
            "plans/PHASE_E_V2_CONTRACT_NOTES.md):"
            "\n  [1] artifact tu khai khong deploy duoc: deployable=" +
            (deployable.empty() ? std::string("<missing>") : deployable) +
            ", iteration=" + (iteration.empty() ? std::string("<missing>") : iteration) +
            ", revision=" + (revision.empty() ? std::string("<missing>") : revision) +
            ". HB khong duoc chay mot policy ma chinh ben export danh dau chua "
            "du tu cach."
            "\n  [2] action index 24 la LENH NHIP, khong phai khop. Policy tu "
            "dat chu ky dong ho (0.35-2.0 s, slew log 1.5/s) va chinh no doc "
            "lai sin/cos cua dong ho do trong obs. HB chua co chu so huu dong "
            "ho kieu nay: GaitScheduler/CommandGatedPhase chay chu ky CO DINH "
            "tu gait_period_s. Can port timing.py::Clock va noi action[24] "
            "nguoc vao no."
            "\n  [3] FSM 8 mode (STAND,WALK,RUN,S2W,W2S,W2R,R2W,TURN) + "
            "substate BRAKE/GATHER/SETTLE + cong dang dung theo FK 12 khop "
            "chan. GaitModeScheduler cua HB chi co 3 mode va khong co FK. Can "
            "port machine.py va leg_fk.py."
            "\n  [4] export chi co 30 khoa metadata va KHONG co khoa hop dong "
            "quan sat nao cua HB (actor_input_schema, base_observation_schema, "
            "actor_history_order/padding/steps, observation_terms, "
            "normalizer_contract, onnx_preprocess_contract, gait_period_s). "
            "Chay no bay gio la tin thu tu 332 chieu dau vao bang loi hua, "
            "khong phai bang kiem tra.");
    }

    void BuildObservation(const ControlContext& /*ctx*/,
                          std::vector<float>& /*obs*/) override {
        // Khong the toi: Init() luon nem o ValidateExtraMetadata().
        throw std::runtime_error(
            "MultiGaitH4V2: BuildObservation goi tren mot contract chua trien khai");
    }
};
