// Model and asset loading: locomotion policy, dances, get-up/lie-down clips.
// Application is split across src/app/Application*.cpp; see Application.hpp.
#include "Application.hpp"
#include "ApplicationInternal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>
#include <dirent.h>

using unitree_hg::msg::dds_::LowState_;
using unitree_hg::msg::dds_::LowCmd_;
using namespace app_internal;

namespace {

bool HasExt(const std::string& name, const std::string& ext) {
    if (name.size() < ext.size()) return false;
    std::string tail = name.substr(name.size() - ext.size());
    std::transform(tail.begin(), tail.end(), tail.begin(), ::tolower);
    return tail == ext;
}

void ScanDanceFolder(const std::string& dir, const std::string& label,
                     std::string& onnx, std::string& npz, std::string& music) {
    static const char* kAudioExt[] = {".mp3", ".wav", ".ogg", ".flac", ".m4a"};
    onnx.clear(); npz.clear(); music.clear();
    DIR* d = opendir(dir.c_str());
    if (!d) {
        std::cerr << "[Application] Failed to open dance folder " << label << ": " << dir << "\n";
        return;
    }
    for (dirent* e = readdir(d); e; e = readdir(d)) {
        std::string name = e->d_name;
        if (name == "." || name == "..") continue;
        auto pick = [&](std::string& slot) {
            if (slot.empty()) slot = name;
            else std::cerr << "[Application] Dance " << label << ": multiple files of same type, using '"
                           << slot << "', ignoring '" << name << "'.\n";
        };
        if (HasExt(name, ".onnx")) pick(onnx);
        else if (HasExt(name, ".npz")) pick(npz);
        else {
            for (const char* ax : kAudioExt) {
                if (HasExt(name, ax)) { pick(music); break; }
            }
        }
    }
    closedir(d);
}

} // namespace

std::unique_ptr<PolicyController> Application::MakeLocomotionController(
        const std::string& contract, const std::string& path) {
    std::unique_ptr<PolicyController> ctrl;
    switch (ParseFlatPolicyProfile(contract)) {
        case FlatPolicyProfile::kLegacy83:
            ctrl = std::make_unique<LocomotionController>();
            break;
        case FlatPolicyProfile::kFlatPlusH4V1:
            ctrl = std::make_unique<FlatPlusController>();
            break;
        case FlatPolicyProfile::kFlatPlusH5V1:
            ctrl = std::make_unique<FlatPlusH5Controller>();
            break;
        case FlatPolicyProfile::kFlatPlusGaitH4V1: {
            auto gait_controller = std::make_unique<FlatPlusGaitController>();
            gait_controller->ConfigureGait({
                tuning_.gait_stop_request_lin,
                tuning_.gait_stop_request_ang,
                tuning_.gait_move_request_lin,
                tuning_.gait_move_request_ang,
                tuning_.gait_stability_gyro_norm,
                tuning_.gait_stability_joint_velocity_rms,
                tuning_.gait_settle_dwell_s,
                tuning_.gait_w2s_timeout_s,
                tuning_.gait_stability_filter_tau_s},
                tuning_.gait_period_s);
            ctrl = std::move(gait_controller);
            break;
        }
        case FlatPolicyProfile::kWholeBodyLegsBlindV1:
            ctrl = std::make_unique<WholeBodyLegsController>();
            break;
        case FlatPolicyProfile::kWholeBodyLegsGaitH4BlindV1:
            // FSM cấu hình hoàn toàn từ metadata ONNX (không từ locomotion.yaml).
            ctrl = std::make_unique<WholeBodyLegsGaitController>();
            break;
    }
    ctrl->Init(path, ort_env_, ort_opts_, tuning_);
    if (const auto* wb = dynamic_cast<const WholeBodyLegsController*>(ctrl.get())) {
        // Phase obs dùng gait_ chung (tuning gait_period_s): phải bằng chu kỳ lúc train.
        if (std::abs(wb->metadata_gait_period_s() - tuning_.gait_period_s) > 1e-3f)
            throw std::runtime_error(
                "gait_period_s=" + std::to_string(tuning_.gait_period_s) +
                " khac metadata gait_period_s=" + std::to_string(wb->metadata_gait_period_s()) +
                " cua " + path);
    }
    return ctrl;
}

void Application::InitControllers() {
    std::cout << "[MotionSafety] mode=" << (tuning_.state_motion_guard ? "state_reference" : "legacy")
              << " dance_error_deg=" << tuning_.dance_tracking_error_deg
              << " debounce_s=" << tuning_.dance_tracking_debounce_s
              << " state_timeout_s=" << tuning_.motion_safety_state_timeout_s << "\n";
    // Hợp đồng máy-đọc cho chuỗi deploy. preflight.sh, deploy_stack.sh và
    // update_model_manifest.sh đọc dòng "[Preflight] MODEL" (walk policy, luôn in
    // ĐẦU TIÊN) thay vì tự suy ra đường dẫn + số chiều theo contract: các script
    // đó từng có bảng contract riêng, lệch với runtime cả hai chiều. Thêm contract
    // hoặc bundle mới thì KHÔNG phải sửa script.
    auto print_model = [&](const char* tag, const std::string& contract,
                           const std::string& rel, const PolicyController& ctrl) {
        std::cout << "[Preflight] " << tag << " contract=" << contract
                  << " rel=" << rel
                  << " input=" << ctrl.ObsSize()
                  << " output=" << ctrl.ActionSize() << "\n";
    };

    std::string walk_contract;
    if (!tuning_.walk_policy.empty()) {
        // Route bundle: policies/locomotion/<tên>/{policy.onnx, policy.yaml}.
        const PolicyBundle walk = PolicyBundle::Load(proj_dir_, tuning_.walk_policy);
        walk_contract = walk.contract;
        walk_ctrl_ = MakeLocomotionController(walk.contract, proj_dir_ + "/" + walk.rel_path);
        walk_policy_label_ = walk.name;
        print_model("MODEL", walk.contract, walk.rel_path, *walk_ctrl_);

        const std::string teleop_name =
            tuning_.teleop_policy.empty() ? tuning_.walk_policy : tuning_.teleop_policy;
        const PolicyBundle teleop = teleop_name == walk.name
            ? walk : PolicyBundle::Load(proj_dir_, teleop_name);
        if (teleop.name != walk.name) {
            teleop_ctrl_ = MakeLocomotionController(teleop.contract,
                                                    proj_dir_ + "/" + teleop.rel_path);
            // Tay: gesture/teleop chốt default tay MỘT lần lúc khởi động (từ walk
            // policy) -> hai policy phải trùng default tay. Chân + eo: các state tư thế
            // nhắm default của policy đang chọn, và đổi nóng blend blend_time_s từ
            // target vừa gửi, nên cho lệch nhỏ (kTeleopLegDefaultGapRad); lệch hơn là
            // từ chối khởi động thay vì kéo khớp một bước lớn lúc đổi chế độ.
            constexpr float kTeleopLegDefaultGapRad = 0.10f;
            const auto& a = walk_ctrl_->default_q();
            const auto& b = teleop_ctrl_->default_q();
            float leg_gap = 0.0f;
            for (int j = 0; j < spec::kNumJoints; ++j) {
                const float gap = std::abs(a[j] - b[j]);
                const bool arm = j >= 14;
                if ((arm && gap > 1e-4f) || (!arm && gap > kTeleopLegDefaultGapRad))
                    throw std::runtime_error(
                        "teleop_policy '" + teleop.name + "' default_joint_pos khac walk_policy '" +
                        walk.name + "' o khop " + std::to_string(j) + " (" +
                        std::to_string(gap) + " rad) -> khong cho doi nong giua hai policy");
                if (!arm) leg_gap = std::max(leg_gap, gap);
            }
            if (leg_gap > 1e-4f)
                std::cout << "[Application] walk/teleop default chan+eo lech toi da " << leg_gap
                          << " rad; doi nong blend " << tuning_.blend_time_s << "s.\n";
        }
        teleop_policy_label_ = teleop.name;
        PolicyController& teleop_ctrl = teleop_ctrl_ ? *teleop_ctrl_ : *walk_ctrl_;
        print_model("TELEOP_MODEL", teleop.contract, teleop.rel_path, teleop_ctrl);

        // R2+A/R2+X chỉ chọn policy. H4 bật theo teleop_h4_enabled và dùng được ở
        // mọi policy có contract hỗ trợ overlay; policy không hỗ trợ thì START bị
        // từ chối ở chế độ đó, không chặn khởi động.
        h4_teleop_approved_ = tuning_.teleop_h4_enabled;
        if (h4_teleop_approved_) {
            if (!walk_ctrl_->SupportsH4TeleopOverlay())
                std::cout << "[Teleop] walk_policy '" << walk.name << "' (" << walk.contract
                          << ") khong ho tro H4 -> START bi tu choi o R2+A.\n";
            if (!teleop_ctrl.SupportsH4TeleopOverlay())
                std::cout << "[Teleop] teleop_policy '" << teleop.name << "' (" << teleop.contract
                          << ") khong ho tro H4 -> START bi tu choi o R2+X.\n";
        }
    } else {
        // Route cũ (rollback/so sánh): flat_model + flat_policy_contract, một policy
        // cho cả hai chế độ. Legacy nằm ở policies/flat, flat_plus ở locomotion/_archive/flat_plus.
        walk_contract = tuning_.flat_policy_contract;
        const std::string policy_dir =
            ParseFlatPolicyProfile(walk_contract) == FlatPolicyProfile::kLegacy83
                ? "policies/flat/"
                : "policies/locomotion/_archive/flat_plus/";
        const std::string rel = policy_dir + tuning_.flat_model;
        const std::string policy_path = proj_dir_ + "/" + rel;
        walk_ctrl_ = MakeLocomotionController(walk_contract, policy_path);
        if (tuning_.teleop_h4_enabled) {
            h4_teleop_approved_ = walk_ctrl_->SupportsH4TeleopOverlay() &&
                teleop_h4::Approved(true, walk_contract, policy_path,
                                    tuning_.teleop_h4_model_sha256);
            if (!h4_teleop_approved_)
                throw std::runtime_error("H4 teleop denied: selected policy contract or SHA-256 differs");
        }
        walk_policy_label_ = teleop_policy_label_ = tuning_.flat_model;
        print_model("MODEL", walk_contract, rel, *walk_ctrl_);
        print_model("TELEOP_MODEL", walk_contract, rel, *walk_ctrl_);
    }
    loco_mode_ = LocoMode::kWalk;
    locomotion_ = walk_ctrl_.get();

    // Đặt ở đây (chứ không ở Run) để --preflight cũng in ra: chu kỳ gait là một phần
    // hợp đồng train giống Kp/Kd, phải đọc được mà không cần cắm DDS.
    gait_.Configure(tuning_.gait_period_s);
    std::cout << "[" << locomotion_->Name() << "] gait_period=" << gait_.period_s()
              << "s (phai khop 'period' cua phase obs luc train " << walk_policy_label_ << ")\n";
    if (ParseFlatPolicyProfile(walk_contract) == FlatPolicyProfile::kFlatPlusGaitH4V1) {
        std::cout << "[FlatPlusGaitH4] gait_settle_dwell_s="
                  << tuning_.gait_settle_dwell_s << "s (runtime config)\n";
    }
    if (const auto* wbg = dynamic_cast<const WholeBodyLegsGaitController*>(locomotion_)) {
        std::cout << "[WholeBodyLegsGaitH4] settle_dwell=" << wbg->gait_config().settle_dwell_s
                  << "s (metadata ONNX; bo qua gait_settle_dwell_s cua locomotion.yaml)\n";
    }
    std::cout << "[Application] Locomotion: R2+A policy '" << walk_policy_label_
              << "', R2+X policy '" << teleop_policy_label_ << "'"
              << (teleop_ctrl_ ? " (hai policy rieng, doi nong khi dung yen)" : " (chung policy)")
              << "\n";

    InitGestures();

    for (int i = 0; i < Tuning::kMaxDances; ++i) {
        if (tuning_.dance_folder[i].empty()) continue;
        int key = i + 1; // dance slots 1-7
        std::cout << "[Preflight] DANCE slot=" << key
                  << " tilt_guard=" << (tuning_.state_motion_guard ? "reference" :
                                        (tuning_.DanceTiltGuard(key) ? "on" : "off"))
                  << " home_exit=" << (tuning_.DanceHomeExit(key) ? "on" : "off") << "\n";
        std::string dir = proj_dir_ + "/policies/dance/" + tuning_.dance_folder[i];

        std::string onnx, npz, music;
        ScanDanceFolder(dir, std::to_string(key), onnx, npz, music);
        // Audio is optional: only policy and reference motion gate the dance.
        if (onnx.empty() || npz.empty()) {
            std::cerr << "[Application] Dance " << key << " (" << tuning_.dance_folder[i]
                      << ") missing required files (ONNX/NPZ), skipping.\n";
            continue;
        }

        try {
            auto motion = std::make_unique<MotionData>();
            motion->Load(dir + "/" + npz);

            int start_frame = tuning_.dance_start_frame;
            if (start_frame < 0) {
                const int legacy_frame =
                    motion->FindSmoothStartFrame(tuning_.dance_start_search_frames);
                start_frame = legacy_frame;
                if (tuning_.mimic_transition_v2) {
                    std::array<float, spec::kNumJoints> zero_dq{};
                    start_frame = motion->FindTransitionStartFrame(
                        tuning_.mimic_transition_search_frames,
                        locomotion_->default_q(), zero_dq);
                    std::cout << "[Application] Dance " << key
                              << " start_frame=-1 -> legacy=" << legacy_frame
                              << ", transition_v2 preview=" << start_frame << "\n";
                } else {
                    std::cout << "[Application] Dance " << key
                              << " start_frame=-1 -> selected frame " << start_frame << "\n";
                }
            }

            auto mimic = std::make_unique<MimicController>(
                *motion, start_frame, tuning_.dance_speed[i], tuning_.mimic_warmup_s);
            mimic->ConfigureTransitionV2(
                tuning_.mimic_transition_v2,
                tuning_.dance_start_frame < 0,
                tuning_.mimic_transition_search_frames,
                tuning_.mimic_transition_clip_ramp_s);
            mimic->Init(dir + "/" + onnx, ort_env_, ort_opts_, tuning_);

            // Tính toán độ lệch giữa tư thế mở màn của bài nhảy và tư thế hiện tại.
            // Mục đích: Kiểm tra xem có cần tăng thời gian chuyển động (khởi động mềm) để đảm bảo an toàn hay không.
            const auto& dq = locomotion_->default_q();
            const auto& sp = mimic->start_pose();
            float leg_gap = 0.0f, other_gap = 0.0f;
            for (int j = 0; j < spec::kNumJoints; ++j) {
                float e = std::abs(sp[j] - dq[j]);
                if (j < 12) leg_gap   = std::max(leg_gap, e);
                else        other_gap = std::max(other_gap, e);
            }
            std::cout << "[Application] Dance " << key << " clip-vs-default gap: legs "
                      << leg_gap << " rad, arms/waist " << other_gap << " rad"
                      << (leg_gap > 0.35f ? "  <-- CHAN LECH NHIEU, kiem tra do vung!" : "") << "\n";

            // default_joint_pos của dance policy phải cùng "thế hệ" với locomotion policy:
            // RunReturning đưa robot về locomotion default và BeginCooldown cũng nhắm về
            // đó, nên lệch bao nhiêu là dance vào/ra ở tư thế lệch bấy nhiêu so với lúc
            // train nó. (vd pokemon_v9/doremon_v9 có hip_roll 0.035 - cặp với policy_v3,
            // không phải policy_goc.)
            float def_gap = 0.0f;
            int   def_gap_j = -1;
            for (int j = 0; j < spec::kNumJoints; ++j) {
                float e = std::abs(mimic->default_q()[j] - dq[j]);
                if (e > def_gap) { def_gap = e; def_gap_j = j; }
            }
            if (def_gap > 0.01f) {
                std::cerr << "[Application] ⚠ Dance " << key << " default_joint_pos LECH "
                          << def_gap << " rad (khop " << def_gap_j << ") so voi "
                          << walk_policy_label_
                          << " -- dance nay train o the he policy khac.\n";
            }

            motions_[key] = std::move(motion);
            mimics_[key] = std::move(mimic);
            if (!music.empty()) music_files_[key] = dir + "/" + music;
            std::cout << "[Application] Loaded Mimic " << key << " ("
                      << tuning_.dance_folder[i] << ")"
                      << (music.empty() ? " — no audio; dance runs without music" : "")
                      << "\n";
        } catch (const std::exception& e) {
            disabled_dance_reasons_[key] = e.what();
            std::cerr << "[Application] Dance " << key << " (" << tuning_.dance_folder[i]
                      << ") DISABLED: " << e.what() << "\n";
        }
    }
    active_ = locomotion_;

    // Nạp quỹ đạo chuyển động đứng dậy (Get up) và nằm xuống (Lie down).
    // Hệ thống sẽ in ra cảnh báo nếu không tìm thấy file ghi quỹ đạo.
    auto load_traj = [&](const std::string& file, const char* label,
                         std::unique_ptr<JointTrajectory>& out) {
        std::string path = proj_dir_ + "/" + file;
        std::ifstream probe(path);
        if (!probe.good()) {
            std::cerr << "[Application] " << label << ": chưa có file " << path
                      << " -> L2+X (" << label << ") sẽ bị chặn tới khi bạn ghi bằng "
                         "tools/record_motion.cpp.\n";
            return;
        }
        try {
            out = std::make_unique<JointTrajectory>();
            out->Load(path);
        } catch (const std::exception& e) {
            std::cerr << "[Application] Lỗi nạp " << label << " (" << path << "): " << e.what() << "\n";
            out.reset();
        }
    };
    load_traj(tuning_.getup_motion_file, "GET UP", getup_motion_);
    load_traj(tuning_.liedown_motion_file, "LIE DOWN", liedown_motion_);
}

int Application::Preflight() {
    std::cout << "[Preflight] Loading models and assets without DDS/motor output...\n";
    InitControllers();
    std::cout << "[Preflight] controller: OK\n";
    return 0;
}
