#pragma once
/**
 * Application.hpp — Vòng lặp chính 500Hz + state machine.
 */

#include <array>
#include <chrono>
#include <memory>
#include <map>
#include <mutex>
#include <string>

#include <onnxruntime_cxx_api.h>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>


#include "../audio/MusicPlayer.hpp"
#include "../config/RobotSpec.hpp"
#include "../config/Tuning.hpp"
#include "../estimation/StateEstimator.hpp"
#include "../gait/GaitScheduler.hpp"
#include "../input/InputManager.hpp"
#include "../motion/MotionData.hpp"
#include "../policy/LocomotionController.hpp"
#include "../policy/MimicController.hpp"
#include "../robot/LowCmdSender.hpp"
#include "../safety/FallDetector.hpp"

enum class AppState {
    kWaitingForState,
    kIdle,
    kStandUp,
    kStandLock,
    kLocomotion,
    kReturningToDefault,
    kMimic,
    kSafeShutdown,
};

enum class PendingAction {
    kNone,
    kToStandLock,
    kToSit,
};

class Application {
public:
    Application(std::string proj_dir, std::string interface_override);

    int Run();

private:
    void InitDds();
    void InitControllers();

    void OnLowState(const void* msg);

    bool Tick();

    void HandleTransitions(const InputCommand& cmd);
    void RequestFromPolicy(PendingAction act);
    void BeginStandUp(const RobotState& rs);
    void BeginSit(const RobotState& rs);
    void UpdateSafeStop(InputCommand& cmd);
    void RunStandUp();
    void RunStandLock();
    void RunReturning();
    void RunPolicy(const InputCommand& cmd);
    void RunSafeShutdown();

    void EnterIdle(const std::string& reason);
    void ActivatePolicy(PolicyController* ctrl);
    void UpdateHud();
    void PrintStatus();

    std::string StateName() const;

    std::string proj_dir_;
    Tuning tuning_;

    Ort::Env ort_env_{ORT_LOGGING_LEVEL_WARNING, "r1_policy"};
    Ort::SessionOptions ort_opts_;

    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> low_state_sub_;
    unitree_hg::msg::dds_::LowState_ shared_low_state_;
    std::mutex state_mutex_;
    bool got_state_ = false;
    std::chrono::steady_clock::time_point last_state_time_;

    StateEstimator estimator_;
    GaitScheduler gait_;
    FallDetector fall_detector_;
    InputManager input_;
    LowCmdSender sender_;

    std::unique_ptr<LocomotionController> locomotion_;
    std::map<int, std::unique_ptr<MotionData>> motions_;
    std::map<int, std::unique_ptr<MimicController>> mimics_;
    std::map<int, std::string> music_files_;
    MusicPlayer music_;
    int pending_dance_key_ = 0;
    PolicyController* active_ = nullptr;

    AppState state_ = AppState::kWaitingForState;
    bool running_ = true;
    long tick_ = 0;

    std::array<float, spec::kNumJoints> ai_target_q_{};
    std::array<float, spec::kNumJoints> stand_gains_kp_{};
    std::array<float, spec::kNumJoints> stand_gains_kd_{};
    std::array<float, spec::kNumJoints> sit_gains_kp_{};
    std::array<float, spec::kNumJoints> sit_gains_kd_{};

    std::array<float, spec::kNumJoints> stand_start_q_{};
    float stand_timer_ = 0.0f;

    std::array<float, spec::kNumJoints> sit_start_q_{};
    float sit_timer_ = 0.0f;
    int   sit_phase_ = 0;

    float blend_alpha_ = 1.0f;
    std::array<float, spec::kNumJoints> blend_start_q_{};

    bool  announcing_ = false;
    float announce_timer_ = 0.0f;

    PendingAction pending_action_ = PendingAction::kNone;
    float settle_timer_ = 0.0f;

    bool input_lost_ = false;
    int  input_lost_count_ = 0;

    float cmd_vx_ = 0.0f, cmd_vy_ = 0.0f, cmd_yaw_ = 0.0f;
};
