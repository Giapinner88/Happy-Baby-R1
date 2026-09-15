#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "../config/RobotSpec.hpp"

// Một sample telemetry đã được copy hoàn toàn khỏi control loop. Writer thread chỉ
// làm việc trên struct này, không chạm DDS/controller nên không thể giữ lock điều khiển.
struct MimicTelemetrySample {
    int64_t monotonic_us = 0;
    int64_t elapsed_us = 0;
    int64_t loop_tick = 0;
    uint32_t lowstate_tick = 0;
    int app_state = 0;
    int dance_key = 0;
    int mimic_stage = -1;  // -1=ngoài mimic, 0=warmup, 1=clip, 2=cooldown, 3=finished
    double clip_phase = -1.0;
    int clip_frame = -1;

    std::array<float, 4> quat{};      // pelvis IMU, wxyz
    std::array<float, 4> ref_quat{};  // torso reference trong world, wxyz
    std::array<float, 3> gravity{};
    std::array<float, 3> gyro{};
    std::array<float, 3> gyro_raw{};
    std::array<float, 3> accel_raw{};

    std::array<float, spec::kNumJoints> ref_q{};
    std::array<float, spec::kNumJoints> action{};
    std::array<float, spec::kNumJoints> q_des{};
    std::array<float, spec::kNumJoints> q{};
    std::array<float, spec::kNumJoints> dq{};
    std::array<float, spec::kNumJoints> dq_raw{};
    std::array<float, spec::kNumJoints> tau_est{};
    std::array<float, spec::kNumJoints> kp{};
    std::array<float, spec::kNumJoints> kd{};
};

// CSV writer bất đồng bộ. Producer là control loop (100 Hz mặc định); consumer là
// một thread riêng. Queue có giới hạn để disk chậm không bao giờ làm phình RAM vô hạn.
class MimicTelemetryLogger {
public:
    MimicTelemetryLogger() = default;
    ~MimicTelemetryLogger();

    MimicTelemetryLogger(const MimicTelemetryLogger&) = delete;
    MimicTelemetryLogger& operator=(const MimicTelemetryLogger&) = delete;

    bool StartSession(const std::string& base_dir, int dance_key,
                      const std::string& dance_label, int sample_hz);
    void Enqueue(const MimicTelemetrySample& sample);
    void StopAsync();
    void Shutdown();

    bool Active() const { return accepting_.load(std::memory_order_acquire); }
    uint64_t Dropped() const { return dropped_.load(std::memory_order_relaxed); }
    const std::string& path() const { return path_; }
    int64_t start_us() const { return start_us_; }

private:
    void WriterLoop(int sample_hz);

    static constexpr size_t kMaxQueuedSamples = 4096;

    std::atomic<bool> accepting_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<uint64_t> dropped_{0};
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<MimicTelemetrySample> queue_;
    std::thread writer_;
    std::string path_;
    int64_t start_us_ = 0;
};
