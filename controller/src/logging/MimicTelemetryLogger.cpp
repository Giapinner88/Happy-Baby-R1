#include "MimicTelemetryLogger.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace {

const char* kJointName[spec::kNumJoints] = {
    "L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee", "L_ank_pitch", "L_ank_roll",
    "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee", "R_ank_pitch", "R_ank_roll",
    "waist_roll",  "waist_yaw",
    "L_sh_pitch",  "L_sh_roll",  "L_sh_yaw",  "L_elbow", "L_wrist",
    "R_sh_pitch",  "R_sh_roll",  "R_sh_yaw",  "R_elbow", "R_wrist"};

int64_t NowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

std::string Timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream os;
    os << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return os.str();
}

std::string SafeLabel(std::string label) {
    for (char& c : label) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                        (c >= '0' && c <= '9') || c == '-' || c == '_';
        if (!ok) c = '_';
    }
    return label.empty() ? "unknown" : label;
}

template <size_t N>
void WriteArray(std::ofstream& out, const std::array<float, N>& values) {
    for (float v : values) out << ',' << v;
}

}  // namespace

MimicTelemetryLogger::~MimicTelemetryLogger() {
    Shutdown();
}

bool MimicTelemetryLogger::StartSession(const std::string& base_dir, int dance_key,
                                        const std::string& dance_label, int sample_hz) {
    Shutdown();

    std::error_code ec;
    std::filesystem::create_directories(base_dir, ec);
    if (ec) {
        std::cerr << "[Telemetry] Không tạo được thư mục " << base_dir << ": "
                  << ec.message() << "\n";
        return false;
    }

    path_ = base_dir + "/mimic_" + Timestamp() + "_dance" +
            std::to_string(dance_key) + "_" + SafeLabel(dance_label) + ".csv";
    start_us_ = NowUs();
    dropped_.store(0, std::memory_order_relaxed);
    stop_requested_.store(false, std::memory_order_release);
    accepting_.store(true, std::memory_order_release);
    writer_ = std::thread(&MimicTelemetryLogger::WriterLoop, this, sample_hz);
    std::cout << "[Telemetry] Bắt đầu ghi " << sample_hz << " Hz -> " << path_ << "\n";
    return true;
}

void MimicTelemetryLogger::Enqueue(const MimicTelemetrySample& sample) {
    if (!accepting_.load(std::memory_order_acquire)) return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= kMaxQueuedSamples) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        queue_.push_back(sample);
    }
    cv_.notify_one();
}

void MimicTelemetryLogger::StopAsync() {
    accepting_.store(false, std::memory_order_release);
    stop_requested_.store(true, std::memory_order_release);
    cv_.notify_one();
}

void MimicTelemetryLogger::Shutdown() {
    StopAsync();
    if (writer_.joinable()) writer_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
}

void MimicTelemetryLogger::WriterLoop(int sample_hz) {
    std::ofstream out(path_, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        std::cerr << "[Telemetry] Không mở được file " << path_ << "\n";
        accepting_.store(false, std::memory_order_release);
        return;
    }

    out << "# sample_hz=" << sample_hz << "\n";
    out << "# app_state=0:disarmed,1:conflict,2:waiting,3:idle,4:zero_torque,"
           "5:stand_up,6:stand_lock,7:locomotion,8:returning,9:mimic,10:safe_shutdown\n";
    out << "# mimic_stage=-1:none,0:warmup,1:clip,2:cooldown,3:finished\n";
    out << "# foot_force=unavailable_in_unitree_hg_LowState; use tau_est plus kinematics\n";
    out << "monotonic_us,elapsed_s,loop_tick,lowstate_tick,app_state,dance_key,"
           "mimic_stage,clip_phase,clip_frame,quat_w,quat_x,quat_y,quat_z,"
           "ref_quat_w,ref_quat_x,ref_quat_y,ref_quat_z,gravity_x,gravity_y,gravity_z,"
           "gyro_x,gyro_y,gyro_z,gyro_raw_x,gyro_raw_y,gyro_raw_z,"
           "accel_raw_x,accel_raw_y,accel_raw_z";
    for (int i = 0; i < spec::kNumJoints; ++i) out << ",ref_" << kJointName[i];
    for (int i = 0; i < spec::kNumJoints; ++i) out << ",action_" << kJointName[i];
    for (int i = 0; i < spec::kNumJoints; ++i) out << ",qdes_" << kJointName[i];
    for (int i = 0; i < spec::kNumJoints; ++i) out << ",q_" << kJointName[i];
    for (int i = 0; i < spec::kNumJoints; ++i) out << ",dq_" << kJointName[i];
    for (int i = 0; i < spec::kNumJoints; ++i) out << ",dq_raw_" << kJointName[i];
    for (int i = 0; i < spec::kNumJoints; ++i) out << ",tau_est_" << kJointName[i];
    for (int i = 0; i < spec::kNumJoints; ++i) out << ",kp_" << kJointName[i];
    for (int i = 0; i < spec::kNumJoints; ++i) out << ",kd_" << kJointName[i];
    out << '\n';
    out << std::fixed << std::setprecision(6);

    std::vector<MimicTelemetrySample> batch;
    batch.reserve(512);
    while (true) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
                return !queue_.empty() || stop_requested_.load(std::memory_order_acquire);
            });
            while (!queue_.empty() && batch.size() < 512) {
                batch.push_back(std::move(queue_.front()));
                queue_.pop_front();
            }
            if (batch.empty() && stop_requested_.load(std::memory_order_acquire)) break;
        }

        for (const auto& s : batch) {
            out << s.monotonic_us << ',' << (s.elapsed_us / 1e6) << ',' << s.loop_tick << ','
                << s.lowstate_tick << ',' << s.app_state << ',' << s.dance_key << ','
                << s.mimic_stage << ',' << s.clip_phase << ',' << s.clip_frame;
            WriteArray(out, s.quat);
            WriteArray(out, s.ref_quat);
            WriteArray(out, s.gravity);
            WriteArray(out, s.gyro);
            WriteArray(out, s.gyro_raw);
            WriteArray(out, s.accel_raw);
            WriteArray(out, s.ref_q);
            WriteArray(out, s.action);
            WriteArray(out, s.q_des);
            WriteArray(out, s.q);
            WriteArray(out, s.dq);
            WriteArray(out, s.dq_raw);
            WriteArray(out, s.tau_est);
            WriteArray(out, s.kp);
            WriteArray(out, s.kd);
            out << '\n';
        }
        batch.clear();
    }

    out.flush();
    out.close();
    std::cout << "[Telemetry] Đã đóng " << path_ << " (dropped="
              << dropped_.load(std::memory_order_relaxed) << ")\n";
}
