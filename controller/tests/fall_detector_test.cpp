#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "config/Tuning.hpp"
#include "safety/FallDetector.hpp"

namespace {

void Require(bool condition, const char* expression, int line) {
    if (!condition) {
        std::cerr << "Requirement failed at line " << line << ": " << expression << "\n";
        std::abort();
    }
}

#define REQUIRE(condition) Require(static_cast<bool>(condition), #condition, __LINE__)

Eigen::Vector3f GravityAtTiltDeg(float tilt_deg) {
    const float rad = tilt_deg * static_cast<float>(M_PI) / 180.0f;
    return {std::sin(rad), 0.0f, -std::cos(rad)};
}

bool CheckRepeated(FallDetector& detector,
                   const Eigen::Vector3f& gravity,
                   const Eigen::Vector3f& gyro,
                   float joint_speed,
                   int ticks,
                   std::vector<std::string>& reasons) {
    bool triggered = false;
    for (int i = 0; i < ticks; ++i)
        triggered = detector.Check(gravity, gyro, joint_speed, 7, reasons);
    return triggered;
}

}  // namespace

int main() {
    Tuning tuning;
    REQUIRE(tuning.LoadFromFile(HB_TUNING_CONFIG_PATH));
    REQUIRE(tuning.fall_enabled);
    REQUIRE(tuning.joint_speed_guard_enabled);

    constexpr float kLoopDt = 0.002f;  // runtime HB chạy guard ở 500 Hz
    const int fall_ticks = std::max(
        1, static_cast<int>(tuning.fall_debounce_ms / 1000.0f / kLoopDt));
    const int joint_ticks = std::max(
        1, static_cast<int>(tuning.joint_speed_debounce_ms / 1000.0f / kLoopDt));
    REQUIRE(fall_ticks > 0);
    REQUIRE(joint_ticks > 0);
    const float effective_fall_debounce_ms = fall_ticks * kLoopDt * 1000.0f;
    const float effective_joint_debounce_ms = joint_ticks * kLoopDt * 1000.0f;
    REQUIRE(effective_fall_debounce_ms <= tuning.fall_debounce_ms);
    REQUIRE(effective_fall_debounce_ms >= tuning.fall_debounce_ms - kLoopDt * 1000.0f - 1e-3f);
    REQUIRE(effective_joint_debounce_ms <= tuning.joint_speed_debounce_ms);
    REQUIRE(effective_joint_debounce_ms >=
            tuning.joint_speed_debounce_ms - kLoopDt * 1000.0f - 1e-3f);

    FallDetector detector;
    detector.Configure(tuning, kLoopDt);
    std::vector<std::string> reasons;

    // Dưới mọi ngưỡng phải tiếp tục chạy và không sinh lý do giả.
    REQUIRE(!CheckRepeated(detector, GravityAtTiltDeg(0.0f), {0.0f, 0.0f, 0.0f},
                           tuning.joint_speed_limit - 0.1f, fall_ticks * 2, reasons));
    REQUIRE(reasons.empty());

    // Nghiêng vượt fall_tilt_deg nhưng chưa đủ debounce thì chưa được kích hoạt.
    detector.Reset();
    reasons.clear();
    REQUIRE(!CheckRepeated(detector, GravityAtTiltDeg(tuning.fall_tilt_deg + 1.0f),
                           {0.0f, 0.0f, 0.0f}, 0.0f, fall_ticks - 1, reasons));
    REQUIRE(CheckRepeated(detector, GravityAtTiltDeg(tuning.fall_tilt_deg + 1.0f),
                          {0.0f, 0.0f, 0.0f}, 0.0f, 1, reasons));
    REQUIRE(!reasons.empty());

    // Nhánh lật nhanh: góc vượt flip threshold và gyro vượt ngưỡng.
    detector.Reset();
    reasons.clear();
    REQUIRE(CheckRepeated(detector, GravityAtTiltDeg(tuning.fall_flip_tilt_deg + 1.0f),
                          {tuning.fall_flip_gyro + 0.1f, 0.0f, 0.0f}, 0.0f,
                          fall_ticks, reasons));

    // Nhánh tốc độ khớp phải độc lập với IMU.
    detector.Reset();
    reasons.clear();
    REQUIRE(CheckRepeated(detector, GravityAtTiltDeg(0.0f), {0.0f, 0.0f, 0.0f},
                          tuning.joint_speed_limit + 0.1f, joint_ticks, reasons));

    // Reset phải bỏ latch ngã.
    detector.Reset();
    reasons.clear();
    REQUIRE(!detector.is_fallen());
    REQUIRE(!detector.Check(GravityAtTiltDeg(0.0f), {0.0f, 0.0f, 0.0f},
                            0.0f, -1, reasons));

    std::cout << "HB_FALL_DETECTOR_OK"
              << " tilt_deg=" << tuning.fall_tilt_deg
              << " flip_tilt_deg=" << tuning.fall_flip_tilt_deg
              << " flip_gyro=" << tuning.fall_flip_gyro
              << " joint_speed_limit=" << tuning.joint_speed_limit
              << " configured_debounce_ms=" << tuning.fall_debounce_ms
              << " effective_fall_debounce_ms=" << effective_fall_debounce_ms
              << " effective_joint_debounce_ms=" << effective_joint_debounce_ms << "\n";
    return 0;
}
