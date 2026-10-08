// LowState hỏng phải dừng ở StateEstimator: một kênh IMU chết (quaternion toàn
// 0) từng cho projected_gravity = (0,0,-1) = "đứng thẳng hoàn hảo" và làm
// FallDetector mù mà không cần một NaN nào.
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

#include "estimation/StateEstimator.hpp"
#include "safety/FallDetector.hpp"

using LowState_ = unitree_hg::msg::dds_::LowState_;

namespace {

void Require(bool condition, const char* expression, int line) {
    if (!condition) {
        std::cerr << "Requirement failed at line " << line << ": " << expression << "\n";
        std::abort();
    }
}

#define REQUIRE(condition) Require(static_cast<bool>(condition), #condition, __LINE__)

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// Nghiêng quanh trục y một góc tilt -> quaternion (w, 0, y, 0), tuỳ chọn nhân
// scale để mô phỏng quaternion chưa chuẩn hoá.
LowState_ Packet(float tilt_rad, float scale = 1.0f) {
    LowState_ low;
    low.imu_state().quaternion({std::cos(tilt_rad / 2.0f) * scale, 0.0f,
                                std::sin(tilt_rad / 2.0f) * scale, 0.0f});
    low.imu_state().gyroscope({0.0f, 0.0f, 0.0f});
    low.imu_state().accelerometer({0.0f, 0.0f, -9.81f});
    for (int i = 0; i < spec::kNumMotorsIdl; ++i) {
        low.motor_state()[static_cast<size_t>(i)].q(0.0f);
        low.motor_state()[static_cast<size_t>(i)].dq(0.0f);
    }
    return low;
}

StateEstimator MakeEstimator(const Tuning& tuning) {
    StateEstimator est;
    est.Configure(tuning, spec::kLoopDt);
    return est;
}

void TestDeadImuRejected() {
    Tuning tuning;
    StateEstimator est = MakeEstimator(tuning);

    REQUIRE(est.Update(Packet(0.9f)));            // nghiêng ~51 độ, hợp lệ
    const Eigen::Vector3f tilted = est.state().projected_gravity;
    REQUIRE(tilted.z() > -0.7f);                  // đúng là đang nghiêng nhiều

    LowState_ dead = Packet(0.9f);
    dead.imu_state().quaternion({0.0f, 0.0f, 0.0f, 0.0f});
    // IMU chết là hỏng LIÊN TỤC -> chốt sau đúng 3 gói (6 ms @500 Hz).
    REQUIRE(est.Update(dead));
    REQUIRE(est.Update(dead));
    REQUIRE(!est.Update(dead));
    // Không được "đứng thẳng hoàn hảo": giữ nguyên mẫu nghiêng hợp lệ cuối.
    REQUIRE(std::fabs(est.state().projected_gravity.z() - tilted.z()) < 1e-6f);
    REQUIRE(!est.valid());
}

// Một gói dị thường lẻ không được kéo robot về damping giữa lúc đang đi: dữ liệu
// hỏng đã bị chặn (state giữ mẫu cuối) nên bỏ qua là an toàn.
void TestSingleBadPacketDoesNotTrip() {
    Tuning tuning;
    StateEstimator est = MakeEstimator(tuning);
    REQUIRE(est.Update(Packet(0.2f)));

    LowState_ glitch = Packet(0.2f);
    glitch.imu_state().quaternion({0.0f, 0.0f, 0.0f, 0.0f});
    for (int i = 0; i < 20; ++i) {
        REQUIRE(est.Update(glitch));          // 1 gói hỏng...
        REQUIRE(est.Update(Packet(0.2f)));    // ...rồi 1 gói lành: không bao giờ chốt
        REQUIRE(est.valid());
    }
    REQUIRE(est.repaired_count() == 20);      // vẫn đếm được để chẩn đoán
}

void TestQuaternionNormalized() {
    Tuning tuning;
    StateEstimator est = MakeEstimator(tuning);
    REQUIRE(est.Update(Packet(0.5f)));
    const Eigen::Vector3f unit = est.state().projected_gravity;

    StateEstimator scaled_est = MakeEstimator(tuning);
    REQUIRE(scaled_est.Update(Packet(0.5f, 1.5f)));   // cùng hướng, norm 1.5
    const Eigen::Vector3f scaled = scaled_est.state().projected_gravity;
    REQUIRE((unit - scaled).norm() < 1e-5f);
    REQUIRE(std::fabs(unit.norm() - 1.0f) < 1e-5f);
}

void TestBadJointRejected() {
    Tuning tuning;
    StateEstimator est = MakeEstimator(tuning);
    LowState_ good = Packet(0.0f);
    good.motor_state()[static_cast<size_t>(spec::MotorIdl(3))].q(0.42f);
    REQUIRE(est.Update(good));
    REQUIRE(std::fabs(est.state().q[3] - 0.42f) < 1e-6f);

    LowState_ bad = Packet(0.0f);
    bad.motor_state()[static_cast<size_t>(spec::MotorIdl(3))].q(kNaN);
    est.Update(bad);
    est.Update(bad);
    REQUIRE(!est.Update(bad));
    REQUIRE(std::fabs(est.state().q[3] - 0.42f) < 1e-6f);   // giữ mẫu hợp lệ cuối
}

// Ngay cả khi qua được estimator, fall detector phải fail-closed: mọi so sánh
// với NaN đều false nên trước đây nó im lặng không bao giờ báo ngã.
void TestFallDetectorFailsClosed() {
    Tuning tuning;
    FallDetector det;
    det.Configure(tuning, spec::kLoopDt);
    std::vector<std::string> reasons;
    const Eigen::Vector3f nan_g(kNaN, kNaN, kNaN);
    REQUIRE(det.Check(nan_g, Eigen::Vector3f::Zero(), 0.0f, -1, reasons));
    REQUIRE(!reasons.empty());
}

}  // namespace

int main() {
    TestDeadImuRejected();
    TestSingleBadPacketDoesNotTrip();
    TestQuaternionNormalized();
    TestBadJointRejected();
    TestFallDetectorFailsClosed();
    std::cout << "HB_STATE_ESTIMATOR_OK\n";
    return 0;
}
