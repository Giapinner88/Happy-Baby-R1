// Trần cổ chân phải nằm ĐÚNG giữa hai ràng buộc:
//   - đủ rộng để không cắt bất kỳ frame nào của quỹ đạo đã ghi trên robot;
//   - đủ chặt để không lệnh ra ngoài vùng robot đã thực sự đi được.
// Trần cũ 0.44 rad vi phạm vế thứ hai: tư thế NGỒI giữ vô hạn ở đúng 0.44 với
// Kp=200 trong khi giá trị lớn nhất từng ghi được là 0.3686.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "../src/config/RobotSpec.hpp"
#include "../src/motion/JointTrajectory.hpp"

namespace {

void Require(bool condition, const char* expression, int line) {
    if (!condition) {
        std::cerr << "Requirement failed at line " << line << ": " << expression << "\n";
        std::abort();
    }
}

#define REQUIRE(condition) Require(static_cast<bool>(condition), #condition, __LINE__)

// Giá trị lớn nhất đo được trên phần cứng, quét 74.442 frame encoder trong
// motions/captures* (xem chú thích ở RobotSpec.hpp). Ghim lại để một lần
// re-record không âm thầm nới trần.
constexpr float kObservedMaxAnkleRoll = 0.3686f;

float MaxAbsAnkleRoll(const std::string& path) {
    JointTrajectory traj;
    traj.Load(path);
    float worst = 0.0f;
    for (int f = 0; f < traj.num_frames(); ++f) {
        const auto& q = traj.frame(f);
        worst = std::max(worst, std::fabs(q[5]));
        worst = std::max(worst, std::fabs(q[11]));
    }
    return worst;
}

}  // namespace

int main(int argc, char** argv) {
    REQUIRE(argc >= 3);

    // 1. Trần không được cắt quỹ đạo đã ghi.
    for (int i = 1; i < 3; ++i) {
        const float worst = MaxAbsAnkleRoll(argv[i]);
        std::cout << argv[i] << ": max |ankle_roll| = " << worst << "\n";
        REQUIRE(worst <= spec::kAnkleRollLimit);
        REQUIRE(worst <= kObservedMaxAnkleRoll + 1e-4f);
    }

    // 2. Trần không được vượt vùng đã chứng minh trên phần cứng.
    REQUIRE(spec::kAnkleRollLimit >= kObservedMaxAnkleRoll);
    REQUIRE(spec::kAnkleRollLimit <= kObservedMaxAnkleRoll + 0.02f);

    // 3. Tư thế ngồi: sit_rest_spread=0.45 luôn đẩy target vượt trần, nên trần
    // CHÍNH LÀ giá trị cổ chân bị giữ vô hạn. Kiểm tra nó nằm trong vùng đã
    // chứng minh — đây là điều kiện mà 0.44 vi phạm.
    constexpr float kSitRestSpread = 0.45f;
    const float sit_target = std::clamp(kSitRestSpread, -spec::kAnkleRollLimit,
                                        spec::kAnkleRollLimit);
    REQUIRE(std::fabs(sit_target - spec::kAnkleRollLimit) < 1e-6f);   // luôn chạm trần
    // -> giá trị giữ vô hạn chính là trần, mà trần đã bị (2) ràng vào vùng đã
    // chứng minh. Với trần cũ 0.44 thì bước này giữ ở 0.44, xa hơn mọi quan sát
    // 0.07 rad, suốt thời gian robot còn ngồi.

    // 4. Trần pitch cố ý giữ đúng MJCF (chặt hơn quan sát = sai về phía an toàn).
    REQUIRE(std::fabs(spec::kAnklePitchMin + 0.87266f) < 1e-6f);
    REQUIRE(std::fabs(spec::kAnklePitchMax - 0.57596f) < 1e-6f);

    std::cout << "HB_ANKLE_LIMITS_OK roll_limit=" << spec::kAnkleRollLimit
              << " observed_max=" << kObservedMaxAnkleRoll << "\n";
    return 0;
}
