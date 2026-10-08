#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "config/Tuning.hpp"
#include "gait/GaitScheduler.hpp"

namespace {
void Require(bool condition) {
    if (!condition) std::abort();
}
bool Near(float a, float b) { return std::fabs(a - b) < 1e-4f; }
}  // namespace

// Standard assert disappears under NDEBUG; CTest also runs in Release.
#undef assert
#define assert(condition) Require(static_cast<bool>(condition))

int main() {
    // Chưa Configure -> giữ hành vi cũ (hằng số train-locked trong RobotSpec).
    GaitScheduler untouched;
    assert(Near(untouched.period_s(), spec::kGaitPeriodS));

    // Chu kỳ từ config thật sự đổi được nhịp: 1/4 chu kỳ -> sin=1, cos=0.
    GaitScheduler gait;
    gait.Configure(0.6f);
    assert(Near(gait.period_s(), 0.6f));
    gait.Reset();
    for (int i = 0; i < 3; ++i) gait.Update(0.05f);   // t = 0.15s = 0.6/4
    auto phase = gait.PhaseObs(1.0f);
    assert(Near(phase[0], 1.0f) && Near(phase[1], 0.0f));

    // Cùng thời điểm, chu kỳ 0.8 phải ra pha KHÁC — đây chính là lỗi nếu deploy
    // chạy lệch period so với lúc train.
    GaitScheduler slow;
    slow.Configure(0.8f);
    slow.Reset();
    for (int i = 0; i < 3; ++i) slow.Update(0.05f);
    auto slow_phase = slow.PhaseObs(1.0f);
    assert(!Near(slow_phase[0], phase[0]));

    // Cổng lệnh giữ nguyên contract lúc train: ‖cmd‖ < 0.1 -> (0, 0).
    assert(Near(gait.PhaseObs(0.05f)[0], 0.0f) && Near(gait.PhaseObs(0.05f)[1], 0.0f));

    // Đầu vào vô lý không được phép biến PhaseObs thành NaN/chia-cho-0.
    GaitScheduler guarded;
    guarded.Configure(0.0f);
    assert(Near(guarded.period_s(), spec::kGaitPeriodS));
    guarded.Configure(std::nanf(""));
    assert(Near(guarded.period_s(), spec::kGaitPeriodS));

    // Khoá đọc được từ config thật và nằm trong dải hợp lệ.
    Tuning tuning;
    assert(tuning.LoadFromFile(HB_TUNING_CONFIG_PATH));
    assert(tuning.gait_period_s >= 0.3f && tuning.gait_period_s <= 1.5f);

    // Validate phải TỪ CHỐI khởi động chứ không âm thầm clamp.
    const auto bad_path = std::filesystem::temp_directory_path() /
                          "hb_high_level_bad_gait.yaml";
    {
        std::ofstream bad(bad_path);
        bad << "gait_period_s: 0\n";
    }
    Tuning zero_period;
    assert(!zero_period.LoadFromFile(bad_path.string()));
    {
        std::ofstream bad(bad_path);
        bad << "gait_period_s: 600\n";   // nhập nhầm mili giây
    }
    Tuning ms_period;
    assert(!ms_period.LoadFromFile(bad_path.string()));
    std::filesystem::remove(bad_path);

    return 0;
}
