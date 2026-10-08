// Contract test cho history bank + packer `flat_plus_h4_v1` phía HB.
//
// Vector kỳ vọng được viết TAY từ bảng offset trong kế hoạch, cố ý không suy ra
// từ chính các hàm đang kiểm tra: packer tự đối chiếu số học của mình thì không
// chứng minh được gì.
//
// Test này phải cho ra CÙNG kết quả với
// unitree_mujoco/simulate_cpp/tests/unit/flat_plus_history_test.cpp.

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "../src/policy/BaseObservationHistory.hpp"

namespace hist = r1::history;

namespace {

std::vector<float> MakeFrame(int id) {
    std::vector<float> f(hist::kBaseDim);
    for (int i = 0; i < hist::kBaseDim; ++i)
        f[static_cast<std::size_t>(i)] = id * 1000.0f + static_cast<float>(i);
    return f;
}

bool Same(const std::vector<float>& a, const std::vector<float>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::fabs(a[i] - b[i]) > 1e-6f) return false;
    return true;
}

void TestOffsets() {
    const int block[7] = {0, 12, 24, 36, 44, 140, 236};
    const int current[7] = {9, 21, 33, 42, 116, 212, 308};
    for (std::size_t t = 0; t < hist::kTerms.size(); ++t) {
        assert(hist::TermBlockStart(t) == block[t]);
        assert(hist::TermFrameStart(t, 0) == current[t]);
    }
    assert(hist::TermFrameStart(4, 3) == 44);   // joint_pos, frame cũ nhất
    assert(hist::TermFrameStart(4, 0) == 116);  // joint_pos, frame mới nhất
    assert(hist::kH4ActorDim == 332);
    assert(hist::kBaseDim == 83);
    assert(hist::kH5ActorDim == 415);
}

void TestBackfillNotZeroPad() {
    hist::BaseObservationHistory bank;
    assert(!bank.ready());
    bank.Append(MakeFrame(7));
    assert(bank.ready() && bank.valid_steps() == hist::kH4Steps);
    for (int lag = 0; lag < hist::kH4Steps; ++lag)
        assert(Same(bank.Frame(lag), MakeFrame(7)));

    std::vector<float> packed;
    bank.PackTermMajorH4(packed);
    for (int lag = 1; lag < hist::kH4Steps; ++lag) {
        const int start = hist::TermFrameStart(4, lag);
        assert(packed[static_cast<std::size_t>(start)] != 0.0f);
    }
}

void TestRoll() {
    hist::BaseObservationHistory bank;
    bank.Append(MakeFrame(0));
    for (int step = 1; step <= 2 * hist::kH4Steps; ++step) {
        bank.Append(MakeFrame(step));
        for (int lag = 0; lag < hist::kH4Steps; ++lag) {
            const int want = step - lag;
            if (want < 0) continue;
            assert(Same(bank.Frame(lag), MakeFrame(want)));
        }
    }
}

void TestTermMajor() {
    hist::BaseObservationHistory bank;
    for (int step = 0; step < hist::kH4Steps; ++step) bank.Append(MakeFrame(step));
    std::vector<float> packed;
    bank.PackTermMajorH4(packed);
    assert(packed.size() == static_cast<std::size_t>(hist::kH4ActorDim));

    for (std::size_t t = 0; t < hist::kTerms.size(); ++t) {
        const int base = hist::BaseTermStart(t);
        for (int lag = 0; lag < hist::kH4Steps; ++lag) {
            const std::vector<float> src = MakeFrame((hist::kH4Steps - 1) - lag);
            const int dst = hist::TermFrameStart(t, lag);
            for (int k = 0; k < hist::kTerms[t].dim; ++k)
                assert(std::fabs(packed[static_cast<std::size_t>(dst + k)] -
                                 src[static_cast<std::size_t>(base + k)]) < 1e-6f);
        }
    }

    // Kiểm tra quyết định: time-major sẽ đặt nguyên frame cũ nhất vào [0,83).
    const std::vector<float> oldest = MakeFrame(0);
    bool time_major = true;
    for (int i = 0; i < hist::kBaseDim; ++i) {
        if (std::fabs(packed[static_cast<std::size_t>(i)] -
                      oldest[static_cast<std::size_t>(i)]) > 1e-6f) {
            time_major = false;
            break;
        }
    }
    assert(!time_major);
}

void TestH5TermMajor() {
    hist::BaseObservationHistory bank(hist::kH5Steps);
    for (int step = 0; step < hist::kH5Steps; ++step)
        bank.Append(MakeFrame(step));

    std::vector<float> packed;
    bank.PackTermMajor(packed);
    assert(packed.size() == static_cast<std::size_t>(hist::kH5ActorDim));
    for (std::size_t term = 0; term < hist::kTerms.size(); ++term) {
        const int base = hist::BaseTermStart(term);
        for (int lag = 0; lag < hist::kH5Steps; ++lag) {
            const auto& frame = bank.Frame(lag);
            const int dst = hist::TermFrameStart(term, lag, hist::kH5Steps);
            for (int k = 0; k < hist::kTerms[term].dim; ++k)
                assert(std::fabs(packed[static_cast<std::size_t>(dst + k)] -
                                 frame[static_cast<std::size_t>(base + k)]) < 1e-6f);
        }
    }

    bool threw = false;
    try {
        bank.PackTermMajorH4(packed);
    } catch (const std::exception&) {
        threw = true;
    }
    assert(threw);
}

void TestNewestSliceRebuildsBaseFrame() {
    hist::BaseObservationHistory bank;
    bank.Append(MakeFrame(1));
    bank.Append(MakeFrame(2));
    std::vector<float> packed;
    bank.PackTermMajorH4(packed);

    std::vector<float> rebuilt;
    for (std::size_t t = 0; t < hist::kTerms.size(); ++t) {
        const int start = hist::TermFrameStart(t, 0);
        for (int k = 0; k < hist::kTerms[t].dim; ++k)
            rebuilt.push_back(packed[static_cast<std::size_t>(start + k)]);
    }
    assert(Same(rebuilt, MakeFrame(2)));
}

void TestResetOnReactivation() {
    hist::BaseObservationHistory bank;
    bank.Append(MakeFrame(1));
    bank.Append(MakeFrame(2));
    bank.Reset();
    assert(!bank.ready() && bank.valid_steps() == 0);

    bool threw = false;
    try {
        std::vector<float> p;
        bank.PackTermMajorH4(p);
    } catch (const std::exception&) {
        threw = true;
    }
    assert(threw);

    // Kích hoạt lại: không được còn frame của lần chạy trước.
    bank.Append(MakeFrame(9));
    for (int lag = 0; lag < hist::kH4Steps; ++lag)
        assert(Same(bank.Frame(lag), MakeFrame(9)));
}

void TestWrongFrameSizeRejected() {
    hist::BaseObservationHistory bank;
    bool threw = false;
    try {
        bank.Append(std::vector<float>(hist::kH4ActorDim, 0.0f));
    } catch (const std::exception&) {
        threw = true;
    }
    assert(threw);
}

void TestAppendRateIsPolicyStepNotControlLoop() {
    // Vòng ngoài chạy 500 Hz, policy 50 Hz -> decimation 10. Nếu ai đó append
    // ở vòng ngoài, cửa sổ sẽ chứa 4 frame gần trùng nhau (cách nhau 2 ms thay
    // vì 20 ms) và không còn tương đương lúc train. Kích thước vector không đổi
    // nên không có kiểm tra dimension nào bắt được.
    constexpr int kDecimation = 10;
    constexpr int kControlTicks = 40;   // 4 policy step

    hist::BaseObservationHistory correct;
    hist::BaseObservationHistory wrong;
    int policy_step = 0;
    for (int tick = 0; tick < kControlTicks; ++tick) {
        wrong.Append(MakeFrame(tick));                 // SAI: mỗi tick
        if (tick % kDecimation == 0)
            correct.Append(MakeFrame(policy_step++));  // ĐÚNG: mỗi policy step
    }

    // Đúng: 4 frame cách nhau đúng một policy step.
    for (int lag = 0; lag < hist::kH4Steps; ++lag)
        assert(Same(correct.Frame(lag), MakeFrame(policy_step - 1 - lag)));

    // Sai: cửa sổ chỉ trải 4 tick = 8 ms thay vì 4 step = 80 ms.
    const float correct_span =
        correct.Frame(0)[0] - correct.Frame(hist::kH4Steps - 1)[0];
    const float wrong_span =
        wrong.Frame(0)[0] - wrong.Frame(hist::kH4Steps - 1)[0];
    assert(std::fabs(correct_span - 3000.0f) < 1e-3f);
    assert(std::fabs(wrong_span - 3000.0f) < 1e-3f);
    // Điểm mấu chốt: hai bank KHÁC nhau, nên gọi sai chỗ là phát hiện được.
    assert(!Same(correct.Frame(hist::kH4Steps - 1),
                       wrong.Frame(hist::kH4Steps - 1)));
}

}  // namespace

int main() {
    TestOffsets();
    TestBackfillNotZeroPad();
    TestRoll();
    TestTermMajor();
    TestH5TermMajor();
    TestNewestSliceRebuildsBaseFrame();
    TestResetOnReactivation();
    TestWrongFrameSizeRejected();
    TestAppendRateIsPolicyStepNotControlLoop();
    std::cout << "flat_plus_history_test: PASS\n";
    return 0;
}
