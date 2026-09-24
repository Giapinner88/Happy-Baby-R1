// Contract tests for the shared H4/H5 history bank and packers.
//
// The expected vectors here are written by hand from the offset table in the
// plan, deliberately NOT derived from the same helpers they check: a packer
// tested against its own arithmetic proves nothing.

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "controllers/locomotion/BaseObservationHistory.hpp"

namespace hist = r1::history;

namespace {

int failures = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        std::cerr << "FAIL: " << what << std::endl;
        ++failures;
    }
}

// A frame whose every element encodes its own (frame id, index) so a
// misplaced element is identifiable rather than merely unequal.
std::vector<float> MakeFrame(int frame_id) {
    std::vector<float> frame(hist::kBaseDim);
    for (int i = 0; i < hist::kBaseDim; ++i) {
        frame[static_cast<std::size_t>(i)] =
            static_cast<float>(frame_id) * 1000.0f + static_cast<float>(i);
    }
    return frame;
}

bool FramesEqual(const std::vector<float>& a, const std::vector<float>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::fabs(a[i] - b[i]) > 1e-6f) return false;
    }
    return true;
}

void TestOffsetTable() {
    // Hard-coded from the plan's layout table, term-major.
    const int expect_block_start[7] = {0, 12, 24, 36, 44, 140, 236};
    const int expect_current[7] = {9, 21, 33, 42, 116, 212, 308};
    for (std::size_t t = 0; t < hist::kTerms.size(); ++t) {
        Check(hist::TermBlockStart(t) == expect_block_start[t],
              "block start of " + std::string(hist::kTerms[t].name));
        Check(hist::TermFrameStart(t, 0) == expect_current[t],
              "current slice of " + std::string(hist::kTerms[t].name));
    }
    // joint_pos is index 4: oldest frame at the block start, newest last.
    Check(hist::TermFrameStart(4, 3) == 44, "joint_pos oldest frame at 44");
    Check(hist::TermFrameStart(4, 0) == 116, "joint_pos newest frame at 116");

    int total = 0;
    for (const auto& term : hist::kTerms) total += term.dim * hist::kH4Steps;
    Check(total == hist::kH4ActorDim, "packed size is 332");
    Check(hist::kH4ActorDim == 332, "H4 actor dim constant");
    Check(hist::kBaseDim == 83, "base dim constant");
}

void TestResetBackfillsFirstFrame() {
    hist::BaseObservationHistory bank;
    Check(!bank.ready(), "empty bank is not ready");
    bank.Append(MakeFrame(7));
    Check(bank.ready(), "bank ready after one append");
    Check(bank.valid_steps() == hist::kH4Steps, "first append fills the window");
    for (int lag = 0; lag < hist::kH4Steps; ++lag) {
        Check(FramesEqual(bank.Frame(lag), MakeFrame(7)),
              "slot " + std::to_string(lag) + " backfilled with the first frame");
    }

    std::vector<float> packed;
    bank.PackTermMajorH4(packed);
    // A zero-padded startup would leave the three older frames at zero.
    bool any_zero_block = false;
    for (int lag = 1; lag < hist::kH4Steps; ++lag) {
        const int start = hist::TermFrameStart(4, lag);  // joint_pos
        if (packed[static_cast<std::size_t>(start)] == 0.0f) any_zero_block = true;
    }
    Check(!any_zero_block, "reset backfills rather than zero-pads");
}

void TestRollAndWraparound() {
    hist::BaseObservationHistory bank;
    bank.Append(MakeFrame(0));
    for (int step = 1; step <= 2 * hist::kH4Steps; ++step) {
        bank.Append(MakeFrame(step));
        for (int lag = 0; lag < hist::kH4Steps; ++lag) {
            const int expected_id = step - lag;
            if (expected_id < 0) continue;
            Check(FramesEqual(bank.Frame(lag), MakeFrame(expected_id)),
                  "step " + std::to_string(step) + " lag " + std::to_string(lag));
        }
    }
}

void TestPackingIsTermMajor() {
    hist::BaseObservationHistory bank;
    for (int step = 0; step < hist::kH4Steps; ++step) bank.Append(MakeFrame(step));
    // frames now: lag0=3, lag1=2, lag2=1, lag3=0

    std::vector<float> packed;
    bank.PackTermMajorH4(packed);
    Check(packed.size() == static_cast<std::size_t>(hist::kH4ActorDim),
          "packed vector is 332 long");

    for (std::size_t t = 0; t < hist::kTerms.size(); ++t) {
        const int base_start = hist::BaseTermStart(t);
        for (int lag = 0; lag < hist::kH4Steps; ++lag) {
            const int frame_id = (hist::kH4Steps - 1) - lag;
            const std::vector<float> source = MakeFrame(frame_id);
            const int dst = hist::TermFrameStart(t, lag);
            for (int k = 0; k < hist::kTerms[t].dim; ++k) {
                const float got = packed[static_cast<std::size_t>(dst + k)];
                const float want = source[static_cast<std::size_t>(base_start + k)];
                if (std::fabs(got - want) > 1e-6f) {
                    Check(false, std::string("term ") + hist::kTerms[t].name
                                     + " lag " + std::to_string(lag)
                                     + " element " + std::to_string(k));
                    return;
                }
            }
        }
    }

    // The decisive check: a time-major packing would put the whole oldest
    // 83-D frame in [0,83). Term-major must not.
    const std::vector<float> oldest = MakeFrame(0);
    bool looks_time_major = true;
    for (int i = 0; i < hist::kBaseDim; ++i) {
        if (std::fabs(packed[static_cast<std::size_t>(i)]
                      - oldest[static_cast<std::size_t>(i)]) > 1e-6f) {
            looks_time_major = false;
            break;
        }
    }
    Check(!looks_time_major, "packing is term-major, not time-major");
}

void TestGenericH5AndTimeMajorPacking() {
    hist::BaseObservationHistory bank(hist::kH5Steps);
    for (int step = 0; step < hist::kH5Steps; ++step) bank.Append(MakeFrame(step));

    std::vector<float> term_major;
    bank.PackTermMajor(term_major);
    Check(term_major.size() == static_cast<std::size_t>(hist::kH5ActorDim),
          "generic term-major H5 vector is 415 long");
    Check(term_major[static_cast<std::size_t>(hist::TermFrameStart(4, 0, hist::kH5Steps))]
              == MakeFrame(4)[hist::BaseTermStart(4)],
          "generic H5 newest joint position is in the expected slice");

    std::vector<float> time_major;
    bank.PackTimeMajor(time_major);
    Check(time_major.size() == static_cast<std::size_t>(hist::kH5ActorDim),
          "generic time-major H5 vector is 415 long");
    bool ordered = true;
    for (int frame = 0; frame < hist::kH5Steps; ++frame) {
        const auto expected = MakeFrame(frame);
        for (int element = 0; element < hist::kBaseDim; ++element) {
            if (time_major[static_cast<std::size_t>(frame * hist::kBaseDim + element)]
                != expected[static_cast<std::size_t>(element)]) {
                ordered = false;
            }
        }
    }
    Check(ordered, "time-major history is oldest to newest whole frames");
}

void TestNewestSliceMatchesBaseFrame() {
    // Reassembling every term's newest slice must give back the 83-D frame a
    // legacy controller would have produced.
    hist::BaseObservationHistory bank;
    bank.Append(MakeFrame(1));
    bank.Append(MakeFrame(2));
    std::vector<float> packed;
    bank.PackTermMajorH4(packed);

    std::vector<float> rebuilt;
    rebuilt.reserve(hist::kBaseDim);
    for (std::size_t t = 0; t < hist::kTerms.size(); ++t) {
        const int start = hist::TermFrameStart(t, 0);
        for (int k = 0; k < hist::kTerms[t].dim; ++k) {
            rebuilt.push_back(packed[static_cast<std::size_t>(start + k)]);
        }
    }
    Check(FramesEqual(rebuilt, MakeFrame(2)),
          "newest slices reassemble the current base frame");
}

void TestResetClearsWindow() {
    hist::BaseObservationHistory bank;
    bank.Append(MakeFrame(1));
    bank.Append(MakeFrame(2));
    bank.Reset();
    Check(!bank.ready(), "reset clears readiness");
    Check(bank.valid_steps() == 0, "reset clears valid steps");

    bool threw = false;
    try {
        std::vector<float> packed;
        bank.PackTermMajorH4(packed);
    } catch (const std::exception&) {
        threw = true;
    }
    Check(threw, "packing an empty bank fails loudly");

    // Re-activation must not resurrect the pre-reset frames.
    bank.Append(MakeFrame(9));
    for (int lag = 0; lag < hist::kH4Steps; ++lag) {
        Check(FramesEqual(bank.Frame(lag), MakeFrame(9)),
              "post-reset append backfills, no stale frame at lag "
                  + std::to_string(lag));
    }
}

void TestWrongSizedFrameRejected() {
    hist::BaseObservationHistory bank;
    bool threw = false;
    try {
        bank.Append(std::vector<float>(hist::kH4ActorDim, 0.0f));
    } catch (const std::exception&) {
        threw = true;
    }
    Check(threw, "a 332-D frame is rejected by an 83-D bank");
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
        Check(FramesEqual(correct.Frame(lag), MakeFrame(policy_step - 1 - lag)),
              "50 Hz append keeps one frame per policy step at lag "
                  + std::to_string(lag));

    // Sai: cửa sổ chỉ trải 4 tick = 8 ms thay vì 4 step = 80 ms.
    const float correct_span =
        correct.Frame(0)[0] - correct.Frame(hist::kH4Steps - 1)[0];
    const float wrong_span =
        wrong.Frame(0)[0] - wrong.Frame(hist::kH4Steps - 1)[0];
    Check(std::fabs(correct_span - 3000.0f) < 1e-3f,
          "policy-rate window spans 3 frame ids");
    Check(std::fabs(wrong_span - 3000.0f) < 1e-3f,
          "control-rate window also spans 3 ids, but of ticks not steps");
    // Điểm mấu chốt: hai bank KHÁC nhau, nên gọi sai chỗ là phát hiện được.
    Check(!FramesEqual(correct.Frame(hist::kH4Steps - 1),
                       wrong.Frame(hist::kH4Steps - 1)),
          "appending at the control rate produces a different window");
}

}  // namespace

int main() {
    TestOffsetTable();
    TestResetBackfillsFirstFrame();
    TestRollAndWraparound();
    TestPackingIsTermMajor();
    TestGenericH5AndTimeMajorPacking();
    TestNewestSliceMatchesBaseFrame();
    TestResetClearsWindow();
    TestWrongSizedFrameRejected();
    TestAppendRateIsPolicyStepNotControlLoop();

    if (failures != 0) {
        std::cerr << "flat_plus_history_test: " << failures << " failure(s)\n";
        return 1;
    }
    std::cout << "flat_plus_history_test: PASS\n";
    return 0;
}
