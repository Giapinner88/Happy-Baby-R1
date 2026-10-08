#pragma once

// ============================================================================
// Cửa sổ ngắn các frame 83-D + bộ đóng gói term-major cho `flat_plus_h4_v1`.
//
// Bank lưu FRAME 83-D NGUYÊN VẸN, không lưu vector 332 đã dẹt. Đóng gói H4 chỉ
// là một VIEW bên trên. A-RMA sau này cần view `[K,83]` time-major từ đúng
// nguồn frame đó, nên bank không được đặc hoá thành buffer 332 float.
//
// Twin: unitree_mujoco/simulate_cpp/src/controllers/locomotion/
//       BaseObservationHistory.{hpp,cpp}
// Nguồn train: src/tasks/velocity/config/r1/history_contract.py
// ============================================================================

#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "../config/RobotSpec.hpp"

namespace r1::history {

inline constexpr int kBaseDim = spec::kFlatObsSize;  // 83
inline constexpr const char* kBaseSchema = "r1_common_pd_base83_v1";

inline constexpr int kH4Steps = 4;
inline constexpr int kH4ActorDim = kBaseDim * kH4Steps;  // 332
inline constexpr const char* kFlatPlusContract = "flat_plus_h4_v1";
inline constexpr const char* kH4Order = "term_major_oldest_to_newest";
inline constexpr const char* kH4Padding = "repeat_first";

inline constexpr int kH5Steps = 5;
inline constexpr int kH5ActorDim = kBaseDim * kH5Steps;  // 415
inline constexpr const char* kFlatPlusH5Contract = "flat_plus_h5_v1";

struct TermSpec {
    const char* name;
    int dim;
};

inline constexpr std::array<TermSpec, 7> kTerms{{
    {"base_ang_vel", 3},
    {"projected_gravity", 3},
    {"command", 3},
    {"phase", 2},
    {"joint_pos", 24},
    {"joint_vel", 24},
    {"actions", 24},
}};

inline int BaseTermStart(std::size_t term_index) {
    if (term_index >= kTerms.size()) throw std::out_of_range("term index");
    int offset = 0;
    for (std::size_t i = 0; i < term_index; ++i) offset += kTerms[i].dim;
    return offset;
}

inline int TermBlockStart(std::size_t term_index, int steps = kH4Steps) {
    if (term_index >= kTerms.size()) throw std::out_of_range("term index");
    if (steps < 1) throw std::invalid_argument("history steps must be >= 1");
    int offset = 0;
    for (std::size_t i = 0; i < term_index; ++i) offset += kTerms[i].dim * steps;
    return offset;
}

// lag 0 = frame mới nhất. Trong mỗi block, frame chạy oldest -> newest.
inline int TermFrameStart(std::size_t term_index, int lag, int steps = kH4Steps) {
    if (term_index >= kTerms.size()) throw std::out_of_range("term index");
    if (lag < 0 || lag >= steps) throw std::out_of_range("history lag");
    const int frame_index = steps - 1 - lag;
    return TermBlockStart(term_index, steps) + frame_index * kTerms[term_index].dim;
}

class BaseObservationHistory {
public:
    explicit BaseObservationHistory(int steps = kH4Steps) : steps_(steps) {
        if (steps_ < 1) throw std::invalid_argument("history steps must be >= 1");
        frames_.assign(static_cast<std::size_t>(steps_),
                       std::vector<float>(kBaseDim, 0.0f));
    }

    // Xoá sạch cửa sổ. Append kế tiếp được coi là lần đầu -> backfill.
    // PHẢI gọi ở MỌI đường kích hoạt lại policy, không chỉ khi reset episode:
    // nếu không, cửa sổ sẽ vắt qua hai lần chạy khác nhau.
    void Reset() {
        valid_steps_ = 0;
        newest_ = 0;
        for (auto& frame : frames_) frame.assign(kBaseDim, 0.0f);
    }

    // Đúng MỘT lần mỗi policy step (50 Hz). Append ở vòng 500 Hz sẽ nhét đầy
    // cửa sổ bằng các frame gần trùng nhau và không còn tương đương lúc train.
    void Append(const std::vector<float>& base_frame) {
        if (static_cast<int>(base_frame.size()) != kBaseDim) {
            throw std::runtime_error(
                "base observation frame must be " + std::to_string(kBaseDim) +
                "-D, got " + std::to_string(base_frame.size()));
        }
        if (valid_steps_ == 0) {
            // Backfill frame hợp lệ đầu tiên vào cả cửa sổ, giống
            // CircularBuffer của mjlab lúc reset. Zero-pad ở đây sẽ làm khởi
            // động khác training.
            for (auto& frame : frames_) frame = base_frame;
            newest_ = steps_ - 1;
            valid_steps_ = steps_;
            return;
        }
        newest_ = (newest_ + 1) % steps_;
        frames_[static_cast<std::size_t>(newest_)] = base_frame;
        if (valid_steps_ < steps_) ++valid_steps_;
    }

    const std::vector<float>& Frame(int lag) const {
        if (lag < 0 || lag >= steps_) throw std::out_of_range("history lag");
        if (valid_steps_ == 0) throw std::runtime_error("history is empty");
        const int index = ((newest_ - lag) % steps_ + steps_) % steps_;
        return frames_[static_cast<std::size_t>(index)];
    }

    // Shared term-major view for H4/H5. Storage remains a ring of base frames.
    void PackTermMajor(std::vector<float>& out) const {
        if (valid_steps_ == 0) throw std::runtime_error("history is empty");
        out.assign(static_cast<std::size_t>(kBaseDim * steps_), 0.0f);
        for (std::size_t term = 0; term < kTerms.size(); ++term) {
            const int base_start = BaseTermStart(term);
            const int width = kTerms[term].dim;
            for (int lag = 0; lag < steps_; ++lag) {
                const std::vector<float>& frame = Frame(lag);
                const int dst = TermFrameStart(term, lag, steps_);
                for (int k = 0; k < width; ++k) {
                    out[static_cast<std::size_t>(dst + k)] =
                        frame[static_cast<std::size_t>(base_start + k)];
                }
            }
        }
    }

    // Compatibility name for the existing H4 contract.
    void PackTermMajorH4(std::vector<float>& out) const {
        if (steps_ != kH4Steps)
            throw std::runtime_error("PackTermMajorH4 requires a 4-step history");
        PackTermMajor(out);
    }

    bool ready() const { return valid_steps_ > 0; }
    int valid_steps() const { return valid_steps_; }
    int steps() const { return steps_; }

private:
    int steps_;
    int valid_steps_ = 0;
    int newest_ = 0;
    std::vector<std::vector<float>> frames_;
};

}  // namespace r1::history
