#include "BaseObservationHistory.hpp"

#include <stdexcept>

namespace r1::history {

int BaseTermStart(std::size_t term_index) {
    if (term_index >= kTerms.size()) throw std::out_of_range("term index");
    int offset = 0;
    for (std::size_t i = 0; i < term_index; ++i) offset += kTerms[i].dim;
    return offset;
}

int TermBlockStart(std::size_t term_index, int steps) {
    if (term_index >= kTerms.size()) throw std::out_of_range("term index");
    int offset = 0;
    for (std::size_t i = 0; i < term_index; ++i) offset += kTerms[i].dim * steps;
    return offset;
}

int TermFrameStart(std::size_t term_index, int lag, int steps) {
    if (lag < 0 || lag >= steps) throw std::out_of_range("history lag");
    // Inside a term block frames run oldest -> newest, so the newest frame
    // (lag 0) sits last.
    const int frame_index = steps - 1 - lag;
    return TermBlockStart(term_index, steps)
         + frame_index * kTerms[term_index].dim;
}

BaseObservationHistory::BaseObservationHistory(int steps) : steps_(steps) {
    if (steps_ < 1) throw std::invalid_argument("history steps must be >= 1");
    frames_.assign(static_cast<std::size_t>(steps_),
                   std::vector<float>(kBaseDim, 0.0f));
}

void BaseObservationHistory::Reset() {
    valid_steps_ = 0;
    newest_ = 0;
    for (auto& frame : frames_) frame.assign(kBaseDim, 0.0f);
}

void BaseObservationHistory::Append(const std::vector<float>& base_frame) {
    if (static_cast<int>(base_frame.size()) != kBaseDim) {
        throw std::runtime_error(
            "base observation frame must be " + std::to_string(kBaseDim)
            + "-D, got " + std::to_string(base_frame.size()));
    }

    if (valid_steps_ == 0) {
        // First append after a reset backfills every slot with this frame.
        // Zero-padding here would make startup differ from training, where
        // mjlab's CircularBuffer copies the first value into all slots.
        for (auto& frame : frames_) frame = base_frame;
        newest_ = steps_ - 1;
        valid_steps_ = steps_;
        return;
    }

    newest_ = (newest_ + 1) % steps_;
    frames_[static_cast<std::size_t>(newest_)] = base_frame;
    if (valid_steps_ < steps_) ++valid_steps_;
}

const std::vector<float>& BaseObservationHistory::Frame(int lag) const {
    if (lag < 0 || lag >= steps_) throw std::out_of_range("history lag");
    if (valid_steps_ == 0) throw std::runtime_error("history is empty");
    const int index = ((newest_ - lag) % steps_ + steps_) % steps_;
    return frames_[static_cast<std::size_t>(index)];
}

void BaseObservationHistory::PackTermMajor(std::vector<float>& out) const {
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

void BaseObservationHistory::PackTimeMajor(std::vector<float>& out) const {
    if (valid_steps_ == 0) throw std::runtime_error("history is empty");
    out.clear();
    out.reserve(static_cast<std::size_t>(kBaseDim * steps_));
    for (int lag = steps_ - 1; lag >= 0; --lag) {
        const auto& frame = Frame(lag);
        out.insert(out.end(), frame.begin(), frame.end());
    }
}

void BaseObservationHistory::PackTermMajorH4(std::vector<float>& out) const {
    if (steps_ != kH4Steps) {
        throw std::runtime_error("PackTermMajorH4 requires a 4-step history");
    }
    PackTermMajor(out);
}

}  // namespace r1::history
