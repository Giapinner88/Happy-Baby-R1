#ifndef BASE_OBSERVATION_HISTORY_HPP
#define BASE_OBSERVATION_HISTORY_HPP

// ============================================================================
// Storage for a short window of canonical 83-D base frames, plus the packers
// that turn that window into a policy input.
//
// Storage and view are deliberately separated:
//
//   * the bank keeps whole `r1_common_pd_base83_v1` frames and nothing else;
//   * `PackTermMajorH4()` is one *view* over those frames.
//
// A later A-RMA adaptation module needs a different, longer `[K,83]`
// time-major view of the same frames, so the bank must never be specialised
// into a flat 332-float buffer.
//
// Twin of HB/high_level_2/src/policy/BaseObservationHistory.hpp and of
// src/tasks/velocity/config/r1/history_contract.py in unitree_rl_mjlab_meta.
// ============================================================================

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace r1::history {

// --- canonical base frame -------------------------------------------------

inline constexpr int kBaseDim = 83;
inline constexpr const char* kBaseSchema = "r1_common_pd_base83_v1";

// --- flat_plus_h4_v1 actor view -------------------------------------------

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

// Declaration order of the actor terms in velocity_env_cfg.py after the Flat
// factory removes `height_scan`. The offsets below are only valid in this
// order, which is asserted against the live environment by
// tests/test_flat_plus_contract.py::test_actor_term_order_matches_contract.
inline constexpr std::array<TermSpec, 7> kTerms{{
    {"base_ang_vel", 3},
    {"projected_gravity", 3},
    {"command", 3},
    {"phase", 2},
    {"joint_pos", 24},
    {"joint_vel", 24},
    {"actions", 24},
}};

// Start of `term`'s whole history block inside the packed vector.
int TermBlockStart(std::size_t term_index, int steps = kH4Steps);

// Slice of one frame of `term`; lag 0 is the newest frame.
int TermFrameStart(std::size_t term_index, int lag, int steps = kH4Steps);

// Offset of `term` inside a single 83-D base frame.
int BaseTermStart(std::size_t term_index);

// ---------------------------------------------------------------------------

class BaseObservationHistory {
public:
    explicit BaseObservationHistory(int steps = kH4Steps);

    // Drop every frame. The next Append() is treated as a first append and
    // backfills all slots, matching mjlab's CircularBuffer on episode reset.
    // Must be called on every policy activation/reactivation, not only on a
    // simulated episode reset, or the window would span two runs.
    void Reset();

    // Append one canonical 83-D frame. Exactly once per policy step (50 Hz).
    // Appending at the 500 Hz control rate would fill the window with near
    // duplicates and stop matching training.
    void Append(const std::vector<float>& base_frame);

    // Frame at `lag` steps back; lag 0 is the newest.
    const std::vector<float>& Frame(int lag) const;

    // Term-major, oldest -> newest inside each term block. `out` is resized.
    void PackTermMajor(std::vector<float>& out) const;
    void PackTimeMajor(std::vector<float>& out) const;
    // Compatibility name for existing H4 callers.
    void PackTermMajorH4(std::vector<float>& out) const;

    bool ready() const { return valid_steps_ > 0; }
    int valid_steps() const { return valid_steps_; }
    int steps() const { return steps_; }

private:
    int steps_;
    int valid_steps_ = 0;
    int newest_ = 0;                            // index of the newest frame
    std::vector<std::vector<float>> frames_;    // ring of 83-D frames
};

}  // namespace r1::history

#endif
