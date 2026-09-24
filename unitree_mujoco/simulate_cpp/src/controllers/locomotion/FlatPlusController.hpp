#ifndef FLAT_PLUS_CONTROLLER_HPP
#define FLAT_PLUS_CONTROLLER_HPP

// ============================================================================
// Controller for the `flat_plus_h4_v1` contract.
//
//   actor input = 4 x canonical 83-D base frame, packed term-major,
//                 oldest -> newest inside every term block   (332 float32)
//   action      = 24 float32, same common-PD joint targets as legacy 83-D
//
// It reuses FlatController::BuildBaseObservation83() rather than restating the
// gravity / joint-order / last-action formulas, so the newest slice of the
// packed vector is by construction the same quantity the legacy controller
// feeds a 83-D policy.
//
// Gesture masks q/dq on the 83-D frame before it enters the history bank.
// Teleop remains on the legacy 83-D route.
// ============================================================================

#include "FlatHistoryController.hpp"

class FlatPlusController : public FlatHistoryController {
public:
    static constexpr int kObsSize = r1::history::kH4ActorDim;  // 332

    FlatPlusController()
        : FlatHistoryController(r1::history::kH4Steps,
                                 r1::history::kFlatPlusContract) {}
    ~FlatPlusController() override = default;

    int GetInputSize() const override { return kObsSize; }
    bool SupportsHistoryGestureOverlay() const override { return true; }
};

#endif
