#pragma once

// ============================================================================
// Controller cho contract `flat_plus_h4_v1`.
//
//   obs    = 4 x frame 83-D chuẩn, đóng gói TERM-MAJOR, trong mỗi block chạy
//            oldest -> newest                                  (332 float32)
//   action = 24 float32, cùng common-PD target như legacy_83
//
// Dùng lại r1::obs83::Build() nên slice mới nhất của vector 332 đúng bằng
// vector 83-D mà LocomotionController sinh ra — không viết lại công thức.
//
// Gesture được hỗ trợ qua đường riêng: q/dq tay được mask trên frame 83-D
// trước khi frame được pack thành history. Teleop UDP vẫn chỉ chạy ở legacy 83-D.
// ============================================================================

#include <string>
#include <vector>

#include "FlatHistoryController.hpp"

class FlatPlusController : public FlatHistoryController {
public:
    static constexpr int kObsSize = r1::history::kH4ActorDim;  // 332

    FlatPlusController()
        : FlatHistoryController(r1::history::kH4Steps,
                                r1::history::kFlatPlusContract) {}
    ~FlatPlusController() override = default;

    std::string Name() const override { return "FlatPlusH4"; }
    bool SupportsHistoryGestureOverlay() const override { return true; }
};
