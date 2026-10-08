#pragma once

#include "FlatHistoryController.hpp"

// `flat_plus_h5_v1`: 5 canonical 83-D frames, term-major, 415-D input.
class FlatPlusH5Controller final : public FlatHistoryController {
public:
    static constexpr int kObsSize = r1::history::kH5ActorDim;  // 415

    FlatPlusH5Controller()
        : FlatHistoryController(r1::history::kH5Steps,
                                r1::history::kFlatPlusH5Contract) {}
    ~FlatPlusH5Controller() override = default;

    std::string Name() const override { return "FlatPlusH5"; }
};
