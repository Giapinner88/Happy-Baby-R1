#ifndef FLAT_PLUS_H5_CONTROLLER_HPP
#define FLAT_PLUS_H5_CONTROLLER_HPP

#include "FlatHistoryController.hpp"

class FlatPlusH5Controller final : public FlatHistoryController {
public:
    static constexpr int kObsSize = r1::history::kH5ActorDim;  // 415

    FlatPlusH5Controller()
        : FlatHistoryController(r1::history::kH5Steps,
                                 r1::history::kFlatPlusH5Contract) {}
    ~FlatPlusH5Controller() override = default;

    int GetInputSize() const override { return kObsSize; }
};

#endif
