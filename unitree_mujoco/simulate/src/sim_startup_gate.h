#pragma once

#include <atomic>

namespace sim_startup {

// Physics remains paused for R1 until the bridge has installed either its
// startup PD hold or a fresh controller command.
inline std::atomic<bool> bridge_control_ready{false};
inline std::atomic<bool> model_state_ready{false};

}  // namespace sim_startup
