#include "motion/HomeDanceExit.hpp"
#include <limits>
#include <stdexcept>
static void require(bool ok) { if (!ok) throw std::runtime_error("HOME exit regression"); }
int main() {
    require(HomeDanceExitReady(0, 0.048f, 0.21f, 0.04f, 0.1f, 0.1f));
    require(!HomeDanceExitReady(80, 0, 0, 0, 0, 0)); // no truncation of the roll
    require(!HomeDanceExitReady(0, 0.2f, 0, 0, 0, 0)); // reference not HOME
    require(!HomeDanceExitReady(0, 0, 0.4f, 0, 0, 0));
    require(!HomeDanceExitReady(0, 0, 0, 3.14f, 0, 0)); // inverted is not settled
    require(!HomeDanceExitReady(0, 0, 0, 0, 0.6f, 0));
    require(!HomeDanceExitReady(0, 0, 0, 0, 0, 0.7f));
    require(!HomeDanceExitReady(0, 0, 0, 0, std::numeric_limits<float>::quiet_NaN(), 0));
}
