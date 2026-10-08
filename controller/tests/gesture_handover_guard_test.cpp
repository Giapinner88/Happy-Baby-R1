#include <cstdlib>

#include "../src/motion/GestureHandoverGuard.hpp"

int main() {
    GestureHandoverGuard guard(0.5f);
    if (guard.Advance(true, 0.05f, 0.02f, 0.03f, 0.15f, 0.08f, 0.25f, 0.25f))
        std::abort();
    // Mẫu xấu phải reset dwell, không được cộng dồn hai khoảng rời nhau.
    if (guard.Advance(false, 0.05f, 0.02f, 0.03f, 0.15f, 0.08f, 0.25f, 0.25f))
        std::abort();
    if (guard.Advance(true, 0.05f, 0.02f, 0.03f, 0.15f, 0.08f, 0.25f, 0.25f))
        std::abort();
    if (!guard.Advance(true, 0.05f, 0.02f, 0.03f, 0.15f, 0.08f, 0.25f, 0.25f))
        std::abort();
    return 0;
}
