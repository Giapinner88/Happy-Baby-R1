#pragma once
#include "StateMotionGuard.hpp"

namespace motion_safety {
// Only the final continuous quiet upright suffix qualifies. A standing pause
// before another roll/flip never qualifies. Computed once when loading a clip;
// no future-state measurements or simulator contact data enter the guard.
struct UprightWindow {
    static constexpr float kReferenceTiltDeg = 10;
    static constexpr float kLegSpeed = .6f;
    static constexpr float kReferenceTiltRate = .5f; // rad/s, yaw invariant
    static constexpr float kMinHoldS = .3f;

    template<class Motion>
    static int StartFrame(const Motion& motion) {
        const int n = motion.num_frames();
        const float fps = motion.fps();
        if (n < 2 || !std::isfinite(fps) || fps <= 0) return n;
        int start = n;
        for (int i = n - 1; i >= 0; --i) {
            if (!QuietReference(motion.torso_quat(i), motion.joint_vel(i))) break;
            if (i + 1 < n) {
                const float rate = TiltErrorDeg(motion.torso_quat(i), motion.torso_quat(i+1))
                                   / kRadToDeg * fps;
                if (!std::isfinite(rate) || rate > kReferenceTiltRate) break;
            }
            start = i;
        }
        return (n - 1 - start) / fps >= kMinHoldS ? start : n;
    }

    template<class Velocities>
    static bool QuietReference(const Eigen::Quaternionf& torso, const Velocities& dq) {
        const float tilt = TiltErrorDeg(torso, Eigen::Quaternionf::Identity());
        if (!std::isfinite(tilt) || tilt > kReferenceTiltDeg) return false;
        // Legs + waist; arms may continue moving during a standing dance.
        for (int i = 0; i < 14; ++i)
            if (!std::isfinite(dq[i]) || std::abs(dq[i]) > kLegSpeed) return false;
        return true;
    }
};
}
