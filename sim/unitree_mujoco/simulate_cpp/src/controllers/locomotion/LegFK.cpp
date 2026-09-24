#include "LegFK.hpp"

#include <algorithm>
#include <cmath>

namespace {

LegFK::Vec3 V(float x, float y, float z) { return {x, y, z}; }

}  // namespace

LegFK::Vec3 LegFK::Add(Vec3 a, Vec3 b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

LegFK::Vec3 LegFK::Scale(Vec3 a, float s) {
    return {a.x * s, a.y * s, a.z * s};
}

LegFK::Mat3 LegFK::Identity() {
    Mat3 result{};
    result.v[0][0] = result.v[1][1] = result.v[2][2] = 1.0f;
    return result;
}

LegFK::Mat3 LegFK::Multiply(const Mat3& a, const Mat3& b) {
    Mat3 result{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            for (int k = 0; k < 3; ++k)
                result.v[row][col] += a.v[row][k] * b.v[k][col];
        }
    }
    return result;
}

LegFK::Vec3 LegFK::Multiply(const Mat3& a, Vec3 b) {
    return {
        a.v[0][0] * b.x + a.v[0][1] * b.y + a.v[0][2] * b.z,
        a.v[1][0] * b.x + a.v[1][1] * b.y + a.v[1][2] * b.z,
        a.v[2][0] * b.x + a.v[2][1] * b.y + a.v[2][2] * b.z,
    };
}

LegFK::Mat3 LegFK::QuaternionToMatrix(const std::array<float, 4>& q) {
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    Mat3 result{};
    result.v[0][0] = 1.0f - 2.0f * (y * y + z * z);
    result.v[0][1] = 2.0f * (x * y - w * z);
    result.v[0][2] = 2.0f * (x * z + w * y);
    result.v[1][0] = 2.0f * (x * y + w * z);
    result.v[1][1] = 1.0f - 2.0f * (x * x + z * z);
    result.v[1][2] = 2.0f * (y * z - w * x);
    result.v[2][0] = 2.0f * (x * z - w * y);
    result.v[2][1] = 2.0f * (y * z + w * x);
    result.v[2][2] = 1.0f - 2.0f * (x * x + y * y);
    return result;
}

LegFK::Mat3 LegFK::AxisAngle(const Vec3& axis, float angle) {
    const float c = std::cos(angle), s = std::sin(angle), t = 1.0f - c;
    const float x = axis.x, y = axis.y, z = axis.z;
    return {{
        {t * x * x + c,     t * x * y - s * z, t * x * z + s * y},
        {t * x * y + s * z, t * y * y + c,     t * y * z - s * x},
        {t * x * z - s * y, t * y * z + s * x, t * z * z + c},
    }};
}

LegFK::Mat3 LegFK::LevelRotation(Vec3 gravity) {
    const float norm = std::sqrt(gravity.x * gravity.x + gravity.y * gravity.y
                                 + gravity.z * gravity.z);
    if (norm < 1.0e-6f) return Identity();
    gravity = Scale(gravity, 1.0f / norm);
    const Vec3 target{0.0f, 0.0f, -1.0f};
    const Vec3 cross{
        gravity.y * target.z - gravity.z * target.y,
        gravity.z * target.x - gravity.x * target.z,
        gravity.x * target.y - gravity.y * target.x,
    };
    const float c = gravity.x * target.x + gravity.y * target.y + gravity.z * target.z;
    Mat3 vx{{
        {0.0f, -cross.z, cross.y},
        {cross.z, 0.0f, -cross.x},
        {-cross.y, cross.x, 0.0f},
    }};
    const Mat3 vx2 = Multiply(vx, vx);
    Mat3 result = Identity();
    const float factor = 1.0f / std::max(1.0e-6f, 1.0f + c);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col)
            result.v[row][col] += vx.v[row][col] + vx2.v[row][col] * factor;
    }
    return result;
}

float LegFK::WrapPi(float angle) {
    while (angle > static_cast<float>(M_PI)) angle -= 2.0f * static_cast<float>(M_PI);
    while (angle < -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

void LegFK::FootPose(const Chain& chain, const float* q,
                    Vec3& position, Mat3& rotation) {
    position = {};
    rotation = Identity();
    for (std::size_t k = 0; k < kLegJoints; ++k) {
        position = Add(position, Multiply(rotation, chain.pos[k]));
        rotation = Multiply(Multiply(rotation, chain.rot[k]), AxisAngle(chain.axis[k], q[k]));
    }
    position = Add(position, Multiply(rotation, chain.site_pos));
    rotation = Multiply(rotation, chain.site_rot);
}

LegFK::Metrics LegFK::RawMetrics(const Chain& left, const Chain& right,
                                 const std::array<float, kInputJoints>& leg_q,
                                 const std::array<float, 3>& gravity) {
    Vec3 left_pos{}, right_pos{};
    Mat3 left_rot{}, right_rot{};
    FootPose(left, leg_q.data(), left_pos, left_rot);
    FootPose(right, leg_q.data() + kLegJoints, right_pos, right_rot);
    const Mat3 level = LevelRotation({gravity[0], gravity[1], gravity[2]});
    left_pos = Multiply(level, left_pos);
    right_pos = Multiply(level, right_pos);
    const Vec3 left_x = Multiply(level, Vec3{left_rot.v[0][0], left_rot.v[1][0], left_rot.v[2][0]});
    const Vec3 right_x = Multiply(level, Vec3{right_rot.v[0][0], right_rot.v[1][0], right_rot.v[2][0]});
    const float left_yaw = std::atan2(left_x.y, left_x.x);
    const float right_yaw = std::atan2(right_x.y, right_x.x);
    return {
        left_pos.y - right_pos.y,
        left_pos.x - right_pos.x,
        WrapPi(left_yaw - right_yaw),
        0.0f, 0.0f, 0.0f,
    };
}

LegFK::LegFK() {
    // Body positions, fixed body quaternions and joint axes copied from the
    // R1 MuJoCo chain.  Keeping them in the runtime makes the simulator use
    // the same deployable signals as HB without requiring contact sensors.
    chains_[0].pos = {{
        V(0.0325f, 0.0704672f, -0.0902351f),
        V(0.0248f, 0.045f, -0.053f),
        V(-0.0194507f, -0.0006f, -0.0618f),
        V(-0.02315f, 0.01866f, -0.159301f),
        V(-0.0168139f, -0.0205811f, -0.309175f),
        V(0.0f, 0.0f, 0.0f),
    }};
    chains_[1].pos = {{
        V(0.0325f, -0.0704672f, -0.0902351f),
        V(0.0248f, -0.045f, -0.053f),
        V(-0.0194507f, 0.0006f, -0.0618f),
        V(-0.02315f, -0.01866f, -0.159301f),
        V(-0.0168139f, 0.0205811f, -0.309175f),
        V(0.0f, 0.0f, 0.0f),
    }};
    const std::array<float, 4> left_hip_pitch_q{0.9762959252f, -0.2164399834f, 0.0f, 0.0f};
    const std::array<float, 4> right_hip_pitch_q{0.9762959252f, 0.2164399834f, 0.0f, 0.0f};
    const std::array<float, 4> left_hip_roll_q{0.9762959252f, 0.2164399834f, 0.0f, 0.0f};
    const std::array<float, 4> right_hip_roll_q{0.9762959252f, -0.2164399834f, 0.0f, 0.0f};
    for (auto& chain : chains_) {
        for (std::size_t k = 0; k < kLegJoints; ++k) {
            chain.rot[k] = Identity();
            chain.axis[k] = V(0.0f, 0.0f, 0.0f);
        }
        chain.site_pos = V(0.04f, 0.0f, -0.055f);
        chain.site_rot = Identity();
    }
    chains_[0].rot[0] = QuaternionToMatrix(left_hip_pitch_q);
    chains_[1].rot[0] = QuaternionToMatrix(right_hip_pitch_q);
    chains_[0].rot[1] = QuaternionToMatrix(left_hip_roll_q);
    chains_[1].rot[1] = QuaternionToMatrix(right_hip_roll_q);
    for (auto& chain : chains_) {
        chain.axis[0] = V(0.0f, 1.0f, 0.0f);
        chain.axis[1] = V(1.0f, 0.0f, 0.0f);
        chain.axis[2] = V(0.0f, 0.0f, 1.0f);
        chain.axis[3] = V(0.0f, 1.0f, 0.0f);
        chain.axis[4] = V(0.0f, 1.0f, 0.0f);
        chain.axis[5] = V(1.0f, 0.0f, 0.0f);
    }
    std::array<float, kInputJoints> home{};
    // The FK contract's HOME is deliberately separate from the common-PD
    // neutral pose: the trained gait gate uses the small hip-roll/yaw offsets
    // below to define an acceptable standing footprint.
    home = {{
        -0.1f, 0.0349f, -0.0477f, 0.3f, -0.2f, -0.0349f,
        -0.1f, -0.0349f, 0.0477f, 0.3f, -0.2f, 0.0349f,
    }};
    const Metrics reference = RawMetrics(chains_[0], chains_[1], home,
                                         {0.0f, 0.0f, -1.0f});
    home_dx_ = reference.dx;
    home_dyaw_ = reference.dyaw;
}

LegFK::Metrics LegFK::Compute(
    const std::array<float, kInputJoints>& leg_q,
    const std::array<float, 3>& projected_gravity) const {
    Metrics result = RawMetrics(chains_[0], chains_[1], leg_q, projected_gravity);
    result.width_err = result.width - 0.212f;
    result.dx_err = result.dx - home_dx_;
    result.dyaw_err = WrapPi(result.dyaw - home_dyaw_);
    return result;
}
