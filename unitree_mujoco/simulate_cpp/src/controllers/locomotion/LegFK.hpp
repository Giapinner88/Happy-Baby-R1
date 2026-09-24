#pragma once

#include <array>
#include <cstddef>

// Small, dependency-free R1 leg FK used by the deployable gait stance gate.
// The constants mirror the six-joint leg chains in unitree_r1/xmls/r1.xml.
// Inputs and outputs use policy order: left leg first, then right leg.
class LegFK {
public:
    struct Metrics {
        float width = 0.0f;
        float dx = 0.0f;
        float dyaw = 0.0f;
        float width_err = 0.0f;
        float dx_err = 0.0f;
        float dyaw_err = 0.0f;
    };

    static constexpr std::size_t kLegJoints = 6;
    static constexpr std::size_t kInputJoints = 12;

    LegFK();

    Metrics Compute(const std::array<float, kInputJoints>& leg_q,
                    const std::array<float, 3>& projected_gravity) const;

    float home_dx() const { return home_dx_; }
    float home_dyaw() const { return home_dyaw_; }

    // Internal matrix types are public only so the dependency-free
    // implementation helpers can stay outside the class body.
    struct Vec3 {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };
    struct Mat3 {
        float v[3][3]{};
    };
    struct Chain {
        std::array<Vec3, kLegJoints> pos{};
        std::array<Mat3, kLegJoints> rot{};
        std::array<Vec3, kLegJoints> axis{};
        Vec3 site_pos{};
        Mat3 site_rot{};
    };

private:
    static Vec3 Add(Vec3 a, Vec3 b);
    static Vec3 Scale(Vec3 a, float s);
    static Mat3 Identity();
    static Mat3 Multiply(const Mat3& a, const Mat3& b);
    static Vec3 Multiply(const Mat3& a, Vec3 b);
    static Mat3 QuaternionToMatrix(const std::array<float, 4>& q);
    static Mat3 AxisAngle(const Vec3& axis, float angle);
    static Mat3 LevelRotation(Vec3 gravity);
    static float WrapPi(float angle);
    static Metrics RawMetrics(const Chain& left, const Chain& right,
                              const std::array<float, kInputJoints>& leg_q,
                              const std::array<float, 3>& gravity);
    static void FootPose(const Chain& chain,
                         const float* q,
                         Vec3& position,
                         Mat3& rotation);

    std::array<Chain, 2> chains_{};
    float home_dx_ = 0.0f;
    float home_dyaw_ = 0.0f;
};
