#include "JointTrajectory.hpp"
#include "MotionArrayValidation.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include <cnpy.h>

void JointTrajectory::Load(const std::string& npz_path) {
    *this = JointTrajectory{};
    JointTrajectory candidate;
    candidate.LoadValidated(npz_path);
    *this = std::move(candidate);
}

void JointTrajectory::LoadValidated(const std::string& npz_path) {
    cnpy::npz_t npz = cnpy::npz_load(npz_path);
    has_torso_ = false;
    has_head_ = false;
    ref_grav_.clear();
    head_pos_.clear();

    auto it_jp = npz.find("joint_pos");
    if (it_jp == npz.end())
        throw std::runtime_error("NPZ missing key 'joint_pos': " + npz_path);
    cnpy::NpyArray& jp = it_jp->second;
    motion_array::Validate(jp, "joint_pos");

    if (jp.shape.size() != 2 || jp.shape[1] != spec::kNumJoints)
        throw std::runtime_error("joint_pos shape must be (frames, " +
                                  std::to_string(spec::kNumJoints) + "): " + npz_path);

    num_frames_ = motion_array::Frames(jp.shape[0]);
    if (num_frames_ < 2)
        throw std::runtime_error("joint_pos needs >= 2 frames: " + npz_path);

    fps_ = motion_array::Fps(npz);
    if (!std::isfinite(duration_s())) throw std::runtime_error("Motion duration overflow");
    joint_pos_.resize(static_cast<size_t>(num_frames_));
    for (int f = 0; f < num_frames_; ++f)
        for (int j = 0; j < spec::kNumJoints; ++j)
            joint_pos_[static_cast<size_t>(f)][static_cast<size_t>(j)] =
                motion_array::Scalar(jp, static_cast<size_t>(f) * spec::kNumJoints + static_cast<size_t>(j));

    // Đầu không nằm trong 24 khớp policy. Gesture nào có head_pos sẽ phát nó
    // qua HeadTarget riêng; file cũ không có key này vẫn tương thích hoàn toàn.
    auto it_hp = npz.find("head_pos");
    if (it_hp != npz.end()) {
        cnpy::NpyArray& hp = it_hp->second;
        motion_array::Validate(hp, "head_pos");
        if (hp.shape.size() != 2 || hp.shape[0] != static_cast<size_t>(num_frames_) ||
            hp.shape[1] != 2) {
            throw std::runtime_error("head_pos shape must be (frames, 2): " + npz_path);
        }
        head_pos_.resize(static_cast<size_t>(num_frames_));
        for (int f = 0; f < num_frames_; ++f) {
            head_pos_[static_cast<size_t>(f)][0] = motion_array::Scalar(hp, static_cast<size_t>(f) * 2);
            head_pos_[static_cast<size_t>(f)][1] = motion_array::Scalar(hp, static_cast<size_t>(f) * 2 + 1);
        }
        has_head_ = true;
    }

    // Hướng thân (tuỳ chọn): file cũ không có -> bỏ qua bù cổ chân. Tính projected gravity
    // = trọng lực world (0,0,-1) chiếu vào hệ thân, để so với gravity đo được lúc phát lại.
    auto it_bq = npz.find("torso_quat");
    if (it_bq != npz.end()) {
        const auto& bq = it_bq->second;
        motion_array::Validate(bq, "torso_quat");
        if (bq.shape.size() != 2 || bq.shape[0] != jp.shape[0] || bq.shape[1] != 4)
            throw std::runtime_error("torso_quat must have shape (frames, 4)");
        ref_grav_.resize(static_cast<size_t>(num_frames_));
        const Eigen::Vector3f g_world(0.0f, 0.0f, -1.0f);
        for (int f = 0; f < num_frames_; ++f) {
            const size_t i = static_cast<size_t>(f) * 4;
            Eigen::Quaternionf quat(motion_array::Scalar(bq, i), motion_array::Scalar(bq, i+1),
                                    motion_array::Scalar(bq, i+2), motion_array::Scalar(bq, i+3));
            if (!std::isfinite(quat.norm()) || quat.norm() < 1e-6f)
                throw std::runtime_error("torso_quat has zero/invalid norm");
            quat.normalize();
            ref_grav_[static_cast<size_t>(f)] = quat.conjugate() * g_world;   // gravity trong hệ thân
        }
        has_torso_ = true;
    }

    std::cout << "[JointTrajectory] Loaded " << num_frames_ << " frames @ " << fps_
              << "fps (" << duration_s() << "s)"
              << (has_torso_ ? " [co torso_quat: bu co chan BAT]" : " [KHONG torso_quat: bu co chan TAT]")
              << " from " << npz_path << "\n";
}

Eigen::Vector3f JointTrajectory::RefGravityAt(float t) const {
    if (!has_torso_ || ref_grav_.empty()) return Eigen::Vector3f(0.0f, 0.0f, -1.0f);
    if (num_frames_ == 1) return ref_grav_[0];
    float f = std::clamp(t, 0.0f, duration_s()) * fps_;
    int f0 = std::clamp(static_cast<int>(f), 0, num_frames_ - 1);
    int f1 = std::min(f0 + 1, num_frames_ - 1);
    float a = f - static_cast<float>(f0);
    Eigen::Vector3f v = (1.0f - a) * ref_grav_[static_cast<size_t>(f0)] +
                        a * ref_grav_[static_cast<size_t>(f1)];
    return v;
}

std::array<float, spec::kNumJoints> JointTrajectory::PoseAt(float t) const {
    std::array<float, spec::kNumJoints> out{};
    if (num_frames_ <= 0) return out;
    if (num_frames_ == 1) return joint_pos_[0];

    float f = std::clamp(t, 0.0f, duration_s()) * fps_;
    int f0 = std::clamp(static_cast<int>(f), 0, num_frames_ - 1);
    int f1 = std::min(f0 + 1, num_frames_ - 1);
    float alpha = f - static_cast<float>(f0);

    const auto& p0 = joint_pos_[static_cast<size_t>(f0)];
    const auto& p1 = joint_pos_[static_cast<size_t>(f1)];
    for (int j = 0; j < spec::kNumJoints; ++j)
        out[static_cast<size_t>(j)] = p0[static_cast<size_t>(j)] +
                                       alpha * (p1[static_cast<size_t>(j)] - p0[static_cast<size_t>(j)]);
    return out;
}
