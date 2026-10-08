#include "MotionData.hpp"
#include "MotionArrayValidation.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include <cnpy.h>

void MotionData::Load(const std::string& npz_path) {
    *this = MotionData{};
    MotionData candidate;
    candidate.LoadValidated(npz_path);
    *this = std::move(candidate);
}

void MotionData::LoadValidated(const std::string& npz_path) {
    cnpy::npz_t npz = cnpy::npz_load(npz_path);

    auto require = [&](const char* key) -> cnpy::NpyArray& {
        auto it = npz.find(key);
        if (it == npz.end())
            throw std::runtime_error("NPZ missing key '" + std::string(key) + "': " + npz_path);
        return it->second;
    };

    cnpy::NpyArray& jp = require("joint_pos");
    cnpy::NpyArray& jv = require("joint_vel");
    cnpy::NpyArray& bq = require("body_quat_w");
    motion_array::Validate(jp, "joint_pos");
    motion_array::Validate(jv, "joint_vel");
    motion_array::Validate(bq, "body_quat_w");

    if (jp.shape.size() != 2 || jp.shape[1] != spec::kNumJoints)
        throw std::runtime_error("joint_pos shape must be (frames, 24)");
    if (jv.shape.size() != 2 || jv.shape[1] != spec::kNumJoints)
        throw std::runtime_error("joint_vel shape must be (frames, 24)");

    if (bq.shape.size() != 3 || bq.shape[1] <= spec::kMotionTorsoIdx || bq.shape[2] != 4)
        throw std::runtime_error("body_quat_w shape must be (frames, N>" +
                                  std::to_string(spec::kMotionTorsoIdx) + ", 4)");
    if ((jp.word_size != 4 && jp.word_size != 8) ||
        (jv.word_size != 4 && jv.word_size != 8) ||
        (bq.word_size != 4 && bq.word_size != 8)) {
        throw std::runtime_error("Motion NPZ only supports float32/float64 for joint_pos, joint_vel, body_quat_w");
    }

    num_frames_ = motion_array::Frames(jp.shape[0]);
    if (jv.shape[0] != jp.shape[0] || bq.shape[0] != jp.shape[0])
        throw std::runtime_error("Frame count mismatch across arrays in NPZ");
    // MimicController nội suy giữa frame f và f+1, và coi clip <2 frame là "đã xong"
    // ngay từ Reset() -> BuildObservation thoát sớm, obs giữ nguyên 0 và policy suy
    // luận trên vector rỗng. Chặn tại đây để dance hỏng bị disable chứ không chạy.
    if (num_frames_ < 2)
        throw std::runtime_error("Motion NPZ needs at least 2 frames, got " +
                                 std::to_string(num_frames_) + ": " + npz_path);

    fps_ = motion_array::Fps(npz);
    if (!std::isfinite(static_cast<float>(num_frames_ - 1) / fps_))
        throw std::runtime_error("Motion duration overflow");

    auto scalar = [](const cnpy::NpyArray& arr, size_t idx) -> float {
        if (arr.word_size == 4) return arr.data<float>()[idx];
        if (arr.word_size == 8) return static_cast<float>(arr.data<double>()[idx]);
        throw std::runtime_error("Motion NPZ has unsupported dtype");
    };

    joint_pos_.resize(static_cast<size_t>(num_frames_));
    joint_vel_.resize(static_cast<size_t>(num_frames_));
    torso_quat_w_.resize(static_cast<size_t>(num_frames_));

    const size_t stride_bq = bq.shape[1] * 4;
    for (int f = 0; f < num_frames_; ++f) {
        for (int j = 0; j < spec::kNumJoints; ++j) {
            const size_t idx = static_cast<size_t>(f) * spec::kNumJoints + j;
            joint_pos_[f][j] = scalar(jp, idx);
            joint_vel_[f][j] = scalar(jv, idx);
        }
        const size_t q_idx = static_cast<size_t>(f * stride_bq + spec::kMotionTorsoIdx * 4);
        Eigen::Quaternionf quat(
            scalar(bq, q_idx + 0),
            scalar(bq, q_idx + 1),
            scalar(bq, q_idx + 2),
            scalar(bq, q_idx + 3)
        );
        if (!std::isfinite(quat.norm()) || quat.norm() < 1e-6f)
            throw std::runtime_error("body_quat_w torso has zero/invalid norm");
        torso_quat_w_[f] = quat.normalized();
    }

    std::cout << "[MotionData] Loaded " << num_frames_ << " frames @ " << fps_
              << "fps from " << npz_path << "\n";
}

int MotionData::FindSmoothStartFrame(int search_frames) const {
    const int window = std::clamp(search_frames, 1, num_frames_);

    int best = 0;
    float best_score = std::numeric_limits<float>::max();
    for (int f = 0; f < window; ++f) {
        float vel = 0.0f;
        for (int j = 0; j < spec::kNumJoints; ++j)
            vel = std::max(vel, std::fabs(joint_vel_[static_cast<size_t>(f)][static_cast<size_t>(j)]));

        float e = 0.0f;
        for (int j = 0; j < 12; ++j) { // 2 legs only
            float d = joint_pos_[static_cast<size_t>(f)][static_cast<size_t>(j)] -
                      spec::kDefaultJointPos[static_cast<size_t>(j)];
            e += d * d;
        }
        float leg_err = std::sqrt(e);

        const auto& q = torso_quat_w_[static_cast<size_t>(f)];
        float gz = std::clamp(1.0f - 2.0f * (q.x() * q.x() + q.y() * q.y()), -1.0f, 1.0f);
        float tilt = std::acos(gz);

        float score = vel + 0.1f * leg_err + 0.05f * tilt;
        if (score < best_score) {
            best_score = score;
            best = f;
        }
    }
    return best;
}

int MotionData::FindTransitionStartFrame(
    int search_frames,
    const std::array<float, spec::kNumJoints>& current_q,
    const std::array<float, spec::kNumJoints>& current_dq) const {
    // Keep f+1 available because the controller starts with a linear clip
    // interpolation.  A one-frame search still remains valid for a two-frame
    // clip (candidate f=0).
    const int max_candidate = std::max(0, num_frames_ - 2);
    const int window = std::clamp(search_frames, 1, max_candidate + 1);

    int best = 0;
    float best_score = std::numeric_limits<float>::max();
    for (int f = 0; f < window; ++f) {
        const auto& q = joint_pos_[static_cast<size_t>(f)];
        const auto& dq = joint_vel_[static_cast<size_t>(f)];

        float q_leg_sq = 0.0f;
        float dq_leg_sq = 0.0f;
        for (int j = 0; j < 12; ++j) {
            const float q_err = q[static_cast<size_t>(j)] - current_q[static_cast<size_t>(j)];
            const float dq_err = dq[static_cast<size_t>(j)] - current_dq[static_cast<size_t>(j)];
            q_leg_sq += q_err * q_err;
            dq_leg_sq += dq_err * dq_err;
        }

        // The first candidate can be quiet while the next frame immediately
        // contains a large kick.  Look a short distance ahead to reject that
        // false minimum without changing the allowed start region.
        float future_leg_vel = 0.0f;
        const int future_end = std::min(num_frames_ - 1, f + 3);
        for (int k = f; k <= future_end; ++k) {
            for (int j = 0; j < 12; ++j) {
                future_leg_vel = std::max(
                    future_leg_vel,
                    std::fabs(joint_vel_[static_cast<size_t>(k)][static_cast<size_t>(j)]));
            }
        }

        // R1 policy indices: ankle roll is 5 and 11.  The symmetry term is
        // deliberately measured relative to the locomotion default, so a
        // mirrored clip posture is not rejected just because both legs move.
        const float ankle_roll_err =
            std::fabs(q[5] - spec::kDefaultJointPos[5]) +
            std::fabs(q[11] - spec::kDefaultJointPos[11]);
        float symmetry_err = 0.0f;
        for (int j = 0; j < 6; ++j) {
            const float left = q[static_cast<size_t>(j)] - spec::kDefaultJointPos[static_cast<size_t>(j)];
            const float right = q[static_cast<size_t>(j + 6)] -
                                spec::kDefaultJointPos[static_cast<size_t>(j + 6)];
            symmetry_err += std::fabs(left - right);
        }
        symmetry_err /= 6.0f;

        const auto& torso = torso_quat_w_[static_cast<size_t>(f)];
        const float gravity_z = std::clamp(
            1.0f - 2.0f * (torso.x() * torso.x() + torso.y() * torso.y()), -1.0f, 1.0f);
        const float tilt = std::acos(gravity_z);

        const float score =
            std::sqrt(q_leg_sq) +
            0.25f * std::sqrt(dq_leg_sq) +
            0.50f * future_leg_vel +
            2.0f * ankle_roll_err +
            2.0f * symmetry_err +
            0.05f * tilt;
        if (std::isfinite(score) && score < best_score) {
            best_score = score;
            best = f;
        }
    }
    return best;
}
