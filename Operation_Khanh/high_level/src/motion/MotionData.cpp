#include "MotionData.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include <cnpy.h>

void MotionData::Load(const std::string& npz_path) {
    cnpy::npz_t npz = cnpy::npz_load(npz_path);

    auto require = [&](const char* key) -> cnpy::NpyArray& {
        auto it = npz.find(key);
        if (it == npz.end())
            throw std::runtime_error("NPZ thiếu key '" + std::string(key) + "': " + npz_path);
        return it->second;
    };

    cnpy::NpyArray& jp = require("joint_pos");
    cnpy::NpyArray& jv = require("joint_vel");
    cnpy::NpyArray& bq = require("body_quat_w");

    if (jp.shape.size() != 2 || jp.shape[1] != spec::kNumJoints)
        throw std::runtime_error("joint_pos phải có shape (frames, 24)");
    // Kiểm tra định dạng shape của dữ liệu
    if (bq.shape.size() != 3 || bq.shape[1] <= spec::kMotionTorsoIdx || bq.shape[2] != 4)
        throw std::runtime_error("body_quat_w phải có shape (frames, N>" +
                                  std::to_string(spec::kMotionTorsoIdx) +
                                  ", 4) — clip khác layout?");

    num_frames_ = static_cast<int>(jp.shape[0]);
    if (jv.shape[0] != jp.shape[0] || bq.shape[0] != jp.shape[0])
        throw std::runtime_error("Số frame không khớp giữa các mảng trong NPZ");

    auto it_fps = npz.find("fps");
    if (it_fps != npz.end() && it_fps->second.word_size == 8) {
        fps_ = static_cast<float>(it_fps->second.data<double>()[0]);
    }

    const float* jp_data = jp.data<float>();
    const float* jv_data = jv.data<float>();
    const float* bq_data = bq.data<float>();

    joint_pos_.resize(static_cast<size_t>(num_frames_));
    joint_vel_.resize(static_cast<size_t>(num_frames_));
    torso_quat_w_.resize(static_cast<size_t>(num_frames_));

    const int stride_bq = static_cast<int>(bq.shape[1]) * 4; // số body THẬT trong file
    for (int f = 0; f < num_frames_; ++f) {
        for (int j = 0; j < spec::kNumJoints; ++j) {
            joint_pos_[f][j] = jp_data[f * spec::kNumJoints + j];
            joint_vel_[f][j] = jv_data[f * spec::kNumJoints + j];
        }
        const float* q = &bq_data[f * stride_bq + spec::kMotionTorsoIdx * 4];
        torso_quat_w_[f] = Eigen::Quaternionf(q[0], q[1], q[2], q[3]); // wxyz
    }

    std::cout << "[MotionData] Đã nạp " << num_frames_ << " frames @ " << fps_
              << "fps từ " << npz_path << "\n";
}

int MotionData::FindSmoothStartFrame(int search_frames) const {
    const int window = std::clamp(search_frames, 1, num_frames_);

    // Chấm điểm bằng giá trị vật lý tuyệt đối để tìm frame ổn định nhất
    int best = 0;
    float best_score = std::numeric_limits<float>::max();
    for (int f = 0; f < window; ++f) {
        float vel = 0.0f;
        for (int j = 0; j < spec::kNumJoints; ++j)
            vel = std::max(vel, std::fabs(joint_vel_[static_cast<size_t>(f)][static_cast<size_t>(j)]));

        float e = 0.0f;
        for (int j = 0; j < 12; ++j) { // chỉ 2 chân — quyết định pose đứng
            float d = joint_pos_[static_cast<size_t>(f)][static_cast<size_t>(j)] -
                      spec::kDefaultJointPos[static_cast<size_t>(j)];
            e += d * d;
        }
        float leg_err = std::sqrt(e);

        const auto& q = torso_quat_w_[static_cast<size_t>(f)];
        float gz = std::clamp(1.0f - 2.0f * (q.x() * q.x() + q.y() * q.y()), -1.0f, 1.0f);
        float tilt = std::acos(gz); // rad

        // Đánh giá dựa trên vel, leg_err, và độ nghiêng (tilt)
        float score = vel + 0.1f * leg_err + 0.05f * tilt;
        if (score < best_score) {
            best_score = score;
            best = f;
        }
    }
    return best;
}
