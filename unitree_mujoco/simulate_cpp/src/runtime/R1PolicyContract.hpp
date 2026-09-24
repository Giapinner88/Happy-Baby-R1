#pragma once

#include <string>
#include <vector>

namespace r1::contract {

inline const std::vector<std::string>& JointNames() {
    static const std::vector<std::string> names = {
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "waist_roll_joint", "waist_yaw_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    };
    return names;
}

inline const std::vector<std::string>& ProprioceptiveObservationNames() {
    static const std::vector<std::string> names = {
        "base_ang_vel", "projected_gravity", "command", "phase",
        "joint_pos", "joint_vel", "actions",
    };
    return names;
}

}  // namespace r1::contract
