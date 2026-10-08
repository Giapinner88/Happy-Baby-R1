// Golden regression cho frame 83-D chuẩn (`r1_common_pd_base83_v1`).
//
// Bài toán này tồn tại vì công thức dựng observation ĐÃ BỊ REFACTOR: thân hàm
// được rút khỏi LocomotionController::BuildObservation() thành
// r1::obs83::Build() để FlatPlusController dùng lại. Refactor kiểu đó có thể
// đổi giá trị mà không đổi kích thước, và không test nào khác ở đây so sánh
// từng phần tử của vector 83-D với một nguồn độc lập.
//
// Nguồn độc lập đó là golden trace do
// unitree_rl_mjlab_meta/scripts/make_flat_plus_golden_trace.py sinh ra: nó khai
// báo frame 83-D theo đúng định nghĩa contract (nối các term theo thứ tự), hoàn
// toàn không đi qua code C++. Trùng khớp từng phần tử cũng chính là dòng đầu
// của yêu cầu cross-runtime trong plan §7.3:
//
//     Python base frame == HB base frame == simulator base frame

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../src/policy/BaseObservation83.hpp"

namespace {

int failures = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        std::cerr << "FAIL: " << what << std::endl;
        ++failures;
    }
}

std::vector<float> Values(std::istringstream& line) {
    std::vector<float> out;
    float value = 0.0f;
    while (line >> value) out.push_back(value);
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: flat_plus_base83_golden_test <trace.txt>\n";
        return 2;
    }
    std::ifstream file(argv[1]);
    if (!file.is_open()) {
        std::cerr << "cannot open trace " << argv[1] << std::endl;
        return 2;
    }

    // default_q khác 0 có chủ đích: nếu dùng toàn 0 thì phép trừ
    // q - default_q luôn đúng một cách tình cờ và bug ở đó sẽ lọt.
    std::array<float, spec::kNumJoints> default_q{};
    for (int i = 0; i < spec::kNumJoints; ++i) {
        default_q[static_cast<std::size_t>(i)] = 0.37f * static_cast<float>(i + 1);
    }

    std::vector<float> gyro, grav, cmd, phase, q_rel, dq, prev, expected;
    int checked = 0;

    std::string raw;
    while (std::getline(file, raw)) {
        if (raw.empty() || raw[0] == '#') continue;
        std::istringstream line(raw);
        std::string tag;
        line >> tag;

        if (tag == "GYRO") { gyro = Values(line); continue; }
        if (tag == "GRAV") { grav = Values(line); continue; }
        if (tag == "CMD") { cmd = Values(line); continue; }
        if (tag == "PHASE") { phase = Values(line); continue; }
        if (tag == "Q") { q_rel = Values(line); continue; }
        if (tag == "DQ") { dq = Values(line); continue; }
        if (tag == "PREV") { prev = Values(line); continue; }
        if (tag != "BASE") continue;

        expected = Values(line);
        Check(static_cast<int>(expected.size()) == r1::obs83::kDim,
              "trace BASE row is 83 long");

        RobotState state;
        state.gyro = Eigen::Vector3f(gyro[0], gyro[1], gyro[2]);
        state.projected_gravity = Eigen::Vector3f(grav[0], grav[1], grav[2]);
        for (int i = 0; i < spec::kNumJoints; ++i) {
            const auto index = static_cast<std::size_t>(i);
            // Trace lưu joint_pos ĐÃ trừ default; dựng lại góc tuyệt đối để
            // Build() phải tự thực hiện phép trừ.
            state.q[index] = q_rel[index] + default_q[index];
            state.dq[index] = dq[index];
        }

        ControlContext ctx{state};
        ctx.cmd_vx = cmd[0];
        ctx.cmd_vy = cmd[1];
        ctx.cmd_yaw = cmd[2];
        ctx.gait_phase = {phase[0], phase[1]};

        std::array<float, spec::kNumJoints> last_action{};
        for (int i = 0; i < spec::kNumJoints; ++i)
            last_action[static_cast<std::size_t>(i)] = prev[static_cast<std::size_t>(i)];

        std::vector<float> built(r1::obs83::kDim, 0.0f);
        r1::obs83::Build(ctx, default_q, last_action, built);

        for (std::size_t i = 0; i < expected.size() && i < built.size(); ++i) {
            if (std::fabs(built[i] - expected[i]) > 1e-4f) {
                std::cerr << "FAIL: base frame element " << i << " expected "
                          << expected[i] << " got " << built[i] << std::endl;
                ++failures;
                break;
            }
        }
        ++checked;
    }

    Check(checked > 0, "at least one BASE row was checked");

    // Các offset mà overlay tay của legacy ghi vào phải đúng chỗ, nếu không
    // mask/bias sẽ rơi vào sai term mà vẫn chạy.
    Check(r1::obs83::kAngVelOffset == 0, "base_ang_vel at 0");
    Check(r1::obs83::kGravityOffset == 3, "projected_gravity at 3");
    Check(r1::obs83::kCommandOffset == 6, "command at 6");
    Check(r1::obs83::kPhaseOffset == 9, "phase at 9");
    Check(r1::obs83::kJointPosOffset == 11, "joint_pos at 11");
    Check(r1::obs83::kJointVelOffset == 35, "joint_vel at 35");
    Check(r1::obs83::kActionsOffset == 59, "actions at 59");
    Check(r1::obs83::kDim == 83, "base frame is 83-D");

    // History gesture must mask only arm q/dq in the canonical frame. The
    // actions term is deliberately untouched because it is the policy's raw
    // previous action, not the motor target after gesture blending.
    std::vector<float> masked(r1::obs83::kDim, 0.0f);
    for (int i = 0; i < r1::obs83::kDim; ++i)
        masked[static_cast<std::size_t>(i)] = static_cast<float>(i + 1);
    const std::vector<float> original = masked;
    r1::obs83::MaskArmState(masked, 0.25f);
    for (int i = 0; i < spec::kNumJoints; ++i) {
        const bool arm = i >= 14;
        const std::size_t q = static_cast<std::size_t>(r1::obs83::kJointPosOffset + i);
        const std::size_t dq_index = static_cast<std::size_t>(r1::obs83::kJointVelOffset + i);
        Check(std::fabs(masked[q] - original[q] * (arm ? 0.25f : 1.0f)) < 1e-6f,
              "history arm q mask at joint " + std::to_string(i));
        Check(std::fabs(masked[dq_index] - original[dq_index] * (arm ? 0.25f : 1.0f)) < 1e-6f,
              "history arm dq mask at joint " + std::to_string(i));
    }
    for (int i = 0; i < spec::kNumJoints; ++i) {
        const std::size_t action = static_cast<std::size_t>(r1::obs83::kActionsOffset + i);
        Check(std::fabs(masked[action] - original[action]) < 1e-6f,
              "history mask leaves action at joint " + std::to_string(i));
    }

    if (failures != 0) {
        std::cerr << "flat_plus_base83_golden_test: " << failures << " failure(s)\n";
        return 1;
    }
    std::cout << "flat_plus_base83_golden_test: PASS (" << checked << " frames)\n";
    return 0;
}
