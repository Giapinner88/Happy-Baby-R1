#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <vector>

#include "../src/motion/ArmGesturePlayer.hpp"

namespace {

using Arm = std::array<float, ArmGesturePlayer::kNumArm>;
using Head = std::array<float, ArmGesturePlayer::kNumHead>;
using Joints = std::array<float, ArmGesturePlayer::kNumJoints>;

Arm Filled(float value) {
    Arm arm{};
    arm.fill(value);
    return arm;
}

Joints Blend(ArmGesturePlayer& player) {
    Joints joints{};
    player.BlendInto(joints);
    return joints;
}

void ExpectNear(float actual, float expected) {
    if (std::fabs(actual - expected) >= 1e-5f)
        std::abort();
}

void Check(bool condition) {
    if (!condition)
        std::abort();
}

}  // namespace

int main() {
    {
        ArmGesturePlayer player;
        player.Init(Filled(0.0f));
        player.AddGesture("one_shot", {Filled(0.0f), Filled(1.0f)}, 2.0f, false);
        player.AddGesture("loop", {Filled(0.0f), Filled(1.0f)}, 2.0f, true);
        Check(!player.Loops("one_shot"));
        Check(player.Loops("loop"));
        player.Trigger("one_shot");
        player.Update(0.25f);
        Check(!player.ActiveClipFinished());
        player.Update(0.25f);
        Check(player.ActiveClipFinished());
    }

    {
        ArmGesturePlayer player;
        player.Init(Filled(0.0f), 0.4f, 1.0f);
        player.AddGesture("a", {Filled(1.0f)}, 50.0f, false);
        player.AddGesture("b", {Filled(2.0f)}, 50.0f, false);

        player.Trigger("a");
        player.Update(0.4f);
        ExpectNear(Blend(player)[14], 1.0f);

        player.Trigger("b");
        Check(player.ActiveName() == "a");
        Check(player.PendingName() == "b");

        player.Update(0.5f);
        ExpectNear(Blend(player)[14], 0.5f);
        Check(player.ActiveName() == "a");

        player.Update(0.5f);
        ExpectNear(Blend(player)[14], 0.0f);
        Check(player.ActiveName() == "b");
        Check(player.PendingName().empty());

        player.Update(0.4f);
        ExpectNear(Blend(player)[14], 2.0f);
    }

    {
        ArmGesturePlayer player;
        player.Init(Filled(0.0f), 0.4f, 1.0f);
        player.AddGesture("a", {Filled(1.0f)}, 50.0f, false);
        player.AddGesture("b", {Filled(2.0f)}, 50.0f, false);
        player.AddGesture("c", {Filled(3.0f)}, 50.0f, false);

        player.Trigger("a");
        player.Update(0.4f);
        player.Trigger("b");
        player.Trigger("c");
        Check(player.PendingName() == "c");
        player.Update(1.0f);
        Check(player.ActiveName() == "c");
    }

    {
        ArmGesturePlayer player;
        player.Init(Filled(0.0f), 0.4f, 1.0f);
        player.AddGesture("a", {Filled(1.0f)}, 50.0f, false);
        player.AddGesture("b", {Filled(2.0f)}, 50.0f, false);

        player.Trigger("a");
        player.Update(0.4f);
        player.Trigger("b");
        player.Retract();
        Check(player.PendingName().empty());
        player.Update(1.0f);
        Check(player.Idle());
    }

    {
        ArmGesturePlayer player;
        player.Init(Filled(0.0f), 0.4f, 1.0f);
        player.AddGesture("a", {Filled(1.0f)}, 50.0f, false);
        player.Trigger("a");
        player.Update(0.4f);
        player.Trigger("a");
        Check(player.PendingName().empty());
        player.Update(1.0f);
        Check(player.Idle());
    }

    {
        ArmGesturePlayer player;
        player.Init(Filled(0.0f), 0.4f, 1.0f);
        const Head recorded_head{0.60f, -0.25f};
        player.AddGesture("head", {Filled(1.0f)}, 50.0f, false,
                          0.0f, 0.0f, {recorded_head});
        player.Trigger("head");
        player.Update(0.4f);
        Check(player.HeadActive());
        ExpectNear(player.ReferenceHead()[0], 0.60f);
        ExpectNear(player.ReferenceHead()[1], -0.25f);

        player.Retract();
        player.Update(0.5f);
        Check(player.HeadActive());
        ExpectNear(player.ReferenceHead()[0], 0.30f);
        ExpectNear(player.ReferenceHead()[1], -0.125f);
        player.Update(0.5f);
        Check(!player.HeadActive());
        ExpectNear(player.ReferenceHead()[0], 0.0f);
        ExpectNear(player.ReferenceHead()[1], 0.0f);
    }

    {
        ArmGesturePlayer player;
        player.Init(Filled(0.0f), 0.4f, 1.0f);
        const Head mismatched_head{0.40f, 0.20f};
        player.AddGesture("arm_only", {Filled(1.0f), Filled(1.0f)}, 50.0f, false,
                          0.0f, 0.0f, {mismatched_head});
        player.Trigger("arm_only");
        player.Update(0.4f);
        Check(!player.HeadActive());
        ExpectNear(player.ReferenceHead()[0], 0.0f);
        ExpectNear(player.ReferenceHead()[1], 0.0f);
    }

    // RetractSafety: trần vận tốc phải KÉO DÀI thời gian thu cho gesture vươn xa, và
    // KHÔNG được rút ngắn dưới safety_retract_s cho gesture vươn gần.
    {
        const float kSafetyS = 0.4f, kMaxVel = 4.0f;
        // Vươn 2.0 rad: 0.4s cố định => đỉnh 2.0*1.5/0.4 = 7.5 rad/s > trần.
        // Phải giãn ra 2.0*1.5/4.0 = 0.75s để đỉnh đúng bằng 4.0.
        ArmGesturePlayer far_player;
        far_player.Init(Filled(0.0f), 0.4f, 1.0f, kSafetyS, kMaxVel);
        far_player.AddGesture("far", {Filled(2.0f)}, 50.0f, false);
        far_player.Trigger("far");
        far_player.Update(0.4f);                 // weight -> 1
        ExpectNear(far_player.Weight(), 1.0f);
        far_player.RetractSafety();
        far_player.Update(0.375f);                // nửa quãng của 0.75s
        ExpectNear(far_player.Weight(), 0.5f);    // 0.4s thì đã về 0 từ lâu
        Check(far_player.Active());

        // Vươn 0.5 rad: 0.5*1.5/4.0 = 0.1875s < 0.4s -> giữ nguyên 0.4s, không nhanh hơn.
        ArmGesturePlayer near_player;
        near_player.Init(Filled(0.0f), 0.4f, 1.0f, kSafetyS, kMaxVel);
        near_player.AddGesture("near", {Filled(0.5f)}, 50.0f, false);
        near_player.Trigger("near");
        near_player.Update(0.4f);
        near_player.RetractSafety();
        near_player.Update(0.4f);
        ExpectNear(near_player.Weight(), 0.0f);
        Check(!near_player.Active());

        // max_vel <= 0 = tắt -> hành vi cũ y nguyên kể cả khi vươn xa.
        ArmGesturePlayer off_player;
        off_player.Init(Filled(0.0f), 0.4f, 1.0f, kSafetyS, 0.0f);
        off_player.AddGesture("far", {Filled(2.0f)}, 50.0f, false);
        off_player.Trigger("far");
        off_player.Update(0.4f);
        off_player.RetractSafety();
        off_player.Update(0.4f);
        ExpectNear(off_player.Weight(), 0.0f);
    }

    // Thu tay không được blend trực tiếp vào target policy đang thay đổi. Sau
    // khi quỹ đạo return kết thúc, player vẫn giữ quyền handover cho tới khi
    // Application xác nhận policy/measured state ổn định rồi mới nhả overlay.
    {
        ArmGesturePlayer player;
        player.Init(Filled(0.0f), 0.1f, 0.1f);
        player.SetReturnLimits({0.5f, 2.0f, 20.0f});
        player.AddGesture("return", {Filled(1.0f)}, 50.0f, false);
        player.Trigger("return");
        player.Update(0.1f);
        Check(Blend(player)[14] > 0.99f);

        player.Retract();
        Joints policy{};
        for (int step = 0; step < 400 && !player.HandoverPending(); ++step) {
            player.Update(0.01f);
            Blend(player);
        }
        Check(player.HandoverPending());
        Check(player.Active());
        Check(!player.Idle());
        Check(player.HandoverMatches(policy, 0.02f, 0.02f));
        player.ReleaseHandover();
        Check(player.Idle());
    }

    // Thu tay từ một clip đang chuyển động phải bắt đầu trong giới hạn tốc độ
    // return, không mang nguyên vận tốc frame của clip vào quỹ đạo quintic.
    {
        ArmGesturePlayer player;
        player.Init(Filled(0.0f), 0.1f, 0.1f);
        constexpr float kDt = 0.002f;
        constexpr float kMaxVelocity = 0.60f;
        player.SetReturnLimits({0.5f, kMaxVelocity, 2.0f});
        player.AddGesture("moving", {Filled(0.0f), Filled(0.15f)}, 50.0f, true);
        player.Trigger("moving");
        Joints command{};
        for (int i = 0; i < 52; ++i) {
            player.Update(kDt);
            command.fill(0.0f);
            player.BlendInto(command);
        }
        const float before = command[14];
        player.Retract();
        player.Update(kDt);
        command.fill(0.0f);
        player.BlendInto(command);  // capture current output and return goal
        const float captured = command[14];
        player.Update(kDt);
        command.fill(0.0f);
        player.BlendInto(command);
        const float return_step = std::fabs(command[14] - captured);
        Check(std::fabs(captured - before) < 1e-5f);
        Check(return_step <= kMaxVelocity * kDt + 1e-5f);
    }

    // RetractSafety được gọi lặp trong lúc move/guard phải giữ nguyên tiến độ,
    // không reset trạng thái return về điểm bắt đầu mỗi tick.
    {
        ArmGesturePlayer player;
        player.Init(Filled(0.0f), 0.1f, 0.1f);
        player.SetReturnLimits({0.5f, 2.0f, 20.0f});
        player.AddGesture("static", {Filled(1.0f)}, 50.0f, false);
        player.Trigger("static");
        player.Update(0.1f);
        Joints command{};
        player.BlendInto(command);
        for (int i = 0; i < 400; ++i) {
            player.RetractSafety();
            player.Update(0.01f);
            command.fill(0.0f);
            player.BlendInto(command);
        }
        Check(player.HandoverPending());
        Check(command[14] < 1e-4f);
    }

    // Khi locomotion nhường quyền điều khiển cho một app state khác, gesture
    // phải bị hủy để không tái xuất hiện với pose cũ khi quay lại policy.
    {
        ArmGesturePlayer player;
        player.Init(Filled(0.0f));
        player.SetReturnLimits({0.5f, 0.6f, 2.0f});
        player.AddGesture("active", {Filled(1.0f)}, 50.0f, false);
        player.Trigger("active");
        player.Update(0.4f);
        player.Cancel();
        Check(player.Idle());
        Check(!player.Active());
        Check(player.PendingName().empty());
        ExpectNear(player.Weight(), 0.0f);
    }

    return 0;
}
