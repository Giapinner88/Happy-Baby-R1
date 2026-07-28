// record_motion - Công cụ ghi lại quỹ đạo khớp (từ rt/lowstate) để tạo file chuyển động.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>

#include <cnpy.h>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "../src/config/RobotSpec.hpp"

using LowState_ = unitree_hg::msg::dds_::LowState_;

namespace {

// Định nghĩa cấu trúc tay cầm R3-1 đơn giản.
typedef union {
    struct {
        uint8_t R1 : 1; uint8_t L1 : 1; uint8_t start : 1; uint8_t select : 1;
        uint8_t R2 : 1; uint8_t L2 : 1; uint8_t F1 : 1;    uint8_t F2 : 1;
        uint8_t A : 1;  uint8_t B : 1;  uint8_t X : 1;     uint8_t Y : 1;
        uint8_t up : 1; uint8_t right : 1; uint8_t down : 1; uint8_t left : 1;
    } c;
    uint16_t value;
} KeyUnion;

typedef struct {
    uint8_t head[2];
    KeyUnion btn;
    float lx, rx, ry, L2, ly;
    uint8_t idle[16];
} RemoteRx;

const char* kBtnName[16] = {
    "R1", "L1", "start", "select", "R2", "L2", "F1", "F2",
    "A",  "B",  "X",     "Y",      "up", "right", "down", "left"};

std::string DecodeButtons(uint16_t v) {
    std::string s;
    for (int i = 0; i < 16; ++i) {
        if (v & (1u << i)) {
            if (!s.empty()) s += "+";
            s += kBtnName[i];
        }
    }
    return s.empty() ? "none" : s;
}

std::atomic<bool> g_running{true};
void OnSigint(int) { g_running = false; }

int64_t NowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

int main(int argc, char** argv) {
    std::string iface   = (argc > 1) ? argv[1] : "eth10";
    std::string out_dir = (argc > 2) ? argv[2] : "/tmp/r1_motions";
    float move_thresh   = (argc > 3) ? std::stof(argv[3]) : 0.15f;   // Ngưỡng phát hiện chuyển động
    float still_thresh  = (argc > 4) ? std::stof(argv[4]) : 0.05f;   // Ngưỡng đứng yên

    constexpr float kSettleS    = 0.6f;   // Thời gian đứng yên để lưu đoạn
    constexpr float kMinDurS    = 0.8f;   // Thời gian tối thiểu của đoạn
    constexpr int   kSaveEveryN = 10;     // Lưu ở tần số 50Hz
    constexpr float kTagWindowS = 0.3f;   // Thời gian lưu nhãn nút bấm

    std::signal(SIGINT, OnSigint);
    ::mkdir(out_dir.c_str(), 0755);  // bỏ qua lỗi EEXIST

    unitree::robot::ChannelFactory::Instance()->Init(0, iface);

    std::mutex mtx;
    bool  moving      = false;
    float still_time  = 0.0f;
    float seg_time     = 0.0f;
    float seg_tag_time = 0.0f;
    uint16_t seg_btn_mask = 0;
    int   seg_count    = 0;
    int   decim_count  = 0;
    std::vector<std::array<float, spec::kNumJoints>> seg_frames;
    std::vector<std::array<float, 4>> seg_quats;   // Hướng thân (IMU wxyz)
    int64_t last_t = 0;

    auto save_segment = [&]() {
        if (seg_frames.empty()) return;
        float dur = static_cast<float>(seg_frames.size()) / (500.0f / kSaveEveryN);
        if (dur < kMinDurS) {
            printf("[record_motion] Đoạn quá ngắn (%.2fs) - bỏ qua (nhiễu?).\n", dur);
            seg_frames.clear();
            return;
        }
        ++seg_count;
        std::string tag = DecodeButtons(seg_btn_mask);
        std::string tag_fs = tag;
        for (auto& ch : tag_fs)
            if (ch == '+') ch = '_';

        char path[512];
        snprintf(path, sizeof(path), "%s/capture_%03d_%s.npz", out_dir.c_str(), seg_count,
                  tag_fs.c_str());

        const int frames = static_cast<int>(seg_frames.size());
        std::vector<float> flat(static_cast<size_t>(frames) * spec::kNumJoints);
        for (int f = 0; f < frames; ++f)
            for (int j = 0; j < spec::kNumJoints; ++j)
                flat[static_cast<size_t>(f) * spec::kNumJoints + j] = seg_frames[static_cast<size_t>(f)][j];

        cnpy::npz_save(path, "joint_pos", flat.data(),
                        {static_cast<size_t>(frames), static_cast<size_t>(spec::kNumJoints)}, "w");
        double fps = 500.0 / kSaveEveryN;
        cnpy::npz_save(path, "fps", &fps, {1}, "a");
        // Lưu hướng thân để bù cổ chân.
        std::vector<float> qflat(static_cast<size_t>(frames) * 4);
        for (int f = 0; f < frames; ++f)
            for (int k = 0; k < 4; ++k)
                qflat[static_cast<size_t>(f) * 4 + k] = seg_quats[static_cast<size_t>(f)][k];
        cnpy::npz_save(path, "torso_quat", qflat.data(),
                        {static_cast<size_t>(frames), 4}, "a");

        printf("\n[record_motion] >> Đoạn #%d: %.2fs, %d frame, phím giữ lúc bắt đầu='%s'\n"
               "                  -> %s\n\n",
               seg_count, dur, frames, tag.c_str(), path);
        seg_frames.clear();
        seg_quats.clear();
    };

    unitree::robot::ChannelSubscriber<LowState_> sub("rt/lowstate");
    sub.InitChannel([&](const void* msg) {
        const LowState_* st = static_cast<const LowState_*>(msg);
        int64_t t = NowUs();
        float dt = (last_t == 0) ? 0.002f : static_cast<float>(t - last_t) / 1e6f;
        last_t = t;
        if (dt <= 0.0f || dt > 0.1f) dt = 0.002f;

        float max_dq = 0.0f;
        std::array<float, spec::kNumJoints> q{};
        for (int i = 0; i < spec::kNumJoints; ++i) {
            const int idl = spec::MotorIdl(i);
            q[static_cast<size_t>(i)] = st->motor_state()[idl].q();
            max_dq = std::max(max_dq, std::fabs(st->motor_state()[idl].dq()));
        }

        std::array<float, 4> quat{};   // Hướng thân
        for (int k = 0; k < 4; ++k) quat[static_cast<size_t>(k)] = st->imu_state().quaternion()[k];

        RemoteRx remote{};
        std::memcpy(&remote, st->wireless_remote().data(), sizeof(remote));

        std::lock_guard<std::mutex> lk(mtx);

        if (!moving) {
            if (max_dq > move_thresh) {
                moving = true;
                seg_frames.clear();
                seg_quats.clear();
                decim_count = 0;
                seg_btn_mask = 0;
                seg_tag_time = 0.0f;
                seg_time = 0.0f;
                still_time = 0.0f;
                printf("\n[record_motion] >> Bắt đầu đoạn mới (|dq| max=%.2f rad/s)...\n", max_dq);
            }
            return;
        }

        seg_time += dt;
        if (seg_tag_time < kTagWindowS) {
            seg_btn_mask |= remote.btn.value;
            seg_tag_time += dt;
        }
        if (++decim_count >= kSaveEveryN) {
            decim_count = 0;
            seg_frames.push_back(q);
            seg_quats.push_back(quat);
        }

        if (max_dq < still_thresh) {
            still_time += dt;
            if (still_time >= kSettleS) {
                moving = false;
                still_time = 0.0f;
                save_segment();
            }
        } else {
            still_time = 0.0f;
        }
    });

    printf("[record_motion] Đang nghe rt/lowstate trên %s (CHỈ NGHE, không gửi gì).\n",
           iface.c_str());
    printf("[record_motion] Dùng tay cầm/built-in điều khiển robot; tool tự phát hiện\n"
           "                từng đoạn chuyển động và lưu vào %s\n",
           out_dir.c_str());
    printf("[record_motion] move_thresh=%.2f still_thresh=%.2f settle=%.1fs min_dur=%.1fs\n",
           move_thresh, still_thresh, kSettleS, kMinDurS);
    printf("[record_motion] Ctrl-C để dừng.\n\n");

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::lock_guard<std::mutex> lk(mtx);
        if (moving) {
            printf("\r  [đang ghi] %.1fs, %zu frame đã lưu...   ", seg_time, seg_frames.size());
            fflush(stdout);
        }
    }

    printf("\n[record_motion] Dừng. Đã ghi %d đoạn vào %s\n", seg_count, out_dir.c_str());
    return 0;
}
