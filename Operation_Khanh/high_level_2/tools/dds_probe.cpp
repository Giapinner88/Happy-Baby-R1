// dds_probe - Công cụ chẩn đoán thụ động đo lường độ lệch sim2real và chất lượng bám khớp.

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "../src/config/RobotSpec.hpp"

using LowCmd_ = unitree_hg::msg::dds_::LowCmd_;
using LowState_ = unitree_hg::msg::dds_::LowState_;

namespace {

constexpr int kMaxSamples = 300 * 500;  // 300 s @ 500 Hz
constexpr int kMaxLag = 25;             // quét trễ 0..25 mẫu = 0..50 ms

const char* kJointName[spec::kNumJoints] = {
    "L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee", "L_ank_pitch", "L_ank_roll",
    "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee", "R_ank_pitch", "R_ank_roll",
    "waist_roll",  "waist_yaw",
    "L_sh_pitch",  "L_sh_roll",  "L_sh_yaw",  "L_elbow", "L_wrist",
    "R_sh_pitch",  "R_sh_roll",  "R_sh_yaw",  "R_elbow", "R_wrist"};

// Giới hạn mô-men xoắn định mức để phát hiện bão hòa.
const float kEffortLimit[spec::kNumJoints] = {
    88.f, 88.f, 88.f, 139.f, 50.f, 50.f,   // chân trái
    88.f, 88.f, 88.f, 139.f, 50.f, 50.f,   // chân phải
    50.f, 88.f,                            // waist roll, yaw
    25.f, 25.f, 25.f, 25.f, 25.f,          // tay trái
    25.f, 25.f, 25.f, 25.f, 25.f};         // tay phải

std::atomic<bool> g_running{true};
void OnSigint(int) { g_running = false; }

int64_t NowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

struct Sample {
    float qd[spec::kNumJoints];   // q_des từ lowcmd
    float q[spec::kNumJoints];    // q thực từ lowstate
    float dq[spec::kNumJoints];
    float tau[spec::kNumJoints];
};

std::mutex g_mtx;
LowCmd_ g_cmd{};
int64_t g_cmd_t = 0;              // thời điểm nhận lowcmd gần nhất
bool g_have_cmd = false;
std::vector<int64_t> g_cmd_periods;

std::vector<Sample> g_samples;
std::vector<int64_t> g_state_periods;
std::vector<int64_t> g_cmd_age;   // tuổi của lowcmd tại thời điểm lowstate tới

// Tính phân vị trên vector đã sắp xếp.
double Pct(std::vector<int64_t>& v, double p) {
    if (v.empty()) return 0.0;
    size_t i = static_cast<size_t>(p * (v.size() - 1));
    return static_cast<double>(v[i]);
}

}  // namespace

int main(int argc, char** argv) {
    std::string iface = (argc > 1) ? argv[1] : "eth10";
    std::string csv_path = (argc > 2) ? argv[2] : "/tmp/r1_probe.csv";

    std::signal(SIGINT, OnSigint);
    g_samples.reserve(kMaxSamples);

    unitree::robot::ChannelFactory::Instance()->Init(0, iface);

    unitree::robot::ChannelSubscriber<LowCmd_> sub_cmd("rt/lowcmd");
    sub_cmd.InitChannel([](const void* msg) {
        int64_t t = NowUs();
        std::lock_guard<std::mutex> lk(g_mtx);
        if (g_have_cmd) g_cmd_periods.push_back(t - g_cmd_t);
        g_cmd = *static_cast<const LowCmd_*>(msg);
        g_cmd_t = t;
        g_have_cmd = true;
    });

    int64_t last_state_t = 0;
    unitree::robot::ChannelSubscriber<LowState_> sub_state("rt/lowstate");
    sub_state.InitChannel([&last_state_t](const void* msg) {
        const LowState_* st = static_cast<const LowState_*>(msg);
        int64_t t = NowUs();

        std::lock_guard<std::mutex> lk(g_mtx);
        if (!g_have_cmd) return;
        if (last_state_t != 0) g_state_periods.push_back(t - last_state_t);
        last_state_t = t;
        if (g_samples.size() >= kMaxSamples) return;

        g_cmd_age.push_back(t - g_cmd_t);
        Sample s{};
        for (int i = 0; i < spec::kNumJoints; ++i) {
            int idl = spec::MotorIdl(i);
            s.qd[i] = g_cmd.motor_cmd()[idl].q();
            s.q[i] = st->motor_state()[idl].q();
            s.dq[i] = st->motor_state()[idl].dq();
            s.tau[i] = st->motor_state()[idl].tau_est();
        }
        g_samples.push_back(s);
    });

    printf("[dds_probe] Đang nghe rt/lowcmd + rt/lowstate trên %s. Ctrl-C để dừng.\n",
           iface.c_str());
    printf("[dds_probe] Cho robot nhảy trọn 1 điệu rồi mới dừng.\n\n");

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::lock_guard<std::mutex> lk(g_mtx);
        printf("\r  đã ghi %zu mẫu (%.1f s)   ", g_samples.size(),
               g_samples.size() / 500.0);
        fflush(stdout);
    }

    std::lock_guard<std::mutex> lk(g_mtx);
    const size_t N = g_samples.size();
    printf("\n\n");
    if (N < 500) {
        printf("[dds_probe] Quá ít mẫu (%zu). run_r1 có đang chạy policy không?\n", N);
        return 1;
    }

    // Ghi dữ liệu ra CSV.
    FILE* f = fopen(csv_path.c_str(), "w");
    if (f) {
        fprintf(f, "sample");
        for (int i = 0; i < spec::kNumJoints; ++i)
            fprintf(f, ",qd_%s,q_%s,dq_%s,tau_%s", kJointName[i], kJointName[i],
                    kJointName[i], kJointName[i]);
        fprintf(f, "\n");
        for (size_t k = 0; k < N; ++k) {
            fprintf(f, "%zu", k);
            for (int i = 0; i < spec::kNumJoints; ++i)
                fprintf(f, ",%.5f,%.5f,%.4f,%.3f", g_samples[k].qd[i], g_samples[k].q[i],
                        g_samples[k].dq[i], g_samples[k].tau[i]);
            fprintf(f, "\n");
        }
        fclose(f);
        printf("Đã ghi %zu mẫu -> %s\n\n", N, csv_path.c_str());
    }

    // Thống kê jitter của vòng điều khiển.
    auto report_period = [](const char* name, std::vector<int64_t> v, double nominal_ms) {
        if (v.empty()) return;
        std::sort(v.begin(), v.end());
        printf("  %-18s p50=%.2f ms  p99=%.2f ms  max=%.2f ms   (danh nghĩa %.1f ms)\n",
               name, Pct(v, 0.50) / 1000.0, Pct(v, 0.99) / 1000.0,
               Pct(v, 1.00) / 1000.0, nominal_ms);
    };
    printf("=== 1. NHỊP VÒNG ĐIỀU KHIỂN (jitter) ===\n");
    report_period("chu kỳ lowcmd", g_cmd_periods, 2.0);
    report_period("chu kỳ lowstate", g_state_periods, 2.0);
    report_period("tuổi lowcmd", g_cmd_age, 0.0);
    printf("  -> p99 chu kỳ lowcmd lệch nhiều so với 2 ms = run_r1 bị trượt nhịp.\n\n");

    // Thống kê sai số bám lệnh và bão hòa khớp.
    printf("=== 2. TỪNG KHỚP ===\n");
    printf("  %-12s %8s %8s %8s %8s %7s   %s\n", "khớp", "err_rms", "err_max", "trễ",
           "tau_max", "%bão hoà", "ghi chú");
    printf("  %-12s %8s %8s %8s %8s %7s\n", "", "(rad)", "(rad)", "(ms)", "(N·m)", "");

    for (int i = 0; i < spec::kNumJoints; ++i) {
        double se = 0.0, emax = 0.0, taumax = 0.0;
        size_t nsat = 0;
        double qd_mean = 0.0;
        for (size_t k = 0; k < N; ++k) {
            double e = g_samples[k].q[i] - g_samples[k].qd[i];
            se += e * e;
            emax = std::max(emax, std::fabs(e));
            double ta = std::fabs(g_samples[k].tau[i]);
            taumax = std::max(taumax, ta);
            if (ta > 0.9 * kEffortLimit[i]) nsat++;
            qd_mean += g_samples[k].qd[i];
        }
        qd_mean /= static_cast<double>(N);
        double err_rms = std::sqrt(se / static_cast<double>(N));

        // Biên độ lệnh: khớp gần như đứng yên thì đo trễ vô nghĩa.
        double qd_var = 0.0;
        for (size_t k = 0; k < N; ++k) {
            double d = g_samples[k].qd[i] - qd_mean;
            qd_var += d * d;
        }
        double qd_std = std::sqrt(qd_var / static_cast<double>(N));

        // Ước lượng trễ pha vòng kín thông qua lag tương quan lớn nhất.
        int best_lag = -1;
        double best_cost = 1e30;
        if (qd_std > 0.02) {
            for (int lag = 0; lag <= kMaxLag; ++lag) {
                double c = 0.0;
                for (size_t k = static_cast<size_t>(lag); k < N; ++k) {
                    double d = g_samples[k].q[i] - g_samples[k - lag].qd[i];
                    c += d * d;
                }
                c /= static_cast<double>(N - lag);
                if (c < best_cost) { best_cost = c; best_lag = lag; }
            }
        }

        double sat_pct = 100.0 * static_cast<double>(nsat) / static_cast<double>(N);
        char lag_s[16];
        if (best_lag < 0) snprintf(lag_s, sizeof(lag_s), "  --");
        else snprintf(lag_s, sizeof(lag_s), "%5.1f", best_lag * 2.0);  // 1 mẫu = 2 ms

        const char* note = "";
        if (sat_pct > 1.0) note = "<< BÃO HOÀ MOMENT";
        else if (best_lag * 2.0 > 20.0) note = "<< trễ nhiều";
        else if (err_rms > 0.08) note = "<< bám kém";

        printf("  %-12s %8.4f %8.4f %8s %8.1f %7.1f   %s\n", kJointName[i], err_rms, emax,
               lag_s, taumax, sat_pct, note);
    }

    // Đo lường biến thiên vận tốc khớp dq để đánh giá độ nhiễu.
    printf("\n=== 3. NHIỄU dq (so với ±0.5 rad/s mà train giả định) ===\n");
    double worst = 0.0;
    int worst_i = 0;
    for (int i = 0; i < spec::kNumJoints; ++i) {
        double s2 = 0.0;
        for (size_t k = 1; k < N; ++k) {
            double d = g_samples[k].dq[i] - g_samples[k - 1].dq[i];
            s2 += d * d;
        }
        double step_rms = std::sqrt(s2 / static_cast<double>(N - 1));
        if (step_rms > worst) { worst = step_rms; worst_i = i; }
    }
    printf("  Biến thiên dq giữa 2 mẫu liên tiếp, lớn nhất: %s = %.3f rad/s\n",
           kJointName[worst_i], worst);
    printf("  -> nếu < 0.5 thì nhiễu nằm trong dải train đã quen: ĐỪNG bật joint_vel_lpf_hz.\n");

    return 0;
}
