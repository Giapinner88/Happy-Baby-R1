// sensor_log - Ghi CSV toàn bộ motor + cảm biến của R1 từ DDS (CHỈ NGHE, không publish gì).
//
// Nguồn: rt/lowstate (~1 kHz trên R1: IMU, 35 slot motor, tay cầm), rt/lowcmd (lệnh run_r1 đang
// gửi, nếu có), rt/lf/bmsstate (pin, ~20 Hz). Mỗi mẫu lowstate (sau --decimate) là một
// dòng CSV; lowcmd/bms là bản mới nhất tại thời điểm đó, kèm tuổi (ms; -1 = chưa nhận).
// Callback DDS chỉ copy vào hàng đợi; luồng chính format và ghi file. Cạnh file CSV là
// <out>.meta.json: lệnh, thời gian, map motor -> IDL, đơn vị, số mẫu.
//
//   sensor_log [--iface eth10] [--domain 0] [--out /tmp/r1_sensors.csv]
//              [--duration S] [--decimate N] [--all-motors] [--no-lowcmd] [--no-bms]
//
// Chạy từ máy trạm: ./hb record (xem docs/operation.md).

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "../src/config/RobotSpec.hpp"

using BmsState_ = unitree_hg::msg::dds_::BmsState_;
using LowCmd_ = unitree_hg::msg::dds_::LowCmd_;
using LowState_ = unitree_hg::msg::dds_::LowState_;

namespace {

const char* kJointName[spec::kNumJoints] = {
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch",
    "left_ankle_roll", "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
    "right_ankle_pitch", "right_ankle_roll", "waist_roll", "waist_yaw",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_roll", "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll"};

struct Motor {
    std::string name;
    int idl;
};

// 24 khớp policy + 2 khớp đầu; --all-motors thêm các slot IDL còn lại (idlNN).
std::vector<Motor> MotorList(bool all) {
    std::vector<Motor> motors;
    bool used[spec::kNumMotorsIdl] = {};
    for (int i = 0; i < spec::kNumJoints; ++i) {
        motors.push_back({kJointName[i], spec::MotorIdl(i)});
        used[spec::MotorIdl(i)] = true;
    }
    motors.push_back({"head_pitch", spec::kHeadPitchIdl});
    motors.push_back({"head_yaw", spec::kHeadYawIdl});
    used[spec::kHeadPitchIdl] = used[spec::kHeadYawIdl] = true;
    if (all) {
        for (int idl = 0; idl < spec::kNumMotorsIdl; ++idl) {
            if (!used[idl]) motors.push_back({"idl" + std::to_string(idl), idl});
        }
    }
    return motors;
}

// Gói tay cầm R3 trong LowState_.wireless_remote (giống tools/record_motion.cpp).
struct RemoteRx {
    uint8_t head[2];
    uint16_t buttons;
    float lx, rx, ry, l2, ly;
};

struct Row {
    int64_t t_us;
    double unix_s;
    LowState_ state;
    LowCmd_ cmd;
    int64_t cmd_age_us;  // -1 = chưa nhận lowcmd
    BmsState_ bms;
    int64_t bms_age_us;
};

std::atomic<bool> g_running{true};
void OnSignal(int) { g_running = false; }

int64_t NowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

double UnixS() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void WriteHeader(FILE* f, const std::vector<Motor>& motors, bool with_cmd, bool with_bms) {
    fprintf(f, "sample,t_s,unix_s,tick,mode_pr,mode_machine,version0,version1,crc");
    fprintf(f, ",imu_qw,imu_qx,imu_qy,imu_qz,imu_gyro_x,imu_gyro_y,imu_gyro_z"
               ",imu_acc_x,imu_acc_y,imu_acc_z,imu_roll,imu_pitch,imu_yaw,imu_temp");
    fprintf(f, ",remote_buttons,remote_lx,remote_rx,remote_ry,remote_l2,remote_ly");
    for (int k = 0; k < 4; ++k) fprintf(f, ",state_reserve%d", k);
    for (const Motor& m : motors) {
        const char* n = m.name.c_str();
        fprintf(f, ",%s_mode,%s_q,%s_dq,%s_ddq,%s_tau_est,%s_temp0,%s_temp1,%s_vol"
                   ",%s_sensor0,%s_sensor1,%s_motorstate",
                n, n, n, n, n, n, n, n, n, n, n);
    }
    if (with_cmd) {
        fprintf(f, ",cmd_age_ms,cmd_mode_pr,cmd_mode_machine");
        for (const Motor& m : motors) {
            const char* n = m.name.c_str();
            fprintf(f, ",cmd_%s_mode,cmd_%s_q,cmd_%s_dq,cmd_%s_tau,cmd_%s_kp,cmd_%s_kd",
                    n, n, n, n, n, n);
        }
    }
    if (with_bms) {
        fprintf(f, ",bms_age_ms,bms_soc,bms_soh,bms_current,bms_cycle,bms_version_high"
                   ",bms_version_low,bms_fn");
        for (int k = 0; k < 3; ++k) fprintf(f, ",bms_voltage%d", k);
        for (int k = 0; k < 12; ++k) fprintf(f, ",bms_temp%d", k);
        for (int k = 0; k < 5; ++k) fprintf(f, ",bms_state%d", k);
        for (int k = 0; k < 40; ++k) fprintf(f, ",bms_cell_vol%d", k);
    }
    fprintf(f, "\n");
}

void WriteRow(FILE* f, uint64_t sample, int64_t t0_us, const Row& r,
              const std::vector<Motor>& motors, bool with_cmd, bool with_bms) {
    const LowState_& s = r.state;
    const auto& imu = s.imu_state();
    fprintf(f, "%llu,%.6f,%.6f,%u,%u,%u,%u,%u,%u", static_cast<unsigned long long>(sample),
            (r.t_us - t0_us) / 1e6, r.unix_s, s.tick(), s.mode_pr(), s.mode_machine(),
            s.version()[0], s.version()[1], s.crc());
    fprintf(f, ",%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%d",
            imu.quaternion()[0], imu.quaternion()[1], imu.quaternion()[2], imu.quaternion()[3],
            imu.gyroscope()[0], imu.gyroscope()[1], imu.gyroscope()[2],
            imu.accelerometer()[0], imu.accelerometer()[1], imu.accelerometer()[2],
            imu.rpy()[0], imu.rpy()[1], imu.rpy()[2], imu.temperature());
    RemoteRx remote{};
    std::memcpy(&remote, s.wireless_remote().data(), sizeof(remote));
    fprintf(f, ",%u,%.5g,%.5g,%.5g,%.5g,%.5g", remote.buttons, remote.lx, remote.rx, remote.ry,
            remote.l2, remote.ly);
    for (int k = 0; k < 4; ++k) fprintf(f, ",%u", s.reserve()[k]);
    for (const Motor& m : motors) {
        const auto& ms = s.motor_state()[m.idl];
        fprintf(f, ",%u,%.7g,%.7g,%.7g,%.7g,%d,%d,%.5g,%u,%u,%u", ms.mode(), ms.q(), ms.dq(),
                ms.ddq(), ms.tau_est(), ms.temperature()[0], ms.temperature()[1], ms.vol(),
                ms.sensor()[0], ms.sensor()[1], ms.motorstate());
    }
    if (with_cmd) {
        if (r.cmd_age_us < 0) {
            fprintf(f, ",-1,,");
            for (size_t i = 0; i < motors.size(); ++i) fprintf(f, ",,,,,,");
        } else {
            fprintf(f, ",%.3f,%u,%u", r.cmd_age_us / 1000.0, r.cmd.mode_pr(), r.cmd.mode_machine());
            for (const Motor& m : motors) {
                const auto& mc = r.cmd.motor_cmd()[m.idl];
                fprintf(f, ",%u,%.7g,%.7g,%.7g,%.7g,%.7g", mc.mode(), mc.q(), mc.dq(), mc.tau(),
                        mc.kp(), mc.kd());
            }
        }
    }
    if (with_bms) {
        const BmsState_& b = r.bms;
        if (r.bms_age_us < 0) {
            fprintf(f, ",-1");
            for (int k = 0; k < 7 + 3 + 12 + 5 + 40; ++k) fprintf(f, ",");
        } else {
            fprintf(f, ",%.3f,%u,%u,%d,%u,%u,%u,%u", r.bms_age_us / 1000.0, b.soc(), b.soh(),
                    b.current(), b.cycle(), b.version_high(), b.version_low(), b.fn());
            for (int k = 0; k < 3; ++k) fprintf(f, ",%u", b.bmsvoltage()[k]);
            for (int k = 0; k < 12; ++k) fprintf(f, ",%d", b.temperature()[k]);
            for (int k = 0; k < 5; ++k) fprintf(f, ",%u", b.bmsstate()[k]);
            for (int k = 0; k < 40; ++k) fprintf(f, ",%u", b.cell_vol()[k]);
        }
    }
    fprintf(f, "\n");
}

std::string JsonEscape(const std::string& s) {
    std::string out;
    for (const char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out;
}

int Usage() {
    fprintf(stderr,
            "usage: sensor_log [--iface eth10] [--domain 0] [--out /tmp/r1_sensors.csv]\n"
            "                  [--duration S] [--decimate N] [--all-motors] [--no-lowcmd] [--no-bms]\n");
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    std::string iface = "eth10";
    std::string out = "/tmp/r1_sensors.csv";
    int domain = 0;
    double duration_s = 0.0;  // 0 = tới khi Ctrl-C
    int decimate = 1;
    bool all_motors = false, with_cmd = true, with_bms = true;
    std::string command_line;
    for (int i = 0; i < argc; ++i) command_line += (i ? " " : "") + std::string(argv[i]);
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto value = [&]() -> std::string {
            if (i + 1 >= argc) { Usage(); std::exit(2); }
            return argv[++i];
        };
        if (a == "--iface") iface = value();
        else if (a == "--domain") domain = std::stoi(value());
        else if (a == "--out") out = value();
        else if (a == "--duration") duration_s = std::stod(value());
        else if (a == "--decimate") decimate = std::stoi(value());
        else if (a == "--all-motors") all_motors = true;
        else if (a == "--no-lowcmd") with_cmd = false;
        else if (a == "--no-bms") with_bms = false;
        else {
            fprintf(stderr, "sensor_log: tham so khong hop le '%s'\n", a.c_str());
            return Usage();
        }
    }
    if (decimate < 1 || duration_s < 0.0) return Usage();

    const std::vector<Motor> motors = MotorList(all_motors);
    FILE* f = fopen(out.c_str(), "w");
    if (!f) { perror(out.c_str()); return 1; }
    static char file_buffer[1 << 20];
    setvbuf(f, file_buffer, _IOFBF, sizeof(file_buffer));
    WriteHeader(f, motors, with_cmd, with_bms);

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);
    std::signal(SIGHUP, OnSignal);
    unitree::robot::ChannelFactory::Instance()->Init(domain, iface);

    std::mutex mtx;
    std::vector<Row> queue;
    LowCmd_ last_cmd{};
    int64_t last_cmd_us = -1;
    BmsState_ last_bms{};
    int64_t last_bms_us = -1;
    uint64_t state_msgs = 0, cmd_msgs = 0, bms_msgs = 0;

    unitree::robot::ChannelSubscriber<LowCmd_> sub_cmd("rt/lowcmd");
    if (with_cmd) {
        sub_cmd.InitChannel([&](const void* msg) {
            std::lock_guard<std::mutex> lk(mtx);
            last_cmd = *static_cast<const LowCmd_*>(msg);
            last_cmd_us = NowUs();
            ++cmd_msgs;
        });
    }
    unitree::robot::ChannelSubscriber<BmsState_> sub_bms("rt/lf/bmsstate");
    if (with_bms) {
        sub_bms.InitChannel([&](const void* msg) {
            std::lock_guard<std::mutex> lk(mtx);
            last_bms = *static_cast<const BmsState_*>(msg);
            last_bms_us = NowUs();
            ++bms_msgs;
        });
    }
    unitree::robot::ChannelSubscriber<LowState_> sub_state("rt/lowstate");
    sub_state.InitChannel([&](const void* msg) {
        const int64_t t = NowUs();
        std::lock_guard<std::mutex> lk(mtx);
        if (state_msgs++ % static_cast<uint64_t>(decimate) != 0) return;
        Row r;
        r.t_us = t;
        r.unix_s = UnixS();
        r.state = *static_cast<const LowState_*>(msg);
        r.cmd = last_cmd;
        r.cmd_age_us = last_cmd_us < 0 ? -1 : t - last_cmd_us;
        r.bms = last_bms;
        r.bms_age_us = last_bms_us < 0 ? -1 : t - last_bms_us;
        queue.push_back(std::move(r));
    });

    const double start_unix = UnixS();
    const int64_t start_us = NowUs();
    int64_t t0_us = -1;
    uint64_t written = 0;
    std::vector<Row> batch;
    printf("[sensor_log] Nghe rt/lowstate%s%s tren %s domain %d -> %s (%zu motor, decimate %d). "
           "Ctrl-C de dung.\n",
           with_cmd ? " + rt/lowcmd" : "", with_bms ? " + rt/lf/bmsstate" : "", iface.c_str(),
           domain, out.c_str(), motors.size(), decimate);
    int64_t last_print_us = start_us;
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        {
            std::lock_guard<std::mutex> lk(mtx);
            batch.swap(queue);
        }
        for (const Row& r : batch) {
            if (t0_us < 0) t0_us = r.t_us;
            WriteRow(f, written++, t0_us, r, motors, with_cmd, with_bms);
        }
        batch.clear();
        const int64_t now = NowUs();
        if (duration_s > 0.0 && (now - start_us) / 1e6 >= duration_s) g_running = false;
        if (now - last_print_us >= 1000000) {
            last_print_us = now;
            printf("\r  %llu dong, %.1f s   ", static_cast<unsigned long long>(written),
                   (now - start_us) / 1e6);
            fflush(stdout);
        }
    }
    {
        std::lock_guard<std::mutex> lk(mtx);
        batch.swap(queue);
    }
    for (const Row& r : batch) {
        if (t0_us < 0) t0_us = r.t_us;
        WriteRow(f, written++, t0_us, r, motors, with_cmd, with_bms);
    }
    fclose(f);
    const double elapsed_s = (NowUs() - start_us) / 1e6;

    const std::string meta_path = out + ".meta.json";
    if (FILE* m = fopen(meta_path.c_str(), "w")) {
        char host[256] = {};
        gethostname(host, sizeof(host) - 1);
        std::lock_guard<std::mutex> lk(mtx);
        fprintf(m, "{\n  \"tool\": \"controller/tools/sensor_log.cpp\",\n");
        fprintf(m, "  \"command\": \"%s\",\n  \"host\": \"%s\",\n", JsonEscape(command_line).c_str(),
                JsonEscape(host).c_str());
        fprintf(m, "  \"iface\": \"%s\",\n  \"domain\": %d,\n", iface.c_str(), domain);
        fprintf(m, "  \"start_unix_s\": %.6f,\n  \"elapsed_s\": %.3f,\n", start_unix, elapsed_s);
        fprintf(m, "  \"decimate\": %d,\n  \"rows\": %llu,\n", decimate,
                static_cast<unsigned long long>(written));
        fprintf(m, "  \"messages\": {\"rt/lowstate\": %llu, \"rt/lowcmd\": %llu, "
                   "\"rt/lf/bmsstate\": %llu},\n",
                static_cast<unsigned long long>(state_msgs), static_cast<unsigned long long>(cmd_msgs),
                static_cast<unsigned long long>(bms_msgs));
        fprintf(m, "  \"motors\": [");
        for (size_t i = 0; i < motors.size(); ++i)
            fprintf(m, "%s{\"name\": \"%s\", \"idl\": %d}", i ? ", " : "", motors[i].name.c_str(),
                    motors[i].idl);
        fprintf(m, "],\n");
        fprintf(m, "  \"units\": {\"t_s\": \"s, steady clock from first row\", "
                   "\"q\": \"rad\", \"dq\": \"rad/s\", \"ddq\": \"rad/s^2\", \"tau_est\": \"N*m\", "
                   "\"imu_gyro\": \"rad/s, IMU body frame\", \"imu_acc\": \"m/s^2\", "
                   "\"imu_rpy\": \"rad\", \"imu_q\": \"wxyz\", "
                   "\"cmd_age_ms\": \"ms since last rt/lowcmd, -1 = none received\", "
                   "\"bms_age_ms\": \"ms since last rt/lf/bmsstate, -1 = none received\", "
                   "\"other\": \"raw unitree_hg SDK fields, units as published by the robot\"}\n}\n");
        fclose(m);
    }
    printf("\n[sensor_log] %llu dong trong %.1f s -> %s (+ .meta.json)\n",
           static_cast<unsigned long long>(written), elapsed_s, out.c_str());
    if (with_cmd && cmd_msgs == 0) printf("[sensor_log] Khong nhan rt/lowcmd (run_r1 chua gui lenh).\n");
    if (state_msgs == 0) printf("[sensor_log] KHONG nhan rt/lowstate: sai --iface/--domain?\n");
    fflush(stdout);
    std::_Exit(state_msgs == 0 ? 1 : 0);
}
