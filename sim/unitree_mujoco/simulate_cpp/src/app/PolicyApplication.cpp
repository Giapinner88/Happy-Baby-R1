#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <filesystem>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "app/PolicyApplication.hpp"

#include <dds/dds.hpp>
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/go2/SportModeState_.hpp"

#include "runtime/PolicyRunner.hpp"
#include "controllers/locomotion/FlatController.hpp"
#include "controllers/locomotion/FlatPlusController.hpp"
#include "controllers/locomotion/FlatPlusH5Controller.hpp"
#include "controllers/locomotion/FlatPlusGaitController.hpp"
#include "controllers/locomotion/GaitModeScheduler.hpp"
#include "controllers/locomotion/GaitModeSchedulerV3.hpp"
#include "controllers/locomotion/RoughController.hpp"
#include "controllers/arma/ArmaController.hpp"
#include "controllers/slope_meta/SlopeMetaController.hpp"
#include "controllers/rma_meta/RmaMetaController.hpp"
#include "controllers/motion/DanceController.hpp"
#include "controllers/motion/SitController.hpp"
#include "runtime/Tuning.hpp"
#include "simulator/SimulatorSceneResolver.hpp"
#include "motion/LocalAudioPlayer.hpp"
#include "motion/ArmGesturePlayer.hpp"
#include "motion/GestureLoader.hpp"
#include "input/TeleopReceiver.hpp"

LocalAudioPlayer g_audio;

// --- Overlay động tác tay khi đang Flat (locomotion) ---
ArmGesturePlayer g_gesture;
TeleopReceiver   g_teleop;                   // teleop UDP (tay + đầu), ưu tiên hơn gesture
std::atomic<bool> g_gesture_toggle{false};   // phím V (rising-edge) đặt = true
std::atomic<bool> g_teleop_armed{false};     // phím T: arm/disarm teleop (mặc định TẮT)
std::atomic<bool> g_arm_overlay_supported{false};
// Bù thăng bằng feedforward khi overlay gesture: obs[3] += Kg*lean, obs[6] -= Kv*lean.
// Hệ số Kg/Kv đọc từ Tuning::Get() (config, tune được không cần build lại).

// Gesture đã nạp (thật, từ folder deploy) + chọn/cycle bằng phím G, chơi bằng phím V.
std::vector<std::string> g_gesture_names;
std::atomic<int>  g_gesture_sel{0};
std::atomic<bool> g_gesture_cycle{false};    // phím G (rising-edge): chọn gesture kế

// ── Autotest: chạy không cần bàn phím, đo trôi thân khi chơi 1 gesture rồi xuất CSV ──
bool        g_autotest      = false;
std::string g_at_name;                       // tên gesture cần test
std::string g_at_file;                       // NPZ bất kỳ, dùng cho --autotest-file
float       g_at_vx         = 0.0f;          // vận tốc tiến khi test (0 = đứng tại chỗ)
float       g_at_vy         = 0.0f;          // vận tốc ngang khi test
float       g_at_yaw        = 0.0f;          // vận tốc quay khi test
float       g_at_warmup_s   = 2.5f;          // đi/đứng ổn định trước khi kích gesture
float       g_at_hold_s     = 8.0f;          // thời gian đo sau khi kích
float       g_at_stop_after_s = -1.0f;       // >=0: trả lệnh về 0 sau N giây đo
std::string g_at_csv;                        // file CSV xuất (rỗng = không ghi)
std::string g_at_sequence;                   // kịch bản locomotion liên tục (flat_demo/mixed_shuttle)
bool        g_at_gesture    = true;          // gesture=0 -> chỉ đo locomotion, không kích tay
bool        g_at_meta_trace = false;         // meta_trace=1 -> nối obs/action/target vào CSV
float       g_at_max_tilt_deg = 15.0f;
float       g_at_max_height_drop_m = 0.20f;
float       g_at_max_stand_drift_m = 0.25f;
int         g_dance_test_policy = 0;       // test-only: --dance-test=2..5

struct AutotestSample {
    std::array<float, 14> motion;
    std::string meta_selected;
    bool meta_safety_hold = false;
    std::vector<float> meta_probabilities;
    std::vector<float> meta_weights;
    std::vector<float> actor_observation;
    std::vector<float> policy_action;
    std::vector<float> joint_target;
};

#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <X11/Xutil.h>

using namespace org::eclipse::cyclonedds;
using namespace unitree_hg::msg::dds_;
using namespace unitree_go::msg::dds_;
using namespace unitree::robot;

// DDS Global Variables
LowState_ robot_state;
SportModeState_ current_sport_state;
std::mutex state_mutex;
bool got_first_state = false;
bool got_first_sport_state = false;

// Controller Variables
float target_vx = 0.0f;
float target_vy = 0.0f;
float target_yaw = 0.0f;
bool running = true;
int active_policy = 1; // 0: Sit, 1: Flat, 2-5: Dance
bool policy_changed = true;
std::atomic<int> requested_policy{1}; // Keyboard requests; control loop owns active_policy.
bool high_speed_mode = false;
bool safe_stop_mode = false;   // Chế độ an toàn bị động: policy giữ chạy, v=0
bool pending_sit = false;      // Đang chờ settle rồi chuyển sang Sit
float pending_sit_timer = 0.0f;
constexpr float kSafeShutdownSettleTime = 1.5f; // giây chờ robot dừng trước khi ngồi

enum class RobotState {
    RUNNING_POLICY,
    FALLEN
};
RobotState current_robot_state = RobotState::RUNNING_POLICY;
LocomotionProfile g_locomotion_profile;

// DDS Handlers
void LowStateHandler(const void* message) {
    std::lock_guard<std::mutex> lock(state_mutex);
    robot_state = *(LowState_*)message;
    got_first_state = true;
}

void SportModeStateHandler(const void* message) {
    std::lock_guard<std::mutex> lock(state_mutex);
    current_sport_state = *(SportModeState_*)message;
    got_first_sport_state = true;
}

// X11 Keyboard Thread
void KeyboardThread() {
    Display *d = XOpenDisplay(NULL);
    if (d != NULL) {
        std::cout << "[GUI] Đã mở cửa sổ điều khiển. Bấm vào 'R1 KEYBOARD CONTROL' để điều khiển.\n";
        Window w = XCreateSimpleWindow(d, RootWindow(d, 0), 10, 10, 350, 250, 1, BlackPixel(d, 0), WhitePixel(d, 0));
        XSelectInput(d, w, ExposureMask | KeyPressMask);
        XMapWindow(d, w);
        XStoreName(d, w, "R1 KEYBOARD CONTROL");

        GC gc = XCreateGC(d, w, 0, NULL);
        XSetForeground(d, gc, BlackPixel(d, 0));

        KeyCode kc_w = XKeysymToKeycode(d, XK_w);
        KeyCode kc_s = XKeysymToKeycode(d, XK_s);
        KeyCode kc_a = XKeysymToKeycode(d, XK_a);
        KeyCode kc_d = XKeysymToKeycode(d, XK_d);
        KeyCode kc_q = XKeysymToKeycode(d, XK_q);
        KeyCode kc_e = XKeysymToKeycode(d, XK_e);
        KeyCode kc_1 = XKeysymToKeycode(d, XK_1);
        KeyCode kc_2 = XKeysymToKeycode(d, XK_2);
        KeyCode kc_3 = XKeysymToKeycode(d, XK_3);
        KeyCode kc_4 = XKeysymToKeycode(d, XK_4);
        KeyCode kc_5 = XKeysymToKeycode(d, XK_5);
        KeyCode kc_0 = XKeysymToKeycode(d, XK_0);
        KeyCode kc_9 = XKeysymToKeycode(d, XK_9);
        KeyCode kc_v = XKeysymToKeycode(d, XK_v);
        KeyCode kc_g = XKeysymToKeycode(d, XK_g);
        KeyCode kc_t = XKeysymToKeycode(d, XK_t);
        KeyCode kc_tab = XKeysymToKeycode(d, XK_Tab);
        KeyCode kc_esc = XKeysymToKeycode(d, XK_Escape);

        XEvent ev;
        char keys_return[32];

        Atom wmDeleteMessage = XInternAtom(d, "WM_DELETE_WINDOW", False);
        XSetWMProtocols(d, w, &wmDeleteMessage, 1);

        while (running) {
            while (XPending(d)) {
                XNextEvent(d, &ev);
                if (ev.type == Expose) {
                    XClearWindow(d, w);
                    XDrawString(d, w, gc, 20, 30, "--- CUA SO DIEU KHIEN R1 ---", 28);
                    XDrawString(d, w, gc, 20, 60, "1: Flat | 2-5: Dance | 0: Ngoi chu dong", 39);
                    XDrawString(d, w, gc, 20, 80, "9: Safe Shutdown (doi policy dung -> ngoi)", 41);
                    XDrawString(d, w, gc, 20, 100, "W / S     : Tien / Lui", 22);
                    XDrawString(d, w, gc, 20, 120, "A / D     : Sang Trai / Phai", 28);
                    XDrawString(d, w, gc, 20, 140, "Q / E     : Xoay Trai / Phai", 28);
                    XDrawString(d, w, gc, 20, 160, "TAB       : Che do nhanh/cham", 29);
                    XDrawString(d, w, gc, 20, 175, "V         : Vay tay (chi Flat, bam lai=thu ve)", 46);
                    XDrawString(d, w, gc, 20, 190, "T         : Teleop (chi Flat 83D)", 33);
                    XDrawString(d, w, gc, 20, 205, "ESC       : Thoat", 17);

                    std::string mode_str;
                    if (pending_sit) {
                        mode_str = ">> SAFE SHUTDOWN: dang cho robot dung han... <<";
                    } else if (safe_stop_mode) {
                        mode_str = "!!! AN TOAN BI DONG !!! (v=0, policy con chay)";
                    } else if (current_robot_state == RobotState::FALLEN) {
                        mode_str = "!!! FALLEN: MOTOR OFF - NHAN BACKSPACE !!!";
                    } else if (active_policy == 0) {
                        mode_str = "Hien tai: NGOI CHU DONG";
                    } else if (active_policy == 1) {
                        mode_str = std::string("Hien tai: ") + g_locomotion_profile.name
                                   + (high_speed_mode ? " [NHANH]" : " [CHAM]");
                    } else {
                        mode_str = std::string("Hien tai: Dance ") + std::to_string(active_policy - 1)
                                   + (high_speed_mode ? " [NHANH]" : " [CHAM]");
                    }
                    XDrawString(d, w, gc, 20, 238, mode_str.c_str(), mode_str.length());
                }
                else if (ev.type == ClientMessage) {
                    if ((Atom)ev.xclient.data.l[0] == wmDeleteMessage) {
                        running = false;
                        break;
                    }
                }
            }

            XQueryKeymap(d, keys_return);
            bool pressed_w = (keys_return[kc_w >> 3] & (1 << (kc_w & 7))) != 0;
            bool pressed_s = (keys_return[kc_s >> 3] & (1 << (kc_s & 7))) != 0;
            bool pressed_a = (keys_return[kc_a >> 3] & (1 << (kc_a & 7))) != 0;
            bool pressed_d = (keys_return[kc_d >> 3] & (1 << (kc_d & 7))) != 0;
            bool pressed_q = (keys_return[kc_q >> 3] & (1 << (kc_q & 7))) != 0;
            bool pressed_e = (keys_return[kc_e >> 3] & (1 << (kc_e & 7))) != 0;
            bool pressed_1 = (keys_return[kc_1 >> 3] & (1 << (kc_1 & 7))) != 0;
            bool pressed_2 = (keys_return[kc_2 >> 3] & (1 << (kc_2 & 7))) != 0;
            bool pressed_3 = (keys_return[kc_3 >> 3] & (1 << (kc_3 & 7))) != 0;
            bool pressed_4 = (keys_return[kc_4 >> 3] & (1 << (kc_4 & 7))) != 0;
            bool pressed_5 = (keys_return[kc_5 >> 3] & (1 << (kc_5 & 7))) != 0;
            bool pressed_0 = (keys_return[kc_0 >> 3] & (1 << (kc_0 & 7))) != 0;
            bool pressed_9 = (keys_return[kc_9 >> 3] & (1 << (kc_9 & 7))) != 0;
            bool pressed_v = (keys_return[kc_v >> 3] & (1 << (kc_v & 7))) != 0;
            bool pressed_g = (keys_return[kc_g >> 3] & (1 << (kc_g & 7))) != 0;
            bool pressed_t = (keys_return[kc_t >> 3] & (1 << (kc_t & 7))) != 0;
            bool pressed_tab = (keys_return[kc_tab >> 3] & (1 << (kc_tab & 7))) != 0;
            bool pressed_esc = (keys_return[kc_esc >> 3] & (1 << (kc_esc & 7))) != 0;

            if (pressed_esc) {
                running = false;
                break;
            }

            if (pressed_1 && requested_policy.load() != 1) {
                requested_policy.store(1); g_audio.Stop();
                safe_stop_mode = false;  // thoát safe stop khi chuyển policy
                XClearArea(d, w, 0, 0, 0, 0, True);
            }
            if (pressed_2 && requested_policy.load() != 2) {
                requested_policy.store(2); safe_stop_mode = false;
                XClearArea(d, w, 0, 0, 0, 0, True);
            }
            if (pressed_3 && requested_policy.load() != 3) {
                requested_policy.store(3); safe_stop_mode = false;
                XClearArea(d, w, 0, 0, 0, 0, True);
            }
            if (pressed_4 && requested_policy.load() != 4) {
                requested_policy.store(4); safe_stop_mode = false;
                XClearArea(d, w, 0, 0, 0, 0, True);
            }
            if (pressed_5 && requested_policy.load() != 5) {
                requested_policy.store(5); safe_stop_mode = false;
                XClearArea(d, w, 0, 0, 0, 0, True);
            }
            if (pressed_0 && active_policy != 0) {
                active_policy = 0; requested_policy.store(0); policy_changed = true; safe_stop_mode = false;
                auto& t = Tuning::Get();
                if (!t.audio_sit.empty()) g_audio.Play(t.audio_dir + t.audio_sit);
                XClearArea(d, w, 0, 0, 0, 0, True);
            }
            // Phím 9: SAFE SHUTDOWN (giống high_level_2 kSafeShutdown)
            // Bước 1: ép v=0, giữ policy chạy (robot tự dừng vững)
            // Bước 2: sau kSettleTime giây → tự động chuyển sang SitController (phím 0)
            static bool last_9 = false;
            if (pressed_9 && !last_9 && !pending_sit && active_policy != 0) {
                pending_sit = true;
                pending_sit_timer = 0.0f;
                safe_stop_mode = false;
                target_vx = target_vy = target_yaw = 0.0f;
                std::cout << "[SafeShutdown] Eph v=0, cho robot dung ("
                          << kSafeShutdownSettleTime << "s) roi ngoi xuong..." << std::endl;
                XClearArea(d, w, 0, 0, 0, 0, True);
            }
            last_9 = pressed_9;

            static bool last_tab = false;
            if (pressed_tab && !last_tab) {
                high_speed_mode = !high_speed_mode;
                const auto& limit = high_speed_mode ? g_locomotion_profile.fast
                                                    : g_locomotion_profile.slow;
                std::cout << "[Command] profile=" << g_locomotion_profile.name
                          << " mode=" << (high_speed_mode ? "FAST" : "SLOW")
                          << " limits=(vx=" << limit[0] << ", vy=" << limit[1]
                          << ", yaw=" << limit[2] << ")" << std::endl;
                XClearArea(d, w, 0, 0, 0, 0, True);
            }
            last_tab = pressed_tab;

            // Phím V: chơi/thu gesture đang chọn (chỉ có tác dụng khi đang Flat).
            static bool last_v = false;
            if (pressed_v && !last_v) g_gesture_toggle.store(true);
            last_v = pressed_v;

            // Phím G: chọn gesture kế tiếp trong danh sách đã nạp.
            static bool last_g = false;
            if (pressed_g && !last_g) g_gesture_cycle.store(true);
            last_g = pressed_g;

            // Phím T: arm/disarm teleop (song song L2+Phải trên robot thật).
            static bool last_t = false;
            if (pressed_t && !last_t) {
                if (!g_arm_overlay_supported.load()) {
                    g_teleop_armed.store(false);
                    std::cout << "[Teleop] Phim T bi tu choi: controller hien tai "
                                 "khong ho tro arm overlay." << std::endl;
                } else {
                    bool now_on = !g_teleop_armed.load();
                    g_teleop_armed.store(now_on);
                    std::cout << "[Teleop] "
                              << (now_on ? "ARM (phim T)" : "DISARM (phim T)")
                              << std::endl;
                }
            }
            last_t = pressed_t;

            const auto& speed = high_speed_mode ? g_locomotion_profile.fast
                                                : g_locomotion_profile.slow;
            const bool command_blocked = pending_sit || safe_stop_mode ||
                                         current_robot_state == RobotState::FALLEN;
            float next_vx = 0.0f;
            float next_vy = 0.0f;
            float next_yaw = 0.0f;
            // pending_sit: đang chờ settle — khóa WASD, v=0
            if (!command_blocked) {
                if (pressed_w) next_vx += speed[0];
                if (pressed_s) next_vx -= speed[0];
                if (pressed_a) next_vy += speed[1];
                if (pressed_d) next_vy -= speed[1];
                if (pressed_q) next_yaw += speed[2];
                if (pressed_e) next_yaw -= speed[2];
            }

            static float last_logged_vx = 0.0f;
            static float last_logged_vy = 0.0f;
            static float last_logged_yaw = 0.0f;
            static bool last_logged_blocked = false;
            const bool command_changed = next_vx != last_logged_vx ||
                                         next_vy != last_logged_vy ||
                                         next_yaw != last_logged_yaw ||
                                         command_blocked != last_logged_blocked;
            if (command_changed) {
                std::cout << "[Command] profile=" << g_locomotion_profile.name
                          << " mode=" << (high_speed_mode ? "FAST" : "SLOW")
                          << " cmd=(" << next_vx << ", " << next_vy << ", "
                          << next_yaw << ")";
                if (command_blocked) {
                    std::cout << " BLOCKED="
                              << (current_robot_state == RobotState::FALLEN ? "FALLEN" : "SAFE_STOP");
                }
                std::cout << std::endl;
                last_logged_vx = next_vx;
                last_logged_vy = next_vy;
                last_logged_yaw = next_yaw;
                last_logged_blocked = command_blocked;
            }
            target_vx = next_vx;
            target_vy = next_vy;
            target_yaw = next_yaw;

            usleep(20000); // 50Hz check
        }
        XFreeGC(d, gc);
        XDestroyWindow(d, w);
        XCloseDisplay(d);
    }
}

// Heading-hold: giữ hướng khi đi thẳng (khớp Application::ApplyHeadingHold deploy).
// Tínhrư đi thẳng hơn, đo gesture sạch hơn.
float g_heading_ref = 0.0f;
bool  g_heading_active = false;
constexpr float kHeadKp = 0.9f, kHeadMaxYaw = 0.4f, kHeadMoveMin = 0.15f, kHeadRelatchGyro = 0.3f;

static float YawFromQuat(float w, float x, float y, float z) {
    return std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}
static float WrapPi(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static float SlewCommand(float current, float target, float accel_rate,
                         float decel_rate, float dt) {
    const float delta = target - current;
    if (std::abs(delta) < 1.0e-7f) return target;
    const float rate = std::abs(target) < std::abs(current)
        ? decel_rate : accel_rate;
    return current + std::clamp(delta, -std::max(0.0f, rate) * dt,
                                std::max(0.0f, rate) * dt);
}

static std::string FlatDemoCommand(float t, float& vx, float& vy, float& yaw) {
    vx = 0.0f; vy = 0.0f; yaw = 0.0f;
    if (t < 2.0f)  return "STAND";
    if (t < 6.0f)  { vx =  0.35f; return "FORWARD"; }
    if (t < 8.0f)  return "STOP_AFTER_FORWARD";
    if (t < 12.0f) { vx = -0.25f; return "BACKWARD"; }
    if (t < 14.0f) return "STOP_AFTER_BACKWARD";
    if (t < 17.0f) { vy =  0.20f; return "STRAFE_LEFT"; }
    if (t < 19.0f) return "STOP_AFTER_LEFT";
    if (t < 22.0f) { vy = -0.20f; return "STRAFE_RIGHT"; }
    if (t < 24.0f) return "STOP_AFTER_RIGHT";
    if (t < 28.0f) { yaw =  0.40f; return "TURN_LEFT"; }
    if (t < 30.0f) return "STOP_AFTER_TURN_LEFT";
    if (t < 34.0f) { yaw = -0.40f; return "TURN_RIGHT"; }
    return "FINAL_STOP";
}

static std::string MixedShuttleCommand(float t, float& vx, float& vy, float& yaw) {
    vx = 0.0f; vy = 0.0f; yaw = 0.0f;
    if (t < 48.0f) { vx = 0.40f; return "OUTBOUND"; }
    if (t < 54.0f) return "TURNAROUND_STOP";
    if (t < 102.0f) { vx = -0.40f; return "INBOUND"; }
    return "FINAL_STOP";
}

int RunPolicyApplication(int argc, char** argv) {
    std::cout << "Loading tuning config...\n";
    const char* tuning_override = std::getenv("SIMULATE_CPP_TUNING_CONFIG");
    const std::string tuning_path = tuning_override && *tuning_override
        ? tuning_override : "../config/tuning.yaml";
    Tuning::Get().Load(tuning_path);

    std::string slope_meta_package;
    std::string rma_meta_package;
    std::string arma_package;
    std::string locomotion_policy_override;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--slope-meta") {
            slope_meta_package = "../policy/locomotion/meta/slope_meta_v6";
        } else if (argument == "--meta-package" && i + 1 < argc) {
            slope_meta_package = argv[++i];
        } else if (argument.rfind("--meta-package=", 0) == 0) {
            slope_meta_package = argument.substr(std::string("--meta-package=").size());
        } else if (argument == "--rma-meta-package" && i + 1 < argc) {
            rma_meta_package = argv[++i];
        } else if (argument.rfind("--rma-meta-package=", 0) == 0) {
            rma_meta_package =
                argument.substr(std::string("--rma-meta-package=").size());
        } else if (argument == "--arma-package" && i + 1 < argc) {
            arma_package = argv[++i];
        } else if (argument.rfind("--arma-package=", 0) == 0) {
            arma_package = argument.substr(std::string("--arma-package=").size());
        } else if (argument == "--locomotion-policy" && i + 1 < argc) {
            locomotion_policy_override = argv[++i];
        } else if (argument.rfind("--locomotion-policy=", 0) == 0) {
            locomotion_policy_override =
                argument.substr(std::string("--locomotion-policy=").size());
        }
    }
    // A-RMA không còn là một policy route riêng: một thư mục bundle được cấu
    // hình ở locomotion_policy chính là Flat policy + adapter + actor tương ứng.
    // CLI --locomotion-policy vẫn có quyền thay thế đường dẫn trong tuning.yaml.
    if (!locomotion_policy_override.empty()) {
        Tuning::Get().flat_policy_path = locomotion_policy_override;
        std::cout << "[Locomotion] CLI policy override: "
                  << locomotion_policy_override << std::endl;
    }
    const std::string locomotion_policy_path = Tuning::Get().LocomotionPolicyPath();
    const bool configured_arma_bundle =
        std::filesystem::is_directory(locomotion_policy_path) &&
        std::filesystem::is_regular_file(
            std::filesystem::path(locomotion_policy_path) / "params" / "deploy.yaml");
    if (configured_arma_bundle &&
        (!slope_meta_package.empty() || !rma_meta_package.empty())) {
        std::cerr << "[Locomotion] tuning.yaml đang trỏ tới A-RMA bundle; "
                     "không thể đồng thời chọn --meta-package/--rma-meta-package."
                  << std::endl;
        return 2;
    }
    if (configured_arma_bundle && !arma_package.empty()) {
        std::cout << "[Locomotion] --arma-package ghi đè bundle A-RMA trong tuning.yaml.\n";
    }
    const std::string arma_package_path =
        arma_package.empty() && configured_arma_bundle
            ? locomotion_policy_path
            : arma_package;
    const int explicit_locomotion_modes =
        static_cast<int>(!slope_meta_package.empty())
        + static_cast<int>(!rma_meta_package.empty())
        + static_cast<int>(!arma_package.empty())
        + static_cast<int>(!locomotion_policy_override.empty());
    if (explicit_locomotion_modes > 1) {
        std::cerr << "[Locomotion] Chỉ dùng một trong --meta-package, "
                     "--rma-meta-package, --arma-package hoặc --locomotion-policy."
                  << std::endl;
        return 2;
    }
    const bool use_slope_meta = !slope_meta_package.empty();
    const bool use_rma_meta = !rma_meta_package.empty();
    const bool use_arma = !arma_package_path.empty();
    const bool use_any_meta = use_slope_meta || use_rma_meta || use_arma;

    // ── Parse autotest CLI: --autotest <ten> hoặc --autotest-file <NPZ>
    // [vx=..] [hold=..] [warmup=..] [csv=..] và ngưỡng fail-closed.
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto val = [&](const std::string& p) { return a.substr(p.size()); };
        if (a == "--autotest" && i + 1 < argc) { g_autotest = true; g_at_name = argv[++i]; }
        else if (a == "--autotest-file" && i + 1 < argc) {
            g_autotest = true;
            g_at_file = argv[++i];
            g_at_name = "__autotest_file__";
        }
        else if (a.rfind("vx=",0)==0)     g_at_vx = std::stof(val("vx="));
        else if (a.rfind("vy=",0)==0)     g_at_vy = std::stof(val("vy="));
        else if (a.rfind("yaw=",0)==0)    g_at_yaw = std::stof(val("yaw="));
        else if (a.rfind("kg=",0)==0)     Tuning::Get().gesture_balance_kg = std::stof(val("kg="));
        else if (a.rfind("kv=",0)==0)     Tuning::Get().gesture_balance_kv = std::stof(val("kv="));
        else if (a.rfind("hold=",0)==0)   g_at_hold_s = std::stof(val("hold="));
        else if (a.rfind("stop_after=",0)==0) g_at_stop_after_s = std::stof(val("stop_after="));
        else if (a.rfind("warmup=",0)==0) g_at_warmup_s = std::stof(val("warmup="));
        else if (a.rfind("csv=",0)==0)    g_at_csv = val("csv=");
        else if (a.rfind("sequence=",0)==0) g_at_sequence = val("sequence=");
        else if (a.rfind("gesture=",0)==0) g_at_gesture = (std::stof(val("gesture=")) != 0.0f);
        else if (a.rfind("meta_trace=",0)==0) g_at_meta_trace = (std::stof(val("meta_trace=")) != 0.0f);
        else if (a.rfind("max_tilt_deg=",0)==0) g_at_max_tilt_deg = std::stof(val("max_tilt_deg="));
        else if (a.rfind("max_drop=",0)==0) g_at_max_height_drop_m = std::stof(val("max_drop="));
        else if (a.rfind("max_drift=",0)==0) g_at_max_stand_drift_m = std::stof(val("max_drift="));
        else if (a.rfind("--dance-test=",0)==0) g_dance_test_policy = std::stoi(val("--dance-test="));
    }
    if (g_dance_test_policy < 0 || g_dance_test_policy > 5) {
        std::cerr << "[DanceTransition] --dance-test must be in [2,5].\n";
        return 2;
    }
    if (g_autotest) {
        std::cout << "[AUTOTEST] gesture='" << g_at_name << "' cmd=("
                  << g_at_vx << ", " << g_at_vy << ", " << g_at_yaw << ")"
                  << " kg=" << Tuning::Get().gesture_balance_kg
                  << " kv=" << Tuning::Get().gesture_balance_kv
                  << " warmup=" << g_at_warmup_s << "s hold=" << g_at_hold_s << "s"
                  << (g_at_sequence.empty() ? "" : (" sequence=" + g_at_sequence))
                  << (g_at_stop_after_s >= 0.0f
                          ? (" stop_after=" + std::to_string(g_at_stop_after_s) + "s")
                          : "")
                  << " max_tilt=" << g_at_max_tilt_deg << "deg"
                  << " max_drop=" << g_at_max_height_drop_m << "m"
                  << (g_at_meta_trace ? " meta_trace=1" : "")
                  << (g_at_csv.empty() ? "" : (" csv=" + g_at_csv)) << "\n";
    }

    std::string networkInterface = "lo";
    setenv("CYCLONEDDS_URI", "file:///home/khanh248/Documents/HB/Mujoco/cyclonedds_lo.xml", 1);
    
    std::cout << "Khởi tạo DDS..." << std::endl;
    ChannelFactory::Instance()->Init(1, networkInterface);
    
    auto lowcmd_publisher = std::make_shared<ChannelPublisher<LowCmd_>>("rt/lowcmd");
    lowcmd_publisher->InitChannel();
    
    auto lowstate_subscriber = std::make_shared<ChannelSubscriber<LowState_>>("rt/lowstate");
    lowstate_subscriber->InitChannel(LowStateHandler, 10);
    
    auto sportstate_subscriber = std::make_shared<ChannelSubscriber<SportModeState_>>("rt/sportmodestate");
    sportstate_subscriber->InitChannel(SportModeStateHandler, 10);

    // Initialize ONNX Env
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "r1_policy");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    // Tự chọn controller theo kích thước input ONNX:
    // 83-D = Flat, 270-D = Rough (height scan), 332/335/415-D = flat history
    // variants (all fail closed on metadata/layout). A-RMA is a bundle and is
    // therefore validated by ArmaRuntime instead of this single-model path.
    int locomotion_obs_dim = FlatController::kObsSize;
    std::string locomotion_policy_contract;
    if (use_arma) {
        // ArmaRuntime validates the bundle's base contract and supplies the
        // final actor input size (e.g. 91/340/423/343).
        locomotion_obs_dim = 91;
    } else if (!use_any_meta) {
        try {
            Ort::Session probe(env, locomotion_policy_path.c_str(), session_options);
            if (probe.GetInputCount() != 1 || probe.GetOutputCount() != 1) {
                std::cerr << "[Locomotion] ONNX phải có đúng 1 input và 1 output: "
                          << locomotion_policy_path << std::endl;
                return 1;
            }
            auto input_dims = probe.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            auto output_dims = probe.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            if (input_dims.size() != 2 || input_dims[1] <= 0 ||
                output_dims.size() != 2 || output_dims[1] != R1Config::NUM_JOINTS) {
                std::cerr << "[Locomotion] Contract ONNX không hợp lệ (cần [1,N] -> [1,24]): "
                          << locomotion_policy_path << std::endl;
                return 1;
            }
            locomotion_obs_dim = static_cast<int>(input_dims[1]);

            // Đọc contract ngay ở bước probe: profile phải được chọn theo cặp
            // (contract, dim), không theo mỗi dim.
            Ort::AllocatorWithDefaultOptions allocator;
            auto metadata = probe.GetModelMetadata();
            auto value = metadata.LookupCustomMetadataMapAllocated(
                "policy_contract", allocator);
            if (value != nullptr) locomotion_policy_contract = value.get();
        } catch (const Ort::Exception& e) {
            std::cerr << "[Locomotion] Không nạp được ONNX '" << locomotion_policy_path
                      << "': " << e.what() << std::endl;
            return 1;
        }
    }

    try {
        g_locomotion_profile = use_arma
            ? Tuning::Get().flat_profile
            : use_any_meta
            ? Tuning::Get().slope_profile
            : Tuning::Get().ResolveLocomotionProfile(locomotion_obs_dim,
                                                     locomotion_policy_contract);
    } catch (const std::exception& error) {
        std::cerr << "[Locomotion] " << error.what() << std::endl;
        return 1;
    }
    const float locomotion_gait_period_s = g_locomotion_profile.gait_period_s;
    bool profile_valid = std::isfinite(locomotion_gait_period_s) &&
                         locomotion_gait_period_s > 0.0f;
    for (float speed : g_locomotion_profile.slow) {
        profile_valid = profile_valid && std::isfinite(speed) && speed >= 0.0f;
    }
    for (float speed : g_locomotion_profile.fast) {
        profile_valid = profile_valid && std::isfinite(speed) && speed >= 0.0f;
    }
    if (!profile_valid) {
        std::cerr << "[Tuning] Profile locomotion không hợp lệ: "
                  << g_locomotion_profile.name << std::endl;
        return 1;
    }
    std::cout << "[Locomotion] profile=" << g_locomotion_profile.name
              << " gait_period=" << locomotion_gait_period_s
              << " slow=(" << g_locomotion_profile.slow[0] << ", "
              << g_locomotion_profile.slow[1] << ", "
              << g_locomotion_profile.slow[2] << ") fast=("
              << g_locomotion_profile.fast[0] << ", "
              << g_locomotion_profile.fast[1] << ", "
              << g_locomotion_profile.fast[2] << ")" << std::endl;

    if (!use_arma && locomotion_obs_dim != FlatController::kObsSize &&
        locomotion_obs_dim != RoughController::kObsSize &&
        locomotion_obs_dim != FlatPlusController::kObsSize &&
        locomotion_obs_dim != FlatPlusH5Controller::kObsSize &&
        locomotion_obs_dim != FlatPlusGaitController::kObsSize) {
        std::cerr << "[Locomotion] Chưa hỗ trợ policy input " << locomotion_obs_dim
                  << "-D; chỉ hỗ trợ 83-D, 270-D, 332-D, 335-D và 415-D." << std::endl;
        return 1;
    }

    FlatController       flat_controller;
    FlatPlusController   flat_plus_controller;
    FlatPlusH5Controller  flat_plus_h5_controller;
    FlatPlusGaitController flat_plus_gait_controller;
    ArmaController        arma_controller;
    SlopeMetaController  slope_meta_controller;
    RmaMetaController    rma_meta_controller;
    std::unique_ptr<RoughController> rough_controller;
    PolicyRunner* locomotion_ptr = nullptr;
    std::string locomotion_controller_name;
    bool rough_scene_auto = false;

    if (use_arma) {
        locomotion_ptr = &arma_controller;
        locomotion_controller_name = "ArmaController (adapter history + actor)";
    } else if (use_rma_meta) {
        locomotion_ptr = &rma_meta_controller;
        locomotion_controller_name =
            "RmaMetaController (RMA adapter + PPO selector + 83-D experts)";
    } else if (use_slope_meta) {
        locomotion_ptr = &slope_meta_controller;
        locomotion_controller_name =
            "SlopeMetaController (registry-driven GRU gate + 83-D experts)";
    } else if (locomotion_obs_dim == RoughController::kObsSize) {
        rough_controller = std::make_unique<RoughController>();
        const std::string& configured_scene = Tuning::Get().rough_scene_path;
        rough_scene_auto = configured_scene.empty() || configured_scene == "auto";
        if (rough_scene_auto) {
            std::cout << "[RoughController] Scene auto: policy sẽ nạp sẵn trước, "
                         "sau đó chờ scene từ simulator R1." << std::endl;
        } else {
            std::cout << "[RoughController] Dùng scene override từ tuning.yaml: "
                      << configured_scene << std::endl;
            if (!rough_controller->LoadScene(configured_scene)) {
                std::cerr << "[RoughController] Dừng để tránh chạy policy với height scan sai."
                          << std::endl;
                return 1;
            }
        }
        locomotion_ptr = rough_controller.get();
        locomotion_controller_name = "RoughController (270-D height scan)";
    } else if (locomotion_obs_dim == FlatPlusController::kObsSize) {
        // 332-D chỉ là điều kiện CẦN. FlatPlusController::Init() còn bắt buộc
        // metadata khai đúng contract/thứ tự/padding, vì một vector 332-D đóng
        // gói time-major cũng vừa khít kích thước này.
        locomotion_ptr = &flat_plus_controller;
        locomotion_controller_name =
            "FlatPlusController (332-D history, term-major oldest->newest)";
    } else if (locomotion_obs_dim == FlatPlusH5Controller::kObsSize) {
        locomotion_ptr = &flat_plus_h5_controller;
        locomotion_controller_name =
            "FlatPlusH5Controller (415-D history, term-major oldest->newest)";
    } else if (locomotion_obs_dim == FlatPlusGaitController::kObsSize) {
        locomotion_ptr = &flat_plus_gait_controller;
        locomotion_controller_name =
            "FlatPlusGaitController (335-D history + current gait one-hot)";
    } else {
        locomotion_ptr = &flat_controller;
        locomotion_controller_name = "FlatController (83-D)";
    }

    try {
        const std::string& selected_policy_path = use_arma
            ? arma_package_path
            : use_rma_meta
            ? rma_meta_package
            : use_slope_meta ? slope_meta_package : locomotion_policy_path;
        locomotion_ptr->Init(selected_policy_path, env, session_options);
    } catch (const std::exception& error) {
        std::cerr << "[Locomotion] Không khởi tạo được controller: "
                  << error.what() << std::endl;
        return 1;
    }
    if (use_arma) locomotion_obs_dim = locomotion_ptr->GetInputSize();
    const bool gait_conditioned = use_arma
        ? arma_controller.GaitConditioned()
        : locomotion_policy_contract == "flat_plus_gait_h4_v1";
    const std::string gait_fsm_contract = gait_conditioned
        ? locomotion_ptr->MetadataValue("gait_fsm_contract") : std::string();
    const std::string gait_task_variant = gait_conditioned
        ? locomotion_ptr->MetadataValue("gait_task_variant") : std::string();
    const bool generic_v3_fg = gait_fsm_contract == "flat_plus_gait_fsm_v3"
        && (gait_task_variant == "F" || gait_task_variant == "G")
        && locomotion_ptr->MetadataValue("gait_fsm_overlay")
               == "gather_first_direction_balanced_v1";
    const bool gait_0917 =
        gait_fsm_contract == "flat_plus_gait_fsm_v1_stand_recovery_v1"
        && (gait_task_variant == "0917"
            || gait_task_variant == "0917V3"
            || gait_task_variant == "0917V4");
    const bool gait_v3 = gait_fsm_contract == "flat_plus_gait_fsm_v3b"
        || gait_fsm_contract == "flat_plus_gait_fsm_v3f" || generic_v3_fg;
    const std::string& selected_policy_path = use_arma
        ? arma_package_path
        : use_rma_meta
        ? rma_meta_package
        : use_slope_meta ? slope_meta_package : locomotion_policy_path;
    std::cout << "[Locomotion] policy: " << selected_policy_path
              << std::endl;
    std::cout << "[Locomotion] input " << locomotion_obs_dim << "-D -> "
              << locomotion_controller_name << std::endl;
    if (gait_conditioned) {
        std::cout << "[GaitMode] fsm_contract="
                  << (gait_fsm_contract.empty() ? "<missing>" : gait_fsm_contract)
                  << " variant="
                  << (gait_task_variant.empty() ? "<missing>" : gait_task_variant)
                  << (gait_v3 ? " (v3 FK/substate/clock)" : " (legacy scheduler)")
                  << std::endl;
    }
    g_arm_overlay_supported.store(locomotion_ptr->SupportsArmOverlay());

    // Khởi gesture: nạp npz từ folder deploy theo thứ tự slot (khớp tay cầm robot).
    // Fallback: demo 'wave' nếu không có npz nào.
    {
        std::array<float, 10> def_arm;
        for (int j = 0; j < 10; ++j) def_arm[j] = R1Config::DEFAULT_JOINT_POS[14 + j];
        constexpr float gspeed = 1.0f;
        g_gesture.Init(def_arm, /*blend_in_s=*/0.4f, /*retract_s=*/1.0f);

        // Nạp gesture THẬT từ folder deploy, THEO ĐÚNG THỨ TỰ SLOT của tay cầm
        // robot (gesture_slot_1..8 = Up/Down/Left/Right/A/B/X/Y) để phím G trong
        // sim duyệt cùng thứ tự người vận hành quen trên robot. Slot trống thì bỏ.
        const std::string gdir = Tuning::Get().gesture_dir;
        std::vector<std::string> want;
        std::vector<int> want_slot;
        for (int sl = 1; sl <= 8; ++sl) {
            const std::string& n = Tuning::Get().gesture_slot[sl - 1];
            if (n.empty()) continue;
            want.push_back(n);
            want_slot.push_back(sl);
        }
        for (size_t wi = 0; wi < want.size(); ++wi) {
            const std::string& name = want[wi];
            const std::pair<std::string,bool> cands[] = {{".loop.npz", true}, {".npz", false}};
            for (const auto& c : cands) {
                float fps = 50.0f; bool ok = false;
                auto frames = LoadGestureNpz(gdir + name + c.first, fps, ok);
                if (!ok || frames.empty()) continue;
                g_gesture.AddGesture(name, std::move(frames), fps * gspeed, c.second);
                g_gesture_names.push_back(name);
                std::cout << "[Gesture] slot " << want_slot[wi] << " = '" << name << "' <- "
                          << (name + c.first) << " (fps=" << fps << " x" << gspeed
                          << ", loop=" << (c.second ? 1 : 0) << ")\n";
                break;
            }
        }
        if (!g_at_file.empty()) {
            float fps = 50.0f;
            bool ok = false;
            auto frames = LoadGestureNpz(g_at_file, fps, ok);
            if (!ok || frames.empty()) {
                std::cerr << "[AUTOTEST] Không nạp được NPZ: " << g_at_file << "\n";
                return 1;
            }
            const bool loop = g_at_file.size() >= 9 &&
                g_at_file.compare(g_at_file.size() - 9, 9, ".loop.npz") == 0;
            g_gesture.AddGesture(g_at_name, std::move(frames), fps * gspeed, loop);
            g_gesture_names.push_back(g_at_name);
            std::cout << "[AUTOTEST] nạp NPZ trực tiếp: " << g_at_file
                      << " (fps=" << fps << ", loop=" << (loop ? 1 : 0) << ")\n";
        }
        if (g_gesture_names.empty()) {   // fallback: demo wave nếu không có npz
            g_gesture.AddGesture("wave", ArmGesturePlayer::MakeDemoWave(def_arm), 50.0f, true);
            g_gesture_names.push_back("wave");
            std::cout << "[Gesture] Khong tim thay npz o " << gdir << " -> dung demo 'wave'.\n";
        }
        std::cout << "[Gesture] San sang. Flat: G=chon gesture, V=choi/thu. Gesture dau: '"
                  << g_gesture_names[0] << "'.\n";

        // Teleop UDP (tay + đầu) — ưu tiên hơn gesture. Test: scripts/teleop_send_test.py.
        g_teleop.Init(def_arm, /*enabled=*/true, /*port=*/5560, /*timeout_ms=*/300.0f,
                      /*blend_in_s=*/0.4f, /*retract_s=*/0.5f, /*smooth_hz=*/8.0f,
                      /*head_yaw_max=*/1.0f, /*head_pitch_max=*/0.6f);
        std::cout << "[Teleop] Lang nghe UDP 5560 (tay+dau) — uu tien hon gesture khi co goi.\n";
    }

    std::string hl2_dance_dir = Tuning::Get().dance_dir;
    const auto& tuning = Tuning::Get();
    std::array<float, R1Config::NUM_JOINTS> zero_joint_dq{};

    MotionData motion_1;
    motion_1.Load(hl2_dance_dir + Tuning::Get().dance_1_npz);
    const int dance_1_legacy_start = motion_1.FindSmoothStartFrame(200);
    const int dance_1_start = tuning.mimic_transition_v2
        ? motion_1.FindTransitionStartFrame(tuning.mimic_transition_search_frames,
                                             R1Config::DEFAULT_JOINT_POS, zero_joint_dq)
        : dance_1_legacy_start;
    DanceController dance_1(
        motion_1, dance_1_start, Tuning::Get().dance_1_speed,
        Tuning::Get().mimic_warmup_s);
    dance_1.ConfigureTransitionV2(tuning.mimic_transition_v2, true,
                                  tuning.mimic_transition_search_frames,
                                  tuning.mimic_transition_clip_ramp_s);
    dance_1.Init(hl2_dance_dir + Tuning::Get().dance_1_onnx, env, session_options);

    MotionData motion_2;
    motion_2.Load(hl2_dance_dir + Tuning::Get().dance_2_npz);
    const int dance_2_legacy_start = motion_2.FindSmoothStartFrame(200);
    const int dance_2_start = tuning.mimic_transition_v2
        ? motion_2.FindTransitionStartFrame(tuning.mimic_transition_search_frames,
                                             R1Config::DEFAULT_JOINT_POS, zero_joint_dq)
        : dance_2_legacy_start;
    DanceController dance_2(
        motion_2, dance_2_start, Tuning::Get().dance_2_speed,
        Tuning::Get().mimic_warmup_s);
    dance_2.ConfigureTransitionV2(tuning.mimic_transition_v2, true,
                                  tuning.mimic_transition_search_frames,
                                  tuning.mimic_transition_clip_ramp_s);
    dance_2.Init(hl2_dance_dir + Tuning::Get().dance_2_onnx, env, session_options);

    MotionData motion_3;
    motion_3.Load(hl2_dance_dir + Tuning::Get().dance_3_npz);
    const int dance_3_legacy_start = motion_3.FindSmoothStartFrame(200);
    const int dance_3_start = tuning.mimic_transition_v2
        ? motion_3.FindTransitionStartFrame(tuning.mimic_transition_search_frames,
                                             R1Config::DEFAULT_JOINT_POS, zero_joint_dq)
        : dance_3_legacy_start;
    DanceController dance_3(
        motion_3, dance_3_start, Tuning::Get().dance_3_speed,
        Tuning::Get().mimic_warmup_s);
    dance_3.ConfigureTransitionV2(tuning.mimic_transition_v2, true,
                                  tuning.mimic_transition_search_frames,
                                  tuning.mimic_transition_clip_ramp_s);
    dance_3.Init(hl2_dance_dir + Tuning::Get().dance_3_onnx, env, session_options);

    MotionData motion_4;
    motion_4.Load(hl2_dance_dir + Tuning::Get().dance_4_npz);
    const int dance_4_legacy_start = motion_4.FindSmoothStartFrame(200);
    const int dance_4_start = tuning.mimic_transition_v2
        ? motion_4.FindTransitionStartFrame(tuning.mimic_transition_search_frames,
                                             R1Config::DEFAULT_JOINT_POS, zero_joint_dq)
        : dance_4_legacy_start;
    DanceController dance_4(
        motion_4, dance_4_start, Tuning::Get().dance_4_speed,
        Tuning::Get().mimic_warmup_s);
    dance_4.ConfigureTransitionV2(tuning.mimic_transition_v2, true,
                                  tuning.mimic_transition_search_frames,
                                  tuning.mimic_transition_clip_ramp_s);
    dance_4.Init(hl2_dance_dir + Tuning::Get().dance_4_onnx, env, session_options);
    std::cout << "[DanceTransition] v2=" << (tuning.mimic_transition_v2 ? "on" : "off")
              << " search_frames=" << tuning.mimic_transition_search_frames
              << " clip_ramp_s=" << tuning.mimic_transition_clip_ramp_s
              << " entry_preview={" << dance_1_start << ", " << dance_2_start
              << ", " << dance_3_start << ", " << dance_4_start << "}"
              << " legacy={" << dance_1_legacy_start << ", " << dance_2_legacy_start
              << ", " << dance_3_legacy_start << ", " << dance_4_legacy_start << "}\n";

    // Sit controller (đọc tham số từ Tuning::Get() — đồng bộ high_level_2)
    auto& t = Tuning::Get();
    SitController sit_controller;

    // Array of Runners: index = active_policy value
    // 0 = Sit, 1 = Flat, 2-5 = Dance
    std::vector<PolicyRunner*> runners = {
        &sit_controller, // 0
        locomotion_ptr,   // 1
        &dance_1,         // 2
        &dance_2,         // 3
        &dance_3,         // 4
        &dance_4          // 5
    };
    const int trace_obs_dim = locomotion_ptr->GetInputSize();

    enum class DanceHandoverState {
        kNone,
        kWaitingForLocomotionSettle,
        kReturningToDefault,
        kCooldown
    };
    DanceHandoverState dance_handover = DanceHandoverState::kNone;
    int pending_dance_policy = 0;
    int cooldown_destination_policy = 1;
    float entry_settle_elapsed_s = 0.0f;
    float return_elapsed_s = 0.0f;
    std::array<float, R1Config::NUM_JOINTS> last_commanded_q{};
    bool last_commanded_q_valid = false;
    std::array<float, R1Config::NUM_JOINTS> controller_blend_start_q{};
    float controller_blend_elapsed_s = 0.0f;
    bool controller_blend_active = false;
    bool dance_audio_pending = false;
    bool mimic_handover_block_logged = false;
    bool entry_settle_wait_logged = false;
    float dance_test_elapsed_s = 0.0f;
    bool dance_test_triggered = false;

    std::thread keyboard_thread;
    if (!g_autotest) keyboard_thread = std::thread(KeyboardThread);

    // ── Trạng thái autotest: đo trôi base khi chơi 1 gesture ──
    float at_time = 0.0f;          // thời gian mô phỏng từ lúc bắt đầu vòng lặp
    bool  at_triggered = false;
    float at_bx0 = 0.0f, at_by0 = 0.0f, at_gx0 = 0.0f, at_yaw0 = 0.0f;
    std::array<float, 3> at_gravity0 = {0.0f, 0.0f, -1.0f};
    // Chiều cao base: đo "hạ trọng tâm khi chơi gesture" (policy tự khuỵu gối bù CoM).
    float at_z0 = 0.0f, at_z_min = 1e9f;
    float at_bx_min=1e9f, at_bx_max=-1e9f, at_by_min=1e9f, at_by_max=-1e9f;
    float at_gx_peak = 0.0f;   // đỉnh |Δ torso pitch| = nhiễu CoM (metric chính, ít nhiễu hơn chân)
    float at_tilt_peak_rad = 0.0f;
    float at_drift_peak = 0.0f;
    double at_gx_abs_sum = 0.0; int at_gx_n = 0;
    int autotest_result = 0;
    std::vector<AutotestSample> at_log;
    std::string at_last_phase;

    const auto write_autotest_csv = [&]() {
        if (g_at_csv.empty()) return;
        std::ofstream f(g_at_csv);
        f << "t,drift_x,drift_y,lean,dpitch,base_z,tilt_rad,gx,gy,gz,"
             "yaw_delta,cmd_vx,cmd_vy,cmd_yaw,meta_selected,meta_safety_hold";
        const auto meta_expert_names = use_rma_meta
            ? rma_meta_controller.ExpertNames()
            : slope_meta_controller.ExpertNames();
        for (const auto& name : meta_expert_names) f << ",prob_" << name;
        if (!meta_expert_names.empty()) {
            f << (use_rma_meta ? ",prob_HOLD" : ",prob_NEUTRAL");
        }
        for (const auto& name : meta_expert_names) f << ",weight_" << name;
        if (g_at_meta_trace) {
            for (int index = 0; index < trace_obs_dim; ++index) {
                f << ",obs_" << index;
            }
            for (int index = 0; index < R1Config::NUM_JOINTS; ++index) {
                f << ",action_" << index;
            }
            for (int index = 0; index < R1Config::NUM_JOINTS; ++index) {
                f << ",target_q_" << index;
            }
        }
        f << "\n";
        for (const auto& sample : at_log) {
            const auto& r = sample.motion;
            f << r[0] << "," << r[1] << "," << r[2] << "," << r[3] << "," << r[4]
              << "," << r[5] << "," << r[6] << "," << r[7] << "," << r[8]
              << "," << r[9] << "," << r[10] << "," << r[11]
              << "," << r[12] << "," << r[13] << "," << sample.meta_selected
              << "," << (sample.meta_safety_hold ? 1 : 0);
            for (float probability : sample.meta_probabilities) {
                f << "," << probability;
            }
            for (float weight : sample.meta_weights) f << "," << weight;
            if (g_at_meta_trace) {
                for (int index = 0; index < trace_obs_dim; ++index) {
                    f << "," << (index < static_cast<int>(sample.actor_observation.size())
                        ? sample.actor_observation[index]
                        : std::numeric_limits<float>::quiet_NaN());
                }
                for (int index = 0; index < R1Config::NUM_JOINTS; ++index) {
                    f << "," << (index < static_cast<int>(sample.policy_action.size())
                        ? sample.policy_action[index]
                        : std::numeric_limits<float>::quiet_NaN());
                }
                for (int index = 0; index < R1Config::NUM_JOINTS; ++index) {
                    f << "," << (index < static_cast<int>(sample.joint_target.size())
                        ? sample.joint_target[index]
                        : std::numeric_limits<float>::quiet_NaN());
                }
            }
            f << "\n";
        }
        std::cout << "[AUTOTEST] Da ghi CSV: " << g_at_csv << " ("
                  << at_log.size() << " dong)\n";
    };

    LowCmd_ low_cmd;
    for (int i = 0; i < 31; ++i) {
        low_cmd.motor_cmd()[i].mode() = 1; // PR_MODE
        low_cmd.motor_cmd()[i].q() = 0.0f;
        low_cmd.motor_cmd()[i].dq() = 0.0f;
        low_cmd.motor_cmd()[i].kp() = 0.0f;
        low_cmd.motor_cmd()[i].kd() = 0.0f;
        low_cmd.motor_cmd()[i].tau() = 0.0f;
    }

    std::cout << "\n>>> POLICY ĐÃ NẠP XONG. ĐANG CHỜ SIMULATOR/ROBOT STATE...\n";
    std::string last_scene_wait_message;
    while (running) {
        if (rough_scene_auto && rough_controller && !rough_controller->SceneLoaded()) {
            const auto detected = SimulatorSceneResolver::DetectActiveR1Scene();
            if (detected.ok) {
                std::cout << "[RoughController] Scene tự động từ simulator ("
                          << detected.message << "): " << detected.scene << std::endl;
                if (!rough_controller->LoadScene(detected.scene.string())) {
                    std::cerr << "[RoughController] Dừng để tránh chạy policy với height scan sai."
                              << std::endl;
                    running = false;
                    break;
                }
                last_scene_wait_message.clear();
            } else if (detected.message != last_scene_wait_message) {
                std::cout << "[RoughController] Đang chờ scene: " << detected.message << std::endl;
                last_scene_wait_message = detected.message;
            }
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            const bool scene_ready = !rough_controller || rough_controller->SceneLoaded();
            if (got_first_state && got_first_sport_state && scene_ready) {
                float q_sum = 0.0f;
                for (int i = 0; i < 24; ++i) {
                    int idl_idx = R1Config::PolicyToIdl(i);
                    q_sum += std::abs(robot_state.motor_state()[idl_idx].q());
                }
                if (q_sum > 0.01f) break;
            }
        }
        usleep(5000);
    }
    if (!running) return 1;
    std::cout << ">>> ĐÃ KẾT NỐI! BẮT ĐẦU VÒNG LẶP ĐIỀU KHIỂN (50Hz)\n";

    float smoothed_commands[3] = {0.0f, 0.0f, 0.0f};
    float alpha = 0.1f;
    float gait_time = 0.0f;
    float dt = 0.02f;
    int step = 0;
    float history_gesture_still_s = 0.0f;
    float history_gesture_release_s = 0.0f;
    GaitModeScheduler gait_mode_scheduler;
    std::unique_ptr<GaitModeSchedulerV3> gait_mode_scheduler_v3;
    if (gait_conditioned) {
        const auto& gait_tuning = Tuning::Get().gait_mode;
        try {
            const auto require_policy_value = [&](const char* key, float configured) {
                const std::string raw = locomotion_ptr->MetadataValue(key);
                if (raw.empty() || std::abs(std::stof(raw) - configured) > 1.0e-6f) {
                    throw std::runtime_error(
                        std::string("tuning/ONNX gait contract mismatch for ") + key
                        + ": tuning=" + std::to_string(configured)
                        + ", model=" + (raw.empty() ? "<missing>" : raw));
                }
            };
            require_policy_value("gait_stop_request_lin", gait_tuning.stop_request_lin);
            require_policy_value("gait_stop_request_ang", gait_tuning.stop_request_ang);
            require_policy_value("gait_move_request_lin", gait_tuning.move_request_lin);
            require_policy_value("gait_move_request_ang", gait_tuning.move_request_ang);
            if (!gait_v3) {
                require_policy_value("gait_stability_ang_vel_max", gait_tuning.settle_gyro_norm);
                require_policy_value("gait_stability_joint_vel_rms_max",
                                     gait_tuning.settle_joint_velocity_rms);
                require_policy_value("gait_t_settle_s", gait_tuning.settle_dwell_s);
                require_policy_value("gait_w2s_timeout_s", gait_tuning.w2s_timeout_s);
                // The v1 exporter records filter_tau only in the golden trace.
                if (std::abs(gait_tuning.stability_filter_tau_s - 0.20f) > 1.0e-6f) {
                    throw std::runtime_error(
                        "gait stability filter tau must remain 0.20 s to match "
                        "gait_golden_trace.txt");
                }
                GaitModeConfig config{
                    gait_tuning.stop_request_lin,
                    gait_tuning.stop_request_ang,
                    gait_tuning.move_request_lin,
                    gait_tuning.move_request_ang,
                    gait_tuning.settle_gyro_norm,
                    gait_tuning.settle_joint_velocity_rms,
                    gait_tuning.settle_dwell_s,
                    gait_tuning.w2s_timeout_s,
                    gait_tuning.stability_filter_tau_s};
                if (gait_0917) {
                    const auto& stand = Tuning::Get().gait_mode_v3;
                    require_policy_value("gait_stand_upright_enabled", 1.0f);
                    require_policy_value("gait_stand_entry_tilt_max_rad",
                                         stand.stand_entry_tilt_max_rad);
                    require_policy_value("gait_stand_push_exit_enabled", 1.0f);
                    require_policy_value("gait_push_exit_tilt_immediate_rad",
                                         stand.push_exit_tilt_immediate_rad);
                    require_policy_value("gait_push_exit_tilt_sustained_rad",
                                         stand.push_exit_tilt_sustained_rad);
                    require_policy_value("gait_push_exit_gyro_immediate",
                                         stand.push_exit_gyro_immediate);
                    require_policy_value("gait_push_exit_gyro_sustained",
                                         stand.push_exit_gyro_sustained);
                    require_policy_value("gait_push_exit_sustain_s",
                                         stand.push_exit_sustain_s);
                    config.stand_upright_enabled = true;
                    config.stand_entry_tilt_max_rad = stand.stand_entry_tilt_max_rad;
                    config.stand_push_exit_enabled = true;
                    config.push_exit_tilt_immediate_rad =
                        stand.push_exit_tilt_immediate_rad;
                    config.push_exit_tilt_sustained_rad =
                        stand.push_exit_tilt_sustained_rad;
                    config.push_exit_gyro_immediate = stand.push_exit_gyro_immediate;
                    config.push_exit_gyro_sustained = stand.push_exit_gyro_sustained;
                    config.push_exit_sustain_s = stand.push_exit_sustain_s;
                }
                gait_mode_scheduler.Configure(config);
            } else {
                const auto& v3 = Tuning::Get().gait_mode_v3;
                const auto require_v3 = [&](const char* key, float configured) {
                    const std::string raw = locomotion_ptr->MetadataValue(key);
                    if (raw.empty() || std::abs(std::stof(raw) - configured) > 1.0e-5f) {
                        throw std::runtime_error(
                            std::string("tuning/ONNX gait v3 contract mismatch for ") + key
                            + ": tuning=" + std::to_string(configured)
                            + ", model=" + (raw.empty() ? "<missing>" : raw));
                    }
                };
                const auto require_v3_string = [&](const char* key, const char* expected) {
                    const std::string actual = locomotion_ptr->MetadataValue(key);
                    if (actual != expected) {
                        throw std::runtime_error(
                            std::string("ONNX gait v3 contract mismatch for ") + key
                            + ": expected=" + expected + ", model="
                            + (actual.empty() ? "<missing>" : actual));
                    }
                };
                require_v3("gait_t_settle_s", v3.t_settle_s);
                require_v3("gait_w2s_timeout_s", gait_tuning.w2s_timeout_s);
                require_v3("gait_stance_home_width_m", 0.212f);
                require_v3("gait_stance_width_tol_in_m", v3.stance_width_tol_in_m);
                require_v3("gait_stance_width_tol_out_m", v3.stance_width_tol_out_m);
                require_v3("gait_stance_dx_tol_m", v3.stance_dx_tol_m);
                require_v3("gait_stance_yaw_tol_rad", v3.stance_yaw_tol_rad);
                require_v3("gait_stance_exit_factor", v3.stance_exit_factor);
                require_v3("gait_gather_max_strides", static_cast<float>(v3.gather_max_strides));
                require_v3("gait_gather_widen_factor", v3.gather_widen_factor);
                require_v3("gait_gather_force_settle_strides", static_cast<float>(v3.gather_force_settle_strides));
                require_v3("gait_forced_settle_exit_margin", v3.forced_settle_exit_margin);
                require_v3("gait_stance_threshold", v3.stance_threshold);
                require_v3("gait_push_exit_tilt_immediate_rad", v3.push_exit_tilt_immediate_rad);
                require_v3("gait_push_exit_tilt_sustained_rad", v3.push_exit_tilt_sustained_rad);
                require_v3("gait_push_exit_gyro_immediate", v3.push_exit_gyro_immediate);
                require_v3("gait_push_exit_gyro_sustained", v3.push_exit_gyro_sustained);
                require_v3("gait_push_exit_joint_rms_sustained", v3.push_exit_joint_rms_sustained);
                require_v3("gait_push_exit_sustain_s", v3.push_exit_sustain_s);
                require_v3("gait_stand_entry_tilt_max_rad", v3.stand_entry_tilt_max_rad);
                require_v3("gait_stand_entry_gyro_max", v3.stand_entry_gyro_max);
                require_v3_string("gait_w2s_substates", "GATHER,SETTLE");
                require_v3_string("gait_stability_joints", "all_joints");
                require_v3_string("gait_stance_gate", "fk_width_band_dx_yaw_v2");
                require_v3_string("gait_phase_clock",
                                  "run_walk_gather_freeze_settle_stand_restart_on_stand_exit_v3");
                require_v3_string("gait_phase_obs", "zero_in_settle_and_stand_v3");
                require_v3_string("gait_leg_offsets", "0.0,0.5");
                require_v3_string("gait_command_accel", "2.5,2.0,3.0");
                require_v3_string("gait_command_decel", "1.2,1.2,1.5");
                require_v3_string("gait_phase_start", "0.0");
                const std::array<float, 3> expected_accel{2.5f, 2.0f, 3.0f};
                const std::array<float, 3> expected_decel{1.2f, 1.2f, 1.5f};
                if (v3.command_accel != expected_accel || v3.command_decel != expected_decel) {
                    throw std::runtime_error(
                        "gait v3 command slew must remain 2.5/2.0/3.0 accel and "
                        "1.2/1.2/1.5 decel");
                }
                GaitModeV3Config config{
                    gait_tuning.stop_request_lin, gait_tuning.stop_request_ang,
                    gait_tuning.move_request_lin, gait_tuning.move_request_ang,
                    gait_tuning.settle_gyro_norm, gait_tuning.settle_joint_velocity_rms,
                    gait_tuning.stability_filter_tau_s, locomotion_gait_period_s,
                    v3.t_settle_s, gait_tuning.w2s_timeout_s,
                    v3.stance_width_tol_in_m, v3.stance_width_tol_out_m,
                    v3.stance_dx_tol_m, v3.stance_yaw_tol_rad, v3.stance_exit_factor,
                    v3.gather_max_strides, v3.gather_widen_factor,
                    v3.gather_force_settle_strides, v3.forced_settle_exit_margin,
                    v3.stance_threshold, v3.push_exit_tilt_immediate_rad,
                    v3.push_exit_tilt_sustained_rad, v3.push_exit_gyro_immediate,
                    v3.push_exit_gyro_sustained, v3.push_exit_joint_rms_sustained,
                    v3.push_exit_sustain_s, v3.stand_entry_tilt_max_rad,
                    v3.stand_entry_gyro_max};
                gait_mode_scheduler_v3 = std::make_unique<GaitModeSchedulerV3>(
                    (gait_fsm_contract == "flat_plus_gait_fsm_v3f" || generic_v3_fg)
                        ? GaitV3Variant::F : GaitV3Variant::B);
                gait_mode_scheduler_v3->Configure(config);
            }
        } catch (const std::exception& error) {
            std::cerr << "[GaitMode] " << error.what() << std::endl;
            return 1;
        }
    }
    std::vector<float> last_meta_observation;
    std::vector<float> last_meta_action;
    std::vector<float> last_meta_joint_target;
    auto next_wake_time = std::chrono::steady_clock::now();

    while (running) {
        LowState_ local_state;
        SportModeState_ local_sport;
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            local_state = robot_state;
            local_sport = current_sport_state;
        }

        if (g_dance_test_policy >= 2 && !dance_test_triggered && active_policy == 1 &&
            dance_handover == DanceHandoverState::kNone) {
            dance_test_elapsed_s += dt;
            if (dance_test_elapsed_s >= 1.0f) {
                requested_policy.store(g_dance_test_policy);
                dance_test_triggered = true;
                std::cout << "[DanceTransition] test-only trigger policy "
                          << g_dance_test_policy << " after " << dance_test_elapsed_s
                          << " s.\n";
            }
        }

        const int requested = requested_policy.load();
        if (dance_handover == DanceHandoverState::kWaitingForLocomotionSettle) {
            if (requested == 1) {
                dance_handover = DanceHandoverState::kNone;
                target_vx = target_vy = target_yaw = 0.0f;
                dance_audio_pending = false;
                std::cout << "[DanceHandover] Dance request cancelled while waiting for "
                             "locomotion settle.\n";
            } else if (requested >= 2 && requested <= 5) {
                pending_dance_policy = requested;
            }
        } else if (dance_handover == DanceHandoverState::kReturningToDefault) {
            if (requested == 1) {
                dance_handover = DanceHandoverState::kNone;
                target_vx = target_vy = target_yaw = 0.0f;
                dance_audio_pending = false;
                std::cout << "[DanceHandover] Dance request cancelled; remaining in locomotion.\n";
            } else if (requested >= 2 && requested <= 5) {
                pending_dance_policy = requested;
            }
        } else if (dance_handover == DanceHandoverState::kCooldown) {
            if (requested >= 1 && requested <= 5) {
                cooldown_destination_policy = requested;
            }
        } else if (requested >= 1 && requested <= 5 && requested != active_policy) {
            if (active_policy == 1 && requested >= 2) {
                pending_dance_policy = requested;
                dance_handover = DanceHandoverState::kWaitingForLocomotionSettle;
                entry_settle_elapsed_s = 0.0f;
                entry_settle_wait_logged = false;
                return_elapsed_s = 0.0f;
                target_vx = target_vy = target_yaw = 0.0f;
                dance_audio_pending = true;
                std::cout << "[DanceHandover] Locomotion -> settle before default_q for Dance "
                          << (pending_dance_policy - 1) << ".\n";
            } else if (active_policy >= 2) {
                auto* dance = dynamic_cast<DanceController*>(runners[active_policy]);
                if (dance) {
                    cooldown_destination_policy = requested;
                    dance->BeginCooldown(
                        locomotion_ptr->DefaultPosition(), t.mimic_cooldown_s, local_state);
                    dance_handover = DanceHandoverState::kCooldown;
                    mimic_handover_block_logged = false;
                    target_vx = target_vy = target_yaw = 0.0f;
                    dance_audio_pending = false;
                    g_audio.Stop();
                    std::cout << "[DanceHandover] Dance " << (active_policy - 1)
                              << " -> cooldown before policy " << requested << ".\n";
                }
            } else {
                active_policy = requested;
                policy_changed = true;
            }
        }

        // active_policy: 0=Sit, 1=Flat, 2-5=Dance — khớp trực tiếp với index runners[]
        int policy_idx = std::clamp(active_policy, 0, (int)runners.size() - 1);
        PolicyRunner* policy = runners[policy_idx];
        SitController* sit_ptr = dynamic_cast<SitController*>(policy);

        // --- Overlay thân trên: teleop chỉ legacy 83-D; H4/gait chỉ gesture. ---
        bool flat_mode = (policy_idx == 1);
        const bool legacy_arm_overlay_allowed =
            flat_mode && !use_any_meta && policy->SupportsArmOverlay();
        const bool history_gesture_contract = flat_mode && !use_any_meta &&
            Tuning::Get().gesture_history_enabled && policy->SupportsHistoryGestureOverlay();
        const bool gesture_overlay_allowed = legacy_arm_overlay_allowed ||
            history_gesture_contract;
        const float history_move_intent = std::sqrt(
            target_vx * target_vx + target_vy * target_vy + target_yaw * target_yaw);
        const bool history_gesture_move_requested = history_gesture_contract &&
            history_move_intent > Tuning::Get().gesture_history_move_threshold;
        bool history_gesture_ready = !history_gesture_contract;
        if (history_gesture_contract) {
            float gyro_sq = 0.0f;
            for (const float value : local_state.imu_state().gyroscope()) gyro_sq += value * value;
            float joint_sq = 0.0f;
            for (int joint = 0; joint < R1Config::NUM_JOINTS; ++joint) {
                const int idl_idx = R1Config::PolicyToIdl(joint);
                const float dq = local_state.motor_state()[idl_idx].dq();
                joint_sq += dq * dq;
            }
            const bool sensor_stable = std::sqrt(gyro_sq) <=
                    Tuning::Get().gait_mode.settle_gyro_norm &&
                std::sqrt(joint_sq / static_cast<float>(R1Config::NUM_JOINTS)) <=
                    Tuning::Get().gait_mode.settle_joint_velocity_rms;
            const bool gait_stand = !gait_conditioned || (gait_v3
                ? (gait_mode_scheduler_v3->mode() == GaitMode::Stand
                   && gait_mode_scheduler_v3->stable())
                : (gait_mode_scheduler.mode() == GaitMode::Stand
                   && gait_mode_scheduler.stable()));
            const bool still = !history_gesture_move_requested && sensor_stable && gait_stand;
            history_gesture_still_s = still
                ? std::min(Tuning::Get().gesture_history_stand_dwell_s,
                           history_gesture_still_s + dt)
                : 0.0f;
            history_gesture_ready = history_gesture_still_s >=
                Tuning::Get().gesture_history_stand_dwell_s;
            const bool history_gesture_busy = !g_gesture.ActiveName().empty() ||
                g_gesture.Active();
            if (!history_gesture_busy) {
                history_gesture_release_s = std::max(0.0f, history_gesture_release_s - dt);
            } else {
                history_gesture_release_s = 4.0f * dt;
            }
        } else {
            history_gesture_still_s = 0.0f;
            history_gesture_release_s = 0.0f;
        }
        const bool history_gesture_holds_base = history_gesture_contract &&
            ((!g_gesture.ActiveName().empty() || g_gesture.Active()) ||
             history_gesture_release_s > 0.0f);
        g_teleop.Update(dt, g_teleop_armed.load());
        bool use_teleop = legacy_arm_overlay_allowed && g_teleop_armed.load()
            && g_teleop.Active();
        // Phím G: chọn gesture kế tiếp (in tên ra).
        if (g_gesture_cycle.exchange(false) && !g_gesture_names.empty()) {
            int n = (int)g_gesture_names.size();
            int sel = (g_gesture_sel.load() + 1) % n;
            g_gesture_sel.store(sel);
            std::cout << "[Gesture] Chon: '" << g_gesture_names[sel] << "' (" << (sel+1) << "/" << n << ")\n";
        }
        // Phím V: chơi/thu gesture đang chọn.
        if (g_gesture_toggle.exchange(false) && gesture_overlay_allowed
            && !use_teleop && !g_gesture_names.empty()) {
            if (!history_gesture_contract || history_gesture_ready) {
                g_gesture.Trigger(g_gesture_names[g_gesture_sel.load()]);
            } else {
                std::cout << "[Gesture] H4/gait chua dung on dinh du "
                          << Tuning::Get().gesture_history_stand_dwell_s << "s.\n";
            }
        }
        if (!gesture_overlay_allowed || use_teleop || history_gesture_move_requested) {
            g_gesture.Retract();
        }
        g_gesture.Update(dt);

        // Autotest: lái lệnh tự động + kích gesture + đo trôi base, xuất CSV.
        if (g_autotest) {
            at_time += dt;
            const float measured_time = at_time - g_at_warmup_s;
            if ((g_at_sequence == "flat_demo" || g_at_sequence == "mixed_shuttle")
                && measured_time >= 0.0f) {
                const std::string phase = g_at_sequence == "mixed_shuttle"
                    ? MixedShuttleCommand(measured_time, target_vx, target_vy, target_yaw)
                    : FlatDemoCommand(measured_time, target_vx, target_vy, target_yaw);
                if (phase != at_last_phase) {
                    std::cout << "[AUTOTEST] PHASE=" << phase << " t=" << measured_time
                              << " cmd=(" << target_vx << ", " << target_vy << ", "
                              << target_yaw << ")\n";
                    at_last_phase = phase;
                }
            } else {
                const bool command_active = measured_time >= 0.0f &&
                    (g_at_stop_after_s < 0.0f || measured_time < g_at_stop_after_s);
                target_vx = command_active ? g_at_vx : 0.0f;
                target_vy = command_active ? g_at_vy : 0.0f;
                target_yaw = command_active ? g_at_yaw : 0.0f;
            }
            float bx = local_sport.position()[0];
            float by = local_sport.position()[1];
            // Torso pitch (projected gravity x) — metric nhiễu CoM chính, ít nhiễu hơn vị trí chân.
            float aqw = local_state.imu_state().quaternion()[0];
            float aqx = local_state.imu_state().quaternion()[1];
            float aqy = local_state.imu_state().quaternion()[2];
            float aqz = local_state.imu_state().quaternion()[3];
            float gx = 2.0f * (aqw * aqy - aqx * aqz);
            float gy = -2.0f * (aqy * aqz + aqw * aqx);
            float gz_at = 2.0f * (aqx * aqx + aqy * aqy) - 1.0f;
            float yaw_at = YawFromQuat(aqw, aqx, aqy, aqz);
            if (!at_triggered && at_time >= g_at_warmup_s && !g_at_gesture) {
                at_triggered = true;
                at_bx0 = bx; at_by0 = by; at_gx0 = gx; at_yaw0 = yaw_at;
                at_gravity0 = {gx, gy, gz_at};
                at_z0 = local_sport.position()[2]; at_z_min = at_z0;
                std::cout << "[AUTOTEST] Bat dau do tai t=" << at_time
                          << "s (khong gesture), base0=(" << bx << ", " << by
                          << ") pitch0=" << gx << "\n";
            }
            if (!at_triggered && at_time >= g_at_warmup_s && g_at_gesture) {
                if (!g_gesture.Has(g_at_name)) {
                    std::cerr << "[AUTOTEST] Khong co gesture '" << g_at_name << "'. Thoat.\n";
                    running = false;
                } else if (!history_gesture_contract || history_gesture_ready) {
                    g_gesture.Trigger(g_at_name);
                    at_triggered = true;
                    at_bx0 = bx; at_by0 = by; at_gx0 = gx; at_yaw0 = yaw_at;
                    at_gravity0 = {gx, gy, gz_at};
                    at_z0 = local_sport.position()[2]; at_z_min = at_z0;
                    std::cout << "[AUTOTEST] Kich gesture tai t=" << at_time
                              << "s, base0=(" << bx << ", " << by << ") pitch0=" << gx << "\n";
                }
            }
            if (at_triggered) {
                float rx = bx - at_bx0, ry = by - at_by0, dpitch = gx - at_gx0;
                const float dot = std::clamp(
                    gx * at_gravity0[0] + gy * at_gravity0[1] + gz_at * at_gravity0[2],
                    -1.0f, 1.0f);
                const float tilt_rad = std::acos(dot);
                at_bx_min = std::min(at_bx_min, rx); at_bx_max = std::max(at_bx_max, rx);
                at_by_min = std::min(at_by_min, ry); at_by_max = std::max(at_by_max, ry);
                at_gx_peak = std::max(at_gx_peak, std::fabs(dpitch));
                at_tilt_peak_rad = std::max(at_tilt_peak_rad, tilt_rad);
                at_drift_peak = std::max(at_drift_peak, std::hypot(rx, ry));
                at_z_min = std::min(at_z_min, local_sport.position()[2]);
                at_gx_abs_sum += std::fabs(dpitch); ++at_gx_n;
                const float dyaw = WrapPi(yaw_at - at_yaw0);
                AutotestSample sample;
                sample.motion = {
                    at_time - g_at_warmup_s, rx, ry, g_gesture.LeanPitchProxy(),
                    dpitch, local_sport.position()[2], tilt_rad, gx, gy, gz_at,
                    dyaw, target_vx, target_vy, target_yaw};
                if (use_rma_meta) {
                    sample.meta_selected = rma_meta_controller.SelectedClass();
                    sample.meta_safety_hold = rma_meta_controller.SafetyHold();
                    sample.meta_probabilities = rma_meta_controller.GateProbabilities();
                    sample.meta_weights = rma_meta_controller.ExpertWeights();
                    if (g_at_meta_trace) {
                        sample.actor_observation = last_meta_observation;
                        sample.policy_action = last_meta_action;
                        sample.joint_target = last_meta_joint_target;
                    }
                } else if (use_slope_meta) {
                    sample.meta_selected = slope_meta_controller.SelectedClass();
                    sample.meta_safety_hold = slope_meta_controller.SafetyHold();
                    sample.meta_probabilities = slope_meta_controller.GateProbabilities();
                    sample.meta_weights = slope_meta_controller.ExpertWeights();
                    if (g_at_meta_trace) {
                        sample.actor_observation = last_meta_observation;
                        sample.policy_action = last_meta_action;
                        sample.joint_target = last_meta_joint_target;
                    }
                } else if (use_arma) {
                    sample.meta_selected = "ARMA";
                    sample.meta_safety_hold = arma_controller.SafetyHold();
                    if (g_at_meta_trace) {
                        sample.actor_observation = last_meta_observation;
                        sample.policy_action = last_meta_action;
                        sample.joint_target = last_meta_joint_target;
                    }
                } else {
                    sample.meta_selected = "DISABLED";
                }
                at_log.push_back(std::move(sample));
                if (at_time >= g_at_warmup_s + g_at_hold_s) {
                    float dx = bx - at_bx0, dy = by - at_by0;
                    float pitch_mean = at_gx_n ? (float)(at_gx_abs_sum / at_gx_n) : 0.0f;
                    const float height_drop = at_z0 - at_z_min;
                    const float tilt_peak_deg = at_tilt_peak_rad * 180.0f / static_cast<float>(M_PI);
                    const bool finite = std::isfinite(height_drop) && std::isfinite(tilt_peak_deg) &&
                                        std::isfinite(at_drift_peak);
                    const bool moving_command = !g_at_sequence.empty() ||
                                                std::hypot(g_at_vx, g_at_vy) > 1e-4f ||
                                                std::fabs(g_at_yaw) > 1e-4f;
                    const bool drift_ok = moving_command ||
                                          at_drift_peak <= g_at_max_stand_drift_m;
                    const bool pass = finite && height_drop <= g_at_max_height_drop_m &&
                                      tilt_peak_deg <= g_at_max_tilt_deg && drift_ok;
                    autotest_result = pass ? 0 : 2;
                    std::cout << "\n===== KET QUA AUTOTEST: " << g_at_name << " =====\n"
                              << "  kg=" << Tuning::Get().gesture_balance_kg
                              << " kv=" << Tuning::Get().gesture_balance_kv
                              << " cmd=(" << g_at_vx << ", " << g_at_vy << ", "
                              << g_at_yaw << ")\n"
                              << (g_at_sequence.empty() ? "" : ("  sequence=" + g_at_sequence + "\n"))
                              << "  [METRIC CHINH] dinh |lech pitch than| = " << at_gx_peak
                              << " (trung binh " << pitch_mean << ")\n"
                              << "  tilt tuong doi lon nhat = " << tilt_peak_deg << " deg\n"
                              << "  [CHIEU CAO] base luc kich = " << at_z0 << " m, thap nhat = "
                              << at_z_min << " m  -> HA " << height_drop << " m\n"
                              << "  troi cuoi (x,y) = (" << dx << ", " << dy << ") m\n"
                              << "  troi ngang lon nhat = " << at_drift_peak << " m\n"
                              << "  bien do x: [" << at_bx_min << ", " << at_bx_max << "]"
                              << "  bien do y: [" << at_by_min << ", " << at_by_max << "] m\n"
                              << "  RESULT=" << (pass ? "PASS" : "FAIL") << "\n"
                              << "=====================================\n";
                    write_autotest_csv();
                    running = false;
                }
            }
        }

        float qw = local_state.imu_state().quaternion()[0];
        float qx = local_state.imu_state().quaternion()[1];
        float qy = local_state.imu_state().quaternion()[2];
        float qz = local_state.imu_state().quaternion()[3];
        float gz = 2.0f * (qx * qx + qy * qy) - 1.0f;

        if (current_robot_state == RobotState::FALLEN) {
            for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
                int idl_idx = R1Config::PolicyToIdl(i);
                low_cmd.motor_cmd()[idl_idx].q() = local_state.motor_state()[idl_idx].q();
                low_cmd.motor_cmd()[idl_idx].dq() = 0.0f;
                low_cmd.motor_cmd()[idl_idx].kp() = 0.0f;
                low_cmd.motor_cmd()[idl_idx].kd() = 0.0f;
                low_cmd.motor_cmd()[idl_idx].tau() = 0.0f;
                low_cmd.motor_cmd()[idl_idx].mode() = 1;
            }
            lowcmd_publisher->Write(low_cmd);

            if (g_autotest) {
                std::cerr << "[AUTOTEST] RESULT=FAIL: fall detector đã kích hoạt.\n";
                autotest_result = 3;
                write_autotest_csv();
                running = false;
            }

            if (gz < -0.9f) {
                std::cout << "[INFO] Robot đã đứng dậy. Đang tiếp tục chạy Policy..." << std::endl;
                current_robot_state = RobotState::RUNNING_POLICY;
                policy_changed = true;
                step = 0;
            }
            next_wake_time += std::chrono::milliseconds(20);
            std::this_thread::sleep_until(next_wake_time);
            continue;
        }

        if (current_robot_state == RobotState::RUNNING_POLICY && gz > -0.5f && step > 100 && active_policy != 0) {
            current_robot_state = RobotState::FALLEN;
            target_vx = target_vy = target_yaw = 0.0f;
            std::cout << "[WARNING] ROBOT NGÃ! Đã ngắt động cơ (nhấn Backspace trên Simulator để đứng dậy)." << std::endl;
            continue;
        }

        if (policy_changed) {
            std::cout << "[INFO] Đã chuyển Mode/Khởi động. Đang chạy Policy..." << std::endl;
            policy->Reset(local_state);
            if (auto* dance = dynamic_cast<DanceController*>(policy)) {
                std::cout << "[DanceTransition] runtime entry frame=" << dance->StartFrame()
                          << (dance->TransitionV2Enabled() ? " (v2)" : " (legacy)") << "\n";
            }
            if (gait_conditioned) {
                if (gait_v3) {
                    gait_mode_scheduler_v3->Reset();
                    policy->SetGaitMode(gait_mode_scheduler_v3->OneHot());
                } else {
                    gait_mode_scheduler.Reset();
                    policy->SetGaitMode(gait_mode_scheduler.OneHot());
                }
                if (gait_0917) gait_time = 0.0f;
            }
            controller_blend_elapsed_s = 0.0f;
            for (int joint = 0; joint < R1Config::NUM_JOINTS; ++joint) {
                controller_blend_start_q[joint] = last_commanded_q_valid
                    ? last_commanded_q[joint]
                    : local_state.motor_state()[R1Config::PolicyToIdl(joint)].q();
            }
            controller_blend_active = t.dance_blend_time_s > 0.0f;
            policy_changed = false;
        }

        const float cmd_vx  = history_gesture_holds_base ? 0.0f : target_vx;
        const float cmd_vy  = history_gesture_holds_base ? 0.0f : target_vy;
        const float cmd_yaw = history_gesture_holds_base ? 0.0f : target_yaw;
        if (use_rma_meta && flat_mode) {
            const auto governed = rma_meta_controller.FilterCommands(
                {cmd_vx, cmd_vy, cmd_yaw}, dt);
            std::copy(governed.begin(), governed.end(), smoothed_commands);
        } else if (use_slope_meta && flat_mode) {
            const auto governed = slope_meta_controller.FilterCommands(
                {cmd_vx, cmd_vy, cmd_yaw}, dt);
            std::copy(governed.begin(), governed.end(), smoothed_commands);
        } else if (gait_v3 && flat_mode) {
            const auto& v3 = Tuning::Get().gait_mode_v3;
            const float targets[3] = {cmd_vx, cmd_vy, cmd_yaw};
            for (int axis = 0; axis < 3; ++axis) {
                smoothed_commands[axis] = SlewCommand(
                    smoothed_commands[axis], targets[axis],
                    v3.command_accel[axis], v3.command_decel[axis], dt);
            }
        } else {
            smoothed_commands[0] =
                alpha * cmd_vx + (1.0f - alpha) * smoothed_commands[0];
            smoothed_commands[1] =
                alpha * cmd_vy + (1.0f - alpha) * smoothed_commands[1];
            smoothed_commands[2] =
                alpha * cmd_yaw + (1.0f - alpha) * smoothed_commands[2];
        }

        // Heading-hold: giữ hướng khi đi thẳng (khớp deploy) -> robot không đi cong.
        if (flat_mode) {
            float move = std::sqrt(smoothed_commands[0]*smoothed_commands[0] +
                                   smoothed_commands[1]*smoothed_commands[1]);
            bool steering = std::abs(target_yaw) > 0.05f;
            if (move < kHeadMoveMin || steering) {
                g_heading_active = false;
            } else {
                const auto& q = local_state.imu_state().quaternion();
                float yaw_now = YawFromQuat(q[0], q[1], q[2], q[3]);
                float gyroz = local_state.imu_state().gyroscope()[2];
                if (!g_heading_active || std::abs(gyroz) > kHeadRelatchGyro) {
                    g_heading_ref = yaw_now; g_heading_active = true;
                } else {
                    float err = WrapPi(g_heading_ref - yaw_now);
                    if (std::abs(err) > 1.0f) g_heading_ref = yaw_now;
                    else smoothed_commands[2] = std::clamp(kHeadKp * err, -kHeadMaxYaw, kHeadMaxYaw);
                }
            }
        }

        std::array<float, 3> gyro_values{};
        const auto& gyro = local_state.imu_state().gyroscope();
        for (std::size_t index = 0; index < gyro_values.size(); ++index)
            gyro_values[index] = gyro[index];
        std::array<float, GaitModeScheduler::kNumJoints> joint_velocities{};
        std::array<float, 12> leg_positions{};
        for (int joint = 0; joint < R1Config::NUM_JOINTS; ++joint) {
            const int idl_idx = R1Config::PolicyToIdl(joint);
            joint_velocities[static_cast<std::size_t>(joint)] =
                local_state.motor_state()[idl_idx].dq();
            if (joint < static_cast<int>(leg_positions.size()))
                leg_positions[static_cast<std::size_t>(joint)] =
                    local_state.motor_state()[idl_idx].q();
        }
        const auto& quaternion = local_state.imu_state().quaternion();
        const std::array<float, 3> projected_gravity{
            2.0f * (quaternion[0] * quaternion[2] - quaternion[1] * quaternion[3]),
            -2.0f * (quaternion[2] * quaternion[3] + quaternion[0] * quaternion[1]),
            2.0f * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]) - 1.0f};
        const float command_linear_norm = std::sqrt(
            smoothed_commands[0] * smoothed_commands[0]
            + smoothed_commands[1] * smoothed_commands[1]);
        const float command_yaw_abs = std::abs(smoothed_commands[2]);

        // v3 and 09/17 own their phase clock and update the mode before the
        // phase is packed into the actor observation. Other legacy policies
        // retain the original episode clock and update order.
        if (gait_conditioned && flat_mode && gait_v3) {
            gait_mode_scheduler_v3->Update(
                command_linear_norm, command_yaw_abs, gyro_values,
                joint_velocities, leg_positions, projected_gravity, dt);
            policy->SetGaitMode(gait_mode_scheduler_v3->OneHot());
            gait_time = gait_mode_scheduler_v3->phase_time_s();
        } else if (gait_conditioned && flat_mode && gait_0917) {
            const GaitMode previous_mode = gait_mode_scheduler.mode();
            gait_mode_scheduler.Update(
                command_linear_norm, command_yaw_abs, gyro_values,
                joint_velocities, projected_gravity, dt);
            policy->SetGaitMode(gait_mode_scheduler.OneHot());
            if (previous_mode == GaitMode::Stand
                && gait_mode_scheduler.mode() != GaitMode::Stand) {
                gait_time = 0.0f;
            } else if (gait_mode_scheduler.mode() != GaitMode::Stand) {
                gait_time = fmod(gait_time + dt, locomotion_gait_period_s);
            }
        } else {
            gait_time += dt;
        }

        std::array<float, 2> gait_phase{};
        if (gait_v3) {
            gait_phase = gait_mode_scheduler_v3->PhaseObservation();
        } else {
            const float phase_ratio = fmod(gait_time, locomotion_gait_period_s)
                / locomotion_gait_period_s;
            gait_phase = {
                sin(2.0f * (float)M_PI * phase_ratio),
                cos(2.0f * (float)M_PI * phase_ratio)};
            const float cmd_norm = std::sqrt(
                smoothed_commands[0] * smoothed_commands[0]
                + smoothed_commands[1] * smoothed_commands[1]
                + smoothed_commands[2] * smoothed_commands[2]);
            if (gait_0917) {
                if (gait_mode_scheduler.mode() == GaitMode::Stand) {
                    gait_phase = {0.0f, 0.0f};
                }
            } else if (cmd_norm < 0.1f) {
                gait_phase = {0.0f, 0.0f};
            }
        }
        if (gait_conditioned && flat_mode && !gait_v3 && !gait_0917) {
            gait_mode_scheduler.Update(
                command_linear_norm, command_yaw_abs, gyro_values,
                joint_velocities, dt);
            policy->SetGaitMode(gait_mode_scheduler.OneHot());
        }

        // Xử lý đếm ngược safe shutdown
        if (pending_sit) {
            pending_sit_timer += dt;
            if (pending_sit_timer >= kSafeShutdownSettleTime) {
                pending_sit = false;
                active_policy = 0;
                policy_changed = true;
                auto& t = Tuning::Get();
                if (!t.audio_sit.empty()) g_audio.Play(t.audio_dir + t.audio_sit);
                std::cout << "[SafeShutdown] Da dung vung. Bat dau ngoi xuong..." << std::endl;
            }
        }

        // Dance kết thúc không được giao Flat ngay. Giữ dance actor chạy reference
        // cooldown về default_q trước, đúng với handover của high_level_2.
        if (active_policy >= 2 && dance_handover == DanceHandoverState::kNone
            && policy->IsFinished()) {
            auto* dance = dynamic_cast<DanceController*>(policy);
            if (dance) {
                requested_policy.store(1);
                cooldown_destination_policy = 1;
                dance->BeginCooldown(
                    locomotion_ptr->DefaultPosition(), t.mimic_cooldown_s, local_state);
                dance_handover = DanceHandoverState::kCooldown;
                mimic_handover_block_logged = false;
                target_vx = target_vy = target_yaw = 0.0f;
                dance_audio_pending = false;
                g_audio.Stop();
                std::cout << "[DanceHandover] Dance " << (active_policy - 1)
                          << " finished; starting cooldown.\n";
            }
        }

        // Tick SitController timer mỗi chu kỳ 50Hz
        if (sit_ptr) sit_ptr->Tick(dt);

        std::array<float, R1Config::NUM_JOINTS> target_q;
        bool use_sit_gains = false;
        bool use_dance_return_gains = false;

        if (dance_handover == DanceHandoverState::kWaitingForLocomotionSettle) {
            const auto& gyro = local_state.imu_state().gyroscope();
            const float gyro_norm = std::sqrt(
                gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
            float max_joint_speed = 0.0f;
            for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
                const int idl_idx = R1Config::PolicyToIdl(i);
                max_joint_speed = std::max(
                    max_joint_speed,
                    std::abs(local_state.motor_state()[idl_idx].dq()));
            }
            const bool quiet = gyro_norm <= t.mimic_entry_gyro_max &&
                               max_joint_speed <= t.mimic_entry_dq_max;
            entry_settle_elapsed_s = mimic_transition::AdvanceQuietDwell(
                entry_settle_elapsed_s, dt, quiet, t.mimic_entry_settle_s);
            if (mimic_transition::QuietDwellReady(
                    entry_settle_elapsed_s, t.mimic_entry_settle_s)) {
                dance_handover = DanceHandoverState::kReturningToDefault;
                return_elapsed_s = 0.0f;
                std::cout << "[DanceHandover] Locomotion settled (gyro=" << gyro_norm
                          << ", max_dq=" << max_joint_speed
                          << "); returning to default_q before Dance "
                          << (pending_dance_policy - 1) << ".\n";
            } else if (!entry_settle_wait_logged) {
                std::cout << "[DanceHandover] waiting for locomotion settle (gyro="
                          << gyro_norm << ", max_dq=" << max_joint_speed
                          << ", limits=" << t.mimic_entry_gyro_max << "/"
                          << t.mimic_entry_dq_max << ").\n";
                entry_settle_wait_logged = true;
            }
        }

        if (dance_handover == DanceHandoverState::kReturningToDefault) {
            use_dance_return_gains = true;
            const auto& goal_q = locomotion_ptr->DefaultPosition();
            const float max_step = std::max(0.05f, t.dance_return_rate_limit) * dt;
            float max_position_error = 0.0f;
            float max_joint_speed = 0.0f;
            for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
                const int idl_idx = R1Config::PolicyToIdl(i);
                const float start_q = last_commanded_q_valid
                    ? last_commanded_q[i]
                    : local_state.motor_state()[idl_idx].q();
                target_q[i] = start_q + std::clamp(goal_q[i] - start_q, -max_step, max_step);
                max_position_error = std::max(
                    max_position_error,
                    std::abs(local_state.motor_state()[idl_idx].q() - goal_q[i]));
                max_joint_speed = std::max(
                    max_joint_speed,
                    std::abs(local_state.motor_state()[idl_idx].dq()));
            }
            return_elapsed_s += dt;
            const bool return_pose_ready =
                max_position_error <= std::max(0.0f, t.dance_return_pos_tol) &&
                (!t.mimic_transition_v2 ||
                 max_joint_speed <= std::max(0.0f, t.mimic_transition_exit_dq_tol));
            if (return_pose_ready) {
                active_policy = pending_dance_policy;
                policy_changed = true;
                dance_handover = DanceHandoverState::kNone;
                std::cout << "[DanceHandover] default_q reached; starting Dance "
                          << (pending_dance_policy - 1) << " warmup (max_dq="
                          << max_joint_speed << ").\n";
            } else if (return_elapsed_s >= std::max(0.5f, t.dance_return_timeout_s)) {
                requested_policy.store(1);
                dance_handover = DanceHandoverState::kNone;
                dance_audio_pending = false;
                std::cerr << "[DanceHandover] return-to-default timed out (max error="
                          << max_position_error << ", max_dq=" << max_joint_speed
                          << "); dance cancelled.\n";
            }
        } else if (sit_ptr) {
            // SitController: dùng IMU để bù cổ chân (giống high_level_2)
            target_q = sit_ptr->ComputeTargetQWithIMU(local_state);
            use_sit_gains = true;
        } else {
            if (history_gesture_contract) {
                const float keep = g_gesture.Active() ? 1.0f - g_gesture.Weight() : 1.0f;
                policy->SetHistoryArmMask(keep);
            }
            std::vector<float> obs = policy->ComputeObservation(
                local_state, local_sport,
                smoothed_commands[0], smoothed_commands[1], smoothed_commands[2],
                gait_time, gait_phase
            );

            // Legacy keeps its current post-build mask/feedforward. History
            // gesture was already masked on its canonical 83-D frame above.
            if (legacy_arm_overlay_allowed && !obs.empty()) {
                float lean = 0.0f;
                if (use_teleop)              { g_teleop.MaskObs(obs);  lean = g_teleop.LeanPitchProxy(); }
                else if (g_gesture.Active()) { g_gesture.MaskObs(obs); lean = g_gesture.LeanPitchProxy(); }
                obs[3] += Tuning::Get().gesture_balance_kg * lean;
                obs[6] -= Tuning::Get().gesture_balance_kv * lean;
            }

            if (!obs.empty()) {
                std::vector<float> action = policy->Infer(obs);
                if (action.size() != R1Config::NUM_JOINTS
                    || !std::all_of(action.begin(), action.end(), [](float value) {
                        return std::isfinite(value);
                    })) {
                    throw std::runtime_error("policy returned an invalid 24-D action");
                }
                target_q = policy->ComputeTargetQ(action);
                if (use_any_meta) {
                    last_meta_observation = obs;
                    last_meta_action = action;
                    last_meta_joint_target.assign(target_q.begin(), target_q.end());
                }
                if (dance_handover == DanceHandoverState::kCooldown) {
                    if (auto* dance = dynamic_cast<DanceController*>(policy)) {
                        // Keep the actor's stabilizing target during the
                        // normal cooldown. If it remains outside the q/dq
                        // gate after max_s, blend it gradually toward the
                        // measured-to-stand reference instead of locking or
                        // making a target step.
                        const float recovery_s = std::max(0.5f, t.mimic_cooldown_s);
                        const float recovery_u = std::clamp(
                            (dance->CooldownElapsedS() - t.mimic_handover_max_s)
                                / recovery_s,
                            0.0f, 1.0f);
                        const float recovery_w = recovery_u * recovery_u
                            * (3.0f - 2.0f * recovery_u);
                        if (recovery_w > 0.0f) {
                            const auto& stand_ref = dance->CooldownReferencePosition();
                            for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
                                target_q[i] = (1.0f - recovery_w) * target_q[i]
                                    + recovery_w * stand_ref[i];
                            }
                        }
                    }
                } else if (dance_handover == DanceHandoverState::kNone
                           && active_policy >= 2
                           && t.mimic_transition_v2) {
                    if (auto* dance = dynamic_cast<DanceController*>(policy)) {
                        // During warmup, the joint-space reference trajectory is the
                        // continuous command. Let the actor take over only
                        // after the reference has reached the selected clip
                        // frame; this prevents an ONNX target jump while the
                        // observation is still being re-anchored.
                        const float u = dance->WarmupProgress();
                        const float actor_w = u * u * (3.0f - 2.0f * u);
                        if (actor_w < 1.0f) {
                            const auto& entry_ref = dance->ReferencePosition();
                            for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
                                target_q[i] = (1.0f - actor_w) * entry_ref[i]
                                    + actor_w * target_q[i];
                            }
                        }
                    }
                }
            } else {
                // Fallback: giữ nguyên vị trí khớp hiện tại
                for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
                    int idl_idx = R1Config::PolicyToIdl(i);
                    target_q[i] = local_state.motor_state()[idl_idx].q();
                }
            }
        }

        const bool dance_handover_holding =
            dance_handover == DanceHandoverState::kWaitingForLocomotionSettle ||
            dance_handover == DanceHandoverState::kReturningToDefault;

        // H4/gait uses the same final arm blend after masking the history frame.
        // During dance entry holding, retract overlays just like HB's announcing state.
        if (gesture_overlay_allowed && !dance_handover_holding) {
            if (use_teleop) g_teleop.BlendInto(target_q);
            else            g_gesture.BlendInto(target_q);
        }

        if (controller_blend_active
            && !dance_handover_holding) {
            const float duration = std::max(0.01f, t.dance_blend_time_s);
            const float u = std::clamp(controller_blend_elapsed_s / duration, 0.0f, 1.0f);
            const float weight = u * u * (3.0f - 2.0f * u);
            for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
                target_q[i] = (1.0f - weight) * controller_blend_start_q[i]
                              + weight * target_q[i];
            }
            controller_blend_elapsed_s += dt;
            controller_blend_active = controller_blend_elapsed_s < duration;
        }

        if (dance_handover == DanceHandoverState::kCooldown) {
            auto* dance = dynamic_cast<DanceController*>(policy);
            if (dance) {
                const auto& gravity = local_state.imu_state().gyroscope();
                const float gyro_norm = std::sqrt(
                    gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
                const auto& quaternion = local_state.imu_state().quaternion();
                const float gravity_x = 2.0f * (quaternion[1] * quaternion[3]
                                                 - quaternion[0] * quaternion[2]);
                const float gravity_y = 2.0f * (quaternion[0] * quaternion[1]
                                                 + quaternion[2] * quaternion[3]);
                const float tilt = std::sqrt(gravity_x * gravity_x + gravity_y * gravity_y);
                const bool settled = tilt < t.mimic_handover_tilt
                    && gyro_norm < t.mimic_handover_gyro;
                const bool pose_ready = dance->CooldownPoseReady(
                    local_state, t.mimic_transition_exit_pos_tol,
                    t.mimic_transition_exit_dq_tol);
                const float cooldown_pos_err = dance->CooldownMaxPositionError(local_state);
                const float cooldown_max_dq = dance->CooldownMaxJointSpeed(local_state);
                const bool bounded_fallback =
                    dance->TransitionV2Enabled() && !pose_ready && settled &&
                    dance->CooldownDone() &&
                    dance->CooldownElapsedS() >= t.mimic_handover_max_s &&
                    cooldown_pos_err <= t.mimic_transition_fallback_pos_tol &&
                    cooldown_max_dq <= t.mimic_transition_fallback_dq_tol;
                const bool handover_ready =
                    (settled && dance->CooldownDone()
                     && dance->CooldownElapsedS() >= t.mimic_handover_min_s
                     && pose_ready) ||
                    (!dance->TransitionV2Enabled() &&
                     dance->CooldownElapsedS() >= t.mimic_handover_max_s) ||
                    bounded_fallback;
                if (handover_ready) {
                    const int destination = cooldown_destination_policy;
                    active_policy = destination;
                    requested_policy.store(destination);
                    policy_changed = true;
                    dance_handover = DanceHandoverState::kNone;
                    dance_audio_pending = destination >= 2;
                    std::cout << "[DanceHandover] cooldown complete (tilt=" << tilt
                              << ", gyro=" << gyro_norm;
                    if (bounded_fallback) {
                        std::cout << ", bounded fallback max_pos_err=" << cooldown_pos_err
                                  << ", max_dq=" << cooldown_max_dq;
                    }
                    std::cout << "); switching to policy "
                              << destination << ".\n";
                } else if (dance->TransitionV2Enabled() &&
                           dance->CooldownElapsedS() >= t.mimic_handover_max_s &&
                           !pose_ready && !mimic_handover_block_logged) {
                    std::cerr << "[DanceHandover] transition v2 holds cooldown past max_s: "
                              << "q/dq gate not ready (tilt=" << tilt
                              << ", gyro=" << gyro_norm
                              << ", max_pos_err=" << cooldown_pos_err
                              << ", max_dq=" << cooldown_max_dq << ").\n";
                    mimic_handover_block_logged = true;
                }
            }
        }

        // Sim CỐ Ý không có cổng làm sạch như deploy (HB/high_level_2
        // LowCmdSender): ở đây mục tiêu là ĐO policy, nên clamp/thay giá trị sẽ
        // che mất một model đang phân kỳ. Thay vào đó là báo to và dừng — deploy
        // fail-closed, sim fail-loud.
        for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
            if (!std::isfinite(target_q[i])) {
                std::cerr << "[FATAL] target_q[" << i << "] khong huu han (NaN/Inf) -> dung sim.\n";
                if (g_autotest) {
                    std::cerr << "[AUTOTEST] RESULT=FAIL: policy tra target khong huu han.\n";
                    autotest_result = 4;
                    write_autotest_csv();
                }
                running = false;
                break;
            }
        }
        if (!running) break;

        std::array<float, R1Config::NUM_JOINTS> effective_kp = policy->Stiffness();
        std::array<float, R1Config::NUM_JOINTS> effective_kd = policy->Damping();
        if (use_dance_return_gains) {
            effective_kp = R1Config::DANCE_RETURN_KP_ARRAY;
            effective_kd = R1Config::DANCE_RETURN_KD_ARRAY;
        }
        if (auto* dance = dynamic_cast<DanceController*>(policy)) {
            const float warmup = dance->WarmupProgress();
            const float weight = warmup * warmup * (3.0f - 2.0f * warmup);
            const auto& handover_kp = R1Config::DANCE_RETURN_KP_ARRAY;
            const auto& handover_kd = R1Config::DANCE_RETURN_KD_ARRAY;
            for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
                const bool is_ankle = i == 4 || i == 5 || i == 10 || i == 11;
                const float gain_weight = is_ankle ? weight * weight * weight : weight;
                effective_kp[i] = (1.0f - gain_weight) * handover_kp[i]
                                  + gain_weight * policy->Stiffness()[i];
                effective_kd[i] = (1.0f - gain_weight) * handover_kd[i]
                                  + gain_weight * policy->Damping()[i];
            }
        }

        if (dance_audio_pending && dance_handover == DanceHandoverState::kNone) {
            if (auto* dance = dynamic_cast<DanceController*>(policy);
                dance && dance->WarmupProgress() >= 1.0f && active_policy >= 2) {
                const std::array<std::string, 4> audio = {
                    t.audio_dance_1, t.audio_dance_2, t.audio_dance_3, t.audio_dance_4};
                const std::string& file = audio[static_cast<std::size_t>(active_policy - 2)];
                if (!file.empty()) g_audio.Play(t.audio_dir + file);
                dance_audio_pending = false;
            }
        }

        for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
            int idl_idx = R1Config::PolicyToIdl(i);
            low_cmd.motor_cmd()[idl_idx].q() = target_q[i];
            low_cmd.motor_cmd()[idl_idx].dq() = 0.0f;
            // SitController dùng gains riêng; policy dùng đúng gains được export trong ONNX.
            low_cmd.motor_cmd()[idl_idx].kp() = use_sit_gains ? sit_controller.kp() : effective_kp[i];
            low_cmd.motor_cmd()[idl_idx].kd() = use_sit_gains ? sit_controller.kd() : effective_kd[i];
            low_cmd.motor_cmd()[idl_idx].tau() = 0.0f;
            low_cmd.motor_cmd()[idl_idx].mode() = 1;
            last_commanded_q[i] = target_q[i];
        }
        last_commanded_q_valid = true;


        int head_pitch_idl = R1Config::joint_idx_in_idl[R1Config::HEAD_PITCH_LOGICAL_INDEX];
        int head_yaw_idl = R1Config::joint_idx_in_idl[R1Config::HEAD_YAW_LOGICAL_INDEX];

        // Đầu: teleop nếu có gói hợp lệ (khi Flat), ngược lại giữ 0.
        float th_yaw = 0.0f, th_pitch = 0.0f;
        if (use_teleop && g_teleop.HeadValid()) { th_yaw = g_teleop.HeadYaw(); th_pitch = g_teleop.HeadPitch(); }

        low_cmd.motor_cmd()[head_pitch_idl].q() = th_pitch;
        low_cmd.motor_cmd()[head_pitch_idl].kp() = 10.0f;
        low_cmd.motor_cmd()[head_pitch_idl].kd() = 1.0f;
        low_cmd.motor_cmd()[head_pitch_idl].mode() = 1;

        low_cmd.motor_cmd()[head_yaw_idl].q() = th_yaw;
        low_cmd.motor_cmd()[head_yaw_idl].kp() = 10.0f;
        low_cmd.motor_cmd()[head_yaw_idl].kd() = 1.0f;
        low_cmd.motor_cmd()[head_yaw_idl].mode() = 1;

        lowcmd_publisher->Write(low_cmd);

        step++;
        next_wake_time += std::chrono::milliseconds(20);
        std::this_thread::sleep_until(next_wake_time);
    }

    if (keyboard_thread.joinable()) keyboard_thread.join();

    // CycloneDDS 0.10.2 can intermittently abort in
    // EntityDelegate::prevent_callbacks() while an autotest process tears down
    // readers that are still receiving simulator samples.  The test result and
    // CSV are already complete here, so flush them and let the OS reclaim the
    // short-lived test process without running the racy DDS destructors.  Keep
    // normal interactive operation on the explicit clean-shutdown path below.
    if (g_autotest) {
        std::cout.flush();
        std::cerr.flush();
        std::fflush(nullptr);
        std::_Exit(autotest_result);
    }

    // Dừng callback DDS trước khi các shared_ptr bị hủy tự động. Nếu để destructor
    // chạy trong lúc simulator vẫn còn publish, CycloneDDS có thể assert tại
    // EntityDelegate::prevent_callbacks() sau khi autotest đã ghi kết quả.
    sportstate_subscriber.reset();
    lowstate_subscriber.reset();
    lowcmd_publisher.reset();
    ChannelFactory::Instance()->Release();
    return autotest_result;
}
