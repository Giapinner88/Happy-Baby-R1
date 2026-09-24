#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <chrono>
#include <algorithm>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

// X11 GUI headers
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>

// Giải quyết xung đột macro Status của X11 với thư viện ONNX Runtime
#ifdef Status
#undef Status
#endif

// DDS and SDK
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

// ONNX Runtime C++ API
#include <onnxruntime_cxx_api.h>
#include "runtime/R1Config.hpp"

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;
using namespace unitree_go::msg::dds_;

// --- CONSTANTS AND CONFIGURATIONS ---
const int NUM_JOINTS = 24;
const int R1_NUM_MOTOR = 26;

// Default joint positions from training
std::array<float, 24> DEFAULT_Q = {
    -0.1f, 0.0f, 0.0f, 0.3f, -0.2f, 0.0f,   // Left leg
    -0.1f, 0.0f, 0.0f, 0.3f, -0.2f, 0.0f,   // Right leg
    0.0f, 0.0f,                             // Waist
    0.35f, 0.18f, 0.0f, 0.87f, 0.0f,        // Left arm
    0.35f, -0.18f, 0.0f, 0.87f, 0.0f        // Right arm
};

// Default action scales (will be customized dynamically at runtime for flat/rough)
std::array<float, 24> ACTION_SCALE = {
    0.22f, 0.22f, 0.22f, 0.3475f, 0.3125f, 0.3125f, // Left leg
    0.22f, 0.22f, 0.22f, 0.3475f, 0.3125f, 0.3125f, // Right leg
    0.125f, 0.22f,                                  // Waist (roll, yaw)
    0.15625f, 0.15625f, 0.15625f, 0.15625f, 0.15625f, // Left arm
    0.15625f, 0.15625f, 0.15625f, 0.15625f, 0.15625f  // Right arm
};

// Joint stiffness (Kp) from training
const std::array<float, 24> KP_ARRAY = {
    100.0f, 100.0f, 100.0f, 100.0f, 40.0f, 40.0f,   // Left leg
    100.0f, 100.0f, 100.0f, 100.0f, 40.0f, 40.0f,   // Right leg
    100.0f, 100.0f,                                 // Waist
    40.0f, 40.0f, 20.0f, 20.0f, 20.0f,              // Left arm
    40.0f, 40.0f, 20.0f, 20.0f, 20.0f               // Right arm
};

// Joint damping (Kd) from training
const std::array<float, 24> KD_ARRAY = {
    2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f,             // Left leg
    2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f,             // Right leg
    2.0f, 2.0f,                                     // Waist
    2.0f, 2.0f, 1.0f, 1.0f, 1.0f,                   // Left arm
    2.0f, 2.0f, 1.0f, 1.0f, 1.0f                    // Right arm
};

// --- REMOTE CONTROL UNION (gamepad.hpp equivalent) ---
typedef union {
    struct {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

typedef struct {
    uint8_t head[2];
    xKeySwitchUnion btn;
    float lx;
    float rx;
    float ry;
    float L2;
    float ly;
    uint8_t idle[16];
} xRockerBtnDataStruct;

typedef union {
    xRockerBtnDataStruct RF_RX;
    uint8_t buff[40];
} REMOTE_DATA_RX;

// --- FALL DETECTOR ---
class G1FallDetector {
public:
    G1FallDetector(float tilt_threshold_deg = 50.0f, float flip_tilt_deg = 30.0f, float gyro_threshold = 6.0f, float accel_threshold = 30.0f) {
        tilt_threshold = -cos(tilt_threshold_deg * M_PI / 180.0f);
        tilt_30_thresh = -cos(flip_tilt_deg * M_PI / 180.0f);
        lay_down_thresh = -cos(85.0f * M_PI / 180.0f);
        this->gyro_threshold = gyro_threshold;
        this->accel_threshold = accel_threshold;
        reset();
    }

    void reset() {
        is_fallen = false;
        is_lay_down = false;
        has_impacted = false;
    }

    bool check(const std::array<float, 3>& projected_gravity, const std::array<float, 3>& gyro, const std::array<float, 3>& accel, std::vector<std::string>& reasons) {
        float accel_norm = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
        if (accel_norm > accel_threshold) {
            has_impacted = true;
        }

        float gz = projected_gravity[2];
        bool lay_down = (gz >= lay_down_thresh) && has_impacted;
        bool new_lay_down = lay_down && !is_lay_down;
        is_lay_down = lay_down;

        if (is_fallen) {
            if (new_lay_down) {
                reasons.push_back("Robot nằm hẳn (90 độ) + Va chạm -> Ngắt toàn bộ momen");
            }
            return true;
        }

        bool fall_by_tilt = (gz > tilt_threshold);
        float gyro_norm = sqrt(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
        bool fall_by_flip = (gz > tilt_30_thresh) && (gyro_norm > gyro_threshold);

        if (fall_by_tilt) reasons.push_back("Nghiêng quá mức (gz=" + std::to_string(gz) + ")");
        if (fall_by_flip) reasons.push_back("Lật nhanh (gyro=" + std::to_string(gyro_norm) + ")");

        if (fall_by_tilt || fall_by_flip) {
            is_fallen = true;
            return true;
        }

        return false;
    }

    bool is_fallen = false;
    bool is_lay_down = false;
    bool has_impacted = false;

private:
    float tilt_threshold;
    float tilt_30_thresh;
    float lay_down_thresh;
    float gyro_threshold;
    float accel_threshold;
};

// --- GLOBAL VARIABLES FOR CONTROL LOOP & THREADS ---
bool running = true;
bool reset_requested = false;
float target_vx = 0.0f;
float target_vy = 0.0f;
float target_yaw = 0.0f;

LowState_ robot_state;
SportModeState_ sport_state;
bool got_first_state = false;
bool got_first_sport_state = false;
std::mutex state_mutex;

// Non-blocking keyboard helper functions
int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if(ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

int getch() {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

// Keyboard thread for user navigation input (with GUI window and terminal fallback)
void KeyboardThread() {
    Display *d = XOpenDisplay(NULL);
    if (d != NULL) {
        std::cout << "[GUI] Đã mở cửa sổ điều khiển đồ họa X11. Hãy bấm chọn cửa sổ 'R1 KEYBOARD CONTROL' để nhập phím!\n";

        Window w = XCreateSimpleWindow(d, RootWindow(d, 0), 10, 10, 320, 220, 1, 
                                      BlackPixel(d, 0), WhitePixel(d, 0));
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
        KeyCode kc_r = XKeysymToKeycode(d, XK_r);
        KeyCode kc_esc = XKeysymToKeycode(d, XK_Escape);

        XEvent ev;
        char keys_return[32];
        bool last_r = false;

        while (running) {
            while (XPending(d)) {
                XNextEvent(d, &ev);
                if (ev.type == Expose) {
                    XClearWindow(d, w);
                    XDrawString(d, w, gc, 20, 30, "--- CUA SO DIEU KHIEN R1 ---", 28);
                    XDrawString(d, w, gc, 20, 60, "W / S   : Tien / Lui", 20);
                    XDrawString(d, w, gc, 20, 80, "A / D   : Sang Trai / Phai", 26);
                    XDrawString(d, w, gc, 20, 100, "Q / E   : Xoay Trai / Phai", 26);
                    XDrawString(d, w, gc, 20, 130, "(Tu dong phanh khi buong tay)", 29);
                    XDrawString(d, w, gc, 20, 160, "R       : Reset Simulator", 25);
                    XDrawString(d, w, gc, 20, 190, "ESC     : Thoat", 15);
                }
            }

            XQueryKeymap(d, keys_return);

            bool is_w = (keys_return[kc_w / 8] & (1 << (kc_w % 8))) != 0;
            bool is_s = (keys_return[kc_s / 8] & (1 << (kc_s % 8))) != 0;
            bool is_a = (keys_return[kc_a / 8] & (1 << (kc_a % 8))) != 0;
            bool is_d = (keys_return[kc_d / 8] & (1 << (kc_d % 8))) != 0;
            bool is_q = (keys_return[kc_q / 8] & (1 << (kc_q % 8))) != 0;
            bool is_e = (keys_return[kc_e / 8] & (1 << (kc_e % 8))) != 0;
            bool is_r = (keys_return[kc_r / 8] & (1 << (kc_r % 8))) != 0;
            bool is_esc = (keys_return[kc_esc / 8] & (1 << (kc_esc % 8))) != 0;

            if (is_w) target_vx = 1.0f;
            else if (is_s) target_vx = -0.5f;
            else target_vx = 0.0f;

            if (is_a) target_vy = 0.5f;
            else if (is_d) target_vy = -0.5f;
            else target_vy = 0.0f;

            if (is_q) target_yaw = 1.0f;
            else if (is_e) target_yaw = -1.0f;
            else target_yaw = 0.0f;

            if (is_r && !last_r) {
                reset_requested = true;
                std::cout << "-> Lệnh (GUI): Reset trạng thái!\n";
            }
            last_r = is_r;

            if (is_esc) {
                running = false;
                std::cout << "-> Đang đóng từ GUI...\n";
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        XCloseDisplay(d);
    } else {
        std::cout << "\n=======================================================\n";
        std::cout << " BÀN PHÍM ĐIỀU KHIỂN R1 HOẠT ĐỘNG (chế độ TERMINAL)\n";
        std::cout << "-------------------------------------------------------\n";
        std::cout << "   W / S : Tiến / Lùi\n";
        std::cout << "   A / D : Sang Trái / Sang Phải\n";
        std::cout << "   Q / E : Xoay Trái / Xoay Phải\n";
        std::cout << "   SPACE : Phanh / Dừng Lại\n";
        std::cout << "   R     : Reset Mô Phỏng (Simulator Reset)\n";
        std::cout << "   X     : Thoát chương trình\n";
        std::cout << "=======================================================\n\n";

        while (running) {
            if (kbhit()) {
                int c = getch();
                if (c == 'w' || c == 'W') {
                    target_vx = std::min(1.0f, target_vx + 0.1f);
                    std::cout << "-> Lệnh (Terminal): Tiến (Vx=" << target_vx << ")\n";
                } else if (c == 's' || c == 'S') {
                    target_vx = std::max(-0.5f, target_vx - 0.1f);
                    std::cout << "-> Lệnh (Terminal): Lùi (Vx=" << target_vx << ")\n";
                } else if (c == 'a' || c == 'A') {
                    target_vy = std::min(0.5f, target_vy + 0.05f);
                    std::cout << "-> Lệnh (Terminal): Sang Trái (Vy=" << target_vy << ")\n";
                } else if (c == 'd' || c == 'D') {
                    target_vy = std::max(-0.5f, target_vy - 0.05f);
                    std::cout << "-> Lệnh (Terminal): Sang Phải (Vy=" << target_vy << ")\n";
                } else if (c == 'q' || c == 'Q') {
                    target_yaw = std::min(1.0f, target_yaw + 0.1f);
                    std::cout << "-> Lệnh (Terminal): Xoay Trái (Yaw=" << target_yaw << ")\n";
                } else if (c == 'e' || c == 'E') {
                    target_yaw = std::max(-1.0f, target_yaw - 0.1f);
                    std::cout << "-> Lệnh (Terminal): Xoay Phải (Yaw=" << target_yaw << ")\n";
                } else if (c == ' ') {
                    target_vx = 0.0f;
                    target_vy = 0.0f;
                    target_yaw = 0.0f;
                    std::cout << "-> Lệnh (Terminal): Dừng khẩn cấp!\n";
                } else if (c == 'r' || c == 'R') {
                    reset_requested = true;
                    std::cout << "-> Lệnh (Terminal): Reset trạng thái!\n";
                } else if (c == 'x' || c == 'X') {
                    running = false;
                    std::cout << "-> Đang đóng...\n";
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
}

// DDS LowState handler
void LowStateHandler(const void *message) {
    LowState_ state = *(const LowState_ *)message;
    std::lock_guard<std::mutex> lock(state_mutex);
    robot_state = state;
    if (!got_first_state) {
        std::cout << "\n[DEBUG] Đã nhận được bản tin LowState_ từ Simulator qua DDS!\n";
        got_first_state = true;
    }
}

void SportModeStateHandler(const void *message) {
    SportModeState_ state = *(const SportModeState_ *)message;
    std::lock_guard<std::mutex> lock(state_mutex);
    sport_state = state;
    if (!got_first_sport_state) {
        got_first_sport_state = true;
    }
}

// CRC32 core logic for unitree package validation
inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }
            if (data & xbit) CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

// Projected gravity calculator
std::array<float, 3> ComputeProjectedGravity(const std::array<float, 4>& quat) {
    float w = quat[0];
    float x = quat[1];
    float y = quat[2];
    float z = quat[3];

    float gx = 2.0f * (w * y - x * z);
    float gy = -2.0f * (y * z + w * x);
    float gz = 2.0f * (x * x + y * y) - 1.0f;

    return {gx, gy, gz};
}

int main(int argc, char const *argv[]) {
    if (argc < 3) {
        std::cout << "Cách sử dụng: ./run_r1_policy <network_interface> <onnx_model_path>\n";
        std::cout << "Ví dụ: ./run_r1_policy lo policy_r1_flat_2.onnx\n";
        return 0;
    }

    std::string networkInterface = argv[1];
    std::string modelPath = argv[2];

    if (networkInterface == "lo") {
        std::cout << ">>> Chạy ở chế độ GIẢ LẬP (Domain 1, Interface 'lo')" << std::endl;
        setenv("CYCLONEDDS_URI", "file:///home/khanh248/Documents/HB/Mujoco/cyclonedds_lo.xml", 1);
        ChannelFactory::Instance()->Init(1, networkInterface); // Domain 1
    } else {
        std::cout << ">>> Chạy ở chế độ ROBOT THẬT (Domain 0, Interface '" << networkInterface << "')" << std::endl;
        unsetenv("CYCLONEDDS_URI"); // Xoá cấu hình loopback để dùng multicast
        ChannelFactory::Instance()->Init(0, networkInterface); // Domain 0
    }

    // DDS Publisher
    auto lowcmd_publisher = std::make_shared<ChannelPublisher<LowCmd_>>("rt/lowcmd");
    lowcmd_publisher->InitChannel();

    // DDS Subscriber
    auto lowstate_subscriber = std::make_shared<ChannelSubscriber<LowState_>>("rt/lowstate");
    lowstate_subscriber->InitChannel(LowStateHandler, 10);

    auto sportstate_subscriber = std::make_shared<ChannelSubscriber<SportModeState_>>("rt/sportmodestate");
    sportstate_subscriber->InitChannel(SportModeStateHandler, 10);

    // --- ONNX RUNTIME INITIALIZATION ---
    std::cout << "Đang tải mô hình ONNX: " << modelPath << std::endl;
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "r1_policy");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    Ort::Session session(env, modelPath.c_str(), session_options);
    Ort::AllocatorWithDefaultOptions allocator;

    // --- READ METADATA FOR DYNAMIC SCALING ---
    try {
        Ort::ModelMetadata metadata = session.GetModelMetadata();
        
        try {
            Ort::AllocatedStringPtr action_scale_ptr = metadata.LookupCustomMetadataMapAllocated("action_scale", allocator);
            if (action_scale_ptr) {
                std::string action_scale_str(action_scale_ptr.get());
                std::stringstream ss(action_scale_str);
                std::string token;
                int i = 0;
                while (std::getline(ss, token, ',') && i < 24) {
                    ACTION_SCALE[i] = std::stof(token);
                    i++;
                }
                std::cout << "-> Đã nạp tự động ACTION_SCALE từ ONNX Metadata.\n";
            }
        } catch (...) {}
        
        try {
            Ort::AllocatedStringPtr default_pos_ptr = metadata.LookupCustomMetadataMapAllocated("default_joint_pos", allocator);
            if (default_pos_ptr) {
                std::string default_pos_str(default_pos_ptr.get());
                std::stringstream ss(default_pos_str);
                std::string token;
                int i = 0;
                while (std::getline(ss, token, ',') && i < 24) {
                    DEFAULT_Q[i] = std::stof(token);
                    i++;
                }
                std::cout << "-> Đã nạp tự động DEFAULT_JOINT_POS từ ONNX Metadata.\n";
            }
        } catch (...) {}
    } catch (...) {
        std::cout << "[INFO] Không thể đọc Metadata, sử dụng thông số mặc định.\n";
    }

    // Get input shapes
    auto type_info = session.GetInputTypeInfo(0);
    auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> input_node_dims = tensor_info.GetShape();
    int expected_dim = input_node_dims[1];
    std::cout << "Nạp model ONNX thành công! Chiều của vector đầu vào: " << expected_dim << std::endl;

    // Customize action scales and print status based on flat vs rough terrain
    if (expected_dim == 83) {
        std::cout << "-> Phát hiện: MODEL DÀNH CHO MẶT PHẲNG (FLAT POLICY - 83 inputs)\n";
    } else if (expected_dim == 270) {
        std::cout << "-> Phát hiện: MODEL ĐỊA HÌNH MẤP MÔ (ROUGH TERRAIN POLICY - 270 inputs)\n";
        std::cout << "-> Đang sử dụng chế độ Fallback địa hình phẳng cho height_scan!\n";
    } else {
        std::cout << "[CẢNH BÁO] Chiều vector đầu vào không quen thuộc: " << expected_dim << ". Hủy chạy.\n";
        return 1;
    }

    std::string input_name = session.GetInputNameAllocated(0, allocator).get();
    std::string output_name = session.GetOutputNameAllocated(0, allocator).get();

    // Start keyboard input thread
    std::thread keyboard_thread(KeyboardThread);

    // Control parameters
    std::array<float, 24> last_action = {0.0f};
    std::array<float, 3> smoothed_commands = {0.0f};
    float alpha = 0.1f; // Command smoothing coefficient
    float gait_time = 0.0f;
    float gait_scale = 1.0f;
    float dt = 0.02f; // Loop time (50Hz)

    G1FallDetector fall_detector;

    // Main Control Loop at 50Hz (20ms)
    auto next_wake_time = std::chrono::steady_clock::now();
    int step = 0;

    std::cout << "\n>>> ĐANG CHỜ PHẢN HỒI THỰC TẾ TỪ ROBOT STATE...\n";
    while (running) {
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            if (got_first_state) {
                // Ensure state isn't zero (e.g. during startup)
                float q_sum = 0.0f;
                for (int i = 0; i < NUM_JOINTS; ++i) {
                    int idl_idx = R1Config::PolicyToIdl(i);
                    q_sum += std::abs(robot_state.motor_state()[idl_idx].q());
                }
                if (q_sum > 0.01f) {
                    break;
                } else {
                    std::cout << "[DEBUG] Nhận được State nhưng q_sum (" << q_sum << ") quá nhỏ (robot chưa nhúc nhích), tiếp tục đợi...\n";
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << ">>> NHẬN STATE OK! BẮT ĐẦU CHẠY POLICY...\n";

    while (running) {
        next_wake_time += std::chrono::milliseconds(20);

        LowState_ current_state;
        SportModeState_ current_sport_state;
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            current_state = robot_state;
            current_sport_state = sport_state;
        }

        // --- 1. RESET LOGIC ---
        if (reset_requested) {
            std::cout << ">>> ĐANG GỬI LỆNH RESET SANG SIMULATOR...\n";
            LowCmd_ reset_cmd;
            reset_cmd.motor_cmd()[0].mode() = 0xFF; // Special simulator reset code
            reset_cmd.crc() = Crc32Core((uint32_t *)&reset_cmd, (sizeof(reset_cmd) >> 2) - 1);
            lowcmd_publisher->Write(reset_cmd);

            // Restore defaults
            fall_detector.reset();
            std::fill(last_action.begin(), last_action.end(), 0.0f);
            std::fill(smoothed_commands.begin(), smoothed_commands.end(), 0.0f);
            target_vx = 0.0f;
            target_vy = 0.0f;
            target_yaw = 0.0f;
            gait_time = 0.0f;
            gait_scale = 1.0f;
            step = 0;
            reset_requested = false;

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            next_wake_time = std::chrono::steady_clock::now();
            continue;
        }

        // --- 2. EXTRACT JOYSTICK / PHYSICAL REMOTE CONTROL ---
        float rc_vx = 0.0f;
        float rc_vy = 0.0f;
        float rc_yaw = 0.0f;
        
        REMOTE_DATA_RX rx;
        std::copy(current_state.wireless_remote().begin(), current_state.wireless_remote().end(), rx.buff);
        
        // If joystick inputs are active (outside deadzone)
        if (std::abs(rx.RF_RX.ly) > 0.05f || std::abs(rx.RF_RX.lx) > 0.05f || std::abs(rx.RF_RX.rx) > 0.05f) {
            rc_vx = rx.RF_RX.ly * 1.0f;       // Vertical Left: Forward/Backward
            rc_vy = rx.RF_RX.lx * 0.5f;      // Horizontal Left: Sideways
            rc_yaw = rx.RF_RX.rx * 1.0f;     // Horizontal Right: Yaw
            
            // Overwrite keyboard targets when RC is actively steered
            target_vx = rc_vx;
            target_vy = rc_vy;
            target_yaw = rc_yaw;
        }

        // --- 3. COMMAND SMOOTHING ---
        smoothed_commands[0] = alpha * target_vx + (1.0f - alpha) * smoothed_commands[0];
        smoothed_commands[1] = alpha * target_vy + (1.0f - alpha) * smoothed_commands[1];
        smoothed_commands[2] = alpha * target_yaw + (1.0f - alpha) * smoothed_commands[2];

        // --- 4. GAIT PHASE GENERATOR ---
        bool moving = (std::abs(smoothed_commands[0]) > 0.01f || std::abs(smoothed_commands[1]) > 0.01f || std::abs(smoothed_commands[2]) > 0.01f);
        if (moving) {
            gait_time += dt;
            gait_scale = std::min(1.0f, gait_scale + dt / 0.3f);
        } else {
            float remainder = fmod(gait_time, 0.6f);
            if (remainder > 0.02f && remainder < 0.58f) {
                gait_time += dt;
                gait_scale = std::min(1.0f, gait_scale + dt / 0.3f);
            } else {
                gait_time = round(gait_time / 0.6f) * 0.6f;
                gait_scale = std::max(0.0f, gait_scale - dt / 0.3f);
            }
        }

        float phase_ratio = fmod(gait_time, 0.6f) / 0.6f;
        std::array<float, 2> gait_phase = {
            sin(2.0f * (float)M_PI * phase_ratio) * gait_scale,
            cos(2.0f * (float)M_PI * phase_ratio) * gait_scale
        };

        if (sqrt(smoothed_commands[0]*smoothed_commands[0] + smoothed_commands[1]*smoothed_commands[1]) < 0.1f) {
            gait_phase[0] = 0.0f;
            gait_phase[1] = 0.0f;
        }

        // --- 5. IMU STATES ---
        std::array<float, 4> quat = current_state.imu_state().quaternion();
        std::array<float, 3> gyro = current_state.imu_state().gyroscope();
        std::array<float, 3> accel = current_state.imu_state().accelerometer();

        std::array<float, 3> projected_gravity = ComputeProjectedGravity(quat);

        // --- 6. FALL DETECTOR ---
        std::vector<std::string> fall_reasons;
        bool is_fallen = fall_detector.check(projected_gravity, gyro, accel, fall_reasons);

        // Bỏ qua cảm biến ngã trong 2 giây đầu tiên (100 steps) để chịu lực rơi từ trên không
        if (step < 100) {
            is_fallen = false;
            fall_reasons.clear();
            fall_detector.reset();
        }

        if (is_fallen) {
            if (!fall_reasons.empty()) {
                std::cout << "\n[CẢNH BÁO] PHÁT HIỆN ROBOT NGÃ: ";
                for (const auto& reason : fall_reasons) std::cout << reason << " | ";
                std::cout << "\n--> TỰ ĐỘNG TẮT ĐỘNG CƠ ĐỂ BẢO VỆ!\n";
            }

            // Publish safety command (Kp = Kd = tau = 0)
            LowCmd_ safety_cmd;
            for (int i = 0; i < R1_NUM_MOTOR; ++i) {
                int idl_idx = R1Config::joint_idx_in_idl[i];
                safety_cmd.motor_cmd()[idl_idx].mode() = 1;
                safety_cmd.motor_cmd()[idl_idx].kp() = 0.0f;
                safety_cmd.motor_cmd()[idl_idx].kd() = 0.0f;
                safety_cmd.motor_cmd()[idl_idx].q() = current_state.motor_state()[idl_idx].q();
                safety_cmd.motor_cmd()[idl_idx].tau() = 0.0f;
            }
            safety_cmd.crc() = Crc32Core((uint32_t *)&safety_cmd, (sizeof(safety_cmd) >> 2) - 1);
            lowcmd_publisher->Write(safety_cmd);

            // Auto-detect if user pressed Backspace in MuJoCo to teleport robot
            if (projected_gravity[2] < -0.9f) {
                std::cout << "\n[INFO] Phát hiện robot đã được dựng đứng (Backspace trong MuJoCo). Tự động Reset Controller!\n";
                for(int i=0; i<24; ++i) last_action[i] = 0.0f;
                smoothed_commands[0] = 0.0f;
                smoothed_commands[1] = 0.0f;
                smoothed_commands[2] = 0.0f;
                gait_time = 0.0f;
                gait_scale = 1.0f;
                fall_detector.reset();
                step = 0; // Reset grace period
            }

            std::this_thread::sleep_until(next_wake_time);
            continue;
        }

        // --- 7. EXTRACT JOINT STATES (24 policy joints) ---
        std::array<float, 24> q_current;
        std::array<float, 24> dq_current;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            int idl_idx = R1Config::PolicyToIdl(i);
            q_current[i] = current_state.motor_state()[idl_idx].q();
            dq_current[i] = current_state.motor_state()[idl_idx].dq();
        }

        // --- 8. BUILD QUANT OBSERVATION VECTOR ---
        std::vector<float> obs(expected_dim, 0.0f);
        int offset = 0;

        // Gyro (3)
        obs[offset++] = gyro[0];
        obs[offset++] = gyro[1];
        obs[offset++] = gyro[2];

        // Projected Gravity (3)
        obs[offset++] = projected_gravity[0];
        obs[offset++] = projected_gravity[1];
        obs[offset++] = projected_gravity[2];

        // Smoothed Commands (3)
        obs[offset++] = smoothed_commands[0];
        obs[offset++] = smoothed_commands[1];
        obs[offset++] = smoothed_commands[2];

        // Gait Phase (2) - Zero out when stationary like in Python rough script
        float cmd_norm = std::sqrt(smoothed_commands[0]*smoothed_commands[0] + 
                                   smoothed_commands[1]*smoothed_commands[1] + 
                                   smoothed_commands[2]*smoothed_commands[2]);
        if (cmd_norm < 0.1f) {
            obs[offset++] = 0.0f;
            obs[offset++] = 0.0f;
        } else {
            obs[offset++] = gait_phase[0];
            obs[offset++] = gait_phase[1];
        }

        // q_rel (24)
        for (int i = 0; i < 24; ++i) {
            obs[offset++] = q_current[i] - DEFAULT_Q[i];
        }

        // dq (24)
        for (int i = 0; i < 24; ++i) {
            obs[offset++] = dq_current[i];
        }

        // last_action (24)
        for (int i = 0; i < 24; ++i) {
            obs[offset++] = last_action[i];
        }

        // Height Scan Fallback for Rough terrain (187 inputs)
        if (expected_dim == 270) {
            float p_pelvis_z = 0.76f;
            if (got_first_sport_state) {
                p_pelvis_z = current_sport_state.position()[2];
            }
            // If ground is flat at z=0, relative_height = p_pelvis_z - 0
            // Scale by 0.2f
            float relative_height_scaled = p_pelvis_z * 0.2f; 
            for (int i = 83; i < 270; ++i) {
                obs[i] = relative_height_scaled;
            }
        }

        // --- 9. ONNX INFERENCE ---
        std::vector<int64_t> input_shape = {1, expected_dim};
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, obs.data(), obs.size(), input_shape.data(), input_shape.size()
        );

        const char* input_names[] = {input_name.c_str()};
        const char* output_names[] = {output_name.c_str()};

        auto output_tensors = session.Run(
            Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1
        );

        float* output_data = output_tensors[0].GetTensorMutableData<float>();

        // Copy action to last_action and calculate joint target positions
        std::array<float, 24> target_q;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            last_action[i] = output_data[i];
            target_q[i] = DEFAULT_Q[i] + last_action[i] * ACTION_SCALE[i];
        }

        // --- 10. PUBLISH COMMANDS TO DDS ---
        LowCmd_ low_cmd;
        low_cmd.mode_machine() = current_state.mode_machine();
        low_cmd.mode_pr() = 0; // PR mode

        // Map policy targets to IDL joints
        for (int i = 0; i < NUM_JOINTS; ++i) {
            int idl_idx = R1Config::PolicyToIdl(i);
            low_cmd.motor_cmd()[idl_idx].mode() = 1; // Enable
            low_cmd.motor_cmd()[idl_idx].q() = target_q[i];
            low_cmd.motor_cmd()[idl_idx].dq() = 0.0f;
            low_cmd.motor_cmd()[idl_idx].kp() = KP_ARRAY[i];
            low_cmd.motor_cmd()[idl_idx].kd() = KD_ARRAY[i];
            low_cmd.motor_cmd()[idl_idx].tau() = 0.0f;
        }

        // Head joints holding straight posture
        int head_pitch_idl = R1Config::joint_idx_in_idl[R1Config::HEAD_PITCH_LOGICAL_INDEX];
        int head_yaw_idl = R1Config::joint_idx_in_idl[R1Config::HEAD_YAW_LOGICAL_INDEX];
        
        low_cmd.motor_cmd()[head_pitch_idl].mode() = 1;
        low_cmd.motor_cmd()[head_pitch_idl].q() = 0.0f;
        low_cmd.motor_cmd()[head_pitch_idl].dq() = 0.0f;
        low_cmd.motor_cmd()[head_pitch_idl].kp() = 10.0f;
        low_cmd.motor_cmd()[head_pitch_idl].kd() = 1.0f;
        low_cmd.motor_cmd()[head_pitch_idl].tau() = 0.0f;

        low_cmd.motor_cmd()[head_yaw_idl].mode() = 1;
        low_cmd.motor_cmd()[head_yaw_idl].q() = 0.0f;
        low_cmd.motor_cmd()[head_yaw_idl].dq() = 0.0f;
        low_cmd.motor_cmd()[head_yaw_idl].kp() = 10.0f;
        low_cmd.motor_cmd()[head_yaw_idl].kd() = 1.0f;
        low_cmd.motor_cmd()[head_yaw_idl].tau() = 0.0f;

        // Calculate CRC and write command
        low_cmd.crc() = Crc32Core((uint32_t *)&low_cmd, (sizeof(low_cmd) >> 2) - 1);
        lowcmd_publisher->Write(low_cmd);

        // DEBUG: In thông tin policy mỗi 50 bước
        if (step % 50 == 0) {
            std::cout << "[DBG s=" << step << "] "
                      << "gz=" << projected_gravity[2] 
                      << " | cmd_vx=" << smoothed_commands[0]
                      << " | hip_L_q=" << target_q[0] 
                      << " | knee_L_q=" << target_q[3]
                      << " | arm_L_q=" << target_q[14] << std::endl;
            if (step == 0) {
                std::cout << "[DBG OBS] ";
                for (int i=0; i<10; i++) std::cout << obs[i] << " ";
                std::cout << "| height=" << obs[83] << " | action=" << last_action[0] << " | DEFAULT_Q[0]=" << DEFAULT_Q[0] << " | ACTION_SCALE[0]=" << ACTION_SCALE[0] << "\n";
            }
        }
        step++;
        std::this_thread::sleep_until(next_wake_time);
    }

    keyboard_thread.join();
    std::cout << "Đã tắt bộ điều khiển C++ RL Policy R1.\n";
    return 0;
}
