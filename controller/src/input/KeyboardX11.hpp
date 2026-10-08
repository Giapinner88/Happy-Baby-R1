#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../config/Tuning.hpp"
#include "InputCommand.hpp"

// Bộ đọc bàn phím qua cửa sổ X11 (hỗ trợ SSH -Y)
class KeyboardX11 {
public:
    ~KeyboardX11() { Stop(); }

    void Configure(const Tuning& tuning) {
        release_ms_ = tuning.x11_release_ms;
        dev_no_keyboard_ = tuning.dev_no_keyboard;
    }

    // Đăng ký hàm xả lực khẩn cấp khi đứt kết nối X11
    static void SetEmergencyDampFn(std::function<void()> fn);

    void Start();
    void Stop();

    InputCommand GetCommand();
    bool WantExit() const { return want_exit_.load(); }
    bool available() const { return display_ok_.load(); }

    // Cập nhật giao diện hướng dẫn phím bấm
    void SetHud(const std::string& status, const std::vector<std::string>& hints);

private:
    void ThreadFn();

    std::thread thread_;
    std::atomic<bool> running_{false};

    // Trạng thái các phím di chuyển
    std::atomic<bool> key_w_{false}, key_s_{false}, key_a_{false};
    std::atomic<bool> key_d_{false}, key_q_{false}, key_e_{false};

    // Sự kiện phím chức năng
    std::atomic<bool> want_stand_lock_{false}, want_locomotion_{false};
    std::atomic<int> want_mimic_key_{0};
    std::atomic<bool> want_reset_{false};
    std::atomic<bool> want_speed_toggle_{false}, want_exit_{false};
    std::atomic<bool> want_safe_shutdown_{false};

    std::mutex hud_mutex_;
    std::string hud_status_ = "KHOI DONG...";
    std::vector<std::string> hud_hints_;
    std::atomic<bool> hud_dirty_{true};

    float release_ms_ = 2500.0f;
    bool dev_no_keyboard_ = false;
    std::atomic<bool> display_ok_{false};
    std::atomic<int64_t> last_event_ms_{0};
};
