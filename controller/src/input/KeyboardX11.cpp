#include "KeyboardX11.hpp"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include <unistd.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#ifdef Status
#undef Status
#endif

namespace {

std::function<void()> g_emergency_damp;

int64_t NowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

int XIOErrorCb(Display*) {
    std::cout << "\n[FATAL] !!! ĐỨT KẾT NỐI X11 (SSH) — dập motor khẩn cấp trước khi sập !!!\n";
    if (g_emergency_damp) g_emergency_damp();
    ::exit(1);
    return 0;
}

} // namespace

void KeyboardX11::SetEmergencyDampFn(std::function<void()> fn) {
    g_emergency_damp = std::move(fn);
}

void KeyboardX11::Start() {
    const char* disp = getenv("DISPLAY");
    if (dev_no_keyboard_ || !disp || disp[0] == '\0') {
        std::cout << "[KeyboardX11] Bỏ qua bàn phím X11 ("
                  << (dev_no_keyboard_ ? "dev_no_keyboard=true" : "$DISPLAY trống")
                  << ") — điều khiển thuần gamepad R3-1.\n";
        display_ok_ = false;
        return;
    }
    if (running_.exchange(true)) return;
    last_event_ms_ = NowMs();
    thread_ = std::thread(&KeyboardX11::ThreadFn, this);
}

void KeyboardX11::Stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void KeyboardX11::SetHud(const std::string& status, const std::vector<std::string>& hints) {
    std::lock_guard<std::mutex> lock(hud_mutex_);
    if (status == hud_status_ && hints == hud_hints_) return;
    hud_status_ = status;
    hud_hints_ = hints;
    hud_dirty_ = true;
}

InputCommand KeyboardX11::GetCommand() {
    InputCommand cmd;
    cmd.is_active = display_ok_.load();

    if (NowMs() - last_event_ms_.load() > static_cast<int64_t>(release_ms_)) {
        key_w_ = key_s_ = key_a_ = key_d_ = key_q_ = key_e_ = false;
    }

    if (want_stand_lock_.exchange(false)) cmd.want_stand_lock = true;
    if (want_locomotion_.exchange(false)) cmd.want_locomotion = true;
    cmd.want_mimic_key = want_mimic_key_.exchange(0);
    if (want_reset_.exchange(false))      cmd.want_reset = true;
    if (want_safe_shutdown_.exchange(false))   cmd.want_safe_shutdown = true;
    if (want_speed_toggle_.exchange(false)) cmd.want_speed_toggle = true;
    if (want_exit_.load())                cmd.want_emergency_stop = true;

    if (key_w_) cmd.vx += 1.0f;
    if (key_s_) cmd.vx -= 1.0f;
    if (key_a_) cmd.vy += 1.0f;
    if (key_d_) cmd.vy -= 1.0f;
    if (key_q_) cmd.yaw += 1.0f;
    if (key_e_) cmd.yaw -= 1.0f;

    return cmd;
}

void KeyboardX11::ThreadFn() {
    XSetIOErrorHandler(XIOErrorCb);

    Display* display = XOpenDisplay(nullptr);
    if (!display) {
        std::cerr << "[KeyboardX11] Không mở được X11 Display ($DISPLAY chưa set?) "
                     "— chỉ điều khiển được bằng gamepad.\n";
        return;
    }

    int screen = DefaultScreen(display);
    Window window = XCreateSimpleWindow(
        display, RootWindow(display, screen), 10, 10, 460, 340, 1,
        BlackPixel(display, screen), WhitePixel(display, screen));
    XStoreName(display, window, "R1 KEYBOARD CONTROL");

    XSelectInput(display, window,
                 ExposureMask | KeyPressMask | KeyReleaseMask | FocusChangeMask);
    XMapWindow(display, window);

    Atom wm_delete = XInternAtom(display, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(display, window, &wm_delete, 1);

    XWMHints wm_hints = {};
    wm_hints.flags = InputHint;
    wm_hints.input = True;
    XSetWMHints(display, window, &wm_hints);
    XFlush(display);

    GC gc = XCreateGC(display, window, 0, nullptr);
    XSetForeground(display, gc, BlackPixel(display, screen));

    bool has_focus = false;
    display_ok_ = true;
    std::cout << "[KeyboardX11] Cửa sổ điều khiển đã mở.\n";

    auto set_key = [&](KeySym sym, bool value) {
        if (sym == XK_w || sym == XK_W) key_w_ = value;
        if (sym == XK_s || sym == XK_S) key_s_ = value;
        if (sym == XK_a || sym == XK_A) key_a_ = value;
        if (sym == XK_d || sym == XK_D) key_d_ = value;
        if (sym == XK_q || sym == XK_Q) key_q_ = value;
        if (sym == XK_e || sym == XK_E) key_e_ = value;
    };

    while (running_) {
        while (XPending(display)) {
            last_event_ms_ = NowMs();
            XEvent ev;
            XNextEvent(display, &ev);

            if (ev.type == FocusIn) {
                has_focus = true;
                hud_dirty_ = true;
            } else if (ev.type == FocusOut) {
                has_focus = false;
                key_w_ = key_s_ = key_a_ = key_d_ = key_q_ = key_e_ = false;
                hud_dirty_ = true;
            } else if (ev.type == KeyPress) {
                KeySym sym = XLookupKeysym(&ev.xkey, 0);
                set_key(sym, true);
                if (sym == XK_0) want_stand_lock_ = true;
                if (sym == XK_1) want_locomotion_ = true;
                if (sym == XK_2) want_mimic_key_ = 2;
                if (sym == XK_3) want_mimic_key_ = 3;
                if (sym == XK_4) want_mimic_key_ = 4;
                if (sym == XK_5) want_mimic_key_ = 5;
                if (sym == XK_6) want_mimic_key_ = 6;
                if (sym == XK_7) want_mimic_key_ = 7;
                if (sym == XK_8) want_mimic_key_ = 8;
                if (sym == XK_9) want_safe_shutdown_ = true;
                if (sym == XK_r || sym == XK_R) want_reset_ = true;
                if (sym == XK_Tab) want_speed_toggle_ = true;
                if (sym == XK_Escape) {
                    std::cout << "[KeyboardX11] ESC — dừng khẩn cấp.\n";
                    want_exit_ = true;
                }
            } else if (ev.type == KeyRelease) {
                if (XPending(display)) {
                    XEvent peek;
                    XPeekEvent(display, &peek);
                    if (peek.type == KeyPress && peek.xkey.keycode == ev.xkey.keycode &&
                        (peek.xkey.time - ev.xkey.time) < 5) {
                        XNextEvent(display, &peek);
                        continue;
                    }
                }
                set_key(XLookupKeysym(&ev.xkey, 0), false);
            } else if (ev.type == Expose) {
                hud_dirty_ = true;
            } else if (ev.type == ClientMessage) {
                if (static_cast<Atom>(ev.xclient.data.l[0]) == wm_delete) {
                    want_exit_ = true;
                }
            }
        }

        if (hud_dirty_.exchange(false)) {
            XClearWindow(display, window);
            if (!has_focus) {
                const char* msg = ">>> CLICK VAO CUA SO NAY DE DIEU KHIEN <<<";
                XDrawString(display, window, gc, 20, 160, msg, static_cast<int>(strlen(msg)));
            } else {
                std::lock_guard<std::mutex> lock(hud_mutex_);
                const char* title = "=== R1 ROBOT CONTROL ===";
                XDrawString(display, window, gc, 20, 30, title, static_cast<int>(strlen(title)));
                XDrawString(display, window, gc, 20, 70, hud_status_.c_str(),
                            static_cast<int>(hud_status_.length()));
                int y = 110;
                for (const auto& line : hud_hints_) {
                    XDrawString(display, window, gc, 20, y, line.c_str(),
                                static_cast<int>(line.length()));
                    y += 20;
                }
                const char* esc = "ESC : Dung khan cap va thoat";
                XDrawString(display, window, gc, 20, 300, esc, static_cast<int>(strlen(esc)));
            }
            XFlush(display);
        }

        usleep(20000);
    }

    display_ok_ = false;
    XFreeGC(display, gc);
    XDestroyWindow(display, window);
    XCloseDisplay(display);
}
