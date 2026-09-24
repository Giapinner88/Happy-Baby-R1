#pragma once
/**
 * LocalAudioPlayer.hpp — Phát file âm thanh MP3/WAV ngầm trên PC giả lập.
 * Dùng `ffplay` (thuộc ffmpeg) để phát ra loa máy tính.
 * Play()/Stop() không chặn luồng điều khiển 50Hz.
 */

#include <string>
#include <cstdio>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <iostream>

class LocalAudioPlayer {
public:
    LocalAudioPlayer() = default;
    ~LocalAudioPlayer() { Stop(); }

    // Phát file âm thanh (MP3/WAV/...). Nếu đang phát thì dừng bài cũ trước.
    void Play(const std::string& path) {
        if (path.empty()) return;
        Stop(); // dừng bài cũ nếu có

        pid_t pid = fork();
        if (pid < 0) {
            std::cerr << "[Audio] fork() thất bại.\n";
            return;
        }
        if (pid == 0) {
            // Tiến trình con: tắt stderr rồi chạy ffplay
            int nul = open("/dev/null", O_WRONLY);
            if (nul >= 0) {
                dup2(nul, STDERR_FILENO);
                dup2(nul, STDOUT_FILENO);
                if (nul > 2) close(nul);
            }
            execlp("ffplay", "ffplay", "-nodisp", "-autoexit",
                   "-loglevel", "quiet", path.c_str(), (char*)nullptr);
            _exit(127); // ffplay không tồn tại
        }
        // Tiến trình cha lưu pid để có thể dừng sau
        child_pid_ = pid;
    }

    // Dừng âm thanh đang phát (nếu có).
    void Stop() {
        if (child_pid_ > 0) {
            kill(child_pid_, SIGKILL);
            waitpid(child_pid_, nullptr, WNOHANG);
            child_pid_ = -1;
        }
    }

    bool IsPlaying() const { return child_pid_ > 0; }

private:
    pid_t child_pid_ = -1;
};
