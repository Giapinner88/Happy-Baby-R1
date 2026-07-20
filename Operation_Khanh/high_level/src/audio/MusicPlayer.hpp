#pragma once
/**
 * MusicPlayer.hpp — Quản lý âm thanh cho robot (nhạc nền + thông báo).
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <unitree/robot/a2/audio/audio_client.hpp>

class MusicPlayer {
public:
    MusicPlayer() = default;
    ~MusicPlayer() {
        {
            std::lock_guard<std::mutex> lk(mu_);
            shutdown_ = true;
            ++gen_;
        }
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
    }

    MusicPlayer(const MusicPlayer&) = delete;
    MusicPlayer& operator=(const MusicPlayer&) = delete;

    // Khởi tạo AudioClient
    void InitAudio() {
        try {
            client_ = std::make_unique<unitree::robot::a2::AudioClient>();
            client_->Init();
            client_->SetTimeout(3.0f);
            ready_ = true;
            thread_ = std::thread(&MusicPlayer::Loop, this);
        } catch (const std::exception& e) {
            fprintf(stderr, "[MusicPlayer] Init AudioClient lỗi: %s -> tắt audio.\n",
                    e.what());
            ready_ = false;
        }
    }

    bool ready() const { return ready_; }

    // Phát nhạc nền
    void Play(const std::string& path, float vol01) {
        Enqueue(kMusic, path, vol01, 0);
    }

    // Thông báo voice (path tới file hoặc text cho TTS)
    void Say(const std::string& text_or_path, float vol01, int speaker_id) {
        if (text_or_path.empty()) return;
        Enqueue(kVoice, text_or_path, vol01, speaker_id);
    }

    // Dừng âm thanh
    void Stop() {
        Enqueue(kStop, "", 0.0f, 0);
    }

private:
    enum JobType { kMusic, kVoice, kStop };

    static constexpr const char* kApp = "hb_audio";
    static constexpr size_t kChunkBytes = 16000;
    static constexpr int kPrimeChunks = 3;
    static constexpr double kBytesPerSec = 32000.0;
    static constexpr double kTailMarginS = 0.3;

    void Enqueue(JobType type, const std::string& arg, float vol, int speaker) {
        if (!ready_) return;
        {
            std::lock_guard<std::mutex> lk(mu_);
            req_type_ = type;
            req_arg_ = arg;
            req_vol_ = vol;
            req_speaker_ = speaker;
            ++gen_;
        }
        cv_.notify_all();
    }

    // Thread audio
    void Loop() {
        uint64_t seen = 0;
        for (;;) {
            JobType type;
            std::string arg;
            float vol;
            int speaker;
            uint64_t mygen;
            {
                std::unique_lock<std::mutex> lk(mu_);
                cv_.wait(lk, [&] { return shutdown_ || gen_ != seen; });
                if (shutdown_) return;
                seen = gen_;
                mygen = gen_;
                type = req_type_;
                arg = req_arg_;
                vol = req_vol_;
                speaker = req_speaker_;
            }
            if (type == kStop) {
                if (ready_) client_->PlayStop(kApp);
            } else if (type == kVoice) {
                PlayVoice(arg, vol, speaker, mygen);
            } else if (type == kMusic) {
                if (!arg.empty()) StreamFile(arg, vol, mygen);
            }
        }
    }

    static bool FileExists(const std::string& p) {
        return !p.empty() && std::ifstream(p).good();
    }

    // Voice: nếu file tồn tại -> phát file, ngược lại -> TTS.
    void PlayVoice(const std::string& text_or_path, float vol, int speaker,
                   uint64_t mygen) {
        if (!ready_) return;
        if (FileExists(text_or_path)) {
            StreamFile(text_or_path, vol, mygen);
        } else {
            float vc = vol < 0.0f ? 0.0f : (vol > 1.0f ? 1.0f : vol);
            client_->SetVolume(static_cast<uint8_t>(vc * 100.0f + 0.5f));
            client_->TtsMaker(text_or_path, speaker);
        }
    }

    void StreamFile(const std::string& path, float vol, uint64_t mygen) {
        if (!std::ifstream(path).good()) {
            fprintf(stderr, "[MusicPlayer] Không thấy file audio: %s\n", path.c_str());
            return;
        }
        float vc = vol < 0.0f ? 0.0f : (vol > 1.0f ? 1.0f : vol);
        client_->SetVolume(static_cast<uint8_t>(vc * 100.0f + 0.5f));

        // Decode ffmpeg thành luồng s16le
        int fds[2];
        if (pipe(fds) != 0) return;
        pid_t pid = fork();
        if (pid < 0) {
            close(fds[0]);
            close(fds[1]);
            return;
        }
        if (pid == 0) {
            dup2(fds[1], STDOUT_FILENO);
            int nul = open("/dev/null", O_WRONLY);
            if (nul >= 0) { dup2(nul, STDERR_FILENO); if (nul > 2) close(nul); }
            close(fds[0]);
            close(fds[1]);
            execlp("ffmpeg", "ffmpeg", "-i", path.c_str(), "-f", "s16le",
                   "-acodec", "pcm_s16le", "-ac", "1", "-ar", "16000",
                   "-loglevel", "quiet", "pipe:1", static_cast<char*>(nullptr));
            _exit(127);
        }
        close(fds[1]);

        std::string stream_id = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());

        std::vector<uint8_t> chunk;
        chunk.reserve(kChunkBytes);
        uint8_t buf[4096];
        int chunk_idx = 0;
        bool aborted = false;
        size_t total_bytes = 0;
        auto t0 = std::chrono::steady_clock::now();

        while (!aborted) {
            if (Aborted(mygen)) { aborted = true; break; }
            ssize_t n = read(fds[0], buf, sizeof(buf));
            if (n <= 0) break;
            chunk.insert(chunk.end(), buf, buf + n);
            while (chunk.size() >= kChunkBytes) {
                std::vector<uint8_t> out(chunk.begin(), chunk.begin() + kChunkBytes);
                chunk.erase(chunk.begin(), chunk.begin() + kChunkBytes);
                client_->PlayStream(kApp, stream_id, out);
                total_bytes += out.size();
                if (++chunk_idx > kPrimeChunks) SleepAbortable(500, mygen);
                if (Aborted(mygen)) { aborted = true; break; }
            }
        }
        if (!aborted && !chunk.empty()) {
            client_->PlayStream(kApp, stream_id, chunk);
            total_bytes += chunk.size();
        }

        kill(pid, SIGKILL);
        waitpid(pid, nullptr, 0);
        close(fds[0]);

        // Chờ nốt phần đã gửi mà chưa phát xong
        if (!aborted) {
            double audio_s   = static_cast<double>(total_bytes) / kBytesPerSec;
            double elapsed_s = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - t0).count();
            double remain_s  = audio_s - elapsed_s + kTailMarginS;
            if (remain_s > 0.0)
                SleepAbortable(static_cast<int>(remain_s * 1000.0), mygen);
        }
        client_->PlayStop(kApp);
    }

    // Kiểm tra xem yêu cầu bị hủy không
    bool Aborted(uint64_t mygen) {
        std::lock_guard<std::mutex> lk(mu_);
        return shutdown_ || gen_ != mygen;
    }

    // Hàm ngủ chia nhỏ thời gian để dễ dàng dừng giữa chừng
    void SleepAbortable(int ms, uint64_t mygen) {
        int steps = (ms + 24) / 25;
        for (int i = 0; i < steps; ++i) {
            if (Aborted(mygen)) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    }

    std::unique_ptr<unitree::robot::a2::AudioClient> client_;
    std::atomic<bool> ready_{false};
    std::thread thread_;

    std::mutex mu_;
    std::condition_variable cv_;
    bool shutdown_ = false;
    uint64_t gen_ = 0;
    JobType req_type_ = kStop;
    std::string req_arg_;
    float req_vol_ = 1.0f;
    int req_speaker_ = 0;
};
