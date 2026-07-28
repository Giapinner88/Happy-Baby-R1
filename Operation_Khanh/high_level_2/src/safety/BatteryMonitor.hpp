#pragma once
/**
 * BatteryMonitor.hpp — giám sát pin qua topic rt/lf/bmsstate
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

class BatteryMonitor {
public:
    void Init(const std::string& topic) {
        using unitree_hg::msg::dds_::BmsState_;
        sub_ = std::make_unique<unitree::robot::ChannelSubscriber<BmsState_>>(topic);
        sub_->InitChannel([this](const void* msg) {
            const auto& b = *static_cast<const BmsState_*>(msg);
            soc_.store(b.soc());
            current_.store(b.current());
            last_ms_.store(NowMs());
            got_.store(true);
        }, 1);
    }

    int  Soc() const { return soc_.load(); }          // % pin
    int  Current() const { return current_.load(); }   // current
    bool Received() const { return got_.load(); }      // đã nhận gói nào chưa

    // Kiểm tra dữ liệu pin có mới không
    bool Fresh(float stale_s) const {
        if (!got_.load()) return false;
        return (NowMs() - last_ms_.load()) <= static_cast<int64_t>(stale_s * 1000.0f);
    }

private:
    static int64_t NowMs() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::BmsState_>> sub_;
    std::atomic<int>     soc_{-1};
    std::atomic<int>     current_{0};
    std::atomic<int64_t> last_ms_{0};
    std::atomic<bool>    got_{false};
};
