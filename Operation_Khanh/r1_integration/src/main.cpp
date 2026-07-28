#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace {

using unitree_hg::msg::dds_::LowState_;
using Clock = std::chrono::steady_clock;

std::atomic<bool> g_running{true};
std::atomic<bool> g_ptt{false};
std::atomic<bool> g_remote_rearm{false};
std::atomic<int64_t> g_remote_ms{0};

int64_t NowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             Clock::now().time_since_epoch())
      .count();
}

void HandleSignal(int) { g_running = false; }

union KeySwitch {
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
};

struct RemoteData {
  uint8_t head[2];
  KeySwitch btn;
  float lx, rx, ry, L2, ly;
  uint8_t idle[16];
};

union RemotePacket {
  RemoteData data;
  uint8_t bytes[40];
};

static_assert(sizeof(RemotePacket) == 40, "Unexpected R3-1 remote packet layout");

struct Options {
  std::string interface = "eth10";
  std::string input_socket = "/run/hb/integration.sock";
  std::string output_socket = "/run/hb/voice_gate.sock";
  std::string status_file = "/run/hb/status.env";
  int high_timeout_ms = 1500;
  int remote_timeout_ms = 3000;
  bool self_test = false;
};

struct GateInputs {
  bool high_alive = false;
  bool high_busy = true;
  bool remote_alive = false;
  bool ptt = false;
  bool require_release = false;
};

struct GateOutputs {
  bool mic = false;
  bool speaker = false;
};

struct RemotePacketState {
  bool valid = false;
  bool ptt = false;
};

GateOutputs ComputeGate(const GateInputs& in) {
  GateOutputs out;
  out.mic = in.high_alive && in.remote_alive && in.ptt && !in.high_busy &&
            !in.require_release;
  out.speaker = in.high_alive && !in.high_busy;
  return out;
}

bool ParseBoolToken(const std::string& message, const std::string& key,
                    bool fallback) {
  const std::string needle = key + "=";
  const size_t pos = message.find(needle);
  if (pos == std::string::npos) return fallback;
  const size_t value_pos = pos + needle.size();
  return value_pos < message.size() && message[value_pos] == '1';
}

std::string ParseStringToken(const std::string& message, const std::string& key,
                             const std::string& fallback) {
  const std::string needle = key + "=";
  const size_t pos = message.find(needle);
  if (pos == std::string::npos) return fallback;
  const size_t begin = pos + needle.size();
  const size_t end = message.find(' ', begin);
  return message.substr(begin, end == std::string::npos ? end : end - begin);
}

sockaddr_un UnixAddress(const std::string& path) {
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (path.size() >= sizeof(addr.sun_path)) {
    throw std::runtime_error("Unix socket path is too long: " + path);
  }
  std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
  return addr;
}

int CreateInputSocket(const std::string& path) {
  const int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (fd < 0) throw std::runtime_error("socket(AF_UNIX) failed");
  unlink(path.c_str());
  const sockaddr_un addr = UnixAddress(path);
  if (bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
    close(fd);
    throw std::runtime_error("bind failed for " + path);
  }
  chmod(path.c_str(), 0660);
  return fd;
}

void SendGate(int fd, const Options& options, const std::string& message) {
  const sockaddr_un addr = UnixAddress(options.output_socket);
  sendto(fd, message.data(), message.size(), MSG_DONTWAIT,
         reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
}

void WriteStatus(const Options& options, const GateInputs& in,
                 const GateOutputs& out, bool high_armed,
                 const std::string& high_state) {
  const std::string tmp = options.status_file + ".tmp";
  std::ofstream file(tmp, std::ios::trunc);
  if (!file.good()) return;
  file << "ready=" << (in.high_alive && in.remote_alive ? 1 : 0) << "\n"
       << "high_alive=" << in.high_alive << "\n"
       << "high_busy=" << in.high_busy << "\n"
       << "high_armed=" << high_armed << "\n"
       << "high_state=" << high_state << "\n"
       << "remote_alive=" << in.remote_alive << "\n"
       << "ptt=" << in.ptt << "\n"
       << "ptt_rearm_required=" << in.require_release << "\n"
       << "mic_allowed=" << out.mic << "\n"
       << "speaker_allowed=" << out.speaker << "\n"
       << "updated_monotonic_ms=" << NowMs() << "\n";
  file.close();
  chmod(tmp.c_str(), 0644);
  rename(tmp.c_str(), options.status_file.c_str());
}

RemotePacketState DecodeRemote(const std::array<uint8_t, 40>& bytes) {
  RemotePacket packet{};
  std::memcpy(packet.bytes, bytes.data(), sizeof(packet.bytes));
  RemotePacketState state;
  for (uint8_t byte : packet.bytes) state.valid = state.valid || byte != 0;
  state.ptt = state.valid && packet.data.btn.components.select;
  return state;
}

void ApplyRemotePacket(const RemotePacketState& state) {
  // Gói 0 đóng mic ngay; sau reconnect phải nhả Select trước khi mở lại.
  if (!state.valid) {
    g_ptt = false;
    g_remote_rearm = true;
    return;
  }
  g_ptt = state.ptt;
  if (!state.ptt) g_remote_rearm = false;
  g_remote_ms = NowMs();
}

void OnLowState(const void* raw) {
  const auto& low = *static_cast<const LowState_*>(raw);
  ApplyRemotePacket(DecodeRemote(low.wireless_remote()));
}

Options ParseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&](const char* flag) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + flag);
      return argv[++i];
    };
    if (arg == "--interface") options.interface = value("--interface");
    else if (arg == "--input-socket") options.input_socket = value("--input-socket");
    else if (arg == "--output-socket") options.output_socket = value("--output-socket");
    else if (arg == "--status-file") options.status_file = value("--status-file");
    else if (arg == "--high-timeout-ms") options.high_timeout_ms = std::stoi(value("--high-timeout-ms"));
    else if (arg == "--remote-timeout-ms") options.remote_timeout_ms = std::stoi(value("--remote-timeout-ms"));
    else if (arg == "--self-test") options.self_test = true;
    else throw std::runtime_error("Unknown argument: " + arg);
  }
  return options;
}

int SelfTest() {
  GateInputs in;
  if (ComputeGate(in).mic || ComputeGate(in).speaker) return 1;
  in.high_alive = in.remote_alive = true;
  in.high_busy = false;
  in.ptt = true;
  if (!ComputeGate(in).mic || !ComputeGate(in).speaker) return 2;
  in.high_busy = true;
  if (ComputeGate(in).mic || ComputeGate(in).speaker) return 3;
  in.high_busy = false;
  in.require_release = true;
  if (ComputeGate(in).mic || !ComputeGate(in).speaker) return 4;

  std::array<uint8_t, 40> bytes{};
  auto remote = DecodeRemote(bytes);
  if (remote.valid || remote.ptt) return 5;
  ApplyRemotePacket(remote);
  if (g_ptt.load() || !g_remote_rearm.load()) return 6;

  bytes[0] = 0x55;
  bytes[1] = 0x51;
  bytes[2] = 0x08;  // Select
  remote = DecodeRemote(bytes);
  if (!remote.valid || !remote.ptt) return 7;
  ApplyRemotePacket(remote);
  if (!g_ptt.load() || !g_remote_rearm.load()) return 8;

  bytes[2] = 0;
  remote = DecodeRemote(bytes);
  ApplyRemotePacket(remote);
  if (g_ptt.load() || g_remote_rearm.load()) return 9;
  std::cout << "hb_integration self-test: OK\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = ParseOptions(argc, argv);
    if (options.self_test) return SelfTest();

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    const char* home = std::getenv("HOME");
    if (home) {
      const std::string uri = std::string(home) +
                              "/unitree_sdk2/thirdparty/cyclonedds/cyclonedds.xml";
      if (std::ifstream(uri).good()) setenv("CYCLONEDDS_URI", ("file://" + uri).c_str(), 1);
    }

    const int input_fd = CreateInputSocket(options.input_socket);
    const int output_fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (output_fd < 0) throw std::runtime_error("output socket failed");

    unitree::robot::ChannelFactory::Instance()->Init(0, options.interface);
    unitree::robot::ChannelSubscriber<LowState_> low_state_sub("rt/lowstate");
    low_state_sub.InitChannel(OnLowState, 1);

    int64_t last_high_ms = 0;
    int64_t last_send_ms = 0;
    int64_t last_status_ms = 0;
    bool high_busy = true;
    bool high_armed = false;
    bool require_release = false;
    std::string high_state = "UNKNOWN";
    std::string previous_message;
    uint64_t sequence = 0;

    std::cout << "hb_integration ready: interface=" << options.interface
              << " input=" << options.input_socket
              << " output=" << options.output_socket << "\n";

    while (g_running) {
      char buffer[1024];
      for (;;) {
        const ssize_t count = recv(input_fd, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);
        if (count <= 0) break;
        buffer[count] = '\0';
        const std::string message(buffer);
        high_busy = ParseBoolToken(message, "busy", true);
        high_armed = ParseBoolToken(message, "armed", false);
        high_state = ParseStringToken(message, "state", "UNKNOWN");
        last_high_ms = NowMs();
      }

      const int64_t now = NowMs();
      GateInputs in;
      in.high_alive = last_high_ms > 0 && now - last_high_ms <= options.high_timeout_ms;
      in.high_busy = !in.high_alive || high_busy;
      in.remote_alive = g_remote_ms.load() > 0 &&
                        now - g_remote_ms.load() <= options.remote_timeout_ms;
      in.ptt = in.remote_alive && g_ptt.load();

      if (in.high_busy && in.ptt) require_release = true;
      if (!in.ptt) require_release = false;
      in.require_release = require_release || g_remote_rearm.load();
      const GateOutputs out = ComputeGate(in);

      std::ostringstream state_line;
      state_line << "high_alive=" << in.high_alive
           << " high_busy=" << in.high_busy << " high_armed=" << high_armed
           << " remote_alive=" << in.remote_alive
           << " ptt=" << in.ptt << " rearm=" << in.require_release
           << " mic=" << out.mic << " speaker=" << out.speaker;
      const std::string current_state = state_line.str();
      if (current_state != previous_message || now - last_send_ms >= 100) {
        const std::string gate_message =
            "v=1 seq=" + std::to_string(++sequence) + " " + current_state;
        SendGate(output_fd, options, gate_message);
        previous_message = current_state;
        last_send_ms = now;
      }
      if (now - last_status_ms >= 250) {
        WriteStatus(options, in, out, high_armed, high_state);
        last_status_ms = now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    close(output_fd);
    close(input_fd);
    unlink(options.input_socket.c_str());
    unlink(options.status_file.c_str());
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "hb_integration fatal: " << error.what() << "\n";
    return 1;
  }
}
