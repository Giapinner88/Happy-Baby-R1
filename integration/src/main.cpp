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
std::atomic<bool> g_conv_mode{false};
std::atomic<bool> g_remote_rearm{false};
std::atomic<int64_t> g_remote_ms{0};
std::atomic<int64_t> g_interrupt_ms{0};   // thời điểm double-tap Select (pulse ngắt lượt voice)
// Preset voice không diễn giải hay sửa packet R3-1. Coordinator chỉ mirror bitmask
// read-only sang service riêng; service đó tự map UP/RIGHT/DOWN/LEFT theo YAML.
std::atomic<uint16_t> g_remote_buttons{0};
std::atomic<uint64_t> g_remote_sequence{0};
// Nguồn mic do NGƯỜI VẬN HÀNH chọn bằng F2: true = mic ngoài PC2, false = mic thân robot.
// Đây là lựa chọn dính, không phải tín hiệu tức thời như ptt/interrupt: nó phải sống
// qua lúc mất tay cầm và qua lúc restart hb_voice, nếu không mic sẽ tự đổi sau lưng.
std::atomic<bool> g_mic_external{true};
// Trạng thái F2 của gói trước. Chỉ thread DDS callback chạm vào.
bool g_f2_prev = false;

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
  std::string preset_socket = "/run/hb/voice_presets.sock";
  std::string preset_control_socket = "/run/hb/voice_presets_control.sock";
  std::string status_file = "/run/hb/status.env";
  int high_timeout_ms = 1500;
  int remote_timeout_ms = 3000;
  int preset_busy_timeout_ms = 1000;
  bool self_test = false;
};

struct GateInputs {
  bool high_alive = false;
  bool high_busy = true;
  bool remote_alive = false;
  bool ptt = false;
  bool conv_mode = false;
  bool require_release = false;
  // Preset player đã nhận quyền loa. Không cho Pipecat/mic chen vào trong lúc
  // này, nhưng hoàn toàn không liên quan tới DDS motor.
  bool preset_busy = false;
};

struct GateOutputs {
  bool mic = false;
  bool speaker = false;
};

struct RemotePacketState {
  bool valid = false;
  bool ptt = false;
  bool f1 = false;
  bool f2 = false;
  uint16_t buttons = 0;
};

GateOutputs ComputeGate(const GateInputs& in) {
  GateOutputs out;
  out.mic = in.high_alive && in.remote_alive && !in.high_busy && !in.preset_busy &&
            (in.conv_mode || (in.ptt && !in.require_release));
  out.speaker = in.high_alive && !in.high_busy && !in.preset_busy;
  return out;
}

// Gesture vẫn thuộc controller. Voice chỉ được phép quảng bá tool sau khi
// run_r1 đã thật sự arm và đã nhận locomotion; Dev Mode riêng của firmware
// (L2+R2) không thỏa điều kiện này.
bool GestureDevReady(const GateInputs& in, bool high_armed,
                     const std::string& high_state) {
  return in.high_alive && in.remote_alive && high_armed &&
         high_state == "LOCOMOTION";
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

void SendDatagram(int fd, const std::string& path, const std::string& message) {
  const sockaddr_un addr = UnixAddress(path);
  sendto(fd, message.data(), message.size(), MSG_DONTWAIT,
         reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
}

void WriteStatus(const Options& options, const GateInputs& in,
                 const GateOutputs& out, bool high_armed,
                 const std::string& high_state, bool gesture_dev_ready,
                 bool preset_busy) {
  const std::string tmp = options.status_file + ".tmp";
  std::ofstream file(tmp, std::ios::trunc);
  if (!file.good()) return;
  file << "ready=" << (in.high_alive && in.remote_alive ? 1 : 0) << "\n"
       << "high_alive=" << in.high_alive << "\n"
       << "high_busy=" << in.high_busy << "\n"
       << "high_armed=" << high_armed << "\n"
       << "high_state=" << high_state << "\n"
       << "gesture_dev_ready=" << gesture_dev_ready << "\n"
       << "remote_alive=" << in.remote_alive << "\n"
       << "ptt=" << in.ptt << "\n"
       << "conv_mode=" << in.conv_mode << "\n"
       << "mic_external=" << (g_mic_external.load() ? 1 : 0) << "\n"
       << "ptt_rearm_required=" << in.require_release << "\n"
       << "preset_busy=" << preset_busy << "\n"
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
  state.buttons = packet.data.btn.value;
  state.ptt = state.valid && packet.data.btn.components.select;
  state.f1 = state.valid && packet.data.btn.components.F1;
  state.f2 = state.valid && packet.data.btn.components.F2;
  return state;
}

// F2 nhấn MỘT lần -> đổi nguồn mic. Không dùng double-click như F1: giữa sự kiện,
// khi mic đeo chết thì phải đổi được ngay. Chỉ bắt cạnh lên nên giữ nút không làm
// đổi liên tục.
void DetectMicSourceToggle(bool f2_now) {
  if (f2_now && !g_f2_prev) g_mic_external = !g_mic_external.load();
  g_f2_prev = f2_now;
}

// Phát hiện DOUBLE-TAP Select (2 cú nhấn-nhả NGẮN liên tiếp) -> pulse ngắt lượt voice.
// Nhấn-GIỮ (PTT nói) không tính là tap -> không lẫn. Chỉ chạy trên thread DDS callback.
void DetectDoubleTap(bool select_now) {
  static bool prev = false;
  static int64_t press_start = 0;
  static int64_t last_tap = 0;
  static int taps = 0;
  constexpr int64_t kTapMaxMs = 400;       // nhấn-nhả < 0.4s = tap (khác giữ để nói)
  constexpr int64_t kDoubleWindowMs = 800; // 2 tap trong 0.8s = double-tap
  const int64_t now = NowMs();
  if (select_now && !prev) {
    press_start = now;                     // bắt đầu nhấn
  } else if (!select_now && prev) {        // nhả
    const int64_t dur = now - press_start;
    if (dur <= kTapMaxMs) {                // 1 cú tap ngắn
      if (taps == 1 && now - last_tap <= kDoubleWindowMs) {
        g_interrupt_ms = now;              // DOUBLE-TAP -> phát pulse
        taps = 0;
      } else {
        taps = 1;
        last_tap = now;
      }
    } else {
      taps = 0;                            // giữ lâu = PTT, reset
    }
  }
  prev = select_now;
}

// Double-click F1 toggles hands-free conversation mode.  Its state is
// independent from Select so the established interrupt gesture is unchanged.
void DetectConversationToggle(bool f1_now) {
  static bool prev = false;
  static int64_t press_start = 0;
  static int64_t last_tap = 0;
  static int taps = 0;
  constexpr int64_t kTapMaxMs = 400;
  constexpr int64_t kDoubleWindowMs = 800;
  const int64_t now = NowMs();
  if (f1_now && !prev) {
    press_start = now;
  } else if (!f1_now && prev) {
    const int64_t dur = now - press_start;
    if (dur <= kTapMaxMs) {
      if (taps == 1 && now - last_tap <= kDoubleWindowMs) {
        g_conv_mode = !g_conv_mode.load();
        taps = 0;
      } else {
        taps = 1;
        last_tap = now;
      }
    } else {
      taps = 0;
    }
  }
  prev = f1_now;
}

// A missing/invalid remote must never preserve a latched hands-free mode.
// The next microphone activation always requires a fresh remote packet and,
// for PTT, a release before another press can open the microphone.
void InvalidateRemoteState() {
  g_ptt = false;
  g_conv_mode = false;
  g_remote_rearm = true;
  g_remote_ms = 0;
  g_remote_buttons = 0;
  ++g_remote_sequence;
  // KHÔNG đụng g_mic_external: đổi nguồn mic sau lưng người vận hành mỗi lần tay
  // cầm chớp là cách chắc chắn để mất tiếng giữa lúc đang nói. Nhưng coi F2 như
  // "đã thấy nhấn" để nếu lúc mất sóng nút đang bị giữ thì lúc nối lại không
  // tính thành một cú nhấn mới; phải nhả ra rồi nhấn lại mới đổi.
  g_f2_prev = true;
}

void ApplyRemotePacket(const RemotePacketState& state) {
  // Gói 0 đóng mic ngay; sau reconnect phải nhả Select trước khi mở lại.
  if (!state.valid) {
    InvalidateRemoteState();
    return;
  }
  g_remote_buttons = state.buttons;
  ++g_remote_sequence;
  DetectDoubleTap(state.ptt);
  DetectConversationToggle(state.f1);
  DetectMicSourceToggle(state.f2);
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
    else if (arg == "--preset-socket") options.preset_socket = value("--preset-socket");
    else if (arg == "--preset-control-socket") options.preset_control_socket = value("--preset-control-socket");
    else if (arg == "--status-file") options.status_file = value("--status-file");
    else if (arg == "--high-timeout-ms") options.high_timeout_ms = std::stoi(value("--high-timeout-ms"));
    else if (arg == "--remote-timeout-ms") options.remote_timeout_ms = std::stoi(value("--remote-timeout-ms"));
    else if (arg == "--preset-busy-timeout-ms") options.preset_busy_timeout_ms = std::stoi(value("--preset-busy-timeout-ms"));
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
  in.conv_mode = true;
  if (!ComputeGate(in).mic || !ComputeGate(in).speaker) return 5;
  in.conv_mode = false;

  // Built-in firmware Dev alone is insufficient: only run_r1 owning
  // locomotion may expose voice gestures. Any lost heartbeat/remote closes it.
  if (GestureDevReady(in, false, "LOCOMOTION")) return 20;
  if (GestureDevReady(in, true, "UNKNOWN")) return 21;
  if (!GestureDevReady(in, true, "LOCOMOTION")) return 22;
  in.remote_alive = false;
  if (GestureDevReady(in, true, "LOCOMOTION")) return 23;
  in.remote_alive = true;

  // Preset đang phát phải khóa audio conversation; coordinator không có
  // bất kỳ quyền điều khiển motor nào.
  in.preset_busy = true;
  if (ComputeGate(in).mic || ComputeGate(in).speaker) return 19;
  in.preset_busy = false;

  std::array<uint8_t, 40> bytes{};
  bytes[0] = 0x55;
  bytes[1] = 0x51;

  // Two short Select taps must always produce the voice-stop pulse.  This
  // uses the same packet decoder and edge detector as the DDS callback.
  g_interrupt_ms = 0;
  bytes[2] = 0x08;  // Select
  ApplyRemotePacket(DecodeRemote(bytes));
  bytes[2] = 0;
  ApplyRemotePacket(DecodeRemote(bytes));
  bytes[2] = 0x08;
  ApplyRemotePacket(DecodeRemote(bytes));
  bytes[2] = 0;
  ApplyRemotePacket(DecodeRemote(bytes));
  if (g_interrupt_ms.load() == 0) return 6;

  bytes.fill(0);
  auto remote = DecodeRemote(bytes);
  if (remote.valid || remote.ptt || remote.f1) return 7;
  ApplyRemotePacket(remote);
  if (g_ptt.load() || g_conv_mode.load() || !g_remote_rearm.load()) return 8;

  bytes[0] = 0x55;
  bytes[1] = 0x51;
  bytes[2] = 0x08;  // Select
  remote = DecodeRemote(bytes);
  if (!remote.valid || !remote.ptt || remote.f1) return 9;
  ApplyRemotePacket(remote);
  if (!g_ptt.load() || !g_remote_rearm.load()) return 10;

  bytes[2] = 0;
  remote = DecodeRemote(bytes);
  ApplyRemotePacket(remote);
  if (g_ptt.load() || g_remote_rearm.load()) return 11;

  // Two short F1 taps enable conversation mode.  A remote-loss packet must
  // clear the mode so reconnecting cannot reopen the microphone by itself.
  bytes[2] = 0x40;  // F1
  ApplyRemotePacket(DecodeRemote(bytes));
  bytes[2] = 0;
  ApplyRemotePacket(DecodeRemote(bytes));
  bytes[2] = 0x40;
  ApplyRemotePacket(DecodeRemote(bytes));
  bytes[2] = 0;
  ApplyRemotePacket(DecodeRemote(bytes));
  if (!g_conv_mode.load()) return 12;
  ApplyRemotePacket(DecodeRemote(std::array<uint8_t, 40>{}));
  if (g_conv_mode.load() || g_remote_ms.load() != 0) return 13;

  // F2 nhấn MỘT lần đổi nguồn mic, nhấn nữa đổi lại. Gói mất tay cầm ở trên vừa
  // đặt g_f2_prev=true, nên phải có một gói F2-nhả trước thì cú nhấn mới tính.
  bytes[2] = 0;
  ApplyRemotePacket(DecodeRemote(bytes));
  if (!g_mic_external.load()) return 14;
  bytes[2] = 0x80;  // F2
  ApplyRemotePacket(DecodeRemote(bytes));
  if (g_mic_external.load()) return 15;
  bytes[2] = 0;
  ApplyRemotePacket(DecodeRemote(bytes));
  bytes[2] = 0x80;
  ApplyRemotePacket(DecodeRemote(bytes));
  if (!g_mic_external.load()) return 16;
  bytes[2] = 0;
  ApplyRemotePacket(DecodeRemote(bytes));

  // Mất tay cầm KHÔNG được đổi nguồn mic (khác hẳn ptt/conv): đổi mic sau lưng
  // người vận hành mỗi lần tay cầm chớp là mất tiếng giữa lúc đang nói.
  const bool source_before_loss = g_mic_external.load();
  ApplyRemotePacket(DecodeRemote(std::array<uint8_t, 40>{}));
  if (g_mic_external.load() != source_before_loss) return 17;
  // Và nối lại trong lúc F2 đang bị giữ cũng không được tính là cú nhấn mới.
  bytes[2] = 0x80;
  ApplyRemotePacket(DecodeRemote(bytes));
  if (g_mic_external.load() != source_before_loss) return 18;

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
    const int preset_control_fd = CreateInputSocket(options.preset_control_socket);
    const int output_fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (output_fd < 0) throw std::runtime_error("output socket failed");

    unitree::robot::ChannelFactory::Instance()->Init(0, options.interface);
    unitree::robot::ChannelSubscriber<LowState_> low_state_sub("rt/lowstate");
    low_state_sub.InitChannel(OnLowState, 1);

    int64_t last_high_ms = 0;
    int64_t last_send_ms = 0;
    int64_t last_preset_send_ms = 0;
    int64_t last_status_ms = 0;
    bool high_busy = true;
    bool high_armed = false;
    bool require_release = false;
    std::string high_state = "UNKNOWN";
    std::string previous_message;
    std::string previous_preset_message;
    uint64_t sequence = 0;
    uint64_t preset_sequence = 0;
    uint64_t preset_cancel_epoch = 0;
    int64_t preset_busy_until_ms = 0;
    bool previous_remote_alive = false;
    bool previous_preset_high_busy = false;

    std::cout << "hb_integration ready: interface=" << options.interface
              << " input=" << options.input_socket
              << " output=" << options.output_socket
              << " preset=" << options.preset_socket << "\n";

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

      // Preset service xin lease ngắn trước khi đẩy PCM. Nếu service chết thì
      // lease hết hạn, voice conversation tự nhận lại loa thay vì bị kẹt BUSY.
      for (;;) {
        const ssize_t count = recv(preset_control_fd, buffer, sizeof(buffer) - 1,
                                   MSG_DONTWAIT);
        if (count <= 0) break;
        buffer[count] = '\0';
        const std::string message(buffer);
        if (ParseStringToken(message, "v", "") != "1" ||
            ParseStringToken(message, "role", "") != "preset") {
          continue;
        }
        if (ParseBoolToken(message, "busy", false)) {
          preset_busy_until_ms = NowMs() + options.preset_busy_timeout_ms;
        } else {
          preset_busy_until_ms = 0;
        }
      }

      const int64_t now = NowMs();
      GateInputs in;
      in.high_alive = last_high_ms > 0 && now - last_high_ms <= options.high_timeout_ms;
      in.high_busy = !in.high_alive || high_busy;
      const int64_t remote_ms = g_remote_ms.load();
      in.remote_alive = remote_ms > 0 && now - remote_ms <= options.remote_timeout_ms;
      if (!in.remote_alive && remote_ms > 0) {
        // DDS can go silent without publishing a zero remote packet.  Treat
        // that the same as an explicit remote-loss packet, so F1 hands-free
        // never resumes by itself when the remote comes back.
        InvalidateRemoteState();
      }
      in.ptt = in.remote_alive && g_ptt.load();
      in.conv_mode = in.remote_alive && g_conv_mode.load();

      // Chỉ busy do high-level báo THẬT mới hủy preset. Mất heartbeat vẫn giữ
      // nguyên yêu cầu của preset; nó không bị coi nhầm thành high_level BUSY.
      const bool preset_high_busy = in.high_alive && high_busy;
      const bool remote_lost = previous_remote_alive && !in.remote_alive;
      const bool high_became_busy = !previous_preset_high_busy && preset_high_busy;
      if (remote_lost || high_became_busy) {
        ++preset_cancel_epoch;
        preset_busy_until_ms = 0;
      }
      if (!in.remote_alive || preset_high_busy) preset_busy_until_ms = 0;
      in.preset_busy = now < preset_busy_until_ms;

      if (in.high_busy && in.ptt) require_release = true;
      if (!in.ptt) require_release = false;
      in.require_release = require_release || g_remote_rearm.load();
      const GateOutputs out = ComputeGate(in);
      const bool gesture_dev_ready = GestureDevReady(in, high_armed, high_state);

      // Pulse ngắt lượt: giữ interrupt=1 trong 300ms sau double-tap để voice bắt được cạnh lên.
      const bool interrupt = g_interrupt_ms.load() > 0 && now - g_interrupt_ms.load() < 300;
      std::ostringstream state_line;
      state_line << "high_alive=" << in.high_alive
           << " high_busy=" << in.high_busy << " high_armed=" << high_armed
           << " high_state=" << high_state
           << " gesture_dev=" << gesture_dev_ready
           << " remote_alive=" << in.remote_alive
           << " ptt=" << in.ptt << " conv=" << in.conv_mode
           << " rearm=" << in.require_release
           << " mic=" << out.mic << " speaker=" << out.speaker
           << " preset_busy=" << in.preset_busy
           << " interrupt=" << interrupt
           << " micext=" << (g_mic_external.load() ? 1 : 0);
      const std::string current_state = state_line.str();
      if (current_state != previous_message || now - last_send_ms >= 100) {
        const std::string gate_message =
            "v=1 seq=" + std::to_string(++sequence) + " " + current_state;
        SendDatagram(output_fd, options.output_socket, gate_message);
        previous_message = current_state;
        last_send_ms = now;
      }
      std::ostringstream preset_line;
      preset_line << "remote_alive=" << in.remote_alive
                  << " buttons=" << g_remote_buttons.load()
                  << " remote_seq=" << g_remote_sequence.load()
                  << " high_armed=" << high_armed
                  << " high_busy=" << preset_high_busy
                  << " preset_busy=" << in.preset_busy
                  << " cancel_epoch=" << preset_cancel_epoch;
      const std::string current_preset_state = preset_line.str();
      if (current_preset_state != previous_preset_message ||
          now - last_preset_send_ms >= 100) {
        const std::string preset_message =
            "v=1 seq=" + std::to_string(++preset_sequence) + " " + current_preset_state;
        SendDatagram(output_fd, options.preset_socket, preset_message);
        previous_preset_message = current_preset_state;
        last_preset_send_ms = now;
      }
      if (now - last_status_ms >= 250) {
        WriteStatus(options, in, out, high_armed, high_state,
                    gesture_dev_ready, in.preset_busy);
        last_status_ms = now;
      }
      previous_remote_alive = in.remote_alive;
      previous_preset_high_busy = preset_high_busy;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    close(output_fd);
    close(input_fd);
    close(preset_control_fd);
    unlink(options.input_socket.c_str());
    unlink(options.preset_control_socket.c_str());
    unlink(options.status_file.c_str());
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "hb_integration fatal: " << error.what() << "\n";
    return 1;
  }
}
