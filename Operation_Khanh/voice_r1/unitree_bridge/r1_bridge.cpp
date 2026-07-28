#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <unitree/common/time/time_tool.hpp>
#include <unitree/robot/r1/audio/audio_client.hpp>

namespace {

constexpr const char* kMicGroupIp = "239.168.123.161";
constexpr int kMicPort = 5555;
constexpr int kSampleRate = 16000;
constexpr int kBytesPerSample = 2;
constexpr int kMaxAudioChunkBytes = 96000;  // Unitree example uses 3 seconds at 16 kHz mono.
constexpr int kSpeakerFlushMs = 120;

std::atomic<bool> g_running{true};

enum class MicPayloadMode {
  kRaw,
  kRtp,
  kRtpL16Be,
};

void HandleSignal(int) {
  g_running = false;
}

void Usage(const char* argv0) {
  std::cerr
      << "Usage:\n"
      << "  " << argv0
      << " speaker <network_interface> [app_name] [volume_0_100]\n"
      << "  " << argv0
      << " mic <network_interface> [seconds] [group_ip] [port] [raw|rtp|rtp-l16be]\n";
}

void InitChannel(const std::string& network_interface) {
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
}

std::string LocalIpForMulticast() {
  struct ifaddrs *ifaddr = nullptr, *ifa = nullptr;
  char host[NI_MAXHOST];
  std::string result;

  if (getifaddrs(&ifaddr) != 0) {
    return result;
  }

  for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) {
      continue;
    }
    getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST,
                nullptr, 0, NI_NUMERICHOST);
    std::string ip(host);
    if (ip.find("192.168.123.") == 0) {
      result = ip;
      break;
    }
  }

  freeifaddrs(ifaddr);
  return result;
}

int SpeakerMode(const std::string& network_interface, const std::string& app_name,
                int volume) {
  InitChannel(network_interface);

  unitree::robot::r1::AudioClient client;
  client.Init();
  client.SetTimeout(10.0f);

  if (volume >= 0) {
    volume = std::clamp(volume, 0, 100);
    int32_t volume_ret = client.SetVolume(static_cast<uint8_t>(volume));
    if (volume_ret != 0) {
      std::cerr << "SetVolume(" << volume << ") ret=" << volume_ret << std::endl;
    }
  }

  const std::string stream_id =
      std::to_string(unitree::common::GetCurrentTimeMillisecond());
  std::vector<uint8_t> read_buffer(kSampleRate * kBytesPerSample / 10);
  std::vector<uint8_t> pending;
  pending.reserve(kMaxAudioChunkBytes);
  auto last_audio_at = std::chrono::steady_clock::now();
  bool stream_failed = false;

  std::cerr << "r1_bridge speaker ready: app=" << app_name
            << " volume=" << volume
            << " stream_id=" << stream_id << std::endl;

  auto send_chunk = [&](const std::vector<uint8_t>& chunk) {
    if (chunk.empty()) {
      return;
    }
    int32_t ret = client.PlayStream(app_name, stream_id, chunk);
    if (ret != 0) {
      std::cerr << "PlayStream ret=" << ret << std::endl;
      stream_failed = true;
      g_running = false;
    }
  };

  auto flush_pending = [&]() {
    while (pending.size() >= kMaxAudioChunkBytes) {
      std::vector<uint8_t> chunk(pending.begin(), pending.begin() + kMaxAudioChunkBytes);
      send_chunk(chunk);
      pending.erase(pending.begin(), pending.begin() + kMaxAudioChunkBytes);
    }
    if (!pending.empty()) {
      send_chunk(pending);
      pending.clear();
    }
  };

  while (g_running) {
    pollfd pfd{};
    pfd.fd = STDIN_FILENO;
    pfd.events = POLLIN | POLLHUP;
    int poll_ret = poll(&pfd, 1, 50);

    if (poll_ret > 0 && (pfd.revents & POLLIN)) {
      ssize_t got = read(STDIN_FILENO, read_buffer.data(), read_buffer.size());
      if (got > 0) {
        pending.insert(pending.end(), read_buffer.begin(), read_buffer.begin() + got);
        last_audio_at = std::chrono::steady_clock::now();

        while (pending.size() >= kMaxAudioChunkBytes) {
          std::vector<uint8_t> chunk(pending.begin(), pending.begin() + kMaxAudioChunkBytes);
          send_chunk(chunk);
          pending.erase(pending.begin(), pending.begin() + kMaxAudioChunkBytes);
        }
      } else {
        flush_pending();
        break;
      }
    }

    if (poll_ret > 0 && (pfd.revents & POLLHUP)) {
      flush_pending();
      break;
    }

    if (!pending.empty()) {
      auto idle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - last_audio_at)
                         .count();
      if (idle_ms >= kSpeakerFlushMs) {
        flush_pending();
      }
    }
  }

  // The R1 SDK serializes this argument as JSON field "app_name".
  client.PlayStop(app_name);
  return stream_failed ? 3 : 0;
}

int RtpHeaderBytes(const char* data, ssize_t len) {
  if (len < 12 || (static_cast<uint8_t>(data[0]) & 0xC0) != 0x80) {
    return 0;
  }

  int header_len = 12 + (static_cast<uint8_t>(data[0]) & 0x0F) * 4;
  if (header_len + 4 <= len && (static_cast<uint8_t>(data[0]) & 0x10)) {
    uint16_t extension_words = 0;
    std::memcpy(&extension_words, data + header_len + 2, sizeof(extension_words));
    header_len += 4 + ntohs(extension_words) * 4;
  }
  if (header_len >= len) {
    return 0;
  }
  return header_len;
}

MicPayloadMode ParseMicPayloadMode(const std::string& mode) {
  if (mode == "rtp") {
    return MicPayloadMode::kRtp;
  }
  if (mode == "rtp-l16be") {
    return MicPayloadMode::kRtpL16Be;
  }
  return MicPayloadMode::kRaw;
}

int MicMode(const std::string& network_interface, int seconds,
            const std::string& group_ip, int port, MicPayloadMode payload_mode) {
  // The R1 firmware only starts publishing the mic multicast stream once a
  // DDS participant + voice-service RPC handshake happens on this interface
  // (mirrors what the official r1_audio_client_example.cpp does before it
  // starts its mic thread). Without this, join succeeds but no packets ever
  // arrive.
  InitChannel(network_interface);
  unitree::robot::r1::AudioClient audio_client;
  audio_client.Init();
  audio_client.SetTimeout(10.0f);
  uint8_t volume = 0;
  int32_t volume_ret = audio_client.GetVolume(volume);
  std::cerr << "voice service handshake (GetVolume) ret=" << volume_ret
            << " volume=" << static_cast<int>(volume) << std::endl;

  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::cerr << "socket() failed" << std::endl;
    return 2;
  }

  int reuse = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  timeval receive_timeout{};
  receive_timeout.tv_sec = 1;
  receive_timeout.tv_usec = 0;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout));

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(port);
  local_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(sock, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) < 0) {
    std::cerr << "bind() failed on UDP port " << port << std::endl;
    close(sock);
    return 2;
  }

  ip_mreq mreq{};
  inet_pton(AF_INET, group_ip.c_str(), &mreq.imr_multiaddr);
  std::string local_ip = LocalIpForMulticast();
  if (local_ip.empty()) {
    std::cerr << "Could not find local 192.168.123.x IP for R1 microphone multicast"
              << std::endl;
    close(sock);
    return 2;
  }
  std::cerr << "joining mic multicast " << group_ip << ":" << port
            << " via local ip: " << local_ip << std::endl;
  mreq.imr_interface.s_addr = inet_addr(local_ip.c_str());
  if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
    std::cerr << "IP_ADD_MEMBERSHIP failed" << std::endl;
    close(sock);
    return 2;
  }

  const int64_t max_bytes =
      seconds > 0 ? static_cast<int64_t>(seconds) * kSampleRate * kBytesPerSample : -1;
  int64_t total_bytes = 0;
  std::vector<char> buffer(kSampleRate * kBytesPerSample * 160 / 1000);
  const auto started_at = std::chrono::steady_clock::now();
  const auto no_packet_deadline =
      started_at + std::chrono::seconds(seconds > 0 ? seconds + 2 : 0);

  while (g_running && (max_bytes < 0 || total_bytes < max_bytes)) {
    ssize_t len = recvfrom(sock, buffer.data(), buffer.size(), 0, nullptr, nullptr);
    if (len > 0) {
      const char* data = buffer.data();
      ssize_t output_len = len;
      if (payload_mode != MicPayloadMode::kRaw) {
        int header_len = RtpHeaderBytes(data, len);
        if (header_len > 0) {
          data += header_len;
          output_len -= header_len;
        }
      }
      if (payload_mode == MicPayloadMode::kRtpL16Be) {
        output_len = (output_len / 2) * 2;
        for (ssize_t i = 0; i + 1 < output_len; i += 2) {
          std::swap(buffer[static_cast<size_t>(data - buffer.data() + i)],
                    buffer[static_cast<size_t>(data - buffer.data() + i + 1)]);
        }
      }
      std::cout.write(data, output_len);
      std::cout.flush();
      total_bytes += output_len;
    } else if (seconds > 0 && total_bytes == 0 &&
               std::chrono::steady_clock::now() >= no_packet_deadline) {
      std::cerr << "no microphone packets received on " << group_ip << ":"
                << port << std::endl;
      close(sock);
      return 3;
    }
  }

  close(sock);
  return 0;
}

}  // namespace

int main(int argc, char const* argv[]) {
  signal(SIGINT, HandleSignal);
  signal(SIGTERM, HandleSignal);

  if (argc < 3) {
    Usage(argv[0]);
    return 1;
  }

  std::string mode = argv[1];
  std::string network_interface = argv[2];

  if (mode == "speaker") {
    return SpeakerMode(network_interface, argc >= 4 ? argv[3] : "pipecat",
                       argc >= 5 ? std::stoi(argv[4]) : -1);
  }
  if (mode == "mic") {
    return MicMode(network_interface, argc >= 4 ? std::stoi(argv[3]) : 0,
                   argc >= 5 ? argv[4] : kMicGroupIp,
                   argc >= 6 ? std::stoi(argv[5]) : kMicPort,
                   argc >= 7 ? ParseMicPayloadMode(argv[6]) : MicPayloadMode::kRaw);
  }
  Usage(argv[0]);
  return 1;
}
