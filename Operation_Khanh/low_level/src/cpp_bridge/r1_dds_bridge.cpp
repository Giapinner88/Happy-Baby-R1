#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <cstring>
#include <csignal>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/BmsState_.hpp>

using namespace unitree::robot;
using namespace unitree::common;
using namespace unitree_hg::msg::dds_;

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
            } else
                CRC32 <<= 1;
            if (data & xbit) CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}

const std::array<int, 26> joint_idx = {
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    12, 13,
    15, 16, 17, 18, 19,
    22, 23, 24, 25, 26,
    29, 30
};

const std::array<float, 26> kp_array_default = {
    200.0f, 200.0f, 200.0f, 200.0f, 200.0f, 200.0f,
    200.0f, 200.0f, 200.0f, 200.0f, 200.0f, 200.0f,
    300.0f, 300.0f,
    100.0f, 100.0f, 100.0f, 100.0f, 50.0f,
    100.0f, 100.0f, 100.0f, 100.0f, 50.0f,
    50.0f,  10.0f
};

const std::array<float, 26> kd_array_default = {
    3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f,
    3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f,
    5.0f, 5.0f,
    2.0f, 2.0f, 2.0f, 2.0f, 2.0f,
    2.0f, 2.0f, 2.0f, 2.0f, 2.0f,
    2.0f, 0.1f
};

std::mutex bms_mutex;
BmsState_ current_bms_state;
std::atomic<bool> got_bms{false};

void BmsStateHandler(const void* message) {
    std::lock_guard<std::mutex> lock(bms_mutex);
    current_bms_state = *(const BmsState_*)message;
    got_bms = true;
}

#pragma pack(push, 1)
struct JointState {
    float q;
    float dq;
    float ddq;
    float tau;
    float vol;
    float temp_coil;
    float temp_inv;
    uint8_t mode;
};

struct BridgeState {
    uint8_t mode_machine;
    uint8_t soc;
    uint8_t bms_status;
    float bms_current;
    float bms_voltage;
    float bms_temp;
    JointState joints[26];
    float roll;
    float pitch;
    float yaw;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    float imu_temp;
};

struct BridgeCommand {
    uint8_t motors_enabled;
    float target_q[26];
    float target_kp[26];
    float target_kd[26];
};
#pragma pack(pop)

std::atomic<bool> got_state{false};
std::atomic<uint8_t> mode_machine{0};
std::array<float, 35> current_q = {};
std::mutex state_mutex;

std::atomic<bool> motors_enabled{false};
std::array<float, 26> target_q = {};
std::array<float, 26> target_kp = {};
std::array<float, 26> target_kd = {};
std::mutex cmd_mutex;

int sock_send = -1;
int sock_recv = -1;
struct sockaddr_in addr_python_state;
struct sockaddr_in addr_bridge_cmd;

std::atomic<bool> running{true};

void StateHandler(const void* message) {
    const LowState_& low_state = *(const LowState_*)message;
    
    BridgeState bs;
    std::memset(&bs, 0, sizeof(BridgeState));
    
    bs.mode_machine = low_state.mode_machine();
    
    if (got_bms) {
        std::lock_guard<std::mutex> lock(bms_mutex);
        bs.soc = current_bms_state.soc();
        bs.bms_current = static_cast<float>(current_bms_state.current());
        bs.bms_voltage = static_cast<float>(current_bms_state.bmsvoltage()[0]);
        bs.bms_temp = static_cast<float>(current_bms_state.temperature()[0]);
        bs.bms_status = 1;
    } else {
        bs.soc = 0;
        bs.bms_current = 0.0f;
        bs.bms_voltage = 0.0f;
        bs.bms_temp = 0.0f;
        bs.bms_status = 0;
    }
    
    const auto& imu = low_state.imu_state();
    auto rpy = imu.rpy();
    auto gyro = imu.gyroscope();
    auto acc = imu.accelerometer();
    auto quat = imu.quaternion();
    
    bs.roll = rpy[0];
    bs.pitch = rpy[1];
    bs.yaw = rpy[2];
    bs.gyro_x = gyro[0];
    bs.gyro_y = gyro[1];
    bs.gyro_z = gyro[2];
    bs.acc_x = acc[0];
    bs.acc_y = acc[1];
    bs.acc_z = acc[2];
    bs.quat_w = quat[0];
    bs.quat_x = quat[1];
    bs.quat_y = quat[2];
    bs.quat_z = quat[3];
    bs.imu_temp = static_cast<float>(imu.temperature());
    
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        mode_machine = bs.mode_machine;
        for (int i = 0; i < 35; ++i) {
            current_q[i] = low_state.motor_state()[i].q();
        }
        for (int i = 0; i < 26; ++i) {
            int sdk_idx = joint_idx[i];
            const auto& ms = low_state.motor_state()[sdk_idx];
            bs.joints[i].q = current_q[sdk_idx];
            bs.joints[i].dq = ms.dq();
            bs.joints[i].ddq = ms.ddq();
            bs.joints[i].tau = ms.tau_est();
            bs.joints[i].vol = ms.vol();
            bs.joints[i].mode = ms.mode();
            
            auto temps = ms.temperature();
            bs.joints[i].temp_coil = static_cast<float>(temps[0]);
            bs.joints[i].temp_inv = static_cast<float>(temps[1]);
        }
    }
    
    if (sock_send >= 0) {
        sendto(sock_send, &bs, sizeof(bs), 0, (struct sockaddr*)&addr_python_state, sizeof(addr_python_state));
    }
    got_state = true;
}

void CommandReceiverLoop() {
    BridgeCommand bc;
    int packet_counter = 0;
    
    while (running) {
        int bytes = recvfrom(sock_recv, &bc, sizeof(bc), 0, nullptr, nullptr);
        if (bytes == sizeof(bc)) {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            motors_enabled = (bc.motors_enabled != 0);
            for (int i = 0; i < 26; ++i) {
                target_q[i] = bc.target_q[i];
                target_kp[i] = bc.target_kp[i];
                target_kd[i] = bc.target_kd[i];
            }
            
            packet_counter++;
            if (packet_counter % 200 == 0) {
                std::cout << "[Bridge] UDP commands received: " << packet_counter 
                          << " packets. Enabled=" << (motors_enabled ? "Y" : "N") << std::endl;
            }
        } else if (bytes > 0) {
            std::cout << "[Bridge] Warning: Invalid packet size: " << bytes << " bytes." << std::endl;
        }
    }
}

void PublisherLoop(ChannelPublisherPtr<LowCmd_> publisher) {
    LowCmd_ lowcmd;
    int dds_pub_counter = 0;
    
    std::array<float, 26> commanded_q = {};
    bool prev_enabled = false;
    
    std::array<float, 26> max_vel = {};
    for (int i = 0; i < 26; i++) {
        if (i < 12) {
            max_vel[i] = 1.0f;
        } else if (i < 14) {
            max_vel[i] = 1.0f;
        } else {
            max_vel[i] = 1.5f;
        }
    }
    
    const float dt = 0.005f;
    
    while (running) {
        bool enabled = false;
        std::array<float, 26> targets;
        std::array<float, 26> kps;
        std::array<float, 26> kds;
        std::array<float, 35> current_qs_snapshot;
        
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            enabled = motors_enabled;
            targets = target_q;
            kps = target_kp;
            kds = target_kd;
        }
        
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            for (int i = 0; i < 35; i++) {
                current_qs_snapshot[i] = current_q[i];
            }
        }
        
        if (enabled && !prev_enabled) {
            for (int i = 0; i < 26; i++) {
                int sdk_idx = joint_idx[i];
                commanded_q[i] = current_qs_snapshot[sdk_idx];
            }
            std::cout << "[Bridge] Transition: Synced targets to physical joint angles." << std::endl;
        }
        prev_enabled = enabled;
        
        lowcmd.mode_machine() = mode_machine;
        
        for (int i = 0; i < 35; i++) {
            lowcmd.motor_cmd()[i].mode() = 0x00;
            lowcmd.motor_cmd()[i].q() = 0.0f;
            lowcmd.motor_cmd()[i].dq() = 0.0f;
            lowcmd.motor_cmd()[i].kp() = 0.0f;
            lowcmd.motor_cmd()[i].kd() = 0.0f;
            lowcmd.motor_cmd()[i].tau() = 0.0f;
        }
        
        if (enabled) {
            for (int i = 0; i < 26; i++) {
                int sdk_idx = joint_idx[i];
                
                float diff = targets[i] - commanded_q[i];
                float max_step = max_vel[i] * dt;
                
                if (diff > max_step) {
                    commanded_q[i] += max_step;
                } else if (diff < -max_step) {
                    commanded_q[i] -= max_step;
                } else {
                    commanded_q[i] = targets[i];
                }
                
                lowcmd.motor_cmd()[sdk_idx].mode() = 0x01;
                lowcmd.motor_cmd()[sdk_idx].q() = commanded_q[i];
                lowcmd.motor_cmd()[sdk_idx].dq() = 0.0f;
                lowcmd.motor_cmd()[sdk_idx].kp() = kps[i];
                lowcmd.motor_cmd()[sdk_idx].kd() = kds[i];
                lowcmd.motor_cmd()[sdk_idx].tau() = 0.0f;
            }
        }
        
        lowcmd.crc() = Crc32Core((uint32_t *)&lowcmd, (sizeof(lowcmd) >> 2) - 1);
        publisher->Write(lowcmd);
        
        dds_pub_counter++;
        if (dds_pub_counter % 200 == 0 && enabled) {
            std::cout << "[Bridge] Sending commands. mode_machine=" << (int)mode_machine << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void HandleExit(int sig) {
    std::cout << "\n🛑 Đang dừng bridge và ngắt toàn bộ động cơ..." << std::endl;
    running = false;
    
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(12346);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    BridgeCommand bc;
    memset(&bc, 0, sizeof(bc));
    sendto(s, &bc, sizeof(bc), 0, (struct sockaddr*)&addr, sizeof(addr));
    close(s);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <interface>" << std::endl;
        return 1;
    }
    
    std::string interface = argv[1];
    
    std::signal(SIGINT, HandleExit);
    std::signal(SIGTERM, HandleExit);
    
    sock_send = socket(AF_INET, SOCK_DGRAM, 0);
    sock_recv = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_send < 0 || sock_recv < 0) {
        std::cerr << "❌ Socket creation failed!" << std::endl;
        return 1;
    }
    
    memset(&addr_python_state, 0, sizeof(addr_python_state));
    addr_python_state.sin_family = AF_INET;
    addr_python_state.sin_port = htons(12345);
    addr_python_state.sin_addr.s_addr = inet_addr("127.0.0.1");
    
    memset(&addr_bridge_cmd, 0, sizeof(addr_bridge_cmd));
    addr_bridge_cmd.sin_family = AF_INET;
    addr_bridge_cmd.sin_port = htons(12346);
    addr_bridge_cmd.sin_addr.s_addr = inet_addr("127.0.0.1");
    
    if (bind(sock_recv, (struct sockaddr*)&addr_bridge_cmd, sizeof(addr_bridge_cmd)) < 0) {
        std::cerr << "❌ Socket bind to port 12346 failed!" << std::endl;
        close(sock_send);
        close(sock_recv);
        return 1;
    }
    
    for (int i = 0; i < 26; i++) {
        target_kp[i] = kp_array_default[i];
        target_kd[i] = kd_array_default[i];
    }
    
    std::cout << "[Bridge] Initializing DDS on: " << interface << std::endl;
    if (interface == "lo") {
        ChannelFactory::Instance()->Init(1, interface);
    } else {
        ChannelFactory::Instance()->Init(0, interface);
    }
    
    ChannelPublisherPtr<LowCmd_> lowcmd_publisher = std::make_shared<ChannelPublisher<LowCmd_>>("rt/lowcmd");
    ChannelSubscriberPtr<LowState_> lowstate_subscriber = std::make_shared<ChannelSubscriber<LowState_>>("rt/lowstate");
    ChannelSubscriberPtr<BmsState_> bmsstate_subscriber = std::make_shared<ChannelSubscriber<BmsState_>>("rt/lf/bmsstate");
    
    lowcmd_publisher->InitChannel();
    lowstate_subscriber->InitChannel(std::bind(StateHandler, std::placeholders::_1), 1);
    bmsstate_subscriber->InitChannel(std::bind(BmsStateHandler, std::placeholders::_1), 1);
    
    std::thread recv_thread(CommandReceiverLoop);
    std::thread pub_thread(PublisherLoop, lowcmd_publisher);
    
    std::cout << "✅ DDS Bridge Initialized" << std::endl;
    std::cout << "   - Port 12346 for commands" << std::endl;
    std::cout << "   - Port 12345 for states" << std::endl;
    
    while (!got_state && running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    if (running) {
        std::cout << "[Bridge] Connection established. Active." << std::endl;
    }
    
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    recv_thread.join();
    pub_thread.join();
    
    lowstate_subscriber.reset();
    bmsstate_subscriber.reset();
    lowcmd_publisher.reset();
    ChannelFactory::Instance()->Release();
    
    close(sock_send);
    close(sock_recv);
    
    std::cout << "[Bridge] Stopped." << std::endl;
    return 0;
}
