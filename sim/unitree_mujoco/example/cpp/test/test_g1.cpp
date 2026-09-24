#include <iostream>  
#include <stdio.h>  
#include <stdint.h>  
#include <math.h>  
#include <unitree/robot/channel/channel_publisher.hpp>  
#include <unitree/robot/channel/channel_subscriber.hpp>  
#include <unitree/idl/hg/LowCmd_.hpp>  
#include <unitree/idl/hg/LowState_.hpp>  
#include <unitree/common/time/time_tool.hpp>  
#include <unitree/common/thread/thread.hpp>  
  
using namespace unitree::common;  
using namespace unitree::robot;  
  
#define TOPIC_LOWCMD "rt/lowcmd"  
#define TOPIC_LOWSTATE "rt/lowstate"  
  
class Custom  
{  
public:  
    Custom(){};  
    ~Custom(){};  
    void Init();  
  
private:  
    void InitLowCmd();  
    void LowStateMessageHandler(const void *message);  
    void LowCmdWrite();  
  
private:  
    // G1 có 23 motors (12 chân + 3 eo + 8 tay)  
    double stand_up_joint_pos[23] = {  
        // 12 motors chân (giống Go2)  
        0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,  
        0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,  
        // 3 motors eo  
        0, 0, 0,  
        // 8 motors tay  
        0, 0, 0, 0, 0, 0, 0, 0  
    };  
      
    double stand_down_joint_pos[23] = {  
        // 12 motors chân  
        0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,  
        1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,  
        // 3 motors eo  
        0, 0, 0,  
        // 8 motors tay  
        0, 0, 0, 0, 0, 0, 0, 0  
    };  
      
    double dt = 0.002;  
    double runing_time = 0.0;  
    double phase = 0.0;  
  
    // Sử dụng raw IDL message types cho DDS  
    unitree_hg::msg::dds_::LowCmd_ low_cmd{};  
    unitree_hg::msg::dds_::LowState_ low_state{};  
  
    ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher;  
    ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber;  
  
    ThreadPtr lowCmdWriteThreadPtr;  
};  
  
void Custom::Init()  
{  
    InitLowCmd();  
      
    // Tạo publisher cho G1 LowCmd với raw IDL type  
    lowcmd_publisher.reset(new ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(TOPIC_LOWCMD));  
    lowcmd_publisher->InitChannel();  
  
    // Tạo subscriber cho G1 LowState với raw IDL type  
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(TOPIC_LOWSTATE));  
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);  
  
    // Thread để publish commands tại 500Hz  
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, int(dt * 1000000), &Custom::LowCmdWrite, this);  
}  
  
void Custom::InitLowCmd()  
{  
    // G1 LowCmd không có head, level_flag, gpio như Go2  
    for (int i = 0; i < 35; i++)  // G1 IDL hỗ trợ tối đa 35 motors  
    {  
        low_cmd.motor_cmd()[i].mode() = 0x01;  // Servo mode  
        low_cmd.motor_cmd()[i].q() = 0.0;      // Position  
        low_cmd.motor_cmd()[i].kp() = 0.0;     // Position gain  
        low_cmd.motor_cmd()[i].dq() = 0.0;     // Velocity  
        low_cmd.motor_cmd()[i].kd() = 0.0;     // Velocity gain  
        low_cmd.motor_cmd()[i].tau() = 0.0;    // Torque  
    }  
}  
  
void Custom::LowStateMessageHandler(const void *message)  
{  
    // Cast trực tiếp sang raw IDL type  
    low_state = *(unitree_hg::msg::dds_::LowState_ *)message;  
}  
  
void Custom::LowCmdWrite()  
{  
    runing_time += dt;  
      
    if (runing_time < 3.0)  
    {  
        // Đứng lên trong 3 giây đầu  
        phase = tanh(runing_time / 1.2);  
        for (int i = 0; i < 23; i++)  
        {  
            low_cmd.motor_cmd()[i].q() = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i];  
            low_cmd.motor_cmd()[i].dq() = 0;  
            low_cmd.motor_cmd()[i].kp() = phase * 50.0 + (1 - phase) * 20.0;  
            low_cmd.motor_cmd()[i].kd() = 3.5;  
            low_cmd.motor_cmd()[i].tau() = 0;  
        }  
    }  
    else  
    {  
        // Sau đó ngồi xuống  
        phase = tanh((runing_time - 3.0) / 1.2);  
        for (int i = 0; i < 23; i++)  
        {  
            low_cmd.motor_cmd()[i].q() = phase * stand_down_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i];  
            low_cmd.motor_cmd()[i].dq() = 0;  
            low_cmd.motor_cmd()[i].kp() = 50;  
            low_cmd.motor_cmd()[i].kd() = 3.5;  
            low_cmd.motor_cmd()[i].tau() = 0;  
        }  
    }  
  
    // Publish raw IDL message  
    lowcmd_publisher->Write(low_cmd);  
}  
  
int main(int argc, char **argv)  
{  
    if (argc < 2)  
    {     
        // Simulation mode: sử dụng domain_id=1 và interface="lo"  
        ChannelFactory::Instance()->Init(1, "lo");  
    }  
    else  
    {     
        // Real robot mode: sử dụng domain_id=0 và interface chỉ định  
        ChannelFactory::Instance()->Init(0, argv[1]);  
    }  
  
    Custom custom;  
    custom.Init();  
  
    while (true)  
    {  
        sleep(1);  
    }  
  
    return 0;  
}