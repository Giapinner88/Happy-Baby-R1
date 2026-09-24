#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <algorithm>
#include <deque>
#include <iostream>
#include <vector>

#include "param.h"
#include "physics_joystick.h"
#include "sim_startup_gate.h"
#include "../../common/R1JointMap.hpp"

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        _apply_sim2real_mismatch();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }

    virtual void start() {}

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    // Sensor data indices
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    // Cố tình làm SIM LỆCH khỏi model lúc train, để test này có ý nghĩa nghiệm thu.
    // XML của sim và XML lúc train hiện là CÙNG một model (27 body, 30.182 kg) — nên
    // "chạy ngon trên sim" không chứng minh được gì về robot thật. Xem
    // HB/high_level_2/docs/PLAN_mimic_sim2real.md mục 3.
    void _apply_sim2real_mismatch()
    {
        const double s = param::config.sim2real_mass_scale;
        if (s != 1.0) {
            for (int i = 0; i < mj_model_->nbody; ++i) {
                mj_model_->body_mass[i] *= s;
                for (int k = 0; k < 3; ++k) mj_model_->body_inertia[3 * i + k] *= s;
            }
            std::cout << "[sim2real] Khối lượng + quán tính x" << s << "\n";
        }
        if (param::config.sim2real_delay_ms > 0)
            std::cout << "[sim2real] Trễ lệnh " << param::config.sim2real_delay_ms << " ms\n";
        if (param::config.sim2real_gain_scale != 1.0)
            std::cout << "[sim2real] Gains motor x" << param::config.sim2real_gain_scale << "\n";
    }

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        // Find sensor addresses by name
        int sensor_id = -1;
        
        // IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) {
            imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) {
            imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) {
            imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) {
            frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) {
            frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_quat");
        if (sensor_id >= 0) {
            secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_gyro");
        if (sensor_id >= 0) {
            secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_acc");
        if (sensor_id >= 0) {
            secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(mjModel *model, mjData *data) : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    // Lệnh xuống 1 motor, đã tách khỏi DDS để có thể làm TRỄ.
    struct MotorCmdSnap { double q = 0, dq = 0, kp = 0, kd = 0, tau = 0; };

    // Hàng đợi làm trễ lệnh. Bridge chạy 1 kHz -> 1 phần tử = 1 ms.
    // Sim mặc định có trễ = 0, còn robot thật thì không: đó là lý do policy chạy ngon
    // trên sim mà nảy trên robot. Đặt sim2real_delay_ms > 0 để sim mô phỏng đúng cái đó.
    std::deque<std::vector<MotorCmdSnap>> cmd_delay_buf_;
    std::vector<MotorCmdSnap> startup_hold_cmd_;
    std::vector<MotorCmdSnap> last_fresh_cmd_;
    bool received_first_lowcmd_ = false;
    bool command_timeout_logged_ = false;

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);

            // 1) Chụp lệnh hiện tại (đã map sang thứ tự khớp của sim)
            const int n = (param::config.robot == "r1") ? 24 : num_motor_;
            std::vector<MotorCmdSnap> snap(n);
            const bool command_fresh = !lowcmd->isTimeout();

            if (param::config.robot == "r1" && !command_fresh && !received_first_lowcmd_) {
                // DDS discovery and Rough scene parsing can take hundreds of ms. Hold the
                // keyframe pose during that gap instead of applying the default zero-gain
                // LowCmd and letting the robot free-fall.
                if (startup_hold_cmd_.empty()) {
                    static constexpr double kHoldKp[24] = {
                        100, 100, 100, 100, 40, 40,
                        100, 100, 100, 100, 40, 40,
                        100, 100,
                        40, 40, 20, 20, 20,
                        40, 40, 20, 20, 20,
                    };
                    static constexpr double kHoldKd[24] = {
                        2, 2, 2, 2, 2, 2,
                        2, 2, 2, 2, 2, 2,
                        2, 2,
                        2, 2, 1, 1, 1,
                        2, 2, 1, 1, 1,
                    };
                    startup_hold_cmd_.resize(n);
                    for (int i = 0; i < n; ++i) {
                        startup_hold_cmd_[i].q = mj_data_->sensordata[i];
                        startup_hold_cmd_[i].kp = kHoldKp[i];
                        startup_hold_cmd_[i].kd = kHoldKd[i];
                    }
                    std::cout << "[R1 startup] Chưa có LowCmd; giữ keyframe bằng PD an toàn.\n";
                }
                snap = startup_hold_cmd_;
            } else if (!command_fresh && received_first_lowcmd_ && !last_fresh_cmd_.empty()) {
                // Fail closed on a stale controller: retain the last finite PD command.
                snap = last_fresh_cmd_;
                if (!command_timeout_logged_) {
                    std::cerr << "[R1 safety] LowCmd timeout; giữ lệnh hợp lệ cuối cùng.\n";
                    command_timeout_logged_ = true;
                }
            } else {
                for(int i(0); i<n; i++) {
                    auto & m = lowcmd->msg_.motor_cmd()[
                        (param::config.robot == "r1") ? R1JointMap::kSimToIdl[i] : i];
                    snap[i].q   = m.q();
                    snap[i].dq  = m.dq();
                    // Gains motor thật lệch so với lệnh gửi xuống (hộp số, driver). gain_scale
                    // != 1.0 buộc policy phải chịu được sai lệch đó thay vì tựa vào gains chính xác.
                    snap[i].kp  = m.kp() * param::config.sim2real_gain_scale;
                    snap[i].kd  = m.kd() * param::config.sim2real_gain_scale;
                    snap[i].tau = m.tau();
                }
                if (param::config.robot == "r1") {
                    last_fresh_cmd_ = snap;
                    if (!received_first_lowcmd_) {
                        std::cout << "[R1 startup] Đã nhận LowCmd đầu tiên; chuyển quyền cho policy.\n";
                    }
                    received_first_lowcmd_ = true;
                    command_timeout_logged_ = false;
                }
            }

            // 2) Áp lệnh của N ms TRƯỚC (N = 0 -> hành vi y như cũ)
            cmd_delay_buf_.push_back(snap);
            const size_t depth = static_cast<size_t>(std::max(0, param::config.sim2real_delay_ms)) + 1;
            while(cmd_delay_buf_.size() > depth) cmd_delay_buf_.pop_front();
            const std::vector<MotorCmdSnap> & cmd = cmd_delay_buf_.front();

            // 3) PD dùng lệnh TRỄ nhưng cảm biến MỚI NHẤT — đúng như robot thật.
            for(int i(0); i<n; i++) {
                mj_data_->ctrl[i] = cmd[i].tau +
                                    cmd[i].kp * (cmd[i].q - mj_data_->sensordata[i]) +
                                    cmd[i].kd * (cmd[i].dq - mj_data_->sensordata[i + num_motor_]);
            }
            if (param::config.robot == "r1") {
                sim_startup::bridge_control_ready.store(true, std::memory_order_release);
            }
        }

        // lowstate
        if(lowstate->trylock()) {
            if (param::config.robot == "r1") {
                for(int i=0; i<35; i++) {
                    lowstate->msg_.motor_state()[i].q() = 0.0;
                    lowstate->msg_.motor_state()[i].dq() = 0.0;
                    lowstate->msg_.motor_state()[i].tau_est() = 0.0;
                }
                for(int i=0; i<24; i++) {
                    int idl_idx = R1JointMap::kSimToIdl[i];
                    lowstate->msg_.motor_state()[idl_idx].q() = mj_data_->sensordata[i];
                    lowstate->msg_.motor_state()[idl_idx].dq() = mj_data_->sensordata[i + num_motor_];
                    lowstate->msg_.motor_state()[idl_idx].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
                }
            } else {
                for(int i(0); i<num_motor_; i++) {
                    lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                    lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                    lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
                }
            }
            
            if(imu_quat_adr_ >= 0) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }
            
            if(imu_gyro_adr_ >= 0) {
                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
            }

            if(imu_acc_adr_ >= 0) {
                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
            }
            
            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            lowstate->unlockAndPublish();
        }
        // highstate
        if(highstate->trylock()) {
            if(frame_pos_adr_ >= 0) {
                highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;
    
private:
    unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(mjModel *model, mjData *data) : RobotBridge(model, data)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");
    }

    void run() override
    {
        RobotBridge::run();

        // secondary IMU state
        if (secondary_imustate->trylock()) {
            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
                secondary_imustate->msg_.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;
};
