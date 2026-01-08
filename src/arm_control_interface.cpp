#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"
#include "ik_client.hpp"
#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <fstream> 
#include <sstream>
#include <thread> 
#include <mutex> 
#include <memory>

using namespace unitree::robot;
using namespace unitree::common;

const std::string TOPIC_P = "rt/arm_Command";
const std::string TOPIC_S = "current_servo_angle";

class D1ArmController
{
public: 
    D1ArmController(): publisher(nullptr), initialized(false) {}

    ~D1ArmController() {}

    bool init() {
        try {
            ChannelFactory::Instance()->Init(0);
            publisher = new ChannelPublisher<unitree_arm::msg::dds_::ArmString_>(TOPIC_P);
            publisher->InitChannel();

            subscriber = new ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>(TOPIC_S);
            subscriber->InitChannel([this](const void* msg) {
                this->servo_handler(msg);
            });

            initialized = true;
            return true;
        } catch (const std::exception& e) {
            initialized = false;
            return false;
        }
    }

    bool enable_joint_control() {
        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}";
        if(publisher->Write(msg))
            return true;
        return false; 
    }

    bool set_all_joint_angles(const std::vector<float>& joint_angles, const float gripper_width) {
        unitree_arm::msg::dds_::ArmString_ msg{};

        if (joint_angles.size() < 6) {
            std::cout << "ERROR: Not enough joint angles to command the arm." << std::endl;
        }

        std::string cmd_msg = "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{"
        "\"mode\":1,"
        "\"angle0\":" + std::to_string(joint_angles[0]) +
        ",\"angle1\":" + std::to_string(joint_angles[1]) +
        ",\"angle2\":" + std::to_string(joint_angles[2]) +
        ",\"angle3\":" + std::to_string(joint_angles[3]) +
        ",\"angle4\":" + std::to_string(joint_angles[4]) +
        ",\"angle5\":" + std::to_string(joint_angles[5]) +
        ",\"angle6\":" + std::to_string(gripper_width) +
        "}}";

        std::cout << cmd_msg << std::endl;

        msg.data_() = cmd_msg;
        if(publisher->Write(msg)) {
            std::cout << "Joint angles successfully commanded" << std::endl; 
            return true;
        }
        return false; 
    }

    bool get_joint_angles(std::vector<float>& joint_angles) 
    {
        std::lock_guard<std::mutex> lock(servo_mutex_); 

        if (!has_servo_data_)
            return false;

        joint_angles = {
            latest_servo_data_.servo0_data_(),
            latest_servo_data_.servo1_data_(),
            latest_servo_data_.servo2_data_(),
            latest_servo_data_.servo3_data_(),
            latest_servo_data_.servo4_data_(),
            latest_servo_data_.servo5_data_(),
            latest_servo_data_.servo6_data_(),
        };

        return true;
    }

private: 
    void servo_handler(const void* msg) 
    {
        std::lock_guard<std::mutex> lock(servo_mutex_);

        const unitree_arm::msg::dds_::PubServoInfo_* pm = 
            (const unitree_arm::msg::dds_::PubServoInfo_*)msg;

        latest_servo_data_ = *pm; 
        has_servo_data_ = true; 
    }

    std::unique_ptr<ChannelPublisher<unitree_arm::msg::dds_::ArmString_>> publisher;
    std::unique_ptr<ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>> subscriber;
    
    std::mutex servo_mutex_; 
    unitree_arm::msg::dds_::PubServoInfo_ latest_servo_data_; 
    bool has_servo_data_ = false; 
    
    bool initialized; 
};

bool parse_position(const std::string& line, float target_pos[3], float target_orientation[4], bool& has_orientation) {
    std::stringstream ss(line); 

    if (!(ss >> target_pos[0] >> target_pos[1] >> target_pos[2])) {
        std::cerr << "Error: Unable to extract position information from line" << std::endl;
        return false; 
    }

    has_orientation = false; 
    if (ss >> target_orientation[0] >> target_orientation[1] >> target_orientation[2] >> target_orientation[3]) {
        has_orientation = true; 
        std::cout << "Orientation found: [" << target_orientation[0] << target_orientation[1] << target_orientation[2] << target_orientation[3] << "]" << std::endl;
    }

    return true; 
}


int main(int argc, char** argv)
{
    /* Setup IK Client for Server Connection */
    IKClient ik_client;
    if (!ik_client.connect()) {
        std::cerr << "Failed to connect to IK Server" << std::endl; 
        return 1; 
    }

    std::cout << "Sending ping to server..." << std::endl; 
    if (ik_client.ping()) {
        std::cout << "OK" << std::endl; 
    } else {
        std::cout << "FAILED" << std::endl; 
        return 1; 
    }

    /* Initialized Arm Controller */
    D1ArmController joint_controller;  
    bool init = joint_controller.init();
    if (init) {
        std::cout << "D1 Arm Controller Initialized" << std::endl; 
    } else {
        std::cout << "ERROR: D1 Arm Controller Failed to Initialize" << std::endl; 
        return 1; 
    }
    joint_controller.enable_joint_control(); 
    float target_pos[3] = {0, 0, 0};
    std::vector<float> joint_angles; 
    while(true)
        if (ik_client.solve_ik(target_pos, joint_angles)) {
            std::cout << "IK Solved!" << std::endl;  
            joint_controller.set_all_joint_angles(joint_angles, 0);
        } else {
            std::cerr << "IK Failed!" << std::endl; 
        }

    return 0; 
}