#include "arm_controller.h"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"
#include "arm_server.h"

#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <fstream> 
#include <sstream>
#include <thread> 
#include <mutex> 
#include <memory>

bool D1ArmController::init() {
    try {
        ChannelFactory::Instance()->Init(0);
        publisher = std::make_unique<ChannelPublisher<unitree_arm::msg::dds_::ArmString_>>(TOPIC_P);
        publisher->InitChannel();

        subscriber = std::make_unique<ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>>(TOPIC_S);
        subscriber->InitChannel([this](const void* msg) {
            this->servo_handler(msg);
        });

        return true;
    } 
    catch (const std::exception& e) 
    {
        return false;
    }
}

bool D1ArmController::enable_joint_control() {
    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}";
    if(publisher->Write(msg))
        return true;
    return false; 
}

bool D1ArmController::set_all_joint_angles(const std::vector<float>& joint_angles, const float gripper_width) {
    unitree_arm::msg::dds_::ArmString_ msg{};

    if (joint_angles.size() < 6) {
        std::cout << "ERROR: Not enough joint angles to command the arm." << std::endl;
        return false; 
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
        std::cout << "Commanded: 0: " << joint_angles[0] 
            << ", 1: " << joint_angles[1]
            << ", 2: " << joint_angles[2]
            << ", 3: " << joint_angles[3]
            << ", 4: " << joint_angles[4]
            << ", 5: " << joint_angles[5]
            << ", Gripper: " << gripper_width
            << joint_angles[1]<< std::endl; 
        return true;
    }
    return false; 
}

bool D1ArmController::home_joint_angles() 
{
    std::vector<float> joint_angles = {0, -90, 90, 90, 0, 90};
    if (!set_all_joint_angles(joint_angles, 0))
        return false;
    return true; 
}

bool D1ArmController::set_gripper_width(const float gripper_width) {
    unitree_arm::msg::dds_::ArmString_ msg{};

    std::string cmd_msg = "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{"
    "\"mode\":1,\"angle6\":" + std::to_string(gripper_width) + "}}";

    std::cout << cmd_msg << std::endl;

    msg.data_() = cmd_msg;
    publisher->Write(msg);
    return true; 
}

bool D1ArmController::get_joint_angles(std::vector<float>& joint_angles) 
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