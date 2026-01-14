#ifndef ARM_CONTROL_INTERFACE_H
#define ARM_CONTROL_INTERFACE_H

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

#include <string>
#include <vector>
#include <fstream> 
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
    D1ArmController(): has_servo_data_(false) {}
    ~D1ArmController() {}

    bool init();
    bool enable_joint_control();
    bool set_all_joint_angles(const std::vector<float>& joint_angles, const float gripper_width);
    bool get_joint_angles(std::vector<float>& joint_angles);

private: 
    /* Get data from the robot arm */
    void servo_handler(const void* msg) 
    {
        std::lock_guard<std::mutex> lock(servo_mutex_);

        const unitree_arm::msg::dds_::PubServoInfo_* pm = 
            (const unitree_arm::msg::dds_::PubServoInfo_*)msg;

        latest_servo_data_ = *pm; 
        has_servo_data_ = true; 
    }

    /* Communication channel to D1 arm - sending data */
    std::unique_ptr<ChannelPublisher<unitree_arm::msg::dds_::ArmString_>> publisher;
    
    /* Communication channel to the D1 arm - receiving data */
    std::unique_ptr<ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>> subscriber;
    
    /* Mutex for accessing robot arm values */
    std::mutex servo_mutex_; 

    /* Most recently pulled robot arm values */
    unitree_arm::msg::dds_::PubServoInfo_ latest_servo_data_; 
    
    /* Flag set if there is currently arm data available */
    bool has_servo_data_; 
};

#endif //ARM_CONTROL_INTERFACE_H