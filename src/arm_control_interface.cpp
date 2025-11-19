#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "ik_client.hpp"

#include <string>
#include <vector>

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

class D1ArmController
{
private: 
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);

public: 
    D1ArmController() {
        ChannelFactory::Instance()->Init(0);
        publisher.InitChannel();
    }

    bool enable_joint_control() {
        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}";
        if(publisher.Write(msg))
            return true;
        return false; 
    }

    bool set_all_joint_angles(const std::vector<float>joint_angles[6], const float gripper_width) {
        unitree_arm::msg::dds_::ArmString_ msg{};

        String cmd_msg = "{\"seq\":4,\"address\":1,\"funcode\":2,
                            \"data\":{\"mode\":1,
                            \"angle0\":" + joint_angles[0] + ",
                            \"angle1\":" + joint_angles[1] + ",
                            \"angle2\":" + joint_angles[2] + ",
                            \"angle3\":" + joint_angles[3] + ",
                            \"angle4\":" + joint_angles[4] + ",
                            \"angle5\":" + joint_angles[5] + ",
                            \"angle6\":" + gripper_width + "}}";
        
        msg.data_() = cmd_msg;
        if(publisher.Write(msg))
            std::cout << "Joint angles successfully commanded" << std::endl; 
            return true;
        return false; 
    }
};


int main()
{
    /* Setup IK Client for Server Connection */
    IKClient ik_client;
    if (!ik_client.connect()) {
        std::cerr << "Failed to connect to IK Server" << std::endl; 
        return 1; 
    }

    std::cout << "Sending ping to server..." << std::endl; 
    if (ik_client.ping()) {
        std::cout << "OK/n" << std::endl; 
    } else {
        std::cout << "FAILED/n" << std::endl; 
        return 1; 
    }

    /* Initialized Arm Controller */
    D1ArmController joint_controller;  
    joint_controller.enable_joint_control(); 
    
    float target_pos[3] = {0.3, 0.5, 1.0}; 
    std::vector<float> joint_angles; 

    /* Command Joint Position to D1 Arm */
    auto start = std::chrono::high_resolution_clock::now();
    if (ik_client.solve_ik(target_pos, joint_angles)) {
        std::cout << "IK Solved!" << std::endl; 
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start); 
        
        std::cout << "IK Solved in " << duration.count() << "ms" << std::endl; 
        joint_controller.set_all_joint_angles(joint_angles, 0);
    } else {
        std::cerr << "IK Failed!" << std::endl; 
    }

    if (!ik_client.solveIK(target_pos, &target_angles)) {
        return 1;
    }

    joint_controller.enable_joint_control(); 
    joint_controller.set_all_join_angles(target_angles); 
}