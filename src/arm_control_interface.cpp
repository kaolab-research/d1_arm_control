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
    
    while(true) {
        if (ik_client.handle_request(joint_controller)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
            std::cerr << "Error: Unable to Handle Request" << std::endl; 
            break; 
        }
    }

    return 0; 
}