#include "arm_server.h"
#include "arm_controller.h"

#include <iostream>
#include <chrono>

int main(int argc, char** argv)
{
    /* Setup IK Client for Server Connection */
    ArmServer arm_server;
    if (!arm_server.connect()) {
        std::cerr << "Failed to connect to IK Server" << std::endl; 
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
        if (arm_server.handle_request(joint_controller)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
            std::cerr << "Error: Unable to Handle Request" << std::endl; 
            break; 
        }
    }

    return 0; 
}