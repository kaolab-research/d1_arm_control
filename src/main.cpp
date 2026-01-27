#include "arm_server.h"
#include "arm_controller.h"

#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    /* Initialized Arm Controller */
    D1ArmController arm_controller;  
    bool init = arm_controller.init();
    if (init) {
        std::cout << "D1 Arm Controller Initialized" << std::endl; 
    } else {
        std::cout << "ERROR: D1 Arm Controller Failed to Initialize" << std::endl; 
        return 1; 
    }

    arm_controller.enable_joint_control(); 

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ArmServer arm_server;
    if (!arm_server.start_listening()) {
        std::cerr << "Failed to start server" << std::endl; 
        return 1; 
    }

    if (!arm_server.accept_connection()) {
        std::cerr << "Failed to accept connection" << std::endl; 
        return 1; 
    }
    
    while(true) {
        if (!arm_server.handle_request(arm_controller)) {
            std::cerr << "Error handling request. Wait for reconnect..." << std::endl; 
            if (!arm_server.accept_connection()) {
                break; 
            }
        }
    }

    return 0; 
}