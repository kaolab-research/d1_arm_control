#include <sys/socket.h> 
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <cstring> 
#include <vector> 
#include <iostream>

#include "arm_server.h"
#include "arm_controller.h"

bool ArmServer::start_listening() 
{
    /* Create a socket */
    server_sock_ = socket(AF_INET, SOCK_STREAM, 0); 
    if (server_sock_ < 0) {
        std::cerr << "Failed to create server socket" << std::endl; 
        return false; 
    }

    int opt = 1; 
    setsockopt(server_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    /* Setup server address */
    struct sockaddr_in server_addr; 
    server_addr.sin_family = AF_INET; 
    server_addr.sin_addr.s_addr = INADDR_ANY; 
    server_addr.sin_port = htons(port_); 

    if (bind(server_sock_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to bind to port " << port_ << std::endl;
        close(server_sock_);
        return false; 
    }

    if (listen(server_sock_, 1) < 0) {
        std::cerr << "Failed to listen on port " << port_ << std::endl; 
        close(server_sock_);
        return false; 
    }

    listening_ = true; 
    std::cout << "Arm server listening on port " << port_ << std::endl; 
    return true; 
}

bool ArmServer::accept_connection() {
    std::cout << "Waiting for Python client to connect" << std::endl; 

    struct sockaddr_in client_addr; 
    socklen_t client_len = sizeof(client_addr); 
    client_sock_ = accept(server_sock_, (struct sockaddr*)&client_addr, &client_len);

    if (client_sock_ < 0) {
        std::cerr << "Failed to accept connection" << std::endl;
        return false;
    }

    std::cout << "Python client connected" << std::endl;
    return true;
}

bool ArmServer::handle_request(D1ArmController& controller) {

    uint8_t buffer[256]; 
    ssize_t received = recv(client_sock_, buffer, sizeof(buffer), 0); 
    if (received < 1) {
        std::cerr << "No request sent from server." << std::endl;
        return false; 
    }

    uint8_t request = buffer[0]; 
    std::cerr << "Request: " << request << std::endl; 

    switch(request) {
        case 0: {
            /* Requesting Joint Angles */
            std::vector<float> cur_joint_angles; 
            controller.get_joint_angles(cur_joint_angles); 

            std::cout << "Current Joint Angles [";
            for (size_t i = 0; i < cur_joint_angles.size(); i++) {
                std::cout << cur_joint_angles[i];
                if (i < cur_joint_angles.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;

            buffer[0] = cur_joint_angles.size();
            memcpy(&buffer[1], cur_joint_angles.data(), sizeof(float) * cur_joint_angles.size());
            size_t num_bytes_to_send = 1 + sizeof(float) * cur_joint_angles.size();

            ssize_t sent = send(client_sock_, buffer, num_bytes_to_send, 0); 
            if (sent != static_cast<ssize_t>(num_bytes_to_send)) {
                std::cerr << "Error: Failed to send all joint angles" << std::endl; 
                std::cerr << "Successfully sent " << sent << "/" << num_bytes_to_send <<  " bytes." << std::endl; 
                return false; 
            }

            return true;
        }

        case 1: {
            /* Setting Arm Position to a Specific Joint Angle */
            uint8_t num_joints = buffer[1]; 
            if (received < 2 + num_joints * sizeof(float) + sizeof(float)) {
                std::cerr << "Error: Incomplete Target Joint Angles from Python Server" << std::endl; 
                return false; 
            }

            std::vector<float> joint_angles; 
            joint_angles.resize(num_joints);
            float gripper_width; 
            memcpy(joint_angles.data(), &buffer[2], num_joints * sizeof(float));
            memcpy(&gripper_width, &buffer[2 + num_joints * sizeof(float)], sizeof(float));

            float gripper_width_command_value_mm = 65.0 * gripper_width; 

            /* Eventually send gripper width over here */
            if (!controller.set_all_joint_angles(joint_angles, gripper_width_command_value_mm)) {
                std::cerr << "Failed to set joint angles" << std::endl;
                send(client_sock_, "ER", 2, 0);  // ✓ Send error
                return false;
            }

            return true; 
        }

        case 3: {
            /* Ping */
            ssize_t sent = send(client_sock_, "OK", 2, 0);
            if (sent != 2) {
                std::cerr << "Failed to send OK message on Ping Command" << std::endl;
                return false;
            }

            return true; 
        }

        case 4: 
            /* Reset Arm to Zero */
            if (!controller.home_joint_angles()) {
                std::cerr << "Failed home joint angles" << std::endl;
                return false;
            }
            return true;
        
        case 5: 
            /* Command Gripper Width Only */
            float gripper_width; 
            memcpy(&gripper_width, &buffer[1], sizeof(float));
            gripper_width = 65.0 * gripper_width; 

            if(!controller.set_gripper_width(gripper_width)) {
                std::cerr << "Failed to command gripper width" << std::endl; 
                return false;
            }
            return true;
            
        default: 
            std::cerr << "Unknown Request Type from Python Server: " << request << std::endl; 
            return false;
    }

}