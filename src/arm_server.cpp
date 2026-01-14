#include <sys/socket.h> 
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <cstring> 
#include <vector> 
#include <iostream>

#include "arm_server.h"
#include "arm_controller.h"

bool ArmServer::connect() {
    if (connected_) return true; 

    /* Create a socket */
    sock_ = socket(AF_INET, SOCK_STREAM, 0); 
    if (sock_ < 0) {
        std::cerr << "Failed to create socket" << std::endl; 
        return false; 
    }

    /* Set a timeout for the socket */
    struct timeval timeout;
    timeout.tv_sec = 5; 
    timeout.tv_usec = 0; 
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    /* Setup server address */
    struct sockaddr_in server_addr; 
    server_addr.sin_family = AF_INET; 
    server_addr.sin_port = htons(port_); 

    if (inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address: " << host_ << std::endl; 
        close(sock_);
        sock_ = -1; 
        return false; 
    }

    /* Connect to server */
    if (::connect(sock_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to connect to " << host_ << ":" << port_ << std::endl; 
        close(sock_); 
        sock_ = -1; 
        return false; 
    }

    connected_ = true; 
    std::cout << "Connected to IK server at " << host_ << ":" << port_ << std::endl;
    return true; 
}

void ArmServer::disconnect() {
    if (sock_ >= 0) {
        close(sock_); 
        sock_ = -1; 
    }
    connected_ = false; 
}

bool ArmServer::handle_request(D1ArmController& controller) {
    if (!connected_ && !connect()) {
        std::cerr << "Error: Not connected to server" << std::endl;
        return false; 
    }

    uint8_t buffer[256]; 
    ssize_t received = recv(sock_, buffer, sizeof(buffer), 0); 
    if (received < 1) {
        std::cerr << "No request sent from server." << std::endl;
        disconnect(); 
        return false; 
    }

    uint8_t request = buffer[0]; 

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

            ssize_t sent = send(sock_, buffer, num_bytes_to_send, 0); 
            if (sent != static_cast<ssize_t>(num_bytes_to_send)) {
                std::cerr << "Error: Failed to send all joint angles" << std::endl; 
                std::cerr << "Successfully sent " << sent << "/" << num_bytes_to_send <<  " bytes." << std::endl; 
                disconnect(); 
                return false; 
            }

            return true;
        }

        case 1: {
            /* Setting Arm Position to a Specific Joint Angle */
            uint8_t num_joints = buffer[1]; 
            if (received < 2 + num_joints * sizeof(float)) {
                std::cerr << "Error: Incomplete Target Joint Angles from Python Server" << std::endl; 
                return false; 
            }

            std::vector<float> joint_angles; 
            joint_angles.resize(num_joints); 
            memcpy(joint_angles.data(), &buffer[2], num_joints * sizeof(float));

            if (!controller.set_all_joint_angles(joint_angles, 0)) {
                std::cerr << "Failed to set joint angles" << std::endl;
                send(sock_, "ER", 2, 0);  // ✓ Send error
                return false;
            }

            return true; 
        }

        case 3: {
            /* Ping */
            ssize_t sent = send(sock_, "OK", 2, 0);
            if (sent != 2) {
                std::cerr << "Failed to send OK message on Ping Command" << std::endl;
                return false;
            }

            return true; 
        }

        default: 
            std::cerr << "Unknown Request Type from Python Server: " << request << std::endl; 
            return false;
    }

}