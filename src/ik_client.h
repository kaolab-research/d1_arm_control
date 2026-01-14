#ifndef IK_CLIENT_H
#define IK_CLIENT_H

#include <sys/socket.h> 
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <cstring> 
#include <vector> 
#include <iostream>

#include "arm_control_interface.h"

class D1ArmController;

class IKClient {
    private: 
        std::string m_host;
        int m_port; 
        int m_sock; 
        bool m_connected; 
    
    public: 
        IKClient(const std::string& host="192.168.123.10", int port=5555)
        : m_host(host), m_port(port), m_sock(-1), m_connected(false) {}

        ~IKClient() { disconnect(); }

        bool is_connected() {
            return m_connected; 
        }

        bool connect() {
            if (m_connected) return true; 

            /* Create a socket */
            m_sock = socket(AF_INET, SOCK_STREAM, 0); 
            if (m_sock < 0) {
                std::cerr << "Failed to create socket" << std::endl; 
                return false; 
            }

            /* Set a timeout for the socket */
            struct timeval timeout;
            timeout.tv_sec = 5; 
            timeout.tv_usec = 0; 
            setsockopt(m_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            setsockopt(m_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

            /* Setup server address */
            struct sockaddr_in server_addr; 
            server_addr.sin_family = AF_INET; 
            server_addr.sin_port = htons(m_port); 

            if (inet_pton(AF_INET, m_host.c_str(), &server_addr.sin_addr) <= 0) {
                std::cerr << "Invalid address: " << m_host << std::endl; 
                close(m_sock);
                m_sock = -1; 
                return false; 
            }

            /* Connect to server */
            if (::connect(m_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
                std::cerr << "Failed to connect to " << m_host << ":" << m_port << std::endl; 
                close(m_sock); 
                m_sock = -1; 
                return false; 
            }

            m_connected = true; 
            std::cout << "Connected to IK server at " << m_host << ":" << m_port << std::endl;
            return true; 
        }

        void disconnect() {
            if (m_sock >= 0) {
                close(m_sock); 
                m_sock = -1; 
            }
            m_connected = false; 
        }

        bool ping() {
            if (!m_connected && !connect()) {
                std::cerr << "Failed to solve IK, not connected to server" << std::endl;
                return false; 
            }

            uint8_t buffer[8]; 
            buffer[0] = 3; 
            
            if (send(m_sock, buffer, 1, 0) != 1) {
                disconnect(); 
                return false; 
            }

            ssize_t received = recv(m_sock, buffer, sizeof(buffer), 0);
            if (received == 2 && buffer[0] == 'O' && buffer[1] == 'K') {
                return true; 
            }

            return false;
        }

        bool handle_request(D1ArmController& controller) {
            if (!m_connected && !connect()) {
                std::cerr << "Error: Not connected to server" << std::endl;
                return false; 
            }

            uint8_t buffer[256]; 
            ssize_t received = recv(m_sock, buffer, sizeof(buffer), 0); 
            if (received < 1) {
                std::cerr << "No request sent from server." << std::endl;
                disconnect(); 
                return false; 
            }

            uint8_t request = buffer[0]; 

            switch(request) {
                case 0: 
                    /* Requesting Joint Angles */
                    std::vector<float> cur_joint_angles; 
                    controller.get_joint_angles(cur_joint_angles); 

                    std::cout << "Current Joint Angles [";
                    for (size_t i = 0; i < cur_joint_angles.size(); i++) {
                        std::cout << cur_joint_angles[i];
                        if (i < cur_joint_angles.size() - 1) std::cout << ", ";
                    }
                    std::cout << "]" << std::endl;

                    buffer[0] = 0;
                    buffer[1] = cur_joint_angles.size();
                    memcpy(&buffer[2], cur_joint_angles.data(), sizeof(float) * cur_joint_angles.size());
                    size_t num_bytes_to_send = 2 + sizeof(float) * cur_joint_angles.size();

                    ssize_t sent = send(m_sock, buffer, num_bytes_to_send, 0); 
                    if (sent != static_cast<ssize_t>(num_bytes_to_send)) {
                        std::cerr << "Error: Failed to send all joint angles" << std::endl; 
                        std::cerr << "Successfully sent " << sent << "/" << num_bytes_to_send <<  " bytes." << std::endl; 
                        disconnect(); 
                        return false; 
                    }

                    return true;

                case 1: 
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
                        send(m_sock, "ER", 2, 0);  // ✓ Send error
                        return false;
                    }

                    return true; 
                
                case 3: 
                    /* Ping */
                    ssize_t sent = send(m_sock, "OK", 2, 0);
                    if (sent != 2) {
                        std::cerr << "Failed to send OK message on Ping Command" << std::endl;
                        return false;
                    }

                    return true; 

                default: 
                    std::cerr << "Unknown Request Type from Python Server: " << request << std::endl; 
                    return false;
            }

        }
};

#endif // IK_CLIENT_H