#ifndef IK_CLIENT_H
#define IK_CLIENT_H

#include <sys/socket.h> 
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <cstring> 
#include <vector> 
#include <iostream>

class IKClient {
    private: 
        std::string m_host;
        int m_port; 
        int m_sock; 
        bool m_connected; 
    
    public: 
        IKClient(const std::string& host="127.0.0.1", int port=5555)
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

        bool solve_ik(const float target_pos[3], std::vector<float>& joint_angles) {
            if (!m_connected && !connect()) {
                std::cerr << "Failed to solve IK, not connected to server" << std::endl;
                return false; 
            }

            uint8_t buffer[256]; 
            buffer[0] = 1; /* Set first byte to position only command (1) */
            memcpy(&buffer[1], target_pos, 3 * sizeof(float)); /* Set next bytes to target position */

            ssize_t sent = send(m_sock, buffer, 13, 0); 
            if (sent != 13) {
                std::cerr << "Failed to send IK request" << std::endl; 
                disconnect(); 
                return false; 
            }

            ssize_t received = recv(m_sock, buffer, sizeof(buffer), 0);
            if (received < 1) {
                std::cerr << "Failed to receive IK response" << std::endl; 
                disconnect(); 
                return false; 
            }

            uint8_t num_joints = buffer[0]; 
            if (received < 1 + num_joints * sizeof(float)) {
                std::cerr << "Incomplete IK response received from IK server" << std::endl; 
                return false; 
            }

            joint_angles.resize(num_joints); 
            memcpy(joint_angles.data(), &buffer[1], num_joints * sizeof(float));

            return true; 
        }

        bool solve_ik(const float target_pos[3], const float target_orientation[4], std::vector<float>& joint_angles) {
            if (!m_connected && !connect()) {
                std::cerr << "Failed to solve IK, not connected to server" << std::endl;
                return false; 
            }

            uint8_t buffer[256]; 
            buffer[0] = 1; /* Set first byte to position only command (1) */
            memcpy(&buffer[1], target_pos, 3 * sizeof(float)); /* Set next bytes to target position */
            memcpy(&buffer[13], target_orientation, 4 * sizeof(float)); 

            ssize_t sent = send(m_sock, buffer, 29, 0); 
            if (sent != 29) {
                std::cerr << "Failed to send IK request" << std::endl; 
                disconnect(); 
                return false; 
            }

            ssize_t received = recv(m_sock, buffer, sizeof(buffer), 0);
            if (received < 1) {
                std::cerr << "Failed to receive IK response" << std::endl; 
                disconnect(); 
                return false; 
            }

            uint8_t num_joints = buffer[0]; 
            if (received < 1 + num_joints * sizeof(float)) {
                std::cerr << "Incomplete IK response received from IK server" << std::endl; 
                return false; 
            }

            joint_angles.resize(num_joints); 
            memcpy(joint_angles.data(), &buffer[1], num_joints * sizeof(float));

            return true; 
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
};

#endif IK_CLIENT_H

