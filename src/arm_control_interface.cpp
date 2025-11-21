#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"
#include "ik_client.hpp"
#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <fstream> 
#include <sstream>
#include <thread> 

using namespace unitree::robot;
using namespace unitree::common;

const std::string TOPIC = "rt/arm_Command";

class D1ArmController
{
private: 
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_>* publisher;
    bool initialized; 

public: 
    D1ArmController(): publisher(nullptr), initialized(false) {
    }

    ~D1ArmController() {
        if (publisher) {
            delete publisher; 
        }
    }

    bool init() {
        try {
            ChannelFactory::Instance()->Init(0);
            publisher = new ChannelPublisher<unitree_arm::msg::dds_::ArmString_>(TOPIC);
            publisher->InitChannel();
            initialized = true;
            return true;
        } catch (const std::exception& e) {
            initialized = false;
            return false;
        }
    }

    bool enable_joint_control() {
        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}";
        if(publisher->Write(msg))
            return true;
        return false; 
    }

    bool set_all_joint_angles(const std::vector<float>& joint_angles, const float gripper_width) {
        unitree_arm::msg::dds_::ArmString_ msg{};

        if (joint_angles.size() < 6) {
            std::cout << "ERROR: Not enough joint angles to command the arm." << std::endl;
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
            std::cout << "Joint angles successfully commanded" << std::endl; 
            return true;
        }
        return false; 
    }
};

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
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <command_file>" << std::endl;
        std::cerr << "Example: " << argv[0] << " motions.txt" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    std::ifstream file(filename); 

    if (!file.is_open()) {
        std::cerr << "Error: File not found - " << filename << std::endl;
        return 1; 
    }

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
    

    std::string line; 
    int line_num = 0; 
    int command_num = 0; 

    while(std::getline(file, line)) {
        line_num++; 

        if(line.empty() || line[0] == '#') {
            continue;
        }

        command_num++; 
        std::cout << "\n=== Command " << command_num << " ===" << std::endl;

        float target_pos[3]; 
        float target_orientation[4];
        bool has_orientation; 
        
        if (!parse_position(line, target_pos, target_orientation, has_orientation)) {
            std::cerr << "Skipping line " << line_num << std::endl;
            continue; 
        }

        std::cout << "Target position: [" << target_pos[0] << ", " 
                  << target_pos[1] << ", " << target_pos[2] << "]" << std::endl;

        std::vector<float> joint_angles; 

        auto start = std::chrono::high_resolution_clock::now();
        if (has_orientation) {
            if (ik_client.solve_ik(target_pos, target_orientation, joint_angles)) {
                std::cout << "IK Solved!" << std::endl; 
                auto end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start); 
                
                std::cout << "IK Solved in " << duration.count() << "ms" << std::endl; 
                joint_controller.set_all_joint_angles(joint_angles, 0);
            } else {
                std::cerr << "IK Failed!" << std::endl; 
            }
        } else {
            if (ik_client.solve_ik(target_pos, joint_angles)) {
                std::cout << "IK Solved!" << std::endl; 
                auto end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start); 
                
                std::cout << "IK Solved in " << duration.count() << "ms" << std::endl; 
                joint_controller.set_all_joint_angles(joint_angles, 0);
            } else {
                std::cerr << "IK Failed!" << std::endl; 
            }
        }

        std::cout << "Waiting 2 seconds before next command..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
    }

    file.close(); 
    std::cout << "\n === Sequence Complete === " << std::endl; 
    return 0; 
}