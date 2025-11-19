#include "ik_client.hpp"

#include <string>
#include <vector>

#define TOPIC "rt/arm_Command"

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
    float target_pos[3] = {0.3, 0.5, 1.0}; 
    std::vector<float> joint_angles; 

    /* Command Joint Position to D1 Arm */
    auto start = std::chrono::high_resolution_clock::now();
    if (ik_client.solve_ik(target_pos, joint_angles)) {
        std::cout << "IK Solved!" << std::endl; 
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start); 
        
        std::cout << "IK Solved in " << duration.count() << "ms" << std::endl; 
    } else {
        std::cerr << "IK Failed!" << std::endl; 
    }
}