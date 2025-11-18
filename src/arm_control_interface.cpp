#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>

#include <string>
#include <vector>

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

class JointAngleControl 
{
private: 
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);

public: 
    JoinAngleControl() {
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

    bool set_all_joint_angles(float joint_angles[7]) {
        unitree_arm::msg::dds_::ArmString_ msg{};

        String cmd_msg = "{\"seq\":4,\"address\":1,\"funcode\":2,
                            \"data\":{\"mode\":1,
                            \"angle0\":" + join_angles[0] + ",
                            \"angle1\":" + join_angles[1] + ",
                            \"angle2\":" + join_angles[2] + ",
                            \"angle3\":" + join_angles[3] + ",
                            \"angle4\":" + join_angles[4] + ",
                            \"angle5\":" + join_angles[5] + ",
                            \"angle6\":" + join_angles[6] + "}}";
        
        msg.data_() = cmd_msg;
        if(publisher.Write(msg))
            return true;
        return false; 
    }
};


int main()
{
    IKClient ik_client;
    JointAngleControl joint_controller;  

    float[3] target_pos = [0.3, 0.5, 1.0]; 
    float[7] target_angles = [];
    if (!ik_client.solveIK(target_pos, &target_angles)) {
        return 1;
    }

    joint_controller.enable_joint_control(); 
    joint_controller.set_all_join_angles(target_angles); 
}