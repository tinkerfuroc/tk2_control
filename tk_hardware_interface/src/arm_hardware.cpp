#include "tk_hardware_interface/arm_hardware.h"
#include <ros/ros.h>
#include <assert.h>
#include <map>
#include <cstdio>

using std::string;
using std::pair;
using XmlRpc::XmlRpcValue;

namespace tinker {
namespace control {

ArmHardware::ArmHardware(XmlRpcValue& arm_info) {
    ROS_ASSERT(arm_info.getType() == XmlRpcValue::TypeStruct);
    cmd_.resize(arm_info.size(), 0);
    pos_.resize(arm_info.size(), 0);
    vel_.resize(arm_info.size(), 0);
    eff_.resize(arm_info.size(), 0);
    int i = 0;
    for (XmlIter it = arm_info.begin(); it != arm_info.end(); it++) {
        ROS_INFO("Registering for joint %s", it->first.c_str());
        ROS_ASSERT(it->second.hasMember("type"));
        string type = it->second["type"];
        MotorPtr motor = MotorController::GetController(type, it->second);
        if (!motor) {
            ROS_ERROR("Unkown motor class %s", type.c_str());
            continue;
        }
        motors.push_back(motor);

        hardware_interface::JointStateHandle state_handle(it->first, &pos_[i],
                                                          &vel_[i], &eff_[i]);
        jnt_state_interface_.registerHandle(state_handle);

        hardware_interface::JointHandle pos_handle(
            jnt_state_interface_.getHandle(it->first), &cmd_[i]);
        jnt_pos_interface_.registerHandle(pos_handle);
        i++;
    }
    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_pos_interface_);
    }

void ArmHardware::Read() {
    for (int i = 0; i < cmd_.size(); i++) 
        pos_[i] = motors[i]->Get();
}

void ArmHardware::Write() {
    for (int i = 0; i < cmd_.size(); i++) 
        motors[i]->Set(cmd_[i]);
}

}
}
