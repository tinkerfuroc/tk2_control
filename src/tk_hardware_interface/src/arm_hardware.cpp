#include "tk_hardware_interface/arm_hardware.h"
#include <ros/ros.h>
#include <assert.h>
#include <map>

using std::string;
using std::pair;
using XmlRpc::XmlRpcValue;

namespace tinker {
namespace control {

ArmHardware::ArmHardware(XmlRpcValue& arm_info) {
    ROS_ASSERT(arm_info.getType() == XmlRpcValue::TypeStruct);
    int i = 0;
    for(XmlcIter it = arm_info.begin(); it != arm_info.end(); it++) {
        hardware_interface::JointStateHandle state_handle(
            it->first, &pos_[i], &vel_[i], &eff_[i]);
        jnt_state_interface_.registerHandle(state_handle);
        hardware_interface::JointHandle pos_handle(
            jnt_state_interface_.getHandle(it->first), &cmd_[i]);
        jnt_pos_interface_.registerHandle(pos_handle);
        i++;
    }
    registerInterface(&jnt_state_interface_);
}

void ArmHardware::Read() {
}

void ArmHardware::Write() {
}

}
}
