#ifndef __TINKER_ARM_HARDWARE_H__
#define __TINKER_ARM_HARDWARE_H__

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <XmlRpc.h>
#include <vector>
#include "tk_hardware_interface/motor_controller.h"

namespace tinker {
namespace control {

class ArmHardware : public hardware_interface::RobotHW {
public:
    ArmHardware(XmlRpc::XmlRpcValue& arm_info);
    // read and write means reading and writing to the private regs here
    void Read();
    void Write();

private:
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;
    std::vector<MotorPtr> motors;
    std::vector<double> cmd_;
    std::vector<double> pos_;
    std::vector<double> vel_;
    std::vector<double> eff_;
};
}
}

#endif
