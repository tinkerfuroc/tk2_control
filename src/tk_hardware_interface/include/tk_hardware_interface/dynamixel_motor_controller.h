#ifndef __TINKER_DYNAMIXEL_MOTOR_CONTROLLER_H__
#define __TINKER_DYNAMIXEL_MOTOR_CONTROLLER_H__

#include <string>
#include <XmlRpc.h>
#include <sys/types.h>
#include "tk_hardware_interface/switch_gpio.h"
#include "tk_hardware_interface/motor_controller.h"
#include <ros/ros.h>

namespace tinker {
namespace control {

class DynamixelMotorController : public MotorController {
public:
    DynamixelMotorController(XmlRpc::XmlRpcValue & pulse_motor_info);
    virtual bool CheckStartLegal();
    virtual double Get();
    virtual void Set(double val);
private:
    std::string name_;
    ros::NodeHandle private_nh_;
};

}
}

#endif
