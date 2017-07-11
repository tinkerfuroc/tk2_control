#include <ros/ros.h>
#include "tk_hardware_interface/motor_controller.h"
#include "tk_hardware_interface/pulse_motor_controller.h"
#include "tk_hardware_interface/pwm_motor_controller.h"

namespace tinker {
namespace control {

MotorPtr MotorController::GetController(const std::string &class_name,
                                        XmlRpc::XmlRpcValue &motor_info) {
    if (class_name == "Pulse") {
        ROS_INFO("Add a new PULSE motor");
        return MotorPtr(new PulseMotorController(motor_info));
    } else if (class_name == "PWM") {
        ROS_INFO("Add a new PWM motor");
        return MotorPtr(new PWMMotorController(motor_info));
    }
    return MotorPtr();
}
}
}
