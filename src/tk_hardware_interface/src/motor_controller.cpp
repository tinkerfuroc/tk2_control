#include "tk_hardware_interface/motor_controller.h"
#include "tk_hardware_interface/pulse_motor_controller.h"
#include "tk_hardware_interface/pwm_motor_controller.h"

namespace tinker {
namespace control {

MotorPtr MotorController::GetController(const std::string &class_name,
                                        XmlRpc::XmlRpcValue &motor_info) {
    if (class_name == "Pulse") {
        return MotorPtr(new PulseMotorController(motor_info));
    } else if (class_name == "PWM") {
        return MotorPtr(new PWMMotorController(motor_info));
    }
    return MotorPtr();
}
}
}
