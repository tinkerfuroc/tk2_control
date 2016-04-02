#ifndef __TINKER_PWM_MOTOR_CONTROLLER_H__
#define __TINKER_PWM_MOTOR_CONTROLLER_H__

#include <string>
#include <XmlRpc.h>
#include <sys/types.h>
#include "tk_hardware_interface/motor_controller.h"
#include "tk_hardware_interface/switch_gpio.h"
#include "tk_hardware_interface/interpolate.h"

namespace tinker {
namespace control {

class PWMMotorController : public MotorController {
public:
    PWMMotorController(XmlRpc::XmlRpcValue& pwm_motor_info);
    virtual bool CheckStartLegal();
    virtual double Get();
    virtual void Set(double val);
    virtual ~PWMMotorController();
private: 
    void Set_(double val);

    std::string name_;
    int dev_fd_;
    SwitchGPIOPtr legal_checker_;
    LinearInterpolation interpolation_;
    double now_angle_;
    bool can_move_;
    unsigned * motor_regs_;
    static const unsigned kFREQ = 100000000;
    static const int kRATE = 50;
    static const int kCYCLE = 0;
    static const int kWIDTH = 1;
};
}
}

#endif
