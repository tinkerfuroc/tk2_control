#ifndef __TINKER_PWM_MOTOR_CONTROLLER_H__
#define __TINKER_PWM_MOTOR_CONTROLLER_H__

#include <string>
#include <XmlRpc.h>
#include <sys/types.h>
#include "tk_hardware_interface/SwitchGPIO.h"
#include "tk_hardware_interface/motor_controller.h"

namespace tinker {
namespace control {

class PWMMotorController : public MotorController {
public:
    PWMMotorController(const XmlRpc::XmlRpcValue& pulse_motor_struct);
    virtual bool CheckStartLegal();
    virtual double Get();
    virtual void Set(double val);
    virtual ~PWMMotorController();

private:
    int dev_fd_;
    int origin_count_;
    double to_meter_fraction_;
    SwitchGPIOPtr legal_checker_;
    unsigned motor_regs_[2];
};
}
}

#endif
