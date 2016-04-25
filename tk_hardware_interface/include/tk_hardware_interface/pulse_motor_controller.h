#ifndef __TINKER_PULSE_MOTOR_CONTROLLER_H__
#define __TINKER_PULSE_MOTOR_CONTROLLER_H__

#include <string>
#include <XmlRpc.h>
#include <sys/types.h>
#include "tk_hardware_interface/switch_gpio.h"
#include "tk_hardware_interface/motor_controller.h"

namespace tinker {
namespace control {

class PulseMotorController : public MotorController {
public:
    PulseMotorController(XmlRpc::XmlRpcValue & pulse_motor_info);
    virtual bool CheckStartLegal();
    virtual double Get();
    virtual void Set(double val);
    virtual ~PulseMotorController();
private:
    std::string name_;
    int dev_fd_;
    double to_meter_fraction_;
    SwitchGPIOPtr legal_checker_; 
    unsigned * motor_regs_;
    unsigned speed_;
    bool can_move_;
    unsigned start_count_; 
    double last_target_;
    static const int kVEL = 0;
    static const int kCOUNT = 1;
    static const int kCTRL = 2;
    static const int kFEEDBACK = 3;
    static const unsigned CTRL_DIR = 1;
    static const unsigned CTRL_ENABLE = 1 << 1;
};

}
}

#endif
