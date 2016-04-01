#ifndef __TINKER_PULSE_MOTOR_CONTROLLER_H__
#define __TINKER_PULSE_MOTOR_CONTROLLER_H__

#include <string>
#include <XmlRpc.h>
#include <sys/types.h>
#include "tk_hardware_interface/SwitchGPIO.h"
#include "tk_hardware_interface/motor_controller.h"

namespace tinker {
namespace control {

class PulseMotorController : public MotorController {
public:
    PulseMotorController(const XmlRpc::XmlRpcValue & pulse_motor_struct);
    virtual bool CheckStartLegal();
    virtual double Get();
    virtual void Set(double val);
    virtual ~PulseMotorController();
private:
    int dev_fd_;
    int origin_count_;
    double to_meter_fraction_;
    SwitchGPIOPtr legal_checker_; 
    unsigned motor_regs_[4];
    static const int kVEL = 0;
    static const int kCOUNT = 4;
    static const int kCTRL = 8;
    static const int kFEEDBACK = 12;
    static const unsigned CTRL_DIR = 1;
    static const unsigned CTRL_ENABLE = 1 << 1;
};

}
}

#endif
