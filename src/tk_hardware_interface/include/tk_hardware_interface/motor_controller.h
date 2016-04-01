#ifndef __TINKER_MOTOR_CONTROLLER_H__
#define __TINKER_MOTOR_CONTROLLER_H__

#include "tk_hardware_interface/common.h"

namespace tinker {
namespace control {

class MotorController {
public:
    virtual bool CheckStartLegal() = 0;
    virtual double Get() = 0;
    virtual void Set(double val) = 0;
    virtual ~MotorController() = 0;
};

}
}


#endif
