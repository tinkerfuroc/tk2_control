#ifndef __TINKER_MOTOR_CONTROLLER_H__
#define __TINKER_MOTOR_CONTROLLER_H__

#include "tk_hardware_interface/common.h"
#include <boost/shared_ptr.hpp>

namespace tinker {
namespace control {

class MotorController {
public:
    virtual bool CheckStartLegal() = 0;
    virtual double Get() = 0;
    virtual void Set(double val) = 0;
    virtual ~MotorController() {}
    static boost::shared_ptr<MotorController> GetController(
        const std::string &class_name, XmlRpc::XmlRpcValue &motor_info);
};

typedef boost::shared_ptr<MotorController> MotorPtr;
}
}

#endif
