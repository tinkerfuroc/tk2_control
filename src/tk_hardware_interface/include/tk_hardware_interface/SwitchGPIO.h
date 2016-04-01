#ifndef __TINKER_SWITCH_GPIO_H__
#define __TINKER_SWITCH_GPIO_H__

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <vector>
#include "tk_hardware_interface/common.h"

namespace tinker {
namespace control {

class SwitchGPIO {
public:
    SwitchGPIO(const std::string & gpio_file_name);
    bool get();
    ~SwitchGPIO();
private:
    int gpio_fd_;
};

typedef boost::shared_ptr<SwitchGPIO> SwitchGPIOPtr;

}
}

#endif
