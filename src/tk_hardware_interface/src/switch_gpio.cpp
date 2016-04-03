#include "tk_hardware_interface/switch_gpio.h"
#include <ros/ros.h>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <math.h>
#include <vector>

namespace tinker {
namespace control {

SwitchGPIO::SwitchGPIO(const std::string& gpio_file_name) : gpio_fd_(-1) {
    gpio_fd_ = open(gpio_file_name.c_str(), O_RDWR, 0);
    if (gpio_fd_ < 0) 
        ROS_ERROR("failed to open gpio file %s", gpio_file_name.c_str());
}

bool SwitchGPIO::Get() {
    if(gpio_fd_ < 0) return 1;
    char c;
    if (read(gpio_fd_, &c, 1) != 1) {
        ROS_ERROR("failed to read gpio file");
    }
    c = c & 0x01;
    lseek(gpio_fd_, 0, SEEK_SET);
    return c;
}

SwitchGPIO::~SwitchGPIO() {
    close(gpio_fd_);
}

}
}
