#include "tk_hardware_interface/pwm_motor_controller.h"
#include <ros/ros.h>
#include <XmlRpcValue.h>
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

using std::string;
using std::vector;

PWMMotorController::PWMMotorController(
    XmlRpc::XmlRpcValue &pwm_motor_info)
    : motor_regs_(NULL), now_angle_(0), commanded_(false){
    ROS_ASSERT(pwm_motor_info.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(pwm_motor_info.hasMember("name"));
    ROS_ASSERT(pwm_motor_info.hasMember("dev_filename"));
    ROS_ASSERT(pwm_motor_info.hasMember("gpio_filename"));
    ROS_ASSERT(pwm_motor_info.hasMember("angles"));
    ROS_ASSERT(pwm_motor_info.hasMember("duty_rates"));
    ROS_ASSERT(pwm_motor_info.hasMember("start_angle"));

    // Open device files
    name_ = (string)pwm_motor_info["name"];
    string dev_file = pwm_motor_info["dev_filename"];
    legal_checker_ =
        SwitchGPIOPtr(new SwitchGPIO(pwm_motor_info["gpio_filename"]));
    dev_fd_ = open(dev_file.c_str(), O_RDWR, 0);
    if (dev_fd_ < 0) {
        ROS_ERROR("Failed to open file %s for motor %s", dev_file.c_str(),
                  name_.c_str());
        return;
    }
    ROS_ASSERT(sizeof(unsigned) == 4);
    motor_regs_ =
        (unsigned *)mmap(NULL, sizeof(unsigned) * 2, PROT_READ | PROT_WRITE,
                         MAP_SHARED, dev_fd_, 0);
    if ((long long)motor_regs_ == -1) {
        close(dev_fd_);
        ROS_ERROR("Failed to create memory map for motor %s", name_.c_str());
        motor_regs_ = NULL;
        return;
    }

    // Check initial position
    ROS_INFO("Motor %s suceessfully mapped file %s", name_.c_str(),
             dev_file.c_str());
    can_move_ = !legal_checker_->Get();

    // Set Interpolation
    XmlRpc::XmlRpcValue angles = pwm_motor_info["angles"];
    XmlRpc::XmlRpcValue duty_rates = pwm_motor_info["duty_rates"];
    ROS_ASSERT(angles.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(duty_rates.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_INFO("angles size %d", angles.size());
    ROS_INFO("duty_rates size %d", duty_rates.size());
    ROS_ASSERT(angles.size() == duty_rates.size());
    vector<double> angles_vec, duty_rates_vec;
    for (int i = 0; i < angles.size(); i++) {
        ROS_ASSERT(angles[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(duty_rates[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        angles_vec.push_back(angles[i]);
        duty_rates_vec.push_back(duty_rates[i]);
    }
    interpolation_ = LinearInterpolation(angles_vec, duty_rates_vec);

    // Setup
    if (!can_move_) ROS_WARN("Motor %s not at initial position", name_.c_str());
    motor_regs_[kCYCLE] = kFREQ / kRATE;
    double start_angle = pwm_motor_info["start_angle"];
    Set_(start_angle);
    now_angle_ = start_angle;
    commanded_ = false;
}

bool PWMMotorController::CheckStartLegal() {
    if (motor_regs_ != NULL) {
        can_move_ = can_move_ || !legal_checker_->Get();
        return can_move_;
    }
    return false;
}

double PWMMotorController::Get() {
    return now_angle_;
}

void PWMMotorController::Set(double val) {
    Set_(val);
}

void PWMMotorController::Set_(double val) {
    if (can_move_ || CheckStartLegal()) {
        if (val == now_angle_) return;
        if (!commanded_ && val == 0) return;
        commanded_ = true;
        double duty_rate = interpolation_.Get(val);
        unsigned duty_width = (kFREQ / kRATE) * duty_rate;
        ROS_DEBUG("Write duty_rate %f", duty_rate);
        motor_regs_[kWIDTH] = duty_width;
        now_angle_ = val;
    }
}

PWMMotorController::~PWMMotorController() {
    munmap(motor_regs_, 8);
    close(dev_fd_);
    // When the motor stops, it should also be at its initial value
    // However it seems to be unwise to abort in a destructor
    if (!legal_checker_->Get()) {
        ROS_WARN("Motor %s failed to go back to initial position",
                 name_.c_str());
    }
}


}
}

