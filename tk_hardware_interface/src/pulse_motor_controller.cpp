#include "tk_hardware_interface/pulse_motor_controller.h"
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <math.h>

namespace tinker {
namespace control {

using std::string;

PulseMotorController::PulseMotorController(
    XmlRpc::XmlRpcValue &pulse_motor_info)
    : motor_regs_(NULL), start_count_(0), last_target_(0) {
    ROS_ASSERT(pulse_motor_info.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(pulse_motor_info.hasMember("name"));
    ROS_ASSERT(pulse_motor_info.hasMember("dev_filename"));
    ROS_ASSERT(pulse_motor_info.hasMember("gpio_filename"));
    ROS_ASSERT(pulse_motor_info.hasMember("to_meter_fraction"));
    ROS_ASSERT(pulse_motor_info.hasMember("speed"));

    // Open device files
    name_ = (string)pulse_motor_info["name"];
    string dev_file = pulse_motor_info["dev_filename"];
    legal_checker_ =
        SwitchGPIOPtr(new SwitchGPIO(pulse_motor_info["gpio_filename"]));
    dev_fd_ = open(dev_file.c_str(), O_RDWR, 0);
    if (dev_fd_ < 0) {
        ROS_ERROR("Failed to open file %s for motor %s", dev_file.c_str(),
                  name_.c_str());
        return;
    }
    ROS_ASSERT(sizeof(unsigned) == 4);
    motor_regs_ =
        (unsigned *)mmap(NULL, sizeof(unsigned) * 4, PROT_READ | PROT_WRITE,
                         MAP_SHARED, dev_fd_, 0);
    if ((long long)motor_regs_ == -1) {
        close(dev_fd_);
        ROS_ERROR("Failed to create memory map for motor %s", name_.c_str());
        motor_regs_ = NULL;
        return;
    }

    // Check initial position
    printf("fuck 1\n");
    ROS_INFO("Motor %s suceessfully mapped file %s", name_.c_str(),
             dev_file.c_str());
    can_move_ = !legal_checker_->Get();
    printf("fuck 2\n");

    // Setup
    to_meter_fraction_ = pulse_motor_info["to_meter_fraction"];
    printf("fuck 3\n");
    ROS_INFO("get to_meter_fraction");
    if (!can_move_) ROS_WARN("Motor %s not at initial position", name_.c_str());
    speed_ = (unsigned)((int)pulse_motor_info["speed"]);
    printf("fuck 4\n");
    motor_regs_[kCTRL] = CTRL_DIR & ~CTRL_ENABLE;
    motor_regs_[kVEL] = speed_;
    start_count_ = motor_regs_[kFEEDBACK];
    printf("fuck 5\n");
    ROS_INFO("Pulse motor %s starts with count %u", name_.c_str(), start_count_);
}

bool PulseMotorController::CheckStartLegal() {
    if (motor_regs_ != NULL) {
        can_move_ = can_move_ || !legal_checker_->Get();
        return can_move_;
    }
    return false;
}

double PulseMotorController::Get() {
    return - ((int)(motor_regs_[kFEEDBACK] - start_count_)) / to_meter_fraction_;
}

void PulseMotorController::Set(double val) {
    if (val == last_target_) return;
    if (can_move_ || CheckStartLegal()) {
        double now_height = Get();
        double move_height = val - now_height;
        if (move_height > 0)
            motor_regs_[kCTRL] &= ~CTRL_DIR;
        else {
            motor_regs_[kCTRL] |= CTRL_DIR;
            move_height = -move_height;
        }
        ROS_DEBUG("move height %f", move_height);
        unsigned move_pulses =
            (unsigned)(move_height * to_meter_fraction_);
        //For whatever reason, the hardware IP requires me to do so
        move_pulses *= 2;
        ROS_DEBUG("Pulse motor %s move pulses %u", name_.c_str(), move_pulses);
        motor_regs_[kCOUNT] = move_pulses;
        last_target_ = val;
    }
}

PulseMotorController::~PulseMotorController() {
    munmap(motor_regs_, 16);
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
