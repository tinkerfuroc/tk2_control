#include <ros/ros.h>
#include "tk_hardware_interface/arm_hardware.h"
#include <controller_manager/controller_manager.h>

using namespace tinker::control;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tinker_arm_hardware");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle n;
    XmlRpc::XmlRpcValue arm_info;
    private_nh.getParam("arm_info", arm_info);
    ArmHardware arm(arm_info);
    controller_manager::ControllerManager cm(&arm);
    ros::Rate r(10);
    ros::Time last_time, now_time;
    last_time = ros::Time::now();
    while(true) {
        arm.Read();
        now_time = ros::Time::now();
        cm.update(ros::Time::now(), now_time-last_time);
        last_time = now_time;
        r.sleep();
    }
    return 0;
}
