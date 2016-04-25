#!/usr/bin/python

from armjoint import *
from constants_arm import *
from motor import Motor

class RoboArm(object):
    def __init__(self):
        self.joints = [
            PWMJoint(
                5, BASE_MIN_ANGLE, BASE_MIN_PWM, BASE_MAX_ANGLE, 
                BASE_MAX_PWM, BASE_COF1, BASE_COF0),
            PWMJoint(
                6, SHOULDER_MIN_ANGLE, SHOULDER_MIN_PWM, SHOULDER_MAX_ANGLE, 
                SHOULDER_MAX_PWM, SHOULDER_COF1, SHOULDER_COF0),
            PWMJoint(
                7, ELBOW_MIN_ANGLE, ELBOW_MIN_PWM, ELBOW_MAX_ANGLE, 
                ELBOW_MAX_PWM, ELBOW_COF1, ELBOW_COF0),
            DyMiJoint(
                1, WRIST_MIN_ANGLE, WRIST_MIN_PWM, WRIST_MAX_ANGLE, 
                WRIST_MAX_PWM, WRIST_COF1, WRIST_COF0)]
        self.motor = Motor(4)

    def move_arm(self, out_pos):
        self.motor.set_vel(out_pos.pos7)
        arm_pos = [0.] * 6
        for i in range(0, 4):
            legal, arm_pos[i] = joints[i].get_pos(arm_pose[i])
            if not legal:
                rospy.logerr('angle over bound for joint %d' % i)
            joints[i].set_pos(arm_pos[i])
    
    def close(self):
        for joint in self.joints:
            joint.close()