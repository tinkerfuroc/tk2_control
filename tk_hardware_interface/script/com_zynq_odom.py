#!/usr/bin/python

__author__ = 'gjc'

import tf
import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry 
from threading import Lock, Thread
from numpy import array
from numpy.linalg import lstsq, norm
from constants_odom import *
from math import cos, sin, fabs
from chassis import Chassis
import numpy as np

import actionlib
from tk_hardware_interface.msg._SimpleMoveFeedback import SimpleMoveFeedback
from tk_hardware_interface.msg._SimpleMoveResult import SimpleMoveResult
from tk_hardware_interface.msg._SimpleMoveAction import SimpleMoveAction


class ChassisControlNode:
    def __init__(self):
        self.lock = Lock()
        self.is_stopped = True
        self.pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.chassis = Chassis()
        self.target_wheel_speed = [0, 0, 0, 0]
        self.now_wheel_speed = [0, 0, 0, 0]
        self.last_moves = self.chassis.get_feedback()
        self.now_x = 0
        self.now_y = 0
        self.now_theta = 0
        self.in_action_service = False
        self._simple_move_feedback = SimpleMoveFeedback()
        self._simple_move_result = SimpleMoveResult()
        self._as = actionlib.SimpleActionServer("simple_move", SimpleMoveAction, execute_cb = self.execut_cb, auto_start = False)
        self._as.start()

    def execut_cb(self, goal):
        rate = rospy.Rate(RATE)
        self._simple_move_feedback.moved_distance.x = 0
        self._simple_move_feedback.moved_distance.y = 0
        self._simple_move_feedback.moved_distance.theta = 0
        with self.lock:
            self.in_action_service = True
            original_moves = array(self.chassis.get_feedback(), dtype=np.float64)
            self.chassis.stop()
            target = goal.target
            distance = array([target.x, target.y, target.theta])
            wheel_moves = MACANUM_MAT.dot(distance.T)
            total_distance = norm(wheel_moves)
            if total_distance < MAX_DELTA_VELOCITY:
                self._simple_move_result.success = True
                self._as.set_succeeded(self._simple_move_result)
                return
            moved_distance = 0
            vels = wheel_moves / total_distance
            self.chassis.set_wheels_speed(vels * MAX_DELTA_VELOCITY * 4)
            real_moves = array([abs(v) for v in wheel_moves])
            self.chassis.set_wheels_count(real_moves)
        success = True
        while True:
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                with self.lock:
                    self.chassis.stop()
                success = False
                break
            with self.lock:
                now_moves = array(self.chassis.get_feedback(), dtype=np.float64)
            moved_distance = norm(now_moves - original_moves)
            now_speed = self.get_speed(moved_distance, total_distance)
            wheel_vels = now_speed * vels
            x, y, theta = tuple(lstsq(MACANUM_MAT, now_moves - original_moves)[0])
            self._simple_move_feedback.moved_distance.x += x
            self._simple_move_feedback.moved_distance.y += y
            self._simple_move_feedback.moved_distance.theta += theta
            self._as.publish_feedback(self._simple_move_feedback)
            with self.lock:
                self.chassis.set_wheels_speed(wheel_vels * 4)
            if (fabs(moved_distance - total_distance) < 10):
                break
            rate.sleep()
        if success:
            self._simple_move_result.success = True
            self._as.set_succeeded(self._simple_move_result)
        with self.lock:
            self.in_action_service = False


    def get_speed(self, moved_distance, total_distance):
        if moved_distance > total_distance:
            rospy.logwarn("Moved too much")
        remain_distance = total_distance - moved_distance
        if remain_distance < moved_distance:
            moved_distance = remain_distance
        factor = (moved_distance / total_distance) / 0.2
        if factor > 1:
            factor = 1
        speed = factor * DEFAULT_VELOCITY
        if speed < MAX_DELTA_VELOCITY:
            speed = MAX_DELTA_VELOCITY
        return speed


    def vel_callback(self, vel):
        if abs(vel.linear.x) > MAX_VELOCITY or abs(vel.linear.y) > MAX_VELOCITY or abs(vel.angular.z) > MAX_A_VELOCITY:
            rospy.logwarn('vel too big!')
            return
        vel = array([vel.linear.x, vel.linear.y, vel.angular.z])
        self.is_stopped = False
        with self.lock:
            self.target_wheel_speed = MACANUM_MAT.dot(vel.T) * PV_RATE


    def run(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            with self.lock:
                if not self.in_action_service:
                    self.set_speed()
                self.feedback()
            rate.sleep()
        
    def set_speed(self):
        delta_wheel_speed = array(self.target_wheel_speed) - array(self.now_wheel_speed)
        if self.is_stopped:
            self.chassis.stop()    
        else:
            if norm(delta_wheel_speed) > MAX_DELTA_VELOCITY:
                delta_wheel_speed /= norm(delta_wheel_speed)
                delta_wheel_speed *= MAX_DELTA_VELOCITY
            self.now_wheel_speed += delta_wheel_speed
            if norm(self.now_wheel_speed) < 1:
                self.now_wheel_speed = [0] * 4
                self.is_stopped = True
                self.chassis.stop()
            else:
                self.chassis.set_wheels_speed(self.now_wheel_speed)
                self.chassis.set_wheels_count([10000] * 4)

    def feedback(self):
        new_moves = self.chassis.get_feedback()
        delta_moves = array(new_moves) - array(self.last_moves)
        self.last_moves = new_moves
        x, y, theta = tuple(lstsq(MACANUM_MAT, delta_moves)[0])
        x_odom = cos(self.now_theta) * x - sin(self.now_theta) * y
        y_odom = sin(self.now_theta) * x + cos(self.now_theta) * y
        self.now_x += x_odom
        self.now_y += y_odom
        self.now_theta += theta
        self.odom_post(self.now_x, self.now_y, self.now_theta)


    def odom_post(self, now_x, now_y, now_theta):
        odom_broadcaster = tf.TransformBroadcaster()   
        current_time = rospy.Time.now()
        odom_quat =  tf.transformations.quaternion_from_euler(0,0,now_theta)      
        odom_broadcaster.sendTransform((now_x, now_y, 0), 
                odom_quat, 
                current_time, 
                "base_link", 
                "odom")
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = now_x
        odom.pose.pose.position.y = now_y
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]
        self.pub.publish(odom)


def init():
    rospy.init_node('com_zynq', anonymous=False)
    chassis_control_node = ChassisControlNode()
    rospy.Subscriber('cmd_vel', Twist, chassis_control_node.vel_callback)
    rospy.loginfo('go!')
    t = Thread(target=chassis_control_node.run)
    t.start()
    rospy.spin()
    t.join()
    chassis_control_node.chassis.close()


if __name__ == '__main__':
    init()

