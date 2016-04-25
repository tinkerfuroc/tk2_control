#!/usr/bin/env python
import roslib
roslib.load_manifest('tk_dynamixel_control')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class Joint:
	def __init__(self, motor_name):
		#arm_name should be b_arm or f_arm
		self.name = motor_name           
		self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		rospy.loginfo('Waiting for joint trajectory action')
		self.jta.wait_for_server()
		rospy.loginfo('Found joint trajectory action!')

		
	def move_joint(self, angles):
		goal = FollowJointTrajectoryGoal()                  
		char = self.name[0] #either 'f' or 'b'
		goal.trajectory.joint_names = ['wrist_extension'+char, 'wrist_deviation'+char,'claw'+char]
		point = JointTrajectoryPoint()
		point.positions = angles
		point.time_from_start = rospy.Duration(3)                   
		goal.trajectory.points.append(point)
		self.jta.send_goal_and_wait(goal)
		  

def main():
	arm = Joint('f_arm')
	arm.move_joint([3.14,3.14,1.0])
				
if __name__ == '__main__':
	rospy.init_node('joint_position_tester')
	main()
