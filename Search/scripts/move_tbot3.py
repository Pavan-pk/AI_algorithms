#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Naman Shah, Rushang Karia"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import tf
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion
from pid import PID
import copy
from problem import Helper,State

class moveTbot3:
	def __init__(self):
		rospy.init_node('move_turtle',anonymous=True, disable_signals=False)
		self.actions = String()
		self.pose = Pose()
		self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
		self.action_subscriber = rospy.Subscriber('/actions',String,self.callback_actions)
		self.pid_subscriber = rospy.Subscriber("/Controller_Status",String,self.callback_pid)
		self.pose_subscriber = rospy.Subscriber('/odom',Odometry,self.pose_callback)
		self.target_publisher = rospy.Publisher("/target_pose",Pose,queue_size = 1)
		self.status_publisher = rospy.Publisher("/status",String,queue_size = 10)
		self.orientation_raw = rospy.Publisher('/orientation',Float64,queue_size = 5)
		self.helper = Helper() 
		self.init_state = self.helper.get_initial_state()
		self.current_state = copy.deepcopy(self.init_state)
		self.free = String(data = "next")
		self.rate = rospy.Rate(30)
		self.last_state = None
		print("Ready!")
		rospy.spin()

	def round_to_state(self,x):
		return round(x*2)/2.0

	def callback_pid(self,data):
		if data.data == "Done":
			if len(self.actions)>0:
				self.execute_next()
			else:					    	
				print("Shutting down")	
				rospy.signal_shutdown("Done")
	
	def callback_actions(self,data):
		self.actions = data.data.split("_")
		self.rate.sleep()
		self.execute_next()

	def execute_next(self):
		action = self.actions.pop(0)
		direction = None

		if action in self.helper.get_actions():

			next_states = self.helper.get_successors(self.current_state)
			for next_action in next_states.keys():
				if next_action == action:
					next_state = next_states[next_action][0]
					break
			print "current state: {}".format(self.current_state)
			print "next state: {}".format(next_state)
			goal_x = next_state.x
			goal_y = next_state.y
			if next_state.orientation == "EAST":
				goal_angel = 0
			elif next_state.orientation == "NORTH":
				goal_angel = math.pi / 2.0
			elif next_state.orientation == "WEST":
				goal_angel = math.pi
			else:
				goal_angel = -math.pi/2.0
			
			current_pose = self.pose
			quat = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
			current_euler = tf.transformations.euler_from_quaternion(quat)
			target_quat = Quaternion(*tf.transformations.quaternion_from_euler(current_euler[0],current_euler[1],goal_angel))
			target_pose = copy.deepcopy(current_pose)
			target_euler = [current_euler[0],current_euler[1],goal_angel]
			target_pose = [goal_x,goal_y,goal_angel]
			if action in ["MoveF", "MoveB"]:
				PID(target_pose,"linear").publish_velocity()
			else:
				PID(target_pose,"rotational").publish_velocity()
			self.current_state = next_state
		else:
			print "Invalid action"
			exit(-1)
			
		if len(self.actions) == 0:
			self.status_publisher.publish(self.free)


	def pose_callback(self,data):
		self.pose = data.pose.pose
		state_x = self.round_to_state(self.pose.position.x)
		state_y = self.round_to_state(self.pose.position.y)
		quat = (self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z, self.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quat)
		z_angel = euler[2]
		if z_angel >= -math.pi/4.0 and z_angel < math.pi/4.0:
			state_rot = "EAST"
		elif z_angel >= math.pi/4.0 and z_angel < 3.0 * math.pi /4.0:
			state_rot = "NORTH"
		elif z_angel >= -3.0 * math.pi/ 4.0 and z_angel < -math.pi/4.0:
			state_rot = "SOUTH"
		else:
			state_rot = "WEST"
		orientation = Float64()
		orientation.data = z_angel
		self.orientation_raw.publish(orientation)
		self.current_state = State(state_x,state_y,state_rot)


if __name__ == "__main__":
	try:
		moveTbot3()
	except rospy.ROSInterruptException:
		pass
