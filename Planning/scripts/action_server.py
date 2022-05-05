#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Chirav Dave", "Ketan Patil", "Pulkit Verma"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel
from hw2.srv import PlaceActionMsg
from hw2.srv import PickActionMsg
from rospy.timer import sleep
from std_msgs.msg import String
from hw2.srv import RemoveBlockedEdgeMsg
from hw2.srv import MoveActionMsg

class RobotActionsServer:
	def __init__(self, object_dict):
		self.failure = -1
		self.success = 1
		self.object_dict = object_dict
		self.empty = True
		self.remove_object = ''
		self.status = String(data='idle')
		self.model_state_publisher = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size = 10)
		self.action_publisher = rospy.Publisher("/actions",String,queue_size= 10)
		self.status_publisher = rospy.Publisher("/status",String,queue_size=10)
		rospy.Service("execute_place_action",PlaceActionMsg,self.execute_place_action)
		rospy.Service("execute_pick_action",PickActionMsg,self.execute_pick_action)
		rospy.Service("execute_move_action",MoveActionMsg,self.execute_move_action)
		print("Action Server Initiated")

	def change_state(self,object_name,target_transform):
		model_state_msg = ModelState()
		model_state_msg.model_name = object_name
		model_state_msg.pose.position.x = target_transform[0]
		model_state_msg.pose.position.y = target_transform[1]
		model_state_msg.pose.position.z = target_transform[2]
		# rospy.logdebug("Updated model state: {}".format(model_state_msg))
		self.model_state_publisher.publish(model_state_msg)

	def remove_edge(self,object_name):
		rospy.wait_for_service('remove_blocked_edge')
		try:
			remove_edge = rospy.ServiceProxy('remove_blocked_edge',RemoveBlockedEdgeMsg)
			_ = remove_edge(object_name)
		except rospy.ServiceException as e:
			print("Sevice call failed: {}".format(e))

	def execute_place_action(self, req):
		object_name = req.object_name
		goal_name = req.goal_name
		robot_state = (req.x , req.y , req.orientation)
		if object_name in self.object_dict["object"] and goal_name in self.object_dict["goal"]:
			if (robot_state[0],robot_state[1]) in self.object_dict["goal"][goal_name]["load_loc"]:
				if self.object_dict["object"][object_name]["size"] == self.object_dict["goal"][goal_name]["size"] and \
					 self.object_dict["object"][object_name]["obj_type"] == self.object_dict["goal"][goal_name]["obj_type"]:
					goal_loc = list(self.object_dict["goal"][goal_name]["loc"])
					if "bin" in goal_name:
						goal_loc[0] = goal_loc[0] + 0.25
						goal_loc[1] = goal_loc[1] + 0.25
					else:
						goal_loc[0] = goal_loc[0]
						goal_loc[1] = goal_loc[1]
					self.change_state(object_name, goal_loc + [0.5])
					self.empty = True
					self.status_publisher.publish(self.status)
					self.remove_object = object_name
					return self.success
		self.status_publisher.publish(self.status)
		return self.failure

	def execute_pick_action(self, req):
		# rospy.logdebug("Pick action")
		object_name = req.object_name
		# rospy.logdebug("reqs: {} {} {} {}".format(req.object_name, req.x, req.y, req.orientation))
		robot_state = [req.x , req.y ,req.orientation]
		if object_name in self.object_dict["object"]:
			if (robot_state[0],robot_state[1]) in self.object_dict["object"][object_name]["load_loc"]:
				if self.empty:
					self.change_state(object_name,robot_state[:2]+[0.5])
					self.empty = False
					_ = self.remove_edge(object_name)
					self.deleteModel()
					self.status_publisher.publish(self.status)
					# rospy.logdebug("Published success to status publisher")
					return self.success
		self.status_publisher.publish(self.status)
		return self.failure

	def execute_move_action(self, req):
		action_seq = req.actions
		self.action_publisher.publish(String(data=action_seq))
		return self.success
	
	def deleteModel(self):
		if self.remove_object is not '':
			rospy.wait_for_service('gazebo/delete_model')
			deleter = rospy.ServiceProxy('gazebo/delete_model',DeleteModel)\
				(self.remove_object)
			self.remove_object = ''


if __name__ == "__main__":
	object_dict = None
	RobotActionsServer(object_dict)