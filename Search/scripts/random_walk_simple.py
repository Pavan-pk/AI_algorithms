#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Abhyudaya Srinet"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import heapq
import os
import time
from problem import * 
import rospy
from std_msgs.msg import String
import numpy as np
import random
from server import generate_maze, initialize_search_server
from utils import cleanup_ros, initialize_ros
import subprocess


obstacles = 4
env = 'canWorld'
seed = int(0xDEADC0DE)

class RandomWalk:

	def __init__(self):
		self.helper = Helper()
		rospy.init_node("random_walk")
		# self.publisher = rospy.Publisher("/actions", String, queue_size = 10)
		# self.subscriber = rospy.Subscriber("/status", String, self.callback)
		self.init_state = self.helper.get_initial_state()
		print("Initial state: "+str(self.init_state))
		self.current_state = self.init_state
		self.last_action = None
		self.count = 0
		rospy.Rate(1).sleep()
		print "Running"
		self.random_walk_actions()


	def random_walk(self):
		'''
		Randomly choses an action to perform among possible actions
		'''
		all_actions = ['MoveF','TurnCCW','TurnCW']
		response = -1
		action = 'MoveB'

		while True:
			while response==-1 or action == 'MoveB':
				action = random.choice(all_actions)
				print(self.current_state.x)
				possible_next_states = self.helper.get_successor(self.current_state)
				response = possible_next_states[possible_next_states.keys()[0]][0].x

			next_state = possible_next_states[possible_next_states.keys()[0]][0]

			return next_state, action

	def next_action(self):
		'''
		Updates current state from chosen action and publishes the action to the /actions topic
		'''
		self.count += 1

		next_state, action = self.random_walk()
		print "[%3d] Executing %8s from %20s to reach %20s" % (self.count,  action, self.current_state, next_state)
		self.current_state = next_state
		action_str = String(data = action)
		self.publisher.publish(action_str)

	def callback(self,data):
		'''
		callback for handling status messages of turtlebot
		executes the next action when the turtlebot is ready
		'''
		if data.data == "next":
			self.next_action()
	
	def random_walk_actions(self):
		'''
		Randomly choses an action to perform among possible actions
		'''
		all_actions = ['MoveF','TurnCCW','TurnCW']
		action_str = []
		while self.count < 1e5:
			response = -1
			while response==-1:
				possible_next_states = self.helper.get_successor(self.current_state)
				idx = random.randint(0, len(possible_next_states.items())-1)
				action, state_cost = possible_next_states.items()[idx]
				next_state, cost = state_cost
				if next_state.x == -1 or next_state.y == -1:
					response = -1
				else:
					response = 0

			self.current_state = next_state
			action_str.append(action)
			self.count += 1 

		action_str_joined = "_".join(action_str)

		if os.path.isfile('./tests/moves.txt'):
			mode = 'a'
		else:
			mode = 'w'

		with open('result.txt',mode) as file_handle:
			if mode == 'w':
				file_handle.write("Dimension, Obstacles, Seed, Algorithm, Time, "
					"Nodes Expanded, Plan Length, Error, Plan, Env\n")
			file_handle.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (
					'3x6', obstacles, seed, 'random',
					'null', 'null', len(action_str), 'null', 
					action_str_joined,env))
		


if __name__ == "__main__":

	random.seed(1234)
	for env in ['canWorld', 'cafeWorld']:
		roscore_process = initialize_ros()
		server_process = initialize_search_server()
		generate_maze(3,6, obstacles, seed, env)
		RandomWalk()
		cleanup_ros([server_process, roscore_process])
