#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Chirav Dave"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import numpy as np
from abc import *
import os, sys
import copy

import rospy

class MazeObject:
	__metaclass__ = ABCMeta
	'''
	Abstract class for maze objects
	__init__ requires coordinates of the object to be inserted
	add_object should generate the XML for the object being created
	add_object_description adds
	'''
	@abstractmethod
	def __init__(self, grid_dims, out_xml, dae, coords=(0,0,0), name="my_mesh", scale=(1,1,1),id=0, static=False):
		self.x, self.y,self.z = coords
		self.f_out = out_xml
		self.name = name
		self.dae = dae
		self.scale = scale
		self.id=id
		self.static = int(static)
		if os.path.isfile(self.dae): 
			self.dae = os.path.abspath(self.dae)

	@abstractmethod
	def add_object(self):
		self.f_out.write('<model name="{}_{}">\n'.format(self.name, self.id)+\
						 '<pose frame=''>{} {} {} 0 0 0</pose>\n'.format(self.x, self.y, self.z)+\
						 '<scale>{} {} {}</scale>\n'.format(self.scale[0], self.scale[1], self.scale[2])+\
						 '<link name="link">\n'+\
						 '<pose frame=''>{} {} {} 0 0 0</pose>\n'.format(self.x, self.y, self.z)+\
						 '<velocity>0 0 0 0 -0 0</velocity>\n'+\
						 '<acceleration>0 0 0 0 -0 0</acceleration>\n'+\
						 '<wrench>0 0 0 0 -0 0</wrench>\n'+\
						 '</link>\n</model>')

	@abstractmethod
	def add_object_description(self):
		self.f_out.write('<model name="{}_{}">\n<pose>0 0 0 0 0 0</pose>\n'.format(self.name, self.id)+\
						 '<static>{}</static>\n<link name="body">\n'.format(self.static)+\
						 '<inertial><mass>0.005</mass><inertia></inertia><pose frame=> 0 0 0 0 -0 0</pose>\n'+\
						 '</inertial><self_collide>0</self_collide><kinematic>0</kinematic><gravity>1</gravity>'+\
						 '<collision name="body_collide">\n<geometry>\n<mesh>'+\
						 '<uri>file://{}</uri><scale> {} {} {} </scale></mesh>\n'.format(self.dae,self.scale[0],self.scale[1],self.scale[2])+\
						 '</geometry>\n</collision>\n'+\
						 '<visual name="visual"><geometry>\n<mesh>'+\
						 '<uri>file://{}</uri></mesh><scale> {} {} {} </scale>\n'.format(self.dae,self.scale[0],self.scale[1],self.scale[2])+\
						 '</geometry>\n</visual>\n</link>\n</model>\n')

class MazeGenerator(object):
	'''
	Object dict format:
	{
		'goal':MazeObject
		'obstacles':[]->list(MazeObject)
		'bounding':[]->list(MazeObject)
	}
	'bounding' objects will demarcate the areas of the maze; walls for example
	'bounding' needs to pass the pose coordinates as well. This will not be 
	supplied by the MazeGenerator
	'obstacles' will be randomly generated within the available areas of the maze
	'goal' will be generated at the end after all the obstacles have been created

	The position of the goal location will be determined from the available edges 
	after the obstacles have been placed

	'''
	def __init__(self,
				 grid_dims=None,
				 objects=[],
				 cell_size=0.5,
				 maze_seed=None,
				 blocked_edges=[],
				 env="canWorld"):

		self.root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),
													 os.path.pardir))
		self.maze_seed = maze_seed
		self.scale = cell_size
		self.grid_size = grid_dims
		self.goalstate = []
		self.objects = self.validate_objects(objects)
		self.blocked_edges = blocked_edges # List of tuples of occupied edges
		self.env = env

		if grid_dims is None:
			raise ValueError("Grid dimensions undefined")

		self.init_setup = self.generate()

	def validate_objects(self, objects_dict):

		objects_dict.setdefault('obstacles',[])
		objects_dict.setdefault('bounding',[])
		objects_dict.setdefault('goal',[])

		return objects_dict

	def init_empty_world(self):
		#File containing empty world description
		f_in = open(os.path.join(self.root_dir,'worlds/empty_world.sdf'), 'r')
		#File to save your maze
		f_out = open(os.path.join(self.root_dir,'worlds/maze.sdf'), 'w')
		#Copying empty world description into output maze
		for line in f_in:
			f_out.write(line)
		f_in.close()
		return f_out

	def get_goal_blocked_edges(self, coords):
		goal_edges = []
		goal_edges.append((coords[0]-self.scale,coords[1],coords[0],coords[1]))
		goal_edges.append((coords[0],coords[1]+self.scale,coords[0],coords[1]))
		goal_edges.append((coords[0],coords[1],coords[0]+self.scale,coords[1]))
		goal_edges.append((coords[0],coords[1],coords[0],coords[1]-self.scale))

		return goal_edges

	def generate(self):
		'''
		One time generation call to create the maze with 
		all the objects passed to the class
		'''
		self.f_out = self.init_empty_world()
		np.random.seed(self.maze_seed)
		self.xml_objects = {'bounding':[], 'goal':[], 'obstacles':[]}
		
		# Setting bounding type objects in the maze
		for bound_obj, config in self.objects['bounding'].items():
			bounding_dae = config.get('file')
			bounding_scale = self.__get_body_scale(config)
			for i in range(config['count']):
				# Only 1 object is expected to be set as the bounding type
				# If multiple must be set, use the mazeObjects subclass to define 
				# multiple of the same variety in a loop inside that class
				bound_instance = bound_obj(coords=(-0.1, 0.1, -0.1423),
								 grid_dims=self.grid_size,
								 scale=bounding_scale,
								 out_xml=self.f_out,
								 dae=bounding_dae)
				bound_instance.add_object()
				self.xml_objects['bounding'].append(bound_instance)

		# Setting obstacle type objects in the maze
		obstacle_ctr = 0
		for obstacle_obj, config in self.objects['obstacles'].items():
			obstacle_dae = config.get('file')
			obj_scale = self.__get_body_scale(config)

			for i in range(config['count']):
				coords, edge = self.__sample_obstacle_location()
				while len(coords) == 0:
					coords, edge = self.__sample_obstacle_location()

				obstacle_instance = obstacle_obj(coords=coords,
												 grid_dims=self.grid_size,
												 scale=obj_scale,
												 out_xml=self.f_out,
												 dae=obstacle_dae,
												 id=obstacle_ctr+1,)

				obstacle_instance.add_object()
				self.xml_objects['obstacles'].append(obstacle_instance)

				self.blocked_edges.extend(edge)
				obstacle_ctr+=1

		# Generate a random goal location and set the goal objects in the maze
		goal_ctr = 0
		for goal_obj, config in self.objects['goal'].items():
			goal_dae = config.get('file')
			obj_scale = self.__get_body_scale(config)
			for i in range(config['count']):
				goal_coords, goal_blocks = self.__generate_goal_coords()
				goal_instance = goal_obj(coords=tuple(goal_coords),
										 grid_dims=self.grid_size,
										 scale=obj_scale,
										 out_xml=self.f_out,
										 dae=goal_dae,
										 id=goal_ctr+1)

				goal_instance.add_object()
				self.xml_objects['goal'].append(goal_instance)
				self.goalstate.append(tuple(goal_coords[:-1]))
			goal_ctr+=1

		rospy.logdebug("goal coords: {}".format(goal_coords[:-1]))

		self.f_out.write('</state>')
		
		for goal in self.xml_objects['goal']:
			goal.add_object_description()
		for bound in self.xml_objects['bounding']:
			bound.add_object_description()
		for obstacle in self.xml_objects['obstacles']:
			obstacle.add_object_description()

		self.f_out.write('</world>\n</sdf>')
		self.f_out.close()
		print ("World generated")
		return self.blocked_edges

	def add_blocked_edges(self, coords):
		blocked_edges = []
		x = coords[0]
		y = coords[1]
		for i in range(2):
			for j in range(2):
				blocked_edges.append((x + i * self.scale, y + j * self.scale, x + i * self.scale + self.scale, y + j * self.scale))
				blocked_edges.append((x + i * self.scale, y + j * self.scale, x + i * self.scale, y + j * self.scale + self.scale))
				blocked_edges.append((x + i * self.scale, y + j * self.scale, x + i * self.scale, y + j * self.scale - self.scale))
				blocked_edges.append((x + i * self.scale - self.scale, y + j * self.scale, x + i * self.scale, y + j * self.scale))

		return blocked_edges

	def round(self,num):
		return round(num*2)/2.0

	def __get_body_scale(self, body_config, default=(1,1,1)):
		
		if body_config.get('scale') is not None:
			scale = tuple(body_config['scale'])
		else:
			scale = default
		
		return scale

	def __sample_obstacle_location(self):

		offset = np.random.uniform(0, 0.07*self.scale)
		flag = np.random.randint(0, 2)
		coords = tuple()
		edge = tuple()

		if self.env == "canWorld":
			x = self.round(np.random.randint(0, self.grid_size[0]+1)*self.scale)
			y = self.round(np.random.randint(0, self.grid_size[1]+1)*self.scale)	

			if flag==0 \
			and (x+self.scale <= self.grid_size[0]*self.scale) \
			and ((x, y, x+self.scale, y) not in self.blocked_edges):
				coords = (x+self.scale/2+offset, y, 0)
				edge = [(x, y, x+self.scale, y)]
			
			elif flag==1 \
			and (y+self.scale <= self.grid_size[1]*self.scale) \
			and ((x, y, x, y+self.scale) not in self.blocked_edges):
				coords = (x, y+self.scale/2-offset, 0)
				edge = [(x, y, x, y+self.scale)]

		elif self.env == "cafeWorld":
			grid_x = 3
			grid_y = 6
			x = self.round(np.random.randint(1, grid_x+1))
			y = self.round(np.random.randint(1, grid_y+1))
			offset = 0.0

			if flag==0 \
			and (x+self.scale <= self.grid_size[0]*self.scale) \
			and ((x, y, x+self.scale, y) not in self.blocked_edges):
				coords = (x, y, 0)
				edge = self.add_blocked_edges(coords)

			elif flag==1 \
			and (y+self.scale <= self.grid_size[1]*self.scale) \
			and ((x, y, x, y+self.scale) not in self.blocked_edges):
				coords = (x, y, 0)
				edge = self.add_blocked_edges(coords)

		return coords, edge

	def __generate_goal_coords(self):

		if self.env == "canWorld":
			x = self.round(np.random.randint(0, self.grid_size[0]+1)*self.scale)
			y = self.round(np.random.randint(0, self.grid_size[1]+1)*self.scale)
			offset = 0.0
			z = 0
		elif self.env == "cafeWorld":
			x = self.round(np.random.uniform(0.2, 1))
			y = self.round(np.random.uniform(7.5, 7.7))
			z = 2.3 * 0.2
		
		goal_coords = [x,y,z]

		goal_blocks = self.get_goal_blocked_edges(goal_coords)

		return goal_coords, goal_blocks

	def initState(self):
		return (0, 0, 'EAST')

	def getGoalState(self):
		'''
		Return the coords of the goal state in the maze
		'''
		# Define a default goal state in case no goal objects are
		# generated
		if self.goalstate == []:
			self.goalstate = [(self.grid_size[0]*self.scale, self.grid_size[1]*self.scale)]

		return self.goalstate

	def getMazeState(self):
		'''
		Return blocked edges 
		'''
		return self.grid_size, self.scale, self.blocked_edges
	
	def update_blocked_edges(self, edges):

		for edge in edges:
			if edge in self.blocked_edges:
				self.blocked_edges.remove(edge)
