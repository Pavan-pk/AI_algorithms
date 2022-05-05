__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Kiran Prasad", "Rashmeet Kaur Nayyar"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah", "Kiran Prasad"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

from io import DEFAULT_BUFFER_SIZE
import json
import importlib
import os
import argparse
import time

import subprocess
import time

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))

class JSONUtils(object):
	def __init__(self, module_name):
		self.module = module_name

	def custom_dict_hook(self, dct):
		result_dict = {}
		for objname in dct.keys():
			obj = self.import_from_name(objname)
			result_dict[obj] = dct[objname]
		return result_dict

	def import_from_name(self, object_name):
		module = importlib.import_module(self.module)
		return getattr(module, object_name, object_name)

def env_json_setup(fcnt, env):
	GAZEBO_PARENT_DIR = os.getenv('HOME')
	food_dict = {
		"obstacles": {
			"Table": {"count": 1,
				"file":os.path.join(GAZEBO_PARENT_DIR, ".gazebo/models/cafe_table/meshes/cafe_table.dae"),
				"scale":[0.4,0.4,0.4]
			}

		},
		"bounding": {
			"DAEBounding": {
				"count":1, 
				"file":os.path.join(GAZEBO_PARENT_DIR, ".gazebo/models/world/meshes/world_final.dae"),
				"scale":[0.4,0.4,0.4]
			}
		},
		"goal": {
		"Cake": {"count": 1,
				"file":os.path.join(GAZEBO_PARENT_DIR, ".gazebo/models/cake/meshes/model.dae"),
				"scale":[0.3,0.3,0.3]
		}
	}
	}

	can_dict = {
		"obstacles": {
			"Can": {"count": 1,
			}

		},
		"bounding": {
			"Wall": {
				"count":1,
			}
		},
		"goal": {
		"Goal": {"count": 1}
		}
	}

	if env == "canWorld":
		default_dict = can_dict
		default_dict['obstacles']['Can']['count'] = fcnt
	elif env == "cafeWorld":
		default_dict = food_dict
		default_dict['obstacles']['Table']['count'] = fcnt


	with open(os.path.join(ROOT_DIR,'config/maze.json'),'w') as of:
		json.dump(default_dict, of)

def cmdline_args():
	parser = argparse.ArgumentParser()
	parser.add_argument('-s', help='for providing random seed', metavar='32', action='store', dest='seed', default=int(time.time()), type=int)
	return parser


def initialize_ros():

	fileHandle=open("/dev/null", "w")

	# Cleanup any previous instances of roscore.
	subprocess.call("pkill roscore", shell=True,
		stdout=fileHandle, stderr=fileHandle)
	
	# Start a new instance.
	proc = subprocess.Popen("roscore", shell=True,
			stdout=fileHandle, stderr=fileHandle)
	
	# Wait a few seconds for ROS Core to come up.
	time.sleep(5)
	return proc

def cleanup_ros(*pids):

	fileHandle=open("/dev/null", "w")
	for pid in pids:
		subprocess.call("kill -9 {}".format(pid), shell=True,
		stdout=fileHandle, stderr=fileHandle)
	# Cleanup any previous instances of roscore.
	subprocess.call("pkill roscore", shell=True,
		stdout=fileHandle, stderr=fileHandle)
