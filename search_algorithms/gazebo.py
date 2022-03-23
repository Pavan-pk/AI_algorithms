#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Rushang Karia"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import argparse
import rospy
import subprocess
import time

from server import generate_maze
from server import initialize_search_server
from std_msgs.msg import String
from utils import cleanup_ros
from utils import initialize_ros

def run_gazebo_simulation(line_no, line):
    """
        Runs a simulation in Gazebo.
        
        Parameters
        ===========
            line_no: int
                The line number that is being run in Gazebo.
            line: str
                The line containing the world description and plan.
    """
    
    line = line.strip()
    line = line.split(",")
    
    dimensions = (line[0].strip())
    dimension_x, dimension_y = dimensions.split("x")
    dimension_x, dimension_y = int(dimension_x), int(dimension_y)
    obstacles = int(line[1].strip())
    seed = int(line[2].strip())
    
    actions = line[-2].strip()
    env = line[-1].strip()
    
    # Generate the maze.
    generate_maze(dimension_x, dimension_y, obstacles, seed, env)
    
    # Execute it on Gazebo.
    print("[BEGIN] Running Gazebo simulation on line_no=%u with dimension=%s"
        ", obstacles=%u, seed=%u, plan=%s" % (line_no, dimensions , obstacles,
        seed, str(actions.split("_"))))

    if actions != "":
    
        execute_on_gazebo(actions)
    
    print("[  END] Running Gazebo simulation on line_no=%u with dimension=%s"
        ", obstacles=%u, seed=%u, plan=%s" % (line_no, dimensions, obstacles,
        seed, str(actions.split("_"))))

def execute_on_gazebo(plan_str):
    """
        Publishes the actions to the publisher topic in ROS.
        
        Parameters
        ===========
            plan_str: str
                The actions to execute expressed as a1_a2_a3..._an.
    """

    null_file_handle = open("/tmp/log", "w")

    # Cleanup any existing processes.
    subprocess.call("pkill gzclient", shell=True,
        stdout=null_file_handle, stderr=null_file_handle)
    subprocess.call("pkill gzserver", shell=True,
        stdout=null_file_handle, stderr=null_file_handle)
    
    rospy.init_node("gazebo_replay")
    publisher = rospy.Publisher("/actions", String, queue_size=10)
    
    # Wait for the ROS node to be created.
    time.sleep(2)
    
    # Start to move the turtle bot.
    x = subprocess.Popen("rosrun hw1 move_tbot3.py", shell=True,
        stdout=null_file_handle, stderr=null_file_handle)
    
    # Start Gazebo.
    subprocess.Popen("roslaunch hw1 maze.launch", shell=True,
        stdout=null_file_handle, stderr=null_file_handle)
    
    # Sleep for some time to allow Gazebo to start.
    time.sleep(5)
    
    # Start up the ROS publisher and publish the action list.
    publisher.publish(String(data=plan_str))

    # Wait till all actions have been published.
    # The move_tbot3.py script will automatically shutdown once all actions
    # have been executed.
    x.wait()
    
    # Sleep for some time to process the results.
    time.sleep(5)
    
    # Kill Gazebo.
    subprocess.call("pkill gzclient", shell=True)
    subprocess.call("pkill gzserver", shell=True)
    
    # Wait for sometime before exiting.
    time.sleep(1)
    
if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--input-file", required=True,
        help="The input solution file to run")
        
    parser.add_argument("--line", default=None,
        type=int,
        help="The line number to run in Gazebo (Line 1 is the header). "
        "If unspecified, then all solutions are run")
        
    # Parse the arguments.
    args = parser.parse_args()
    
    # Initialize ROS core.
    initialize_ros()

    # Initialize the search server.
    initialize_search_server()

    # Run Gazebo on the input file.
    file_handle = open(args.input_file, "r")
    
    line_no = 0
    for line in file_handle:
    
        line_no += 1
        if line_no != 1:
        
            if args.line is None:
            
                run_gazebo_simulation(line_no, line)
            elif args.line == line_no:
    
                run_gazebo_simulation(line_no, line)
                break

    # Cleanup ROS core.
    cleanup_ros()
