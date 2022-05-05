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
import ast
import problem
from problem import State

from server import generate_maze
from server import initialize_planning_server
from std_msgs.msg import String
from utils import cleanup_ros
from utils import initialize_ros

class GazeboRunner:

    def __init__(self):
    
        self.action_index = float("inf")
        self.actions_queue = None
        
        self.helper = problem.Helper()
        self.move_tbot3 = None
        self.done = False
        self.current_state = None

    def setup_gazebo(self, objtypes, objcount, seed, env):
    
        # Initialize ROS core.
        initialize_ros()

        time.sleep(0.25)

        # Initialize the search server.
        initialize_planning_server()
    
        time.sleep(0.25)
        
        # Generate the maze.
        generate_maze(objtypes, objcount, seed, env)
    
        time.sleep(0.25)

        rospy.init_node("gazebo_replay")
        self.status_subscriber = rospy.Subscriber('/status', String, 
            self.status_callback)

        null_file_handle = open("/tmp/log", "w")

        # Cleanup any existing processes.
        subprocess.call("pkill -f gzclient", shell=True,
            stdout=null_file_handle, stderr=null_file_handle)
        subprocess.call("pkill -f gzserver", shell=True,
            stdout=null_file_handle, stderr=null_file_handle)
        
        # Wait for the ROS node to be created.
        time.sleep(2)
        
        # Start to move the turtle bot.
        # self.move_tbot3 = subprocess.Popen("rosrun hw2 move_tbot3.py",
            # shell=True, stdout=null_file_handle, stderr=null_file_handle)
        self.move_tbot3 = subprocess.Popen("rosrun hw2 move_tbot3.py",
            shell=True)

        # Start Gazebo.
        subprocess.Popen("roslaunch hw2 maze.launch", shell=True,
            stdout=null_file_handle, stderr=null_file_handle)
        
        # Sleep for some time to allow Gazebo to start.
        time.sleep(5)
    

    def cleanup_gazebo(self):
    
        # Sleep for some time to process the results.
        time.sleep(5)
        
        # Kill Gazebo.
        subprocess.call("pkill -f gzclient", shell=True)
        subprocess.call("pkill -f gzserver", shell=True)
        
        # Kill the turtlebot3 scripts.
        self.move_tbot3.kill()
        
        # Signal the shutdown.
        rospy.signal_shutdown("Done executing")
        
        time.sleep(0.25)
        
        # Cleanup ROS core.
        cleanup_ros()
        
        # Wait for sometime before exiting.
        time.sleep(1)

    def status_callback(self, data):
        """
        Callback function for subscriber to the `/status` rostopic
        
        """
        if(data.data == "idle"):
            self.execute_action()

    def execute_action(self):
        """
        This method picks an action from self.actions and send it to 
        RobotActionServer for execution.

        :rtype: None
        """
        if(self.action_index >= len(self.actions_queue)):
            self.done = True
            return
            
        action = self.actions_queue[self.action_index]
        self.action_index += 1
        print(action)

        if(action[0] == "move"):
            move_seq = action[1]
            goal_location = State(action[2][0], action[2][1], action[2][2])
            self.helper.execute_move_action(move_seq)
            self.current_state = goal_location
        elif(action[0] == "pick"):
            book_name = action[1]
            response = self.helper.execute_pick_action(book_name, self.current_state)
            if(response == -1):
                print("Unsuccessful Pick")
        elif(action[0] == "place"):
            book_name = action[1]
            bin_name = action[2]
            response = self.helper.execute_place_action(book_name, bin_name, self.current_state)
            if(response == -1):
                print("Unsuccessful Place")
        else:
            assert False
        
    def run_gazebo(self, actions_queue):
    
        self.action_index = 0
        self.actions_queue = actions_queue
        
        self.execute_action()
        
        while not self.done:
        
            # Sleep for some time to allow the other events to be handled.
            rospy.sleep(0.1)

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
    line = line.split(";")
    
    objtypes = int(line[0].strip())
    objcount = int(line[1].strip())
    seed = int(line[2].strip())
    exception = line[3].strip()
    
    action_line = line[4].strip()
    actions = ast.literal_eval(line[4].strip())
    env= line[5].strip()
    
    # Execute it on Gazebo.
    print("[BEGIN] Running Gazebo simulation on line_no=%u with objtypes=%u"
        ", objcount=%u, seed=%u" % (line_no, objtypes, objcount, seed))

    if exception != "None":
    
        print("[ WARN]: Running Gazebo even though run had Exception=%s" % (
            exception))

    gazebo_runner = GazeboRunner()
    gazebo_runner.setup_gazebo(objtypes, objcount, seed, env)
    gazebo_runner.run_gazebo(actions)
    gazebo_runner.cleanup_gazebo()
    
    print("[  END] Running Gazebo simulation on line_no=%u with objtypes=%u"
        ", objcount=%u, seed=%u" % (line_no, objtypes, objcount, seed))
    
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

    # Run Gazebo on the input file.
    file_handle = open(args.input_file, "r")
    
    line_no = 0
    last_line = None
    for line in file_handle:
    
        line_no += 1
        if line_no == 1:
        
            continue
        elif args.line is None:
        
            last_line = line
        elif args.line == line_no:

            run_gazebo_simulation(line_no, line)
            break
        else:

            pass
            
    if args.line is None:
        run_gazebo_simulation(line_no - 1, last_line)

