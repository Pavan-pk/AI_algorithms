#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Ketan Patil"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

from hw1.srv import *
from hw1.msg import *
import rospy
from gen_maze import *
from maze_objects import *
from utils import JSONUtils, cmdline_args, env_json_setup
import json
import cPickle
import time
import subprocess

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))

def manhattanDistance(x1, y1, x2, y2):
    """
    This function returns manhattan distance between two points.
    """
    return abs(x1-x2) + abs(y1-y2)

class Server(object):

    def __init__(self, maze,blocked_edges,maze_objects):
        if maze is not None:
            self.maze = maze
            self.blocked_edges = blocked_edges
            self.grid_size, self.cell_size, self.blocked_edges = self.maze.getMazeState()
            self.init_x, self.init_y, _ = self.maze.initState()
            self.goalstates = self.maze.getGoalState()
        self.cell_size = 0.5
        self.blocked_edges = blocked_edges
        self.action_list = ["TurnCW", "TurnCCW", "MoveF"]
        self.direction_list = ["NORTH", "EAST", "SOUTH", "WEST"]
    
    def start(self):
        rospy.init_node('get_successor_server',log_level = rospy.ERROR)
        rospy.Service('get_successor', GetSuccessor, self.handle_get_successor)
        rospy.Service('get_initial_state', GetInitialState, self.handle_get_initial_state)
        rospy.Service('is_goal_state', IsGoalState, self.handle_is_goal_state)
        rospy.Service('get_goal_state', GetGoalState, self.handle_get_goal_state)
        rospy.Service("generate_maze", GenerateMaze, self.ros_generate_maze)
        print ("Ready!")
        rospy.spin()
    
    def is_valid_edge(self, curr_x, curr_y, next_x, next_y):

        # Out of bounds check
        if next_x < self.init_x or next_x> self.grid_size[0] \
            or next_y < self.init_y or next_y > self.grid_size[1]:
            return False
        # Blocked edges check
        elif (curr_x, curr_y, next_x, next_y) in self.blocked_edges \
            or (next_x, next_y, curr_x, curr_y) in self.blocked_edges:
            # edge found in blocked edges
            return False
        else:
            # edge NOT found in blocked edges
            return True

    def handle_get_initial_state(self, req):
        x, y, orientation = self.maze.initState()
        return GetInitialStateResponse(x, y, orientation)

    def handle_is_goal_state(self, req):
        if (req.x, req.y)in self.goalstates:
            return IsGoalStateResponse(1)
        else:
            return IsGoalStateResponse(0)

    def handle_get_goal_state(self, req):

        goals = []
        for goal in self.goalstates:
            pose = poseTuple()
            pose.x = goal[0]
            pose.y = goal[1]
            goals.append(pose)

        return GetGoalStateResponse(goals)
    
    def ros_generate_maze(self,req):

        blocked_edges = []
        if(req.env == "canWorld"):
            grid_dimensions = [req.dimension_x/self.cell_size, req.dimension_y/self.cell_size]
            max_obstacles =  grid_dimensions[0]* (grid_dimensions[1] + 1) * 2
            blocked_edges = []
        else:
            dimension_x = 4
            dimension_y = 12
            obstacle_dims = [3,6]
            # grid_dimensions = [dimension_x/self.cell_size,dimension_y/self.cell_size]
            grid_dimensions = [dimension_x,dimension_y]
            max_obstacles = obstacle_dims[0]* (obstacle_dims[1] + 1) * 2
            if os.path.isfile(os.path.join(ROOT_DIR, 'config/env_precompute.pkl')):
                with open(os.path.join(ROOT_DIR, 'config/env_precompute.pkl'), 'r') as envfile:
                    blocked_edges = cPickle.load(envfile)
            
            blocked_edges_temp = (np.array(blocked_edges['map'])*float(blocked_edges['scale'])).tolist()
            blocked_edges = []

            for i in blocked_edges_temp:
                blocked_edges.append(tuple(i))
        if req.obstacles > max_obstacles:
            return GenerateMazeResponse(1)
        
        env_json_setup(req.obstacles, req.env)

        maze_objects = json.load(open(os.path.join(ROOT_DIR,'config/maze.json'),'r'),
                               object_hook=JSONUtils('maze_objects').custom_dict_hook)

        self.maze = MazeGenerator(grid_dimensions,
                                  maze_objects,
                                  self.cell_size,
                                  req.seed,
                                  blocked_edges,
                                  req.env)
        self.grid_size, self.cell_size, self.blocked_edges = self.maze.getMazeState()
        self.init_x, self.init_y, _ = self.maze.initState()
        self.goalstates = self.maze.getGoalState()
        time.sleep(5.0)

        return GenerateMazeResponse(0)

    def handle_get_successor(self, req):
        
        state_x, state_y, state_direction, state_cost = [],[],[],[]

        for action in self.action_list:
        #Checking requested action and making changes in states
            x_cord, y_cord, direction = req.x, req.y, req.direction
            if action == 'TurnCW':
                index = self.direction_list.index(req.direction)
                direction = self.direction_list[(index+1)%4]
                g_cost = 2

            elif action == 'TurnCCW':
                index = self.direction_list.index(req.direction)
                direction = self.direction_list[(index-1)%4]
                g_cost = 2

            elif action == 'MoveF':
                if direction == "NORTH":
                    y_cord += self.cell_size
                elif direction == "EAST":
                    x_cord += self.cell_size
                elif direction == "SOUTH":
                    y_cord -= self.cell_size
                elif direction == "WEST":
                    x_cord -= self.cell_size
                g_cost = 1

            elif action == 'MoveB':
                if direction == "NORTH":
                    y_cord -= self.cell_size
                elif direction == "EAST":
                    x_cord -= self.cell_size
                elif direction == "SOUTH":
                    y_cord += self.cell_size
                elif direction == "WEST":
                    x_cord += self.cell_size
                g_cost = 3
            
            isValidEdge = self.is_valid_edge(curr_x=req.x,
                                            curr_y=req.y,
                                            next_x=x_cord,
                                            next_y=y_cord)

            if not isValidEdge:
                state_x.append(-1)
                state_y.append(-1)
                state_direction.append(direction)
                state_cost.append(-1)
            else:
                state_x.append(x_cord)
                state_y.append(y_cord)
                state_direction.append(direction)
                state_cost.append(g_cost)

        return GetSuccessorResponse(state_x,
                                    state_y,
                                    state_direction,
                                    state_cost,
                                    self.action_list)




def generate_maze(dimension_x,dimension_y, obstacles, seed, env):

    rospy.wait_for_service('generate_maze')

    response =  rospy.ServiceProxy('generate_maze', GenerateMaze) \
        (dimension_x,dimension_y, obstacles, seed, env)
        
    if response.done != 0:
        
        raise Exception("Cannot generate the maze")

def initialize_search_server():

    fileHandle=open("/dev/null", "w")

    # Start the server.
    proc = subprocess.Popen("rosrun hw1 server.py", shell=True)
    return proc


def main():

    parser = cmdline_args()
    args = parser.parse_args()
    
    # Fixed dims to make the obstacles behave
    cell_size = 0.5
    grid_x = 3
    grid_y = 6
    grid_dimensions = [grid_x/cell_size, grid_y/cell_size]

    maze_objects = json.load(open(os.path.join(ROOT_DIR,'config/maze.json'),'r'),
                               object_hook=JSONUtils('maze_objects').custom_dict_hook)

    blocked_edges = []
    if os.path.isfile(os.path.join(ROOT_DIR,'config/env_precompute.pkl')):
        with open(os.path.join(ROOT_DIR,'config/env_precompute.pkl'), 'r') as envfile:
            blocked_edges = cPickle.load(envfile)

	blocked_edges = (np.array(blocked_edges['map'])*float(blocked_edges['scale'])).tolist()

    Server(None,blocked_edges,maze_objects).start()

if __name__ == "__main__":
    main()
