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

import sys
import rospy
from hw2.srv import *
from std_msgs.msg import String
import collections

class State:
    """
    This class defines the state of the TurtleBot.

    """
    
    def __init__(self,x,y,orientation):
        """
        :param x: current x-coordinate of turtlebot
        :type x: float
        :param y: current y-coordinate of turtlebot
        :type y: float   
        :param orientation: current orientation of turtlebot, can be either NORTH, SOUTH, EAST, WEST
        :type orientation: str

        """    
        self.x  = x 
        self.y = y
        self.orientation = orientation

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y and self.orientation == other.orientation:
            return True
        else:
            return False
            
    def __hash__(self):
    
        return hash(str(self))

    def __repr__(self):
        return "({}, {}, {})".format(str(self.x), str(self.y), str(self.orientation))
        
    def get_gazebo_representation(self):
        """
            Returns
            ========
                list
                    List containing x, y, and orientation.
        """
        
        return [self.x, self.y, self.orientation]

    def get_x(self):
        """
            Returns
            ========
                int
                    The x-value of the state (-1 if the state is invalid).
        """
        
        return self.x
        
    def get_y(self):
        """
            Returns
            ========
                int
                    The y-value of the state (-1 if the state is invalid).
        """
        
        return self.y
        
    def get_orientation(self):
        """
            Returns
            ========
                str
                    The direction (orientation) of the state.
        """
        
        return self.orientation

class Helper:
    """
    This class provides the methods used to control TurtleBot.
        
    Example:
        .. code-block:: python

            from problem import Helper

            h = Helper()
            init_state = h.get_init_state()

    """

    def remove_edge(self, book_name):
    
        rospy.wait_for_service('remove_blocked_edge')
        try:
            remove_edge = rospy.ServiceProxy('remove_blocked_edge', RemoveBlockedEdgeMsg)
            _ = remove_edge(book_name)
        except rospy.ServiceException,e:
            print "Sevice call failed: %s" % (e)

    def execute_pick_action(self, object_name, state):
        """
        This action picks the object named object_name located at State robot_state.

        :param object_name: Name of the object
        :type object_name: str
        :param robot_state: State of the robot.
        :type robot_state:  State

        :returns: True if the place action was successful. False, otherwise. 
        :rtype: bool

        :raises: ServiceException: When call to rospy fails.

        Example:
            .. code-block:: python

                from problem import Helper, State

                h = Helper()
                execute_status = h.execute_pick_action(object_name, curr_state)
                if execute_status == False:
                    print "Pick Action Failed"

        .. warning::
            This action will fail (Return False) if:

            .. hlist::
                :columns: 1
                
                * object name is not valid.
                * Robot Location is not within the load location of the object, i.e. robot is not in the vicinity of the object.

        """
        rospy.wait_for_service('execute_pick_action')
        try:
            pick_action = rospy.ServiceProxy('execute_pick_action',PickActionMsg)
            response = pick_action(object_name,state.x,state.y,state.orientation)

            if response.result == -1:
                return False
            elif response.result == 1:
                return True

        except rospy.ServiceException,e:
            print "Sevice call failed: %s"%e

    def execute_place_action(self, object_name, goal_name, state):
        """
        This action places the object named object_name in the location named goal_name when the TurtleBot is at State robot_state.

        :param object_name: Name of the object that has to placed in the location.
        :type object_name: str
        :param goal_name: Name of the location where the object named object_name will be placed. 
        :type goal_name: str
        :param robot_state: State of the robot and and the location.
        :type robot_state: State

        :returns: True if the place action was successful. False, otherwise. 
        :rtype: bool

        :raises: ServiceException: When call to rospy fails.

        Example:
            .. code-block:: python

                from problem import Helper, State

                h = Helper()
                execute_status = h.execute_place_action(object_name, goal_name, curr_state)
                if execute_status == False:
                    print "Place Action Failed"

        .. warning::
            This action will fail (Return False) if:

            .. hlist::
                :columns: 1
                
                * object name or location name is not valid.
                * object size and location size does not match.
                * Type of the object and location does not match.
                * Robot Location is not within the load location of the location, i.e. robot is not in the vicinity of the location.

        """
        rospy.wait_for_service('execute_place_action')
        try:
            place_action = rospy.ServiceProxy('execute_place_action',PlaceActionMsg)
            response = place_action(object_name,goal_name,state.x,state.y,state.orientation)

            if response.result == -1:
                return False
            elif response.result == 1:
                return True
                        
        except rospy.ServiceException,e:
            print "Sevice call failed: %s"%e

    def execute_move_action(self, action_list):
        """
        This action executes the actions in the action_seq. The actions are executed in the same sequence as they appear in action_seq list.

        :param action_seq: Sequence of actions to be executed by the TurtleBot.
        :type action_seq: list(str)

        :rtype: None

        :raises: ServiceException: When call to rospy fails.

        Example:
            .. code-block:: python

                from problem import Helper, State

                h = Helper()
                h.execute_move_action(action_list)

        .. warning::
            This method will execute the actions even if they are wrong semantically, i.e. do not have the desired effect. It is the responsibility of the caller to ensure that the action sequence is applicable from the current state of the robot.

        """
        rospy.wait_for_service('execute_move_action')
        try:
            action_str = "_".join(action for action in action_list)
            move_action = rospy.ServiceProxy("execute_move_action",MoveActionMsg)
            response = move_action(action_str)
            return True
        except rospy.ServiceException,e:
            print "Sevice call failed: %s"%e

    def get_successor(self, curr_state):
        """
        This function calls get_successor service with current state as input and receives a dictionary as output. Possible actions are key to this dictionary and the value of each action is the state that can be reached by applying that action and the corresponding cost.

        :param curr_state: current state of the TurtleBot
        :type curr_state: State

        :returns: An **ordered** dictionary of actions {action_i : (state_i, cost_i)} such that applying action_i on curr_state results in state_i and it incurs a cost cost_i.

        :rtype: OrderedDict {str: tuple(State, float)}

        :raises: ServiceException: When call to rospy fails.

        Example:
            .. code-block:: python

                from problem import Helper, State

                h = Helper()
                initial_state = h.get_initial_state()
                possible_successors_dict = h.get_successor(initial_state)

        .. warning::
            Process the output of this function in the same order it is returned. If you need to break ties between multiple nodes, use the dictionary element that appears earlier in the dictionary returned by get_successor(). Since this is an ordered dictionary, the order will be preserved unlike normal python dictionary.

            Example:
                Assume that a call to get_successor(initial_state) returns {a1: (s1, 2), a2: (s2, 2)}. Now we have 2 nodes in fringe with equal cost, so we will choose **a1** for expansion as it appears before a2 in the dictionary.

        """
        rospy.wait_for_service('get_successor')

        try:
            get_successor = rospy.ServiceProxy('get_successor', GetSuccessor)
            response = get_successor(curr_state.x, curr_state.y, curr_state.orientation)
            states = collections.OrderedDict()

            for i in range(3):
                states[response.action[i]] = (
                State(response.x[i], response.y[i], response.direction[i]), response.g_cost[i])
            return states

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_initial_state(self):
        rospy.wait_for_service('get_initial_state')
        try:
            get_initial_state = rospy.ServiceProxy('get_initial_state', GetInitialState)
            response = get_initial_state()
            return State(response.x, response.y, response.direction)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def is_goal_state(self, current_state, goal_state):
        """
        Checks if the current_state is goal_state or not. 
        If you are wondering why we are checking orientation, remember this is a different Homework. :)

        """
        if current_state.x == goal_state.x and current_state.y == goal_state.y and current_state.orientation == goal_state.orientation:
            return True
        return False

    def get_goal_state(self):
        rospy.wait_for_service('get_goal_state')
        try:
            get_goal_state = rospy.ServiceProxy('get_goal_state', GetGoalState)
            response = get_goal_state()
            return State(response.x,response.y,"EAST")
        except rospy.ServiceException,e:
            print "Service call failed: %s"%e

    def get_actions(self):
        return ["TurnCW","TurnCCW","MoveF"]

    def usage(self):
        return "%s [x y]"%sys.argv[0]
