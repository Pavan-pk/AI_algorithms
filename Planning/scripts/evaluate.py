#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Rushang Karia", "Rashmeet Kaur Nayyar", "Pulkit Verma"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

def compute_g(algorithm, node, goal_state):
    """
        Evaluates the g() value.
        
        Parameters
        ===========
            algorithm: str
                The algorithm type based on which the g-value will be computed.
            node: Node
                The node whose g-value is to be computed.
            goal_state: State
                The goal state for the problem.
                
        Returns
        ========
            int
                The g-value for the node.
    """
    
    if algorithm == "bfs":
    
        return node.get_depth()
    
    if algorithm == "astar":
    
        return node.get_total_action_cost()

    elif algorithm == "gbfs":
        
        return 0

    elif algorithm == "ucs":
        
        return node.get_total_action_cost()

    elif algorithm == "custom-astar":
    
        return node.get_total_action_cost()
        
    # Should never reach here.
    assert False
    return float("inf")
    
def compute_h(algorithm, node, goal_state):
    """
        Evaluates the h() value.
        
        Parameters
        ===========
            algorithm: str
                The algorithm type based on which the h-value will be computed.
            node: Node
                The node whose h-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The h-value for the node.
    """
    
    if algorithm == "bfs":
    
        return 0

    if algorithm == "astar":
        
        return get_manhattan_distance(node.get_state(), goal_state)

    elif algorithm == "gbfs":
    
        return get_manhattan_distance(node.get_state(), goal_state) 

    elif algorithm == "ucs":

        return 0
        
    # Should never reach here.
    assert False
    return float("inf")

def get_manhattan_distance(from_state, to_state):
    '''
    Returns the manhattan distance between 2 states
    '''
    return abs(from_state.x - to_state.x) + abs(from_state.y - to_state.y)

