#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Abhyudaya Srinet", "Pulkit Verma", "Rushang Karia"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'


def get_goal_string(object_dict, obj_list, obj_loc_list, goal_list, 
    goal_loc_list,env):
    """
        Returns
        ========
            str:
                A generic goal condition that will place every object based on
                its type and size at the correct goal.
    """

    # Append your goal condition to this string.
    #
    # Below are a few hints to help you along the way:
    # =================================================
    #
    # You can print the parameters to help you in coming up with a generalized
    # goal condition.
    #
    # You can also look at the (:objects) and (:init) of problem.pddl to give
    # you an idea of where your goal string is going wrong.
    #
    # Keep in mind the subject type and sizes of the bins and the books must
    # match.
    #
    # High level plans do not need actual co-ordinates, rather they use the
    # high-level locations found in the parameters of this method.
    #
    # Finally, there exists a way to write the goal condition using universal
    # and existential quantifiers.
    #
    # Executing wrong plans on Gazebo might not give you the right execution!
    #
    # Remember that the two environments have unique objects, goals and predicates!
    
    goal_string = "(:goal (and "

    if env == "bookWorld":
        goal_string += "(forall (?bk - book)"
        goal_string += "(exists (?bn - bin ?lc - location ?sz - size ?sb - subject)"
        goal_string += "(and"
        goal_string += "(Book_At ?bk ?lc)"
        goal_string += "(Bin_At ?bn ?lc)"
        goal_string += "(Book_Subject ?bk ?sb)"
        goal_string += "(Bin_Subject ?bn ?sb)"
        goal_string += "(Book_Size ?bk ?sz)"
        goal_string += "(Bin_Size ?bn ?sz)"
        goal_string += ")))"

    elif env == "cafeWorld":
        goal_string += "(forall (?fd - food)"
        goal_string += "(exists (?tb - table ?lc - location ?sz - size ?ty - food_type)"
        goal_string += "(and"
        goal_string += "(Food_At ?fd ?lc)"
        goal_string += "(Table_At ?tb ?lc)"
        goal_string += "(Food_Type ?fd ?ty)"
        goal_string += "(Ordered ?tb ?ty)"
        goal_string += "(Portion_Size ?fd ?sz)"
        goal_string += "(Ordered_Portion ?tb ?sz)"
        goal_string += ")))"
    
    goal_string += "))\n"
    
    return goal_string

def sample_goal_condition(object_dict, obj_list, obj_loc_list, goal_list, 
    goal_loc_list):
    """
        Returns
        ========
            str:
                A generic goal condition that moves the robot to any one of
                the object locations.

        
        .. note ::

            You can replace the contents of get_goal_string() with the text below
            to get an idea of what is expected.
            
            The goal condition in the stock task here is VASTLY different from the
            expectation from you. Please review the homework documentation to identify
            your task.
            
            Here are some instructions to run this in Gazebo.
            1. Replace the content of get_goal_string() with this method.
            2. rosrun hw2 refinement.py \
                --objtypes <object types> \
                --objcount <number of objects> \
                --seed <seed>
            3. rosrun hw2 gazebo.py

            The generic goal condition here is to move the robot to a object location.
            
            The stock task below generates a generic goal condition that moves the
            robot to a random object location and this is independent of the total 
            number of locations and objects. 
            
    """

    import random
    assert len(obj_loc_list) > 0
    i = random.randint(0, len(obj_loc_list) - 1)
    
    goal_string = "(:goal (and "
    goal_string += "(Robot_At tbot3 %s)" % (obj_loc_list[i])
    goal_string += "))\n"
    
    return goal_string
