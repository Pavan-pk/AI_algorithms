#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Rushang Karia", "Rashmeet Kaur Nayyar"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import subprocess
import re
import utils
import time
import sys

def run_planner(planner_name, domain_filepath, problem_filepath,
    log_filepath = utils.PLANNER_LOG_FILEPATH):
    """
        Runs the planner and parses its output..
        
        Parameters
        ===========
            planner_name: str
                The name of the planner.
            domain_filepath: str
                The domain.pddl filepath.
            problem_filepath: str
                The problem.pddl filepath.
            log_filepath: str
                The path to the planner log file.
                
        Returns
        ========
            list
                The list of actions as a list of parameterized actions.
                Example: 2 actions move A B, pick B would be returned as
                [[move, A, B], [pick, B]] where all the members are strings.
    """
    if planner_name.lower() == "fd":
    
        planner = FD()
    else:
    
        raise Exception("Unsupported Planner!")
        return None

    command = planner.get_planner_command(domain_filepath, problem_filepath)
    log_filehandle = open(log_filepath, "w")
    
    planner_process = subprocess.Popen(command, shell=True,
        stdout=log_filehandle, stderr=log_filehandle)
        
    end_time = time.time() + utils.PLANNER_TIMEOUT_IN_SECS
    while time.time() < end_time and planner_process.returncode is None:
        
        planner_process.poll()

    try:        

        planner_process.terminate()
        planner_process.kill()
    except Exception:
    
        pass

    log_filehandle.close()

    if time.time() >= end_time:
    
        raise Exception("Timeout in high level planner!")

    return planner.parse_output(log_filepath)

class Planner:
    
    def __init__(self):
    
        pass
        
    def get_planner_command(self, domain_filepath, problem_filepath, *args,
        **kwargs):
        """
            Parameters
            ===========
                domain_filepath: str
                    The path to the domain.pddl file.
                problem_filepath: str
                    The path to the problem.pddl file.
            
            Returns
            ========
                str
                    The string command that can be used to execute the planner
                    using subprocess.
        """

        # Any new planner must implement this method.
        raise NotImplementedError
        
    def parse_output(self, log_filepath):
        """
            Parameters
            ===========
                log_filepath: str
                    The path to the planner output log.
                    
            Returns
            ========
                list(list)
                    A list of list of parameterized actions.
        """
        
        # Any new planner must implement this method.
        raise NotImplementedError
        
class FD(Planner, object):

    def __init__(self):
    
        super(FD, self).__init__()

    def get_planner_command(self, domain_filepath, problem_filepath, *args, 
        **kwargs):

        # Tip: Leave spaces at the end so that the next += does not have to
        #   worry about them.
        command = utils.ROOT_PATH + "/planners/FD/fast-downward.py "
        command += "--plan-file /tmp/plan.txt "
        command += "%s %s " % (domain_filepath, problem_filepath)
        command += "--search \"lazy_greedy([ff()], preferred=[ff()])\""

        return command
        
    def parse_output(self, log_filepath):
        
        PLAN_REGEX = re.compile("(?P<action>(\w|\W)*) \\(\d+\\)($|\n)")
        
        log_filehandle = open(log_filepath, "r")
        action_list = []
        for line in log_filehandle:
        
            action_match = PLAN_REGEX.match(line)
            if action_match is not None:
            
                action = action_match.group("action")
                action = action.strip()
                action = action.lower()
                action = action.split(" ")
                
                # We use append() instead of += since we want a list of lists.
                # action is list[] due to using split().
                #
                # [] + [1, 2] = [1, 2]
                # [].append([1, 2]) = [ [1, 2] ]
                action_list.append(action)

        return action_list
