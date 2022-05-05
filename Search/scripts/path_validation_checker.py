#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Ketan Patil", "Naman Shah"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

def check_is_edge(edge, valueFlag):
	"""
	This function checks if two points are connected via edge or not.
	"""
	mazeInfo = [(0, 5, 'N'), {(2, 1, 3, 1), (0, 2, 1, 2), (4, 2, 4, 3), (1, 3, 2, 3), (3, 4, 4, 4)}]
	invalid_edges = mazeInfo[1]
	if edge in invalid_edges:
		return False
	else:
		return True


def pathValidationChecker(path):
	mazeInfo = [(0, 5, 'N'), {(2, 1, 3, 1), (0, 2, 1, 2), (4, 2, 4, 3), (1, 3, 2, 3), (3, 4, 4, 4)}]
	directionList = ["N", "E","S","W"]
	isValidEdge = True
	prevState = None
	for state in path:
		directionChange = False
		yCordChange = False
		xCordChange = False
		changeDict = {"True":0}
		#Intial state checking
		if prevState == None:
			if state[0] != mazeInfo[0][0] or state[1] != mazeInfo[0][0] or state[2] != mazeInfo[0][2]:
				return "Invalid start state"
			else:
				print("continue")
		else:
			#Direction change check
			if directionList[((directionList.index(prevState[2]))+1)%4] == state[2] or directionList[((directionList.index(prevState[2]))-1)%4] == state[2]:
				directionChange = True
				changeDict["True"] += 1  
			if (abs(state[0]-prevState[0]) == 0 and abs(state[1]-prevState[1]) == 1) and state[1] > mazeInfo[0][0] and state[1]<mazeInfo[0][1]:
				yCordChange = True
				changeDict["True"] += 1
			if (abs(state[1]-prevState[1]) == 0 and abs(state[0]-prevState[0]) == 1) and state[0] > mazeInfo[0][0] and state[0]<mazeInfo[0][1]:
				xCordChange = True
				changeDict["True"] += 1
			print(changeDict)
			if changeDict["True"] > 1 or changeDict["True"] == 0:
				return "Invalid state"

			#checking if robot is not going through blocked edge
			if yCordChange or xCordChange:
				if prevState[0] <= state[0] and prevState[1] <= state[1]:
					isValidEdge = check_is_edge((prevState[0], prevState[1], state[0], state[1]), "changedValuesLater")
				else:
					isValidEdge = check_is_edge((state[0], state[1], prevState[0], prevState[1]), "changedValuesBefore")
				
			if not isValidEdge:
				return "Going through blocked edge"
		prevState = state
	return True 

path = [(0, 0, "N"), (0, 0, "E"), (1, 0, "E"), (2, 0, "E"), (2, 0, "S"), (2, 1, "S"), (2, 2, "S")]
print(pathValidationChecker(path))