DISTANCE_SHORT = 0.254  # in m
from typing import List, Optional


DISTANCE_LONG = 0.3535  # in m
#DISTANCE_SHORT = 1
#DISTANCE_LONG = 1.2
VMAX = 5
TIME_SHORT = DISTANCE_SHORT / 5
TIME_LONG = DISTANCE_LONG / 5
VMAX_TURN45 = 3
VMAX_TURN90 = 3
VMAX_TURN135 = 3
VMAX_TURN_MIDDLE = 3
A = 13
DECEL = 11
MAZE_INFO = 16
MAZE_INFO_HALF = 8
RADIUS_TURN45 = 0.33
RADIUS_TURN90 = 0.162
RADIUS_TURN135 = 0.15



class IsBlocked:
    def __init__(self, left: bool, right: bool, ahead: bool):
        self.left = left
        self.right = right
        self.ahead = ahead

    def __repr__(self):
        return f"IsBlocked(left={self.left}, right={self.right}, ahead={self.ahead} "

class Node:
    def __init__(self, cost: float, xaxis : int, yaxis : int, parent = None, pathToThisNode: Optional[List['Node']] = None, time : float = 0, heuristic : float = 0, v0 : float = 0):
        self.cost = cost
        self.xaxis = xaxis
        self.yaxis = yaxis
        self.parent = parent
        self.pathToThisNode : List['Node']= pathToThisNode if pathToThisNode is not None else []
        self.time = time
        self.heuristic = heuristic
        self.v0 = v0

    def distance_to(self, other_node) -> float:
        return ((self.xaxis - other_node.xaxis) ** 2 + (self.yaxis - other_node.yaxis) ** 2) ** 0.5
    
    def __repr__(self):
        return f"Node(cost={self.cost:.2f}, x={self.xaxis}, y={self.yaxis}, time ={self.time}, heuristic ={self.heuristic})"

STARTING_NODE = Node(0,0,0, None,[])

def calcGoalNodes(MAZE_INFO_HALF : int):
    return {
        (MAZE_INFO_HALF - 1, MAZE_INFO_HALF - 1): Node(0, MAZE_INFO_HALF - 1, MAZE_INFO_HALF - 1),
        (MAZE_INFO_HALF - 1, MAZE_INFO_HALF): Node(0, MAZE_INFO_HALF - 1, MAZE_INFO_HALF),
        (MAZE_INFO_HALF, MAZE_INFO_HALF - 1): Node(0, MAZE_INFO_HALF, MAZE_INFO_HALF - 1),
        (MAZE_INFO_HALF, MAZE_INFO_HALF): Node(0, MAZE_INFO_HALF, MAZE_INFO_HALF)
    }

GOAL_NODES = calcGoalNodes(MAZE_INFO_HALF)