from typing import List
import math
import API
import sys
import Helper

# Micro Mouse starts in Center of tile, but its easier for diagonal calculation if its at corner, therfore an addition drive is added at the calculations
#
#
#
#
#

def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

def main():
    log("Starting...")
    API.setColor(0, 0, "G")
    API.setText(0, 0, "abc")
    currentNode : Helper.Node = Helper.Node(0,0,0)
    onGoal : bool = False
    v0 : float ; totalTime : float = firstHalf() # 0 standing(only on start / return) 1 speed after 1 acceleration 2 max speed
    vend : float = 0
    direction : int = 0# north = 0, south = 4, west = 2, east = -2
    openNodes = [Helper.Node(0,0,0)]
    while not onGoal:
        currentNodes = getCurrentNodes(currentNode, direction)
        a = calculateCostForCurrentNodes(v0, direction, openNodes, currentNode)
        newNode = determinFastetNode(currentNodes)
        openNodes.append(newNode)

def firstHalf():

    return speedAfterDistance(0, Helper.VMAX, Helper.A, Helper.DISTANCE_SHORT / 2), timeCalculation(0, Helper.VMAX, Helper.A, Helper.DISTANCE_SHORT / 2)

if __name__ == "__main__":
    main()

def calculateCost(v0 : float, direction, currentNode, node):
    heuristic : float = calculateHeuristic(currentNode, v0, direction)
    return calculateTimeCost(v0, direction, currentNode, node) + heuristic

def  calculateTimeCost(v0 : float, direction, currentNode, node):
    turn = checkWhatTurn(direction, currentNode, node)
    newdirection : int = (direction + turn) % 8
    distance = getDistance(newdirection)
    if v0 == Helper.vmax:
        if newdirection == direction:
            match newdirection:
                case 0, -2, 2, 4:
                    return Helper.TIME_SHORT
                case _:
                    return Helper.TIME_LONG
        else:
            return timeWithoutTurn(v0, turn, distance) + 0#set tun time
    else:
        if newdirection == direction:
            t1 = (Helper.vmax - v0) / Helper.a
            accelerated_distance = v0 * t1 + 0.5 * Helper.a * t1 ** 2

            if accelerated_distance >= Helper.DISTANCE_SHORT:
                discriminant = v0**2 + 2 * Helper.a * Helper.DISTANCE_SHORT
                return (-v0 + math.sqrt(discriminant)) / Helper.a
            else:
                return t1 + (Helper.DISTANCE_SHORT - accelerated_distance) / Helper.vmax
        else:
            return timeWithoutTurn(v0, turn, distance) + 0#set tun time

def getDistance(newdirection):
    match newdirection:
        case 0, -2, 2, 4:
            return Helper.DISTANCE_SHORT
        case _: 
            return Helper.DISTANCE_LONG 

def timeWithoutTurn(v0 : float,turn, distance ):
    match turn:
        case 1:
            vmax_turn = Helper.VMAX_TURN45
        case 2:
            vmax_turn = Helper.VMAX_TURN90
        case _: 
            vmax_turn = Helper.VMAX_TURN135
    turnTime : float = (Helper.DISTANCE_SHORT/(turn*(math.pi/180)))/v0
    return ((v0 - vmax_turn) / Helper.decel) + turnTime

# 0° = 0
# 45°/ 225° = 1
# 90°/270 = 2
# 135°/ 315° = 3
# 180 ° = 4
def checkWhatTurn(direction, currentNode : Helper.Node, node):
    nodeDirection = checkTurn(currentNode, node)

    # Always work with absolute direction difference
    diff = abs(direction - nodeDirection)

    # Normalize to range 0-4 (circular)
    normalized = diff % 5

    return normalized

def checkTurn(currentNode, node):
    dx = node.xaxis - currentNode.xaxis
    dy = node.yaxis - currentNode.yaxis

    direction_map = {
        (0,  1): 0,   # north
        (1,  0): -2,  # east
        (-1, 0): 2,   # west
        (0, -1): 4,   # south
        (1,  1): -1,  # northeast
        (1, -1): -3,  # southeast
        (-1, -1): 3,  # southwest
        (-1, 1): 1    # northwest
    }

    return direction_map.get((dx, dy))
        

def countTime(time, distance, speed):
    newTime : int = distance/speed + time
    log("new run time is")
    return newTime

def checkCurrent():
    stateAhead = Helper.IsBlocked(API.wallLeft, API.wallRight, API.wallFront)
    goBack()
    return stateAhead

# def checkDiagonal():
#     return Helper.LeftRightBlocked( checkLeft() ,checkRight())


# def checkLeft():
#     API.moveForward()
#     if API.wallLeft():
#         goBack(True)
#         return False
#     goBack(True)
#     return True

# def checkRight():
#     API.moveForward()
#     if API.wallRight():
#         goBack(False)
#         return False
#     goBack(False)
#     return True

# false is left true is right
def goBack():
        API.turnLeft()
        API.turnLeft()
        API.moveForward()
        API.turnLeft()
        API.turnLeft()

def speedAfterDistance(v0 : float, vmax, a, d):
    v : float = math.sqrt(v0 *v0 + 2*a*d)
    return vmax if v < vmax  else v
        

def timeCalculation(v0 : float, vmax, a, d):
    vend = speedAfterDistance(v0, vmax, a, d)
    if a == 0:
        return d / v0
    if vend != vmax :
        return (vend - v0)/a
    dForAccel : float = (vend**2 - v0**2) / (2 * a)
    return (vmax - v0)/a + (d - dForAccel) / v0

def distanceToSlowDown(v0 : float, vmax, a):
    return (vmax**2 - v0**2) / (2 * a)

def timeToSlowDown(v0 : float, vmax, a):
    return (vmax - v0) / a

#Movment is always calculated as starting at 0,0, always +1, always facing north
# def getCurrentNodes(currentNode, direction):
#     openNodes = []
#     openDirections = checkCurrent()
#     if openDirections.ahead :
#         nodeAhead = Helper.Node(currentNode.cost, currentNode.xaxis, currentNode.yaxis + 1)
#         openNodes.append(nodeAhead)
#     if openDirections.left :
#         nodeLeft = Helper.Node(currentNode.cost, currentNode.xaxis - 1, currentNode.yaxis)
#         openNodes.append(nodeLeft)
#     if openDirections.right :
#         nodeRight = Helper.Node(currentNode.cost, currentNode.xaxis, currentNode.yaxis - 1)
#         openNodes.append(nodeRight)
#     return openNodes

def getCurrentNodes(currentNode : Helper.Node, direction):
    openNodes = []
    openDirections = checkCurrent()

    # Define relative movement vectors for "forward", "left", "right"
    movements = {
        "ahead": (0, 1),
        "left": (-1, 0),
        "right": (1, 0)
    }

    # Convert direction to degrees-like format for easy mapping
    direction_map = {
        0: (0, 1),     # North
        -2: (1, 0),    # East
        2: (-1, 0),    # West
        4: (0, -1)     # South
    }

    dx, dy = direction_map[direction]

    if openDirections.ahead:
        mx, my = rotate(*movements["ahead"], direction)
        openNodes.append(Helper.Node(currentNode.cost, currentNode.xaxis + mx, currentNode.yaxis + my))
    if openDirections.left:
        mx, my = rotate(*movements["left"], direction)
        openNodes.append(Helper.Node(currentNode.cost, currentNode.xaxis + mx, currentNode.yaxis + my))
    if openDirections.right:
        mx, my = rotate(*movements["right"], direction)
        openNodes.append(Helper.Node(currentNode.cost, currentNode.xaxis + mx, currentNode.yaxis + my))
    
    openDirections = checkAhead()
    match direction:
        case 0:
            currentNode.yaxis + 1
        case 4:
            currentNode.yaxis - 1
        case 2:
            currentNode.xaxis -1
        case -2:
            currentNode.xaxis +1

    if openDirections.left:
        mx, my = rotate(*movements["left"], direction)
        openNodes.append(Helper.Node(currentNode.cost, currentNode.xaxis + mx, currentNode.yaxis + my))
    if openDirections.right:
        mx, my = rotate(*movements["right"], direction)
        openNodes.append(Helper.Node(currentNode.cost, currentNode.xaxis + mx, currentNode.yaxis + my))

    return openNodes

def rotate(x, y, direction):
    if direction == 0:   # North
        return x, y
    elif direction == -2:  # East
        return y, -x
    elif direction == 4:  # South
        return -x, -y
    else:  # West
        return -y, x
    
def checkAhead():
    API.moveForward
    a = checkCurrent()
    goBack()
    return a


def calculateCostForCurrentNodes(v0 : float, direction : int, openNodes : List[Helper.Node], currentNode : Helper.Node):
    for node in openNodes :
        node.cost = calculateCost(v0, direction, currentNode, node)
    return openNodes

def determinFastetNode(currentNodes : List[Helper.Node]):
    currentFastest = Helper.Node(10000,0,0)
    for node in currentNodes:
        if node.cost < currentFastest.cost:
            currentFastest = node
    return currentFastest

def calculateHeuristic(currentNode : Helper.Node, v0 : float, direction):
    nearestNode = calculateNearestNode(currentNode)
    distance : float; noTurn : bool = calculateDistanceTwoNodes(nearestNode, currentNode, direction)
    return calculateTimeToGoal(distance, noTurn, v0)

def calculateNearestNode(currentNode):
    nearest = min(Helper.GOAL_NODES, key=lambda node: currentNode.distance_to(node))
    return nearest

def calculateDistanceTwoNodes(nearestNode, currentNode, direction):
    dx = abs(nearestNode.xaxis - currentNode.xaxis)
    dy = abs(nearestNode.yaxis - currentNode.yaxis)
    distanceVertical = Helper.DISTANCE_SHORT * abs(dx - dy) 
    distanceDiagonal = Helper.DISTANCE_LONG * min(dx, dy)
    distance = math.hypot(distanceVertical, distanceDiagonal)#sqrt(distanceDiagonal**2 + distanceVertical**2)
    noTurn : bool = False
    if dx == dy:#true means no Turn false means no Turn
        directionForDiagonal = getDirectionForDiagonal(nearestNode, currentNode)
        if directionForDiagonal == direction:
            noTurn = True
    elif dy == 0:
        if nearestNode.xaxis < currentNode.xaxis and direction == -2: # left side
            noTurn = True
        elif nearestNode.xaxis < currentNode.xaxis and direction == 2: # right side
            noTurn = True
    elif dx == 0:
        if nearestNode.yaxis < currentNode.yaxis and direction == 4: # top side
            noTurn = True
        elif nearestNode.xaxis < currentNode.xaxis and  direction == 0: # bottom side
            noTurn = True
    return distance, noTurn


def getDirectionForDiagonal(nearestNode, currentNode):
    if nearestNode.xaxis > currentNode.xaxis:
        if nearestNode.yaxis  > currentNode.yaxis: #bottom left corner
            return -1
        return -3 #top left corner
    if nearestNode.yaxis  > currentNode.yaxis:
        return 1#bottom right
    return 3

def calculateTimeToGoal(distance, noTurn, v0 : float):
    if noTurn:
        turn : float = 0
    else:
        turn : float = timeToSlowDown(v0, Helper.VMAX_TURN_MIDDLE, Helper.A)
        v0 = Helper.VMAX_TURN_MIDDLE
    return timeCalculation(v0, Helper.VMAX, Helper.A, distance) + turn
