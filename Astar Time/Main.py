from typing import List, Tuple
import math
import mice_API
import sys
import Helper

def log(string: str) -> None:
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

def main() -> None:
    log("Starting...")
    mice_API.setColor(0, 0, "G")
    mice_API.setText(0, 0, "abc")
    currentNode: Helper.Node = Helper.STARTING_NODE
    onGoal: bool = False
    v0: float = 0
    totalTime: float
    direction: int = 0
    openNodesDict = {}
    closedDict = {}
    pathToNode : List[Helper.Node] = []

    while not onGoal:
        currentNodes = []
        currentNodes = getCurrentNodes(currentNode, direction, v0, pathToNode)
        for node in currentNodes:
            pos = (node.xaxis, node.yaxis)
            if pos in closedDict and node.cost < closedDict[pos].cost:
                openNodesDict[pos] = node
                del closedDict[pos]
            elif pos not in closedDict:
                if pos not in openNodesDict or node.cost < openNodesDict[pos].cost:
                    openNodesDict[pos] = node
        writeInfo(openNodesDict , closedDict)
        newNode = determinFastetNode(list(openNodesDict.values()), currentNode)

        if currentNode is not Helper.STARTING_NODE:
            pathToNode = newNode.parent.pathToThisNode
        pathToNode, direction = moveToNewNodeOrParent(currentNode, newNode, direction, pathToNode)
        v0 = newNode.v0

        pos_current = (currentNode.xaxis, currentNode.yaxis)
        pos_new = (newNode.xaxis, newNode.yaxis)
        if pos_current in openNodesDict:
            del openNodesDict[pos_current]
        if pos_new in openNodesDict:
            del openNodesDict[pos_new]
        closedDict[pos_current] = currentNode
        currentNode = newNode
        if (currentNode.xaxis, currentNode.yaxis) in Helper.GOAL_NODES:
            onGoal = True
            writeFinishedPath(currentNode.pathToThisNode)
            log(f"The Time would be {currentNode.time:.2f}s")

            

def moveToNewNodeOrParent(currentNode : Helper.Node, newNode : Helper.Node, direction : int, pathToThisNode : List[Helper.Node])-> Tuple[ List[Helper.Node], int]:
    if checkIfNodeIsNext(currentNode, newNode):
        pathToThisNode.append(newNode)
        return pathToThisNode, moveToNewNode(currentNode, newNode, direction)
        
    newDirection = direction
    newCurrentNode = currentNode

    if newCurrentNode in pathToThisNode:
        pathToThisNode.remove(newCurrentNode)
    while(newCurrentNode.parent not in pathToThisNode):
        newDirection = moveToNewNode(newCurrentNode, newCurrentNode.parent, newDirection)
        newCurrentNode = newCurrentNode.parent
    positionInPath = pathToThisNode.index(newCurrentNode.parent)
    if positionInPath < pathToThisNode.index(newNode.parent):
        x = 1
    else:
        x = -1
    while(newCurrentNode is not newNode.parent):
        newDirection = moveToNewNode(newCurrentNode, pathToThisNode[positionInPath], newDirection)
        newCurrentNode = pathToThisNode[positionInPath]
        positionInPath += x
    pathToThisNode.append(newCurrentNode)
    return pathToThisNode, moveToNewNode(newCurrentNode, newNode, newDirection)

def checkIfNodeIsNext(currentNode : Helper.Node, newNode : Helper.Node) -> bool:
    if currentNode != newNode.parent:
        return False
    return True

def calculateCost(v0: float, direction: int, currentNode: Helper.Node, node: Helper.Node) -> Tuple[float, float, float]:
    heuristic: float = calculateHeuristic(node, v0, direction)
    vOne, vTwo = calculateTimeCost(v0, direction, currentNode, node)
    return vOne, vTwo , heuristic

def calculateTimeCost(v0: float, direction: int, currentNode: Helper.Node, node: Helper.Node) -> Tuple[float, float]:
    dx = node.xaxis - currentNode.xaxis
    dy = node.yaxis - currentNode.yaxis
    newdirection = direction_map.get((dx, dy))
    turn = checkWhatTurn(direction, currentNode, node)
    distance = Helper.DISTANCE_SHORT

    if v0 == Helper.VMAX:
        if newdirection == direction:
            match newdirection:
                case 0 | -2 | 2 | 4:
                    return Helper.TIME_SHORT, v0
                case _:
                    return Helper.TIME_LONG, v0
        else:
            return timeWithTurn(v0, turn, distance)
    else:
        if newdirection == direction:
            return timeCalculation(v0, Helper.VMAX, Helper.A, Helper.DISTANCE_SHORT)
        else:
            return timeWithTurn(v0, turn, distance)

def timeWithTurn(v0: float, turn: int, distance: float) -> Tuple[float, float]:
    radius = Helper.RADIUS_TURN90
    v_turn = Helper.VMAX_TURN90

    arc_length = radius * math.radians(90)
    decel_time = timeToSlowDown(v0,v_turn, Helper.DECEL)
    turn_time = arc_length / v_turn
    total_time = decel_time + turn_time

    return total_time, v_turn

def checkWhatTurn(direction: int, currentNode: Helper.Node, node: Helper.Node) -> int:
    nodeDirection = checkTurn(currentNode, node)
    diff = abs(direction - nodeDirection)
    return diff % 5

def checkTurn(currentNode: Helper.Node, node: Helper.Node) -> int:
    dx = node.xaxis - currentNode.xaxis
    dy = node.yaxis - currentNode.yaxis
    return direction_map.get((dx, dy), 0)

def checkCurrent() -> Helper.IsBlocked:
    return Helper.IsBlocked(mice_API.wallLeft(), mice_API.wallRight(), mice_API.wallFront(0))

def speedAfterDistance(v0: float, vmax: float, a: float, d: float) -> float:
    v: float = math.sqrt(v0 * v0 + 2 * a * d)
    return Helper.VMAX if v > vmax else v

def timeCalculation(v0: float, vmax: float, a: float, d: float) -> Tuple[float, float]:
    vend = speedAfterDistance(v0, vmax, a, d)
    if v0 == vmax or a == 0:
        return d / v0, vmax
    if vend != vmax:
        return (vend - v0) / a, vend
    dAccel = (vmax**2 - v0**2) / (2 * a)
    return ((vmax - v0) / a) + ((d - dAccel) / vmax), vend

def timeToSlowDown(v0: float, vmax: float, a: float) -> float:
    if v0 < vmax:
        return 0
    return (v0 - vmax) / a

def getCurrentNodes(currentNode: Helper.Node, direction: int, v0 : int, pathToThisNode : List[Helper.Node]) -> List[Helper.Node]:
    openNodes: List[Helper.Node] = []
    openDirections : Helper.IsBlocked = checkCurrent()
    movements = {"ahead": (0, 1), "left": (-1, 0), "right": (1, 0)}
    openList = []
    if not openDirections.left:
        openList.append("left")
    if not openDirections.right:
        openList.append("right")
    if not openDirections.ahead:
        openList.append("ahead")
    writeWalls(openList, direction, currentNode)
    for key in openList:
        if not getattr(openDirections, key):
            mx, my = rotate(*movements[key], direction)
            openNodes.append(Helper.Node(currentNode.cost, currentNode.xaxis + mx, currentNode.yaxis + my))
    return calculateCostForCurrentNodes(v0, direction, openNodes, currentNode)

def rotate(dx: int, dy: int, direction: int) -> Tuple[int, int]:
    # direction: 0 = North, -2 = East, 4 = South, 2 = West
    # Mapping of rotation based on the mouse's facing direction
    if direction == 0:     # North
        return dx, dy
    elif direction == -2:  # East
        return dy, -dx
    elif direction == 4:   # South
        return -dx, -dy
    elif direction == 2:   # West
        return -dy, dx
    else:
        raise ValueError(f"Unsupported direction {direction}")

def calculateCostForCurrentNodes(v0: float, direction: int, openNodes: List[Helper.Node], currentNode: Helper.Node) -> List[Helper.Node]:
    for node in openNodes:
        time : float = 0
        heuristic : float = 0
        time, v0, heuristic = calculateCost(v0, direction, currentNode, node)
        node.cost = time + heuristic + currentNode.time
        node.time = time + currentNode.time
        node.parent = currentNode
        path : List[Helper.Node] = currentNode.pathToThisNode
        node.pathToThisNode = path + [node]
        node.v0 = v0
    return openNodes

def determinFastetNode(currentNodes: List[Helper.Node], currentNode : Helper.Node) -> Helper.Node:
    currentFastest = Helper.Node(10000, 0, 0)
    for node in currentNodes:
        if node.cost < currentFastest.cost and node is not currentNode:
            currentFastest = node
    return currentFastest

def calculateHeuristic(currentNode: Helper.Node, v0: float, direction: int) -> float:
    nearestNode = calculateNearestNode(currentNode)
    dx , dy, turn, dxOrdy, = calculateDistanceTwoNodes(nearestNode, currentNode, direction)
    return calculateTimeToGoal(dx, dy, turn, v0, dxOrdy)

def calculateNearestNode(currentNode: Helper.Node) -> Helper.Node:
    nearest = min(Helper.GOAL_NODES.values(), key=lambda node: currentNode.distance_to(node))
    return nearest

def calculateDistanceTwoNodes(nearestNode: Helper.Node, currentNode: Helper.Node, direction: int) -> Tuple[float,float, int, bool]:
    dx = abs(nearestNode.xaxis - currentNode.xaxis)
    dy = abs(nearestNode.yaxis - currentNode.yaxis)
    turn: int = 1
    dxOrdy : bool = False
    if direction == -2 or direction == 2:
        dxOrdy : bool = True

    if dy == 0:
        turn = 0
    elif dx == 0:
        turn = 0

    return dx, dy, turn, dxOrdy

def calculateTimeToGoal(dx: float, dy : float, turn: int, v0: float, dxOrdy) -> float: # if dx true else false
    timeForDecel : float = 0
    dx *= Helper.DISTANCE_SHORT
    dy *= Helper.DISTANCE_SHORT
    if turn == 1 :
        if dxOrdy:
            first : float = dx
            second : float = dy
        else:
            first : float = dy
            second : float = dx
        timeBeforeDecel : float = 0
        vend : float = 0
        timeAfterDecel : float = 0
        timeBeforeDecel, vend = timeCalculation(Helper.VMAX, Helper.VMAX, Helper.A, first)
        timeForDecel += timeToSlowDown(vend, Helper.VMAX_TURN_MIDDLE, Helper.A)
        timeAfterDecel, vunimportant = timeCalculation(Helper.VMAX, Helper.VMAX, Helper.A, second)
        return timeBeforeDecel + timeForDecel + timeAfterDecel  
    time, v0  = timeCalculation(v0, Helper.VMAX, Helper.A, dx + dy)
    return time

def moveToNewNode(currentNode : Helper.Node, newNode : Helper.Node, direction : int)-> int:
    dx = newNode.xaxis - currentNode.xaxis
    dy = newNode.yaxis - currentNode.yaxis
    newdirection = direction_map.get((dx, dy))
    if newdirection == direction:
        mice_API.moveForward()
    else:
        start_angle = direction_angles[direction]
        end_angle = direction_angles[newdirection]

        # Clockwise and counterclockwise angles
        right_turn = (end_angle - start_angle) % 360
        left_turn = (start_angle - end_angle) % 360

        # Decide the shorter turn direction
        if right_turn <= left_turn:
            while right_turn >= 90:
                mice_API.turnRight90()
                right_turn -= 90
        else:
            while left_turn >= 90:
                mice_API.turnLeft90()
                left_turn -= 90
        mice_API.moveForward()
    return newdirection

    

direction_map = {
    (0, 1): 0, #north
    (1, 0): -2, #east
    (-1, 0): 2,# west
    (0, -1): 4, #south
}


direction_angles = {
    0: 0,     # North
    -1: 45,   # Northeast
    -2: 90,   # East
    -3: 135,  # Southeast
    4: 180,   # South
    3: 225,   # Southwest
    2: 270,   # West
    1: 315    # Northwest
}

def writeInfo(openNodesDict , closedDict):
    for node in openNodesDict.values():
        mice_API.setColor(node.xaxis, node.yaxis, 'G')
        mice_API.setText(node.xaxis, node.yaxis, str(node.cost))

    for node in closedDict.values():
        mice_API.setColor(node.xaxis, node.yaxis, "R")
        mice_API.setText(node.xaxis, node.yaxis, str(node.cost))

def writeWalls(openList : List[str], direction : int, currentNode : Helper.Node):
    # Absolute directions in clockwise order
    absolute_dirs = [0, -2, 4, 2]  # North, East, South, West
    dir_to_char = {0: 'n', -2: 'e', 4: 's', 2: 'w'}

    # Relative directions with their index offsets
    relative_offsets = {
        'ahead': 0,
        'right': 1,
        'back': 2,
        'left': -1,
    }

def writeFinishedPath(pathToThisNode : List[Helper.Node]):
    for node in pathToThisNode:
        mice_API.setColor(node.xaxis, node.yaxis, 'B')
        mice_API.setText(node.xaxis, node.yaxis, str(node.cost))

def writeWalls(openList : List[str], direction : int, currentNode : Helper.Node):
    # Absolute directions in clockwise order
    absolute_dirs = [0, -2, 4, 2]  # North, East, South, West
    dir_to_char = {0: 'n', -2: 'e', 4: 's', 2: 'w'}

    # Relative directions with their index offsets
    relative_offsets = {
        'ahead': 0,
        'right': 1,
        'back': 2,
        'left': -1,
    }

    # Find the index of the current absolute direction
    facing_idx = absolute_dirs.index(direction)

    # Compute set of relative directions that have no wall (open)
    open_set = set(openList)
    all_rels = {'ahead', 'left', 'right'}
    wall_rels = all_rels - open_set  # directions where there IS a wall

    for rel_dir in wall_rels:
        offset = relative_offsets[rel_dir]
        abs_idx = (facing_idx + offset) % 4
        abs_dir = absolute_dirs[abs_idx]
        dir_char = dir_to_char[abs_dir]
        mice_API.setWall(currentNode.xaxis, currentNode.yaxis, dir_char)

if __name__ == "__main__":
    main()