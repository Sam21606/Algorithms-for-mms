from typing import List, Tuple
import math
import API
import sys
import Helper

def log(string: str) -> None:
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

def main() -> None:
    log("Starting...")
    API.setColor(0, 0, "G")
    API.setText(0, 0, "abc")
    currentNode: Helper.Node = Helper.STARTING_NODE
    onGoal: bool = False
    v0: float
    totalTime: float
    v0, totalTime = firstHalf()
    direction: int = 0
    openNodes: List[Helper.Node] = []
    openNodesDict = {}
    closedDict = {}
    pathToNode : List[Helper.Node] = []
    #API.moveForwardHalf()

    while not onGoal:
        log("New Calculation --------------------------------------------------------------------------------")
        currentNodes = []
        #log(f"These are the lists at the beningin current{currentNodes}, open{openNodes}, closedDict{closedDict}")
        currentNodes = getCurrentNodes(currentNode, direction, v0, pathToNode)
        for node in currentNodes:
            pos = (node.xaxis, node.yaxis)
            if pos in closedDict and node.cost < closedDict[pos].cost:
                openNodes.append(node)
                del closedDict[pos]
            elif pos not in closedDict and node not in openNodes:
                openNodes.append(node)
        log(f"WE are node {currentNode}")
        log(f"here are all open nodes {openNodes}")    
        log(f"these are the current Nodes{currentNodes}")
        newNode = determinFastetNode(openNodes, currentNode)
        log(f"this is the newNode={newNode}")
        #log(f"|||||||||||||||{direction}")
        #pathToNode = newNode.parent.pathToThisNode
        #log(f"here is the path")
        if currentNode is not Helper.STARTING_NODE:
            pathToNode = newNode.parent.pathToThisNode
        pathToNode, direction = moveToNewNodeOrParent(currentNode, newNode, direction, pathToNode)
        #log(f"|||||||||||||||{direction}")
        if currentNode in Helper.GOAL_NODES:
            onGoal = True
        if currentNode in openNodes:
            openNodes.remove(currentNode)
        if newNode in openNodes:
            openNodes.remove(newNode)
        closedDict[(currentNode.xaxis, currentNode.yaxis)] = currentNode
        #path.append(currentNode)
        currentNode = newNode

def moveToNewNodeOrParent(currentNode : Helper.Node, newNode : Helper.Node, direction : int, pathToThisNode : List[Helper.Node])-> Tuple[ List[Helper.Node], int]:
    #log(f"!!!!!!!!!!!!!!!!!!!!This is the newNode {newNode}, This is the pathToThisNode {pathToThisNode}")
    if checkIfNodeIsNext(currentNode, newNode):
        pathToThisNode.append(newNode)
        return pathToThisNode, moveToNewNode(currentNode, newNode, direction)
    
    newDirection = direction
    newCurrentNode = currentNode
    if newCurrentNode in pathToThisNode:
        pathToThisNode.remove(newCurrentNode)
        #log(f"11111111111111111111111111111 YES ITS ALSO REMOVED This is the newNode {newNode},This is the newCurrentNode{newCurrentNode}, This is the pathToThisNode {pathToThisNode}")
    while(newCurrentNode.parent not in pathToThisNode):
        #log(f"11111111111111111111111111111This is the newNode {newNode},This is the newCurrentNode{newCurrentNode}, This is the parentNode {newCurrentNode.parent}")
        newDirection = moveToNewNode(newCurrentNode, newCurrentNode.parent, newDirection)
        #if newCurrentNode in pathToThisNode:
            #pathToThisNode.remove(newCurrentNode) 
            #log(f"11111111111111111111111111111 YES ITS ALSO REMOVED This is the newNode {newNode},This is the newCurrentNode{newCurrentNode}, This is the pathToThisNode {pathToThisNode}")
        newCurrentNode = newCurrentNode.parent
    positionInPath = pathToThisNode.index(newCurrentNode.parent)
    if positionInPath < pathToThisNode.index(newNode.parent):
        x = 1
    else:
        x = -1
    while(newCurrentNode is not newNode.parent):
        #log(f"2222222222222222222222222222This is the newNode {newNode},This is the newCurrentNode{newCurrentNode}, Moving To {pathToThisNode[positionInPath]}, whole path{pathToThisNode}")
        newDirection = moveToNewNode(newCurrentNode, pathToThisNode[positionInPath], newDirection)
        newCurrentNode = pathToThisNode[positionInPath]
        positionInPath += x
    pathToThisNode.append(newCurrentNode)
    log(f"3333333333333333333333333333This is the newNode {newNode},This is the newCurrentNode{newCurrentNode}, This is the pathToThisNode {pathToThisNode}")
    return pathToThisNode, moveToNewNode(newCurrentNode, newNode, newDirection)

def checkIfNodeIsNext(currentNode : Helper.Node, newNode : Helper.Node) -> bool:
    try:
        dx = newNode.xaxis - currentNode.xaxis
        dy = newNode.yaxis - currentNode.yaxis
        direction_map[(dx, dy)]  
        return True
    except Exception as e:
        #log(f"moveToNewNodeDEBUG Error: {e}")
        return False
    
    
def firstHalf() -> Tuple[float, float]:
    return (
        speedAfterDistance(0, Helper.VMAX, Helper.A, Helper.DISTANCE_SHORT / 2),
        timeCalculation(0, Helper.VMAX, Helper.A, Helper.DISTANCE_SHORT / 2)
    )

def calculateCost(v0: float, direction: int, currentNode: Helper.Node, node: Helper.Node) -> Tuple[float, float]:
    heuristic: float = calculateHeuristic(currentNode, v0, direction)
    #log(f"The heuristic is {heuristic} for Node {currentNode}")
    return calculateTimeCost(v0, direction, currentNode, node) , heuristic

def calculateTimeCost(v0: float, direction: int, currentNode: Helper.Node, node: Helper.Node) -> float:
    turn = checkWhatTurn(direction, currentNode, node)
    newdirection: int = (direction + turn) % 8
    distance = getDistance(newdirection)

    if v0 == Helper.VMAX:
        if newdirection == direction:
            match newdirection:
                case 0 | -2 | 2 | 4:
                    return Helper.TIME_SHORT
                case _:
                    return Helper.TIME_LONG
        else:
            return timeWithoutTurn(v0, turn, distance) + 0
    else:
        if newdirection == direction:
            t1 = (Helper.VMAX - v0) / Helper.a
            accelerated_distance = v0 * t1 + 0.5 * Helper.a * t1 ** 2
            if accelerated_distance >= Helper.DISTANCE_SHORT:
                discriminant = v0**2 + 2 * Helper.a * Helper.DISTANCE_SHORT
                return (-v0 + math.sqrt(discriminant)) / Helper.a
            else:
                return t1 + (Helper.DISTANCE_SHORT - accelerated_distance) / Helper.VMAX
        else:
            return timeWithoutTurn(v0, turn, distance) + 0

def getDistance(newdirection: int) -> float:
    match newdirection:
        case 0 | -2 | 2 | 4:
            return Helper.DISTANCE_SHORT
        case _:
            return Helper.DISTANCE_LONG

def timeWithoutTurn(v0: float, turn: int, distance: float) -> float:
    if turn == 1:
        v_turn = Helper.VMAX_TURN45
        radius = Helper.RADIUS_TURN45
    elif turn == 2:
        v_turn = Helper.VMAX_TURN90
        radius = Helper.RADIUS_TURN90
    else:
        v_turn = Helper.VMAX_TURN135
        radius = Helper.RADIUS_TURN135

    # Calculate angle in radians (45°, 90°, 135°)
    theta = math.radians(45 * turn)
    
    # Arc length of the turn
    arc_length = radius * theta

    # Time to decelerate to turning speed
    decel_time = max(0, (v0 - v_turn) / Helper.DECEL)

    # Time to complete the turn arc at turn speed
    turn_time = arc_length / v_turn

    # Total time is deceleration + arc traversal
    return decel_time + turn_time


def checkWhatTurn(direction: int, currentNode: Helper.Node, node: Helper.Node) -> int:
    nodeDirection = checkTurn(currentNode, node)
    diff = abs(direction - nodeDirection)
    return diff % 5

def checkTurn(currentNode: Helper.Node, node: Helper.Node) -> int:
    dx = node.xaxis - currentNode.xaxis
    dy = node.yaxis - currentNode.yaxis
    return direction_map.get((dx, dy), 0)

def countTime(time: float, distance: float, speed: float) -> float:
    newTime: float = distance / speed + time
    #log(" COUNTTIME new run time is")
    return newTime

def checkCurrent() -> Helper.IsBlocked:
    #log("CHECKCURRENT checking Current Nodes...")
    stateAhead = Helper.IsBlocked(API.wallLeft(), API.wallRight(), API.wallFront(0))
    return stateAhead

def goBack() -> None:
    API.turnLeft()
    API.turnLeft()
    API.moveForward()
    API.turnLeft()
    API.turnLeft()

def speedAfterDistance(v0: float, vmax: float, a: float, d: float) -> float:
    v: float = math.sqrt(v0 * v0 + 2 * a * d)
    return vmax if v < vmax else v

def timeCalculation(v0: float, vmax: float, a: float, d: float) -> float:
    vend = speedAfterDistance(v0, vmax, a, d)
    if v0 == vmax or a == 0:
        return d / v0
    if vend != vmax:
        return (vend - v0) / a
    dAccel = (vmax**2 - v0**2) / (2 * a)
    return ((vmax - v0) / a) + ((d - dAccel) / vmax)

def distanceToSlowDown(v0: float, vmax: float, a: float) -> float:
    return (vmax**2 - v0**2) / (2 * a)

def timeToSlowDown(v0: float, vmax: float, a: float) -> float:
    return (vmax - v0) / a

def getCurrentNodes(currentNode: Helper.Node, direction: int, v0 : int, pathToThisNode : List[Helper.Node]) -> List[Helper.Node]:
    #log("getCurrentNodes geting Current Nodes")
    openNodes: List[Helper.Node] = []
    openDirections : Helper.IsBlocked = checkCurrent()
    movements = {"ahead": (0, 1), "left": (-1, 0), "right": (1, 0)}
    openList = []
    if not openDirections.left:
        #log("APPEND LEFT")
        openList.append("left")
    if not openDirections.right:
        #log("APPEND Right")
        openList.append("right")
    if not openDirections.ahead:
        #log("APPEND Ahead")
        openList.append("ahead")
    for key in openList:
        if not getattr(openDirections, key):
            #log(f"This is the curremt Node {currentNode}")
            mx, my = rotate(*movements[key], direction)
            #log(f"This is the mx={mx}, my={my}")
            openNodes.append(Helper.Node(currentNode.cost, currentNode.xaxis + mx, currentNode.yaxis + my))
            #log(f"This is the resulting Node in openNodes {openNodes[-1]}")
    return calculateCostForCurrentNodes(v0, direction, openNodes, currentNode, pathToThisNode)

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

def checkAhead() -> Helper.IsBlocked:
    if not API.wallFront:
        API.moveForward()
        a = checkCurrent()
        goBack()
    else:
        a : Helper.IsBlocked = Helper.IsBlocked(False, False, False)
        if not API.wallLeft():
            API.turnLeft()
            API.moveForward
            a.left = API.wallRight()
            API.turnLeft()
            API.turnLeft()
            API.moveForward()
            API.turnLeft
        if not API.wallRight():
            API.turnRight()
            API.moveForward
            a.Right = API.wallLeft()
            API.turnRight()
            API.turnRight()
            API.moveForward()
            API.turnRight
    return a

def calculateCostForCurrentNodes(v0: float, direction: int, openNodes: List[Helper.Node], currentNode: Helper.Node, pathToThisNode : List[Helper.Node]) -> List[Helper.Node]:
    #log(f"these are all openodes {openNodes}")
    for node in openNodes:
        #log(f"This is the current {node}")
        time : float = 0
        heuristic: float = 0
        time , heuristic = calculateCost(v0, direction, currentNode, node)
        node.cost = time + heuristic + currentNode.time
        node.time = time + currentNode.time
        node.parent = currentNode
        node.pathToThisNode = pathToThisNode + [node]
    return openNodes

def determinFastetNode(currentNodes: List[Helper.Node], currentNode : Helper.Node) -> Helper.Node:
    #log(f"determinFastetNode These are all currentNodes={currentNodes}")
    currentFastest = Helper.Node(10000, 0, 0)
    for node in currentNodes:
        if node.cost < currentFastest.cost and node is not currentNode:
            currentFastest = node
    #log(f"determinFastetNodeThis is the currentFastest={currentFastest}")
    return currentFastest

def calculateHeuristic(currentNode: Helper.Node, v0: float, direction: int) -> float:
    nearestNode = calculateNearestNode(currentNode)
    #log(f"The nearest Node is {nearestNode}")
    distance, noTurn = calculateDistanceTwoNodes(nearestNode, currentNode, direction)
    return calculateTimeToGoal(distance, noTurn, v0)

def calculateNearestNode(currentNode: Helper.Node) -> Helper.Node:
    nearest = min(Helper.GOAL_NODES, key=lambda node: currentNode.distance_to(node))
    return nearest

def calculateDistanceTwoNodes(nearestNode: Helper.Node, currentNode: Helper.Node, direction: int) -> Tuple[float, bool]:
    dx = abs(nearestNode.xaxis - currentNode.xaxis)
    dy = abs(nearestNode.yaxis - currentNode.yaxis)
    distanceVertical = Helper.DISTANCE_SHORT * abs(dx - dy)
    distanceDiagonal = Helper.DISTANCE_LONG * min(dx, dy)
    #distance = math.hypot(distanceVertical, distanceDiagonal)
    distance = distanceDiagonal + distanceVertical
    noTurn: bool = False

    if dx == dy:
        directionForDiagonal = getDirectionForDiagonal(nearestNode, currentNode)
        if directionForDiagonal == direction:
            noTurn = True
    elif dy == 0:
        if nearestNode.xaxis < currentNode.xaxis and direction in (-2, 2):
            noTurn = True
    elif dx == 0:
        if nearestNode.yaxis < currentNode.yaxis and direction == 4:
            noTurn = True
        elif nearestNode.xaxis < currentNode.xaxis and direction == 0:
            noTurn = True

    return distance, noTurn

def getDirectionForDiagonal(nearestNode: Helper.Node, currentNode: Helper.Node) -> int:
    if nearestNode.xaxis > currentNode.xaxis:
        if nearestNode.yaxis > currentNode.yaxis:
            return -1
        return -3
    if nearestNode.yaxis > currentNode.yaxis:
        return 1
    return 3

def calculateTimeToGoal(distance: float, noTurn: bool, v0: float) -> float:
    turn: float = 0 if noTurn else timeToSlowDown(v0, Helper.VMAX_TURN_MIDDLE, Helper.A)
    if not noTurn:
        v0 = Helper.VMAX_TURN_MIDDLE
    return timeCalculation(v0, Helper.VMAX, Helper.A, distance) + turn

def moveToNewNode(currentNode : Helper.Node, newNode : Helper.Node, direction : int)-> int:
    #log(f"moveToNewNode current node {currentNode} new Node {newNode}")
    dx = newNode.xaxis - currentNode.xaxis
    dy = newNode.yaxis - currentNode.yaxis
    #log(f"moveToNewNodeDEBUG moveToNewNode: dx={dx}, dy={dy}, direction={direction}")
    newdirection = direction_map.get((dx, dy))
    if newdirection is None:
        log(f"moveToNewNodeERROR: Kein Mapping für Vektor ({dx},{dy})!")
    if newdirection == direction:
        API.moveForward()
    else:
        #log(f"moveToNewNodehere is all info direction{direction} newDirection{newdirection}")
        start_angle = direction_angles[direction]
        end_angle = direction_angles[newdirection]

        # Clockwise and counterclockwise angles
        right_turn = (end_angle - start_angle) % 360
        left_turn = (start_angle - end_angle) % 360

        # Decide the shorter turn direction
        if right_turn <= left_turn:
            while right_turn >= 90:
                API.turnRight90()
                right_turn -= 90
        else:
            while left_turn >= 90:
                API.turnLeft90()
                left_turn -= 90
        API.moveForward()
    return newdirection

    

direction_map = {
    (0, 1): 0, #north
    (1, 0): -2, #east
    (-1, 0): 2,# west
    (0, -1): 4, #south
    (1, 1): -1, #Northeast
    (1, -1): -3, #Southeast
    (-1, -1): 3, #Southwest
    (-1, 1): 1 #Northwest
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

if __name__ == "__main__":
    main()