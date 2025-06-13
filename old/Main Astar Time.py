from typing import List, Tuple
import math
import API
import sys
import Helper

def log(string: str) -> None:
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

def main() -> None:
   #log("Starting...")
    API.setColor(0, 0, "G")
    API.setText(0, 0, "abc")
    currentNode: Helper.Node = Helper.STARTING_NODE
    onGoal: bool = False
    v0: float = 0
    totalTime: float
    direction: int = 0
    openNodesDict = {}
    closedDict = {}
    pathToNode : List[Helper.Node] = []

    while not onGoal:
       #log("New Calculation --------------------------------------------------------------------------------")
       #log(f"WE are node {currentNode}")
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

       #log(f"here are all open nodes {list(openNodesDict.values())}")    
       #log(f"these are the current Nodes{currentNodes}")
        newNode = determinFastetNode(list(openNodesDict.values()), currentNode)
       #log(f"this is the newNode={newNode}")
        #log(f"|||||||||||||||{direction}")
        #pathToNode = newNode.parent.pathToThisNode
        #log(f"here is the path")

        if currentNode is not Helper.STARTING_NODE:
            pathToNode = newNode.parent.pathToThisNode
        pathToNode, direction = moveToNewNodeOrParent(currentNode, newNode, direction, pathToNode)
        v0 = newNode.v0
        #log(f"|||||||||||||||{direction}")

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
           #log(f"Shortest path is {currentNode.pathToThisNode}")
            writeFinishedPath(currentNode.pathToThisNode)
            pathToNode = newNode.parent.pathToThisNode
            pathToNode, direction = moveToNewNodeOrParent(currentNode, Helper.STARTING_NODE, direction, pathToNode)
            pathToNode, direction = moveToNewNodeOrParent(Helper.STARTING_NODE, currentNode, direction, pathToNode)


def writeInfo(openNodesDict , closedDict):
    for node in openNodesDict.values():
        API.setColor(node.xaxis, node.yaxis, 'G')
        API.setText(node.xaxis, node.yaxis, str(node.cost))

    for node in closedDict.values():
        API.setColor(node.xaxis, node.yaxis, "R")
        API.setText(node.xaxis, node.yaxis, str(node.cost))

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
        API.setColor(node.xaxis, node.yaxis, 'B')
        API.setText(node.xaxis, node.yaxis, str(node.cost))

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
        API.setWall(currentNode.xaxis, currentNode.yaxis, dir_char)

def moveToNewNodeOrParent(currentNode : Helper.Node, newNode : Helper.Node, direction : int, pathToThisNode : List[Helper.Node])-> Tuple[ List[Helper.Node], int]:
    #log(f"!!!!!!!!!!!!!!!!!!!!This is the newNode {newNode}, This is the pathToThisNode {pathToThisNode}")
    if checkIfNodeIsNext(currentNode, newNode):
       #log("Wee should not be here at this node")
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
   #log(f"3333333333333333333333333333This is the newNode {newNode},This is the newCurrentNode{newCurrentNode}, This is the pathToThisNode {pathToThisNode}")
    return pathToThisNode, moveToNewNode(newCurrentNode, newNode, newDirection)

def checkIfNodeIsNext(currentNode : Helper.Node, newNode : Helper.Node) -> bool:
    if currentNode != newNode.parent:
        return False
    return True

def calculateCost(v0: float, direction: int, currentNode: Helper.Node, node: Helper.Node) -> Tuple[float, float, float]:
    heuristic: float = calculateHeuristic(node, v0, direction)
    log(f"The heuristic is {heuristic} for Node {node}")
    vOne, vTwo = calculateTimeCost(v0, direction, currentNode, node)
    return vOne, vTwo , heuristic

def calculateTimeCost(v0: float, direction: int, currentNode: Helper.Node, node: Helper.Node) -> Tuple[float, float]:
    dx = node.xaxis - currentNode.xaxis
    dy = node.yaxis - currentNode.yaxis
    newdirection = direction_map.get((dx, dy))
    turn = checkWhatTurn(direction, currentNode, node)
    #newdirection: int = (direction + turn) % 8
    #log(f"this newdirection {newdirection} and this direction {direction}")
    distance = getDistance(newdirection)

    if v0 == Helper.VMAX:
        if newdirection == direction:
            match newdirection:
                case 0 | -2 | 2 | 4:
                    log("1")
                    return Helper.TIME_SHORT, v0
                case _:
                    log("2")
                    return Helper.TIME_LONG, v0
        else:
            log("3")
            return timeWithTurn(v0, turn, distance)
    else:
        if newdirection == direction:
            log(f"444 ")#{timeCalculation(v0, Helper.VMAX, Helper.A, Helper.DISTANCE_SHORT)}")
            return timeCalculation(v0, Helper.VMAX, Helper.A, Helper.DISTANCE_SHORT)
        else:
            log("5")
            return timeWithTurn(v0, turn, distance)

def getDistance(newdirection: int) -> float:
    match newdirection:
        case 0 | -2 | 2 | 4:
            return Helper.DISTANCE_SHORT
        case _:
            return Helper.DISTANCE_LONG

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
    log(f"the vmx is {vmax}, this vo {v0}, this a {a}")
    v: float = math.sqrt(v0 * v0 + 2 * a * d)
    #log(f"This is new v= {vmax if v > vmax else v}")
    #log(f"speed for v0 = 0 => {math.sqrt(0 * 0 + 2 * 15 * Helper.DISTANCE_SHORT)}")
    return Helper.VMAX if v > vmax else v

def timeCalculation(v0: float, vmax: float, a: float, d: float) -> Tuple[float, float]:
    vend = speedAfterDistance(v0, vmax, a, d)
    if v0 == vmax or a == 0:
        return d / v0, vmax
    if vend != vmax:
        return (vend - v0) / a, vend
    dAccel = (vmax**2 - v0**2) / (2 * a)
    #log(f" 1 term {((vmax - v0) / a) + ((d - dAccel) / vmax)} vend ={vend}")
    return ((vmax - v0) / a) + ((d - dAccel) / vmax), vend

def timeToSlowDown(v0: float, vmax: float, a: float) -> float:
    if v0 < vmax:
        return 0
    return (v0 - vmax) / a

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
    writeWalls(openList, direction, currentNode)
    for key in openList:
        if not getattr(openDirections, key):
            #log(f"This is the curremt Node {currentNode}")
            mx, my = rotate(*movements[key], direction)
            #log(f"This is the mx={mx}, my={my}")
            openNodes.append(Helper.Node(currentNode.cost, currentNode.xaxis + mx, currentNode.yaxis + my))
            #log(f"This is the resulting Node in openNodes {openNodes[-1]}")
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

def calculateCostForCurrentNodes(v0: float, direction: int, openNodes: List[Helper.Node], currentNode: Helper.Node) -> List[Helper.Node]:
    #log(f"these are all openodes {openNodes}")
    for node in openNodes:
       #log(f"This is the NODE {node}")
        time : float = 0
        heuristic : float = 0
        time, v0, heuristic = calculateCost(v0, direction, currentNode, node)
       #log(f"CURRENT Data Node = {node}, time {time}, heuristic {heuristic}, currentNode.time {currentNode.time}")
        node.cost = time + heuristic + currentNode.time
       #log(f"RESULTING in node.cost{node.cost}")
        node.time = time + currentNode.time
        node.parent = currentNode
        path : List[Helper.Node] = currentNode.pathToThisNode
        node.pathToThisNode = path + [node]
       #log(f"This is the NODE {node}")
       #log(f"we are at x={currentNode.xaxis} y={currentNode.yaxis}")
       #log(f"going to x={node.xaxis} y={node.yaxis}")
       #log(f" with path {currentNode.pathToThisNode}")
       #log(f" new path is {node.pathToThisNode} ")
        node.heuristic = heuristic
        node.v0 = v0
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
    log(f"The nearest Node is {nearestNode}")
    dx , dy, turn, dxOrdy, = calculateDistanceTwoNodes(nearestNode, currentNode, direction)
    log(f"heuristic dx is{dx} we have dy{dy}")
    return calculateTimeToGoal(dx, dy, turn, v0, dxOrdy)
    #return distance

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


    # if dx == dy:
    #     directionForDiagonal = getDirectionForDiagonal(nearestNode, currentNode)
    #     if directionForDiagonal == direction:
    #         noTurn = True
    if dy == 0:
        turn = 0
    elif dx == 0:
        turn = 0
    # elif currentNode.xaxis < nearestNode.xaxis and currentNode.yaxis < nearestNode.yaxis and direction is not 4 or 2: # 2 is wesr
    #     turn = 1
    # elif currentNode.xaxis < nearestNode.xaxis and currentNode.yaxis > nearestNode.yaxis and direction is not 0 or 2:
    #     turn = 1
    # elif currentNode.xaxis > nearestNode.xaxis and currentNode.yaxis < nearestNode.yaxis and direction is not 0 or -2:
    #     turn = 1
    # elif currentNode.xaxis > nearestNode.xaxis and currentNode.yaxis > nearestNode.yaxis and direction is not 4 or -2:
    #     turn = 1

    return dx, dy, turn, dxOrdy

def getDirectionForDiagonal(nearestNode: Helper.Node, currentNode: Helper.Node) -> int:
    if nearestNode.xaxis > currentNode.xaxis:
        if nearestNode.yaxis > currentNode.yaxis:
            return -1
        return -3
    if nearestNode.yaxis > currentNode.yaxis:
        return 1
    return 3

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
        log(f" this is the result{timeCalculation(v0, Helper.VMAX, Helper.A, first)}")
        timeBeforeDecel, vend = timeCalculation(Helper.VMAX, Helper.VMAX, Helper.A, first)
        timeForDecel += timeToSlowDown(vend, Helper.VMAX_TURN_MIDDLE, Helper.A)
        timeAfterDecel, vunimportant = timeCalculation(Helper.VMAX, Helper.VMAX, Helper.A, second)
        return timeBeforeDecel + timeForDecel + timeAfterDecel  
    time, v0  = timeCalculation(v0, Helper.VMAX, Helper.A, dx + dy)
    return time

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
    #(1, 1): -1, #Northeast
    #(1, -1): -3, #Southeast
    #(-1, -1): 3, #Southwest
    #(-1, 1): 1 #Northwest
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