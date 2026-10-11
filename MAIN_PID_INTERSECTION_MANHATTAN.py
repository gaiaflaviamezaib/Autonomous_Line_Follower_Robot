import time
import numpy as np
from collections import deque
import threading
import traceback
import os
import argparse


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run the robot in Manhattan-grid mode or normal line-following mode."
    )
    parser.add_argument(
        "--mode",
        choices=["grid", "line"],
        default="grid",
        help="grid uses intersections/Manhattan path planning; line follows curves or any continuous line.",
    )
    parser.add_argument(
        "--no-blinkers",
        action="store_true",
        help="Disable turn signal LEDs during intersection turns.",
    )
    parser.add_argument(
        "--preset",
        choices=["safe", "balanced", "curve", "fast", "power"],
        default="balanced",
        help="Driving/PID preset. Start with balanced; use power only after safe testing.",
    )
    return parser.parse_args()


ARGS = parse_args()

from robotControlClass import robot
import manhattan_graph_dijkstra as grid_dijkstra

def save_list_npz(values, folder_name="data", filename="values.npz"): #for PID error graphing
    """
    Save a list of numbers to a .npz file inside a folder
    located in the same directory as the current script.
    """
    # Directory where the script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Create folder if it doesn't exist
    folder_path = os.path.join(script_dir, folder_name)
    os.makedirs(folder_path, exist_ok=True)

    # Full file path
    file_path = os.path.join(folder_path, filename)

    # Save under a known key, e.g. "values"
    np.savez(file_path, values=np.array(values))

    return file_path

def getError(infraState, lastNonNullSideState):

    if infraState == [0, 1, 0]:
        error = 0

    elif infraState == [1, 1, 0]:
        error = smallError

    elif infraState == [1, 0, 0]:
        error = mediumError

    elif infraState == [0, 1, 1]:
        error = -smallError

    elif infraState == [0, 0, 1]:
        error = -mediumError

    elif infraState == [0, 0, 0] and lastNonNullSideState[2] != 0: #no line, last detection was on left of line
        assert lastNonNullSideState[0] == 0

        error = -bigError

    elif infraState == [0, 0, 0] and lastNonNullSideState[0] != 0: #no line, last detection was on right of line
        assert lastNonNullSideState[2] == 0

        error = bigError

    else: #shouldnt happen
        error = 0
        # raise ValueError(f"InfraState {infraState} was passed to error! (line 42)")
        # print(f"!!!InfraState {infraState} was passed to error! (line 42)!!!")

    return error

def convertOutputToAngle(output):

    return output/maxOutput * 25 #angle proportional to output, between -25 and 25 (max vals for wheel turn)

def convertOutputToSpeed(output):

    # return constantSpeed
    speed = -(maxSpeed-minSpeed)/maxOutput*abs(output) + maxSpeed #speed inversely proportional to output, between maxSpeed and minSpeed
    return max(minSpeed, min(maxSpeed, speed))

def getObstacleDist(): #tochange
    # global obstacleDist

    # s = time.perf_counter()
    # print(time.perf_counter()-testTime)
    # s = time.perf_counter()
    return robot.measureDistance()

def sample_min_distance(duration, min_valid_distance=0):
    """
    Collect ultrasonic measurements for duration seconds and return the nearest
    valid reading. Returns None if the sensor produces no valid readings.
    """
    dists = []
    start = time.time()
    while time.time() - start < duration:
        dist = robot.measureDistance()

        if dist is not None and dist > min_valid_distance:
            dists.append(dist)

    if not dists:
        return None

    return min(dists)

    # print(time.perf_counter()-s)

    # print()

    # if dist is not None:

    #     return dist < distToStop

    # return False

def backwardsTillIntersection(): #straight backwards till intersection detected
    
    robot.turnWheels(0)
    robot.stopMotors()
    time.sleep(.25)
    
    robot.runMotors(1, 22)
    while robot.getInfraredState(False) != [1, 1, 1]:
        time.sleep(.1)
        
    robot.stopMotors()
    
    time.sleep(.25)

def uTurnVer3():
    
    robot.stopMotors()
    time.sleep(.2)
    for i in range(4):
        robot.turnWheels(-25)
        robot.runMotors(1, 30)
        
        time.sleep(1)
        
        robot.stopMotors()
        time.sleep(.2)
        
        if i == 3:
            break
        robot.turnWheels(25)
        robot.runMotors(0, 30)
        
        time.sleep(.75)
    
    time.sleep(.25)
    
    robot.turnWheels(-7)
    robot.runMotors(0, 25)
    
    while not robot.getInfraredState(False)[1]: #loop till line detected by middle sensor
        time.sleep(.1)

def tankTurn(raw_output): #to be threaded

    global runningTankTurn

    try:
        sign = np.sign(raw_output)

        if sign < 0:
            index = 2
            leftDir = 0
            rightDir = 1
        elif sign > 0:
            index = 0
            leftDir = 1
            rightDir = 0
        else:
            raise ValueError("raw_output is exactly 0. (line 107)")

        switch = True
        lastSwitch = time.time()
        while not robot.getInfraredState(False)[index]:

            if time.time() - lastSwitch > .3: #only drifts for .3 secs at a time
                switch = not(switch)
                lastSwitch = time.time()
                # if not switch:
                #     lastSwitch = time.time()-.1
            if switch:
                robot.runLeftMotor(leftDir, 100)
                robot.runRightMotor(rightDir, 100)
            else:
                time.sleep(.01)
    finally:
        robot.stopMotors()
        runningTankTurn = False

def turn90DegGrid(dir):
    
    backTurnCoeff = 25
    frontTurnCoeff = 15
    
    frontSpeed = 25
    backSpeed = 40
    
    if dir == "left":
        
        robot.turnWheels(-backTurnCoeff)
        
        robot.runMotors(1, backSpeed)
        
        time.sleep(.55)
        
        robot.turnWheels(frontTurnCoeff)
        robot.runMotors(0, frontSpeed)
        
        # time.sleep(1)
        # while not any(robot.getInfraredState(False)):# == 0:
        #     time.sleep(.01)
        
    elif dir == "right":
        
        robot.turnWheels(backTurnCoeff)
        
        robot.runMotors(1, backSpeed)
        
        time.sleep(.55)
        
        robot.turnWheels(-frontTurnCoeff)
        robot.runMotors(0, frontSpeed)
        
    time.sleep(1)
    while not any(robot.getInfraredState(False)):
        time.sleep(.01)
              
def isObstacle_intersection(dir): #checks if obstacle in next turn direction (except backwards), if there is obstacle waits 5 secs to see if move 

    robot.stopMotors()

    if dir == "left":
        robot.turnHeadHoriz(80)

    elif dir == "right":
        robot.turnHeadHoriz(-80)

    time.sleep(1.5)

    dist = sample_min_distance(.5, minObstacleDetectDist)

    if dist is None:
        robot.turnHeadHoriz(0)
        time.sleep(1.5)
        return False

    if dist > maxObstacleDetectDist:
        robot.turnHeadHoriz(0)
        time.sleep(1.5)
        return False
    
    # isBlocked = False
    # if dist < : #wait 5 secs to see if object is permanent
    
    isBlocked = True
    print(f"\n{dir} path blocked! Waiting 5 seconds...") 
    
    S = time.time()
    for _ in range(5):
        s = time.time()
        
        
        
        while time.time() - s < 1:
            print(round(time.time()-S, 1), end='\r')
            time.sleep(.05)
        dist = sample_min_distance(.2, minObstacleDetectDist)
        if dist is None:
            continue
        
        if dist > maxObstacleDetectDist + 5:
            isBlocked = False
            break
    
    print()

    
    robot.turnHeadHoriz(0)
    time.sleep(1.5)
        
    return isBlocked

def obstaclePermanence(): #check if obstacle in front of robot is permanent for 5 secs

    
    isBlocked = True
        
    print(f"\nObstacle detected at less than {minDistToObstacle_noIntersections}! Waiting 5 seconds...") 
    
    S = time.time()
    for _ in range(5):
        s = time.time()
        
        
        
        while time.time() - s < 1:
            print(round(time.time()-S, 1), end='\r')
            time.sleep(.05)
        dist = sample_min_distance(.2, 0)
        if dist is None:
            continue
        
        if dist > minDistToObstacle_noIntersections + 5:
            isBlocked = False
            break
    
    print()


    robot.turnHeadHoriz(0)
    time.sleep(1)
        
    return isBlocked

def getNewInstructions(currNode, lastNode, stops_left):

    """
        stops_left: end and any other pitstop not visited yet
    """

    stops_left = stops_left[:]

    stops_left.insert(0, currNode)

    node_path = grid.multi_path_dijkstra(stops_left)
    
    if node_path == []:
        return None, None

    node_path.insert(0, lastNode) #to get first direction to next point
    
    dir_path = grid.convertNodePathToDirections(node_path) #left, right...

    return dir_path, node_path

def backUpLight(): #to be threaded
    
    s = time.time()
    while turning or time.time() - s < minBlinkTime:
        robot.topLED(1)
        time.sleep(blinkDelay)
        robot.topLED(0)
        time.sleep(blinkDelay)
                
def rightBlinker(): #to be threaded
    
    s = time.time()
    while turning or time.time() - s < minBlinkTime:
        robot.rightLED(1)
        time.sleep(blinkDelay)
        robot.rightLED(0)
        time.sleep(blinkDelay)
        
def leftBlinker(): #to be threaded
    
    s = time.time()
    while turning or time.time() - s < minBlinkTime:
        robot.leftLED(1)
        time.sleep(blinkDelay)
        robot.leftLED(0)
        time.sleep(blinkDelay)

def intersectionLogic():
    
    global turning, intersectionInstructions, pathNodes, lastIntersectionTime, currNode, lastNode, nextNode
    
    turnDir = intersectionInstructions.pop(0)
    lastNode = currNode
    
    currNode = pathNodes.pop(0)
    
    print(currNode, turnDir)
    
    try:
        nextNode = pathNodes[0]
    except:
        print("End point reached!")

    if currNode in all_stops_nodes and not(currNode == grid.endNode and turnDir != "end"): #do not remove endNode from all_stops if we are just passing over it
        # print(f"currNode {currNode} was in all_stops_nodes")
        all_stops_nodes.remove(currNode)
        
        if currNode in grid.pitstops_for_display:
            grid.pitstops_for_display.remove(currNode)




    if turnDir == "end":

        robot.stopMotors()
        
        time.sleep(.25)
        if robot.getInfraredState(False) != [1, 1, 1]:
            backwardsTillIntersection()
        robot.celebrate(3)
        return "break"

    ###get on intersection exactly###
    robot.stopMotors()
    time.sleep(.25)
    if robot.getInfraredState(False) != [1, 1, 1]:
        backwardsTillIntersection()
        # print('b')
    ################################


    if turnDir == "backwards":
        print("Turning backwards")
        
        if doBlinkers:
            blinkerThread = threading.Thread(target=backUpLight)
            turning = True #for lights
            blinkerThread.start()
        
        uTurnVer3()
        
        turning = False #for lights
        
        lastIntersectionTime = time.time()-1    
        
        return "continue"

    if not isObstacle_intersection(turnDir): #if path not blocked, continue along precalculated path 
        if turnDir == "straight":
            print("Going straight")
            
            robot.runMotors(0, speed)
            time.sleep(.2)

        elif turnDir in ("right", "left"):
            print(f"Turning {turnDir}")
            
            if doBlinkers:
                if turnDir == "left":
                    blinkerThread = threading.Thread(target=leftBlinker)
                elif turnDir == "right":
                    blinkerThread = threading.Thread(target=rightBlinker)
                    
                turning = True
                blinkerThread.start()
            
            turn90DegGrid(turnDir)
            
            turning = False

        lastIntersectionTime = time.time()

        return "continue"

    else: #if path is blocked

        currNode.removeConnection(nextNode)
        
        otherDirNodes = list(currNode.neighbors - {lastNode, nextNode})

        assert len(otherDirNodes) <= 2

        #options: other turn directions to check for obstacles since direction was supposed to take is blocked

        for node_option in otherDirNodes:
            dir_option = grid.dir3Nodes(lastNode, currNode, node_option)

            isBlocked_option = isObstacle_intersection(dir_option)

            if isBlocked_option:
                currNode.removeConnection(node_option)
                

        intersectionInstructions, pathNodes = getNewInstructions(currNode, lastNode, all_stops_nodes)
        
        if intersectionInstructions == None and pathNodes == None: #Dijkstra did not find path
            print("No possible path to next pitstops and/or to the end!")
            return "nopath"
        
        print("New path from current position:")
        print(intersectionInstructions)

        #for display#
        grid.path_connections = {frozenset([pathNodes[i], pathNodes[i+1]]) for i in range(len(pathNodes)-1)}
        grid.node_path = pathNodes
        grid.display()
        #############
        
        currNode = pathNodes.pop(0) #so next instruction runs correctly
        
        
        lastIntersectionTime = time.time() - 10 #"reset" intersection time so next loop detects intersection directly
                
        input("continue:") #wait for user confirmation of new path
        
        time.sleep(.2)

        return "continue"
        

minObstacleDetectDist = 5
maxObstacleDetectDist = 80
blinkDelay = .5
minBlinkTime = 3

last = time.perf_counter() #for precise loop dt control

PRESETS = {
    # Very steady first run. Best for checking wiring, sensors, and direction.
    "safe": {
        "constantSpeed": 20,
        "maxSpeed": 22,
        "minSpeed": 15,
        "lineLostSpeed": 12,
        "Kp": 0.55,
        "Kd": 0.25,
        "Ki": 0.0,
    },
    # Best first real setting for normal line following and Manhattan grid.
    "balanced": {
        "constantSpeed": 24,
        "maxSpeed": 26,
        "minSpeed": 18,
        "lineLostSpeed": 14,
        "Kp": 0.60,
        "Kd": 0.30,
        "Ki": 0.02,
    },
    # Better for curved tracks, circles, and figure-8/infinity lines.
    "curve": {
        "constantSpeed": 22,
        "maxSpeed": 24,
        "minSpeed": 17,
        "lineLostSpeed": 14,
        "Kp": 0.70,
        "Kd": 0.38,
        "Ki": 0.02,
    },
    # Faster once balanced/curve already work well.
    "fast": {
        "constantSpeed": 30,
        "maxSpeed": 36,
        "minSpeed": 22,
        "lineLostSpeed": 16,
        "Kp": 0.75,
        "Kd": 0.45,
        "Ki": 0.01,
    },
    # Highest-power preset. Use only with wheels tested, charged battery, and enough track space.
    "power": {
        "constantSpeed": 36,
        "maxSpeed": 45,
        "minSpeed": 26,
        "lineLostSpeed": 18,
        "Kp": 0.85,
        "Kd": 0.55,
        "Ki": 0.0,
    },
}

selected_preset = PRESETS[ARGS.preset]

constantSpeed = selected_preset["constantSpeed"]
maxSpeed = selected_preset["maxSpeed"]
minSpeed = selected_preset["minSpeed"]
lineLostSpeed = selected_preset["lineLostSpeed"]

dt = .005


integral = 0

smallError = 1
mediumError = 2
bigError = 4

maxOutput = bigError

Kp = selected_preset["Kp"]
Kd = selected_preset["Kd"]
Ki = selected_preset["Ki"]

# print(f"Max error: {Kp*bigError + Kd * 2 + Ki * bigError}")


lastNonNullSideState = [0, 1, 0] #setup like this so that if robot not placed on line at first, will keep going straight
lastState = None

lastError = getError(robot.getInfraredState(False), lastNonNullSideState)

lookup_time_in_past = .15 #how long in past outputs kept for integral component calculation
numPastStates = max(1, int(round(lookup_time_in_past / dt)))

pastStates = deque(maxlen=numPastStates) #automatic dequeue when append new and maxlen exceeded

runningTankTurn = False
enableTankTurnRecovery = False
tankTurnThreshold = 3.5

doIntersections = ARGS.mode == "grid"
doBlinkers = not ARGS.no_blinkers

print(f"Starting robot in {ARGS.mode} mode.")
print(
    f"Preset {ARGS.preset}: speed {minSpeed}-{maxSpeed}, "
    f"lineLostSpeed {lineLostSpeed}, Kp {Kp}, Kd {Kd}, Ki {Ki}."
)
if doIntersections:
    print("Grid mode: Manhattan intersections/path planning enabled.")
else:
    print("Line mode: follows continuous lines/curves and ignores intersections.")

lastIntersectionTime = time.time()-10


minDistToObstacle_noIntersections = 25

# all_output = [] #storing outputs for PID tuning with output graph

if doIntersections:
    intersectionInstructions, grid, pathNodes, all_stops_nodes, INITIAL_DIR_FACING_OUT  = grid_dijkstra.getGridInstructions()

    currNode = pathNodes.pop(0)
    all_stops_nodes = all_stops_nodes[1:]

    if INITIAL_DIR_FACING_OUT:
        all_stops_nodes.pop(0)
        backwardsTillIntersection() #rejoin initial point intersection
        
else:
    counter = 0
    

robot._measureSetup()

try: #<- error collection, no uncontrolled crashes
    
    # s = time.time()
    #MAINLOOP
    while True:# and time.time() - s < 10:

        ###DT LOGIC### <- more accurate time.sleep(dt) basically
        now = time.perf_counter()

        if now - last < dt:
            continue

        last = now
        ##############

        
        if not doIntersections: #obstacle logic if not grid
            counter+=1
            if counter % 50 == 0:#only check distance every 100 loops (every 100*dt=.5 seconds)
                counter=0
                
                while True:
                    dist = robot.measureDistance()
                    
                    if dist is not None:
                        break
                
                if dist < minDistToObstacle_noIntersections:
                    robot.stopMotors()
                    robot.turnWheels(0)
                    robot.runMotors(1, 100)
                    time.sleep(.2)
                    robot.stopMotors()
                    
                    
                    if obstaclePermanence():
                        print("Obstacle in the way! U-turning!")
                        uTurnVer3()
                        continue
                        # break

        currentInfraState = robot.getInfraredState(False) #infraState: [l, m, r]: on line = 1, off line = 0

        if currentInfraState == [1, 1, 1]: #on intersection
            if doIntersections:
                if time.time() - lastIntersectionTime > 2: #do intersection turn, has been > 2 secs since last intersec (no unwanted intersec detection)
                
                    postIntersecAction = intersectionLogic()
                    
                    if postIntersecAction == "break":
                        break
                    elif postIntersecAction == "continue":
                        continue
                    elif postIntersecAction == "nopath":
                        break
                    else:
                        raise ValueError("intersectionLogic returned weird")
                
            else:
                if lastState is None:
                    continue
                currentInfraState = lastState
        elif currentInfraState == [1, 0, 1]: #ignore weird data
            if lastState is None:
                continue
            currentInfraState = lastState
            
        ###PID###
        error = getError(currentInfraState, lastNonNullSideState)

        pastStates.append(error)
        
        integral = np.average(pastStates) #average PID output over last numPastStates loops

        derivative = (error - lastError) #/ dt

        raw_output = Kp * error + Kd * derivative + Ki * integral

        # all_output.append(raw_output) #for PID tuning, graph output

        capped_output = raw_output
        if abs(raw_output) > maxOutput: #capping output between -maxOutput, maxOutput
            capped_output = np.sign(raw_output) * maxOutput


        turnAngle = convertOutputToAngle(capped_output)

        speed = convertOutputToSpeed(capped_output)

        if currentInfraState == [0, 0, 0]:
            speed = lineLostSpeed

        if enableTankTurnRecovery and currentInfraState == [0, 0, 0] and abs(raw_output) > tankTurnThreshold: #optional hard recovery if the line is fully lost
            if not runningTankTurn:
                runningTankTurn = True
                tankTurnThread = threading.Thread(target=tankTurn, args=(raw_output,))
                tankTurnThread.start()


        robot.turnWheels(turnAngle) #adjust wheel angle based on PID

        robot.runMotors(0, speed) #adjust motor speed based on PID
        
        
        ###FOR NEXT LOOP### <- sets up "last loop" variables for next loop
        lastError = error

        if currentInfraState[0] != 0 or currentInfraState[2] != 0:
            lastNonNullSideState = currentInfraState

        lastState = currentInfraState
        ###################


except KeyboardInterrupt:
    print("User terminated the program.")
except Exception as e:
    # print(f"Program terminated with exception:\n{e}")
    traceback.print_exc()
else:
    print("Program terminated normally.")
finally:
    robot.resetAll() #always resets the robot

    # save_list_npz(all_output, "outputs_to_graph", f"Kp:{Kp}Ki:{Ki}Kd:{Kd}") #for PID tuning, output to graph
    # print(f"Wanted: {1/dt}, Actual: {counter/10}") #dt check
