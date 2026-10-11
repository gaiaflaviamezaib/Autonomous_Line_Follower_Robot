from robotControlClass import robot

import time
import numpy as np
from collections import deque
import threading
import traceback


def tankTurn(dir): #to be threaded

    global runningTankTurn

    # sign = np.sign(raw_output)

    if dir == "right":
        index = 2
        leftDir = 0
        rightDir = 1
    elif dir == "left":
        index = 0
        leftDir = 1
        rightDir = 0
    else:
        raise ValueError("raw_output is exactly 0. (line 107)")

    switch = True
    lastSwitch = time.time()
    while dist := robot.measureDistance():

        if dist is not None:
            if dist < 1.5 * dist_ref:
                break
        if time.time() - lastSwitch > .3: #only drifts for .3 secs at a time
            switch = not(switch)
            lastSwitch = time.time()
            if not switch:
                lastSwitch = time.time()-.1 
        if switch:
            robot.runLeftMotor(leftDir, 100)
            robot.runRightMotor(rightDir, 100)

    runningTankTurn = False
        
def convertOutputToAngle(output):

    return output/maxOutput * 25 #angle proportional to output, between -25 and 25 (max vals for wheel turn)


last = time.perf_counter() #for precise loop dt control

constantSpeed = 25
maxSpeed = 30
minSpeed = 20

dt = .005


integral = 0
maxOutput = 10

#OLD VALUES:
# Kp = 1
# Kd = 0.5
# Ki = 0.2
Kp = .8
Kd = 0.5
Ki = 0.2
lastError = 0


lookup_time_in_past = .05 #how long in past outputs kept for integral component calculation
numPastStates = int(lookup_time_in_past // dt)

pastStates = deque(maxlen=numPastStates) #automatic dequeue when append new and maxlen exceeded

runningTankTurn = False

doIntersections = False
doBlinkers = True

lastIntersectionTime = time.time()-10


minDistToObstacle_noIntersections = 25

robot._measureSetup()

dist_ref = 20

robot.turnHeadHoriz(80)

lastCheck = time.time() - 1
checkTime = 1
distToObstacleInFront = 40
try: #<- error collection, no uncontrolled crashes
    
    #MAINLOOP
    while True:

        ###DT LOGIC### <- more accurate time.sleep(dt) basically
        now = time.perf_counter()

        if now - last < dt:
            continue

        last = now
        ##############

        ###PID###
        
        if time.time() - lastCheck > checkTime:
            
            
            robot.turnHeadHoriz(0)
            
            time.sleep(.3)
            distInFront = robot.measureDistance()
            
            if distInFront is not None:
                if distInFront < distToObstacleInFront:
                    
                    robot.runMotors(1, 25)
                    robot.turnWheels(0)
                    while True:
                        dist = robot.measureDistance()
                        if dist is not None:
                            if dist > distToObstacleInFront * 0.75:
                                break
                    robot.stopMotors()
                    time.sleep(2)
                    robot.runMotors(0, constantSpeed)
                    robot.turnWheels(-25)
                    
                    s = time.time()
                    time.sleep(2)
                    robot.turnHeadHoriz(80)
                    while dist := robot.measureDistance():
                        if dist < 1.25 * dist_ref:# and time.time() - s > 5:
                            robot.turnWheels(0)
                            lastCheck = time.time()
                            break
                            
                    continue
            
            robot.turnHeadHoriz(80)
            time.sleep(.3)
            
            lastCheck = time.time()
            
        distFromWall = robot.measureDistance()#getError(currentInfraState, lastNonNullSideState)

        if distFromWall is None:
            continue
        
        if distFromWall > 1.5 * dist_ref:
            robot.turnWheels(25)
            
            if not runningTankTurn:
                runningTankTurn = True
                tankTurnThread = threading.Thread(target=tankTurn, args=("left",))
                tankTurnThread.start()
                
            while dist := robot.measureDistance():
                if dist is not None:
                    if dist < 1.5 * dist_ref:
                        
                        break
                time.sleep(.1)
                
            robot.turnWheels(0)
            
        error = distFromWall - dist_ref
        
        pastStates.append(error)
        
        integral = np.average(pastStates) #average PID output over last numPastStates loops

        derivative = (error - lastError) #/ dt

        raw_output = Kp * error + Kd * derivative + Ki * integral


        capped_output = raw_output
        if abs(raw_output) > maxOutput: #capping output between -maxOutput, maxOutput
            capped_output = np.sign(raw_output) * maxOutput


        turnAngle = convertOutputToAngle(capped_output)

        robot.turnWheels(turnAngle) #adjust wheel angle based on PID

        robot.runMotors(0, constantSpeed) #adjust motor speed based on PID
        
        
        ###FOR NEXT LOOP### <- sets up "last loop" variables for next loop
        lastError = error
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
