import time
import numpy as np
import math
import threading
import traceback
from collections import deque

import robotControlClass as roboClass


""""
attempt to make inverted pendulum with robot on back wheels,
PID
"""
robot = roboClass.robotControl()


acc_ref = 18

pitch_ref = 79

target_dt = 0.05

integral = 0

prev_error = 0

alpha = 0.2

Kp = 2
Kd = .1
Ki = 0.05

maxError = 15
print(f"max output = {Kp*maxError+Ki*maxError+Kd*maxError}")

last = time.perf_counter()

roll, pitch_raw, acc = robot.getAccelerometerData(False)

pitch_filtered = pitch_raw

lookup_time_in_past = .15 #how long in past outputs kept for integral component calculation
numPastStates = int(lookup_time_in_past // target_dt)
pastStates = deque(maxlen=numPastStates)

lastError = None

maxOutput = Kp * maxError

minSpeed = 25
maxSpeed = 100

# counter = 0
try:
    s = time.time()
    while True:# and time.time()-s < 10:
        
        now = time.perf_counter()
        
        dt = now - last
        
        
        if dt < target_dt:
            continue
        
        last = now
        # counter += 1
        #######################
        
        roll, pitch_raw, acc = robot.getAccelerometerData(False)

        acc = acc[2]
        
        
        # pitch_filtered = pitch_raw
        pitch_filtered = alpha * pitch_filtered + (1-alpha) * pitch_raw
        
                
        if acc > 0:
            error = pitch_ref - pitch_filtered
        
        elif acc < 0:
            error = - (180 - (pitch_filtered + pitch_ref))
        else:
            assert False
        
        pastStates.append(error)
        
        integral = sum(pastStates) * target_dt #average PID output over last numPastStates loops

        if lastError is not None:
            derivative = (error - lastError) / dt
        else:
            derivative = 0

        raw_output = Kp * error + Kd * derivative + Ki * integral

        # all_output.append(raw_output) #for PID tuning, graph output

        capped_output = raw_output
        if abs(raw_output) > maxOutput: #capping output between -maxOutput, maxOutput
            capped_output = np.sign(raw_output) * maxOutput


        # speed = -(maxSpeed-minSpeed)/maxOutput*abs(capped_output) + maxSpeed
        speed = abs(capped_output)/maxOutput * 75 + 25
        
        # speed = min(100, max(20, abs(output))) #deadband: 25
        
        print(round(pitch_raw, 1), round(pitch_filtered, 1), raw_output, capped_output, speed)
        
        if np.sign(capped_output) < 0:
            robot.runMotors(1, speed)
        else:
            robot.runMotors(0, speed)
        
        lastError = error
        
        # time.sleep(dt)

except KeyboardInterrupt:
    print("User terminated the program.")
except Exception as e:
    traceback.print_exc()
    # print(f"Program terminated with exception:\n{e}")
else:
    print("Program terminated normally.")
finally:
    robot.resetAll()

    # print(counter/10, 1/target_dt)
