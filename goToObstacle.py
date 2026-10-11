from robotControlClass import robot

import time
import numpy as np



def detectObstacle(min_distance, max_distance):
    
    distAngle = []
    
    
    robot.turnHeadHoriz(-80)
    time.sleep(.2)
    for a in range(-80, 80, 1):
        robot.turnHeadHoriz(a)
        
        dist = robot.measureDistance()
        # time.sleep(.1)
        # dist2 = robot.measureDistance()
        
        # dist = (dist1+dist2)/2
        if dist is not None and dist > 0:
            # print(dist)
            
            if dist < max_distance:
                distAngle.append((dist, a))
                # print(dist)
                # if dist < min:
                #     return "break"
                
                # return a, dist
        
        time.sleep(.01)
    
    
    if len(distAngle) == 0:
        return None

    if len(distAngle) >= 5:
        distAngle = [(round(np.average([distAngle[j][0] for j in range(i, i+4)]), 2), distAngle[i+2][1]) for i in range(0, len(distAngle)-4, 4)]

    distAngle.sort(key = lambda x: x[0])
    robot.turnHeadHoriz(distAngle[0][1])
    
    time.sleep(.5)
    return distAngle[0]


def main():
    minDist = 10
    maxDist = 300

    try:
        while True:
            angleDist = detectObstacle(minDist, maxDist)

            if angleDist is None:
                print("No obstacle detected!")
                break

            distance, angle = angleDist

            if distance < minDist:
                print("Reached obstacle!")
                break

            print(angle, distance)

            robot.turnWheels(25/80*angle)

            time.sleep(.25)
            robot.runMotors(0, 30)
            time.sleep(max(0.5, 3/100*distance))

            robot.stopMotors()

            maxDist = distance * 1.2
    finally:
        robot.stopMotors()
        robot.resetAll()


if __name__ == "__main__":
    main()
