import TSK04 as t4
import RPIservo_vS as servo
import time
import DCmotor_vS as DC
import numpy as np

import robotControlClass as roboClass

robot = roboClass.robotControl()

def main():
    wantedDist = 75
    maxError = 5

    try:
        while True:
            distToObstacle = robot.measureDistance()

            if distToObstacle is None:
                continue

            delta = distToObstacle-wantedDist

            if delta > 300:
                robot.stopMotors()
                continue

            if abs(delta) < maxError:
                robot.stopMotors()
                continue

            speed = min(75, max(25, abs(delta*2)))

            if delta > 0:
                robot.runMotors(0, speed)
            elif delta < 0:
                robot.runMotors(1, speed)

            time.sleep(.2)
    finally:
        robot.resetAll()


if __name__ == "__main__":
    main()
