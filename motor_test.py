import time

from robotControlClass import robot


speed = 100
test_time = 2


def pause():
    robot.stopMotors()
    time.sleep(1)


try:
    print("Motor test starting.")
    print("Lift the robot wheels off the ground if you can.")

    print("Both motors forward.")
    robot.runMotors(0, speed)
    time.sleep(test_time)
    pause()

    print("Left motor forward.")
    robot.runLeftMotor(0, speed)
    time.sleep(test_time)
    pause()

    print("Right motor forward.")
    robot.runRightMotor(0, speed)
    time.sleep(test_time)
    pause()

    print("Both motors backward.")
    robot.runMotors(1, speed)
    time.sleep(test_time)
    pause()

    print("Motor test done.")

except KeyboardInterrupt:
    print("Stopped by user.")

except Exception as e:
    print(f"Motor test error: {e}")

finally:
    try:
        robot.stopMotors()
    finally:
        robot.resetAll()
