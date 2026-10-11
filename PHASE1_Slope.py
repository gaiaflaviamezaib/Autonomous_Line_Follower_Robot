from robotControlClass import robot

import time

if __name__ == "__main__":
    try:


        speed = 50
        goingBackwards = False
        robot.runMotors(0, speed)


        print("Waiting for motion...")
        while True:
            roll, pitch, acc = robot.getAccelerometerData() # Calculate and print angles

            if not(-15 < roll < 15) or not(-23 < pitch < 7):
                print('INCLINE TOO STEEP! STOP THE ROBOT!')
                robot.runMotors(1,speed)
                goingBackwards = True

            if (-10 < pitch < -5) and goingBackwards:
                print("pitch is zero")


                robot.runMotors(1, 100)

                time.sleep(.5)

                robot.turnWheels(-25)

                robot.runLeftMotor(1, 100)
                robot.runRightMotor(0, 100)

                time.sleep(.9)

                robot.turnWheels(0)
                robot.runMotors(0, speed)

                time.sleep(2)

                robot.stopMotors()
                break

            time.sleep(0.1)             # Delay between readings

    except KeyboardInterrupt:
        print("User terminated the program.")
    except Exception as e:
        print(f"Program terminated with exception:\n{e}")
    else:
        print("Program terminated normally.")
    finally:
        robot.resetAll()
