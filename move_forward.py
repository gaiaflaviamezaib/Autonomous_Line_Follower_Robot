import time

from robotControlClass import robot


start_speed = 100
drive_speed = 80
duration = 5


try:
    print("Starting move_forward.py")
    print(f"Driving forward for {duration} seconds.")

    try:
        robot.turnWheels(0)
    except Exception as e:
        print(f"Could not center wheels, continuing with motors anyway: {e}")

    print(f"Starting motors at speed {start_speed}.")
    robot.runMotors(0, start_speed)
    time.sleep(0.5)

    print(f"Continuing at speed {drive_speed}.")
    robot.runMotors(0, drive_speed)
    time.sleep(duration)
    robot.stopMotors()
    print("Done.")

except KeyboardInterrupt:
    print("Stopped by user.")

except Exception as e:
    print(f"Error while driving: {e}")

finally:
    try:
        robot.stopMotors()
    finally:
        try:
            robot.resetAll()
        except Exception as e:
            print(f"Could not fully reset robot: {e}")
