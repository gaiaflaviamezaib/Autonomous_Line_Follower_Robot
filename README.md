# Autonomous_Line_Follower_Robot

An autonomous line-following robot built using a Raspberry Pi, utilizing IR sensors for path detection (left, center, right) and an ultrasonic sensor for obstacle detection. The system processes sensor input to control motors, enabling navigation and collision avoidance.

## Hardware

- Raspberry Pi
- Two rear DC motors and a front steering servo
- Three infrared line sensors (left, middle, right)
- Ultrasonic distance sensor on a two-servo head
- PCA9685 servo driver
- MPU6050 accelerometer/gyroscope
- LEDs and an optional SSD1306 OLED screen

## Main program

`MAIN_PID_INTERSECTION_MANHATTAN.py` follows the line with a PID controller and has two modes:

- `line`: follow any line or curve, with intersections ignored.
- `grid`: drive on a Manhattan-style grid. At each intersection the robot follows a route planned with Dijkstra's algorithm (`manhattan_graph_dijkstra.py`), checks for obstacles before turning, and plans a new route if the way is blocked.

Speed and PID settings come from presets: `safe`, `balanced`, `curve`, `fast` and `power`.

```bash
pip install -r requirements.txt
python3 run_checks.py            # syntax check, does not move the robot
python3 follow_any_line.py       # line mode, curve preset
python3 manhattan_grid.py        # grid mode, balanced preset
python3 MAIN_PID_INTERSECTION_MANHATTAN.py --mode line --preset safe
```

Stop any program with `Ctrl+C`.

## Other scripts

| Script | What it does |
|---|---|
| `PID_followWall.py` | Follows a wall at a set distance using the ultrasonic sensor and PID |
| `goToObstacle.py` | Scans for the nearest object and drives towards it |
| `handFollow.py` | Keeps a set distance from an object in front |
| `drawSurroundings.py` | Sweeps the ultrasonic sensor and plots distances in the terminal |
| `PHASE1_Slope.py` | Detects a steep slope with the accelerometer and backs off |
| `wheelie.py` | Experimental balancing on the rear wheels |
| `TSK04.py` | Ultrasonic sweep while turning the head |
| `motor_test.py`, `move_forward.py` | Basic motor tests |

`robotControlClass.py` wraps the hardware (motors, servos, sensors, LEDs). The `*_vS.py` files are the low-level hardware drivers provided by the course.

See [docs/run-guide.md](docs/run-guide.md) for presets and PID tuning.
