# Run and tuning guide

Copy the repository to the Raspberry Pi, then run commands from the repository root:

```bash
python3 run_checks.py
```

## Hardware Test

Lift the robot wheels first:

```bash
python3 motor_test.py
```

Short forward drive test:

```bash
python3 move_forward.py
```

## Follow Any Line Or Curve

Use this for a normal track, curved line, circle, or figure-8/infinity track:

```bash
python3 follow_any_line.py
```

This runs the PID line follower with intersections disabled and the `curve` preset.

Equivalent command:

```bash
python3 MAIN_PID_INTERSECTION_MANHATTAN.py --mode line --preset curve
```

Faster versions:

```bash
python3 follow_any_line_fast.py
python3 follow_any_line_power.py
```

## Manhattan Grid

Use this for the intersection/grid challenge:

```bash
python3 manhattan_grid.py
```

Equivalent command:

```bash
python3 MAIN_PID_INTERSECTION_MANHATTAN.py --mode grid --preset balanced
```

Grid mode asks for path information through `manhattan_graph_dijkstra.py` and uses intersection logic.

## Included Support Files

The low-level robot files are included locally:

- `DCmotor_vS.py`
- `FLED_vS.py`
- `MPU6050_vS.py`
- `OLED_vS.py`
- `RPIservo_vS.py`
- `Tracking_vS.py`
- `robotControlClass.py`

The main behavior files are also included:

- `MAIN_PID_INTERSECTION_MANHATTAN.py`
- `manhattan_graph_dijkstra.py`
- `PID_followWall.py`
- `PHASE1_Slope.py`
- `TSK04.py`
- `wheelie.py`

## What Still Needs Real-Robot Calibration

Even when the code is complete, a line-following robot always needs some values tuned on the real track. This is normal. The exact values depend on battery level, floor color, line thickness, wheel grip, sensor height, and servo alignment.

The most important values are in `MAIN_PID_INTERSECTION_MANHATTAN.py`. The file now has named presets so you do not need to edit values for every test:

```python
--preset safe
--preset balanced
--preset curve
--preset fast
--preset power
```

The steering limit is still fixed in `robotControlClass.py`:

```python
MAX_ANGLE_WHEELS = 25
minWheelsAngle = -25
```

That means the front wheels can turn from `-25` to `+25` degrees. The PID output is automatically converted into that steering range.

## Recommended Presets

Use these in order. Do not start with `power` until the robot already follows the track well.

### 1. Safe Preset

Best for first test, weak battery, narrow track, or if the robot is unpredictable.

```python
constantSpeed = 20
maxSpeed = 22
minSpeed = 15
lineLostSpeed = 12
Kp = 0.55
Kd = 0.25
Ki = 0.0
```

Run:

```bash
python3 MAIN_PID_INTERSECTION_MANHATTAN.py --mode line --preset safe
```

### 2. Balanced Preset

Most likely to work well for a normal robot and Manhattan grid.

```python
constantSpeed = 24
maxSpeed = 26
minSpeed = 18
lineLostSpeed = 14
Kp = 0.60
Kd = 0.30
Ki = 0.02
```

Run:

```bash
python3 MAIN_PID_INTERSECTION_MANHATTAN.py --mode grid --preset balanced
```

### 3. Curve Preset

Most likely to work well for a curved track, circle, or figure-8.

```python
constantSpeed = 22
maxSpeed = 24
minSpeed = 17
lineLostSpeed = 14
Kp = 0.70
Kd = 0.38
Ki = 0.02
```

Run:

```bash
python3 follow_any_line.py
```

### 4. Fast Preset

Use after `curve` works. This gives more speed but still tries to stay stable.

```python
constantSpeed = 30
maxSpeed = 36
minSpeed = 22
lineLostSpeed = 16
Kp = 0.75
Kd = 0.45
Ki = 0.01
```

Run:

```bash
python3 follow_any_line_fast.py
```

### 5. Power Preset

Highest-power preset. Only use after testing motors, steering, sensors, and lower presets. Keep your hand near the robot or be ready to stop with `Ctrl+C`.

```python
constantSpeed = 36
maxSpeed = 45
minSpeed = 26
lineLostSpeed = 18
Kp = 0.85
Kd = 0.55
Ki = 0.0
```

Run:

```bash
python3 follow_any_line_power.py
```

Motor power in this project is from `0` to `100`, but line following usually should not run at `100`. At `100`, the robot often moves faster than the sensors and steering can correct. The `power` preset uses `45` as the top line-following speed because that is much more realistic for staying on the line.

Do not change many values at once. Change one thing, test, then write down what happened.

## Before Tuning PID

Check these first, because PID cannot fix bad hardware setup:

1. Battery is charged.
2. Wheels are not slipping.
3. The front wheels are straight when `robot.turnWheels(0)` runs.
4. The three line sensors correctly detect black and white.
5. The robot can drive forward using:

```bash
python3 move_forward.py
```

6. The motors are connected in the correct direction using:

```bash
python3 motor_test.py
```

If the robot goes backward when it should go forward, fix motor wiring or motor direction before PID tuning.

## How To Check The Line Sensors

Run the main program in line mode:

```bash
python3 follow_any_line.py
```

The useful sensor states are:

```text
[0, 1, 0] = centered on the line
[1, 1, 0] = line slightly left
[1, 0, 0] = line far left
[0, 1, 1] = line slightly right
[0, 0, 1] = line far right
[1, 1, 1] = intersection or very thick line
[0, 0, 0] = line lost
```

If the sensors seem reversed, change this in `Tracking_vS.py` or `robotControlClass.py` only after confirming with a simple sensor test.

## How To Tune PID

PID tuning is partly trial and error, but it should be organized.

Start with:

```python
Kp = 0.6
Kd = 0.0
Ki = 0.0
```

Test on a simple curved line.

### Step 1: Tune Kp

`Kp` controls how strongly the robot turns toward the line.

If the robot reacts too slowly and leaves the line:

```python
Kp = Kp + 0.1
```

If the robot wiggles left-right too much:

```python
Kp = Kp - 0.1
```

Good `Kp` means the robot follows the line but may still be a little shaky.

### Step 2: Tune Kd

`Kd` calms the steering and reduces wiggle.

After `Kp` is decent, try:

```python
Kd = 0.2
```

If the robot still wiggles, increase gradually:

```python
Kd = 0.3
Kd = 0.4
```

If the robot becomes slow to correct or feels stiff, reduce `Kd`.

### Step 3: Tune Ki Last

`Ki` fixes small long-term drift, but too much makes the robot unstable.

Usually keep it small:

```python
Ki = 0.0
Ki = 0.02
Ki = 0.05
```

If the robot starts steering strangely after a few seconds, reduce `Ki` or set it back to `0.0`.

## Speed Tuning

Tune speed after PID is mostly working.

If the robot loses the line on curves:

```python
constantSpeed = constantSpeed - 2
maxSpeed = maxSpeed - 2
```

If the robot follows well but is too slow:

```python
constantSpeed = constantSpeed + 2
maxSpeed = maxSpeed + 2
```

If the robot stalls during recovery when the line is lost:

```python
lineLostSpeed = lineLostSpeed + 2
```

Good starting values if you want to edit manually:

```python
constantSpeed = 22
maxSpeed = 25
minSpeed = 16
lineLostSpeed = 14
Kp = 0.6
Kd = 0.3
Ki = 0.02
```

## What To Test

Use this order:

1. Straight line.
2. Gentle curve.
3. Full circle or figure-8.
4. Sharp turn.
5. Intersection.
6. Manhattan grid.

For Manhattan grid, use:

```bash
python3 manhattan_grid.py
```

For curves and figure-8 tracks, use:

```bash
python3 follow_any_line.py
```

## Simple Tuning Table

Write results like this after each test:

```text
Test 1
Kp = 0.6, Kd = 0.0, Ki = 0.0
Speed = 24
Result: follows straight, loses line on curve
Next change: Kp to 0.7
```

This makes tuning much faster than guessing.
