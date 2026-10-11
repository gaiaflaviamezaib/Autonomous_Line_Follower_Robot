from robotControlClass import robot

import time
import numpy as np

def plot_terminal(values, width=80, height=20, title="Chart", show_values=True):
    """
    Plot a list of values as a chart in the terminal.
    
    Args:
        values: List of numeric values to plot
        width: Width of the chart in characters (default 80)
        height: Height of the chart in lines (default 20)
        title: Title for the chart (default "Chart")
        show_values: Whether to show min/max values on axis (default True)
    """
    if not values or len(values) == 0:
        print("No data to plot")
        return
    
    # Get min and max values
    min_val = min(values)
    max_val = max(values)
    val_range = max_val - min_val
    
    # Handle case where all values are the same
    if val_range == 0:
        val_range = 1
    
    # Print title
    print(f"\n{title}")
    print("=" * width)
    
    # Create the chart grid
    chart = [[' ' for _ in range(width)] for _ in range(height)]
    
    # Plot the values
    for i, val in enumerate(values):
        # Map value index to x position
        x = int(i * (width - 1) / (len(values) - 1)) if len(values) > 1 else width // 2
        
        # Map value to y position (inverted because terminal prints top to bottom)
        y = height - 1 - int((val - min_val) * (height - 1) / val_range)
        
        # Ensure within bounds
        x = max(0, min(width - 1, x))
        y = max(0, min(height - 1, y))
        
        # Plot point
        chart[y][x] = '●'
        
        # Draw line to previous point
        if i > 0:
            prev_val = values[i - 1]
            prev_x = int((i - 1) * (width - 1) / (len(values) - 1)) if len(values) > 1 else width // 2
            prev_y = height - 1 - int((prev_val - min_val) * (height - 1) / val_range)
            
            prev_x = max(0, min(width - 1, prev_x))
            prev_y = max(0, min(height - 1, prev_y))
            
            # Draw vertical line if y differs
            # if y != prev_y:
            #     start_y = min(y, prev_y)
            #     end_y = max(y, prev_y)
            #     # for draw_y in range(start_y, end_y + 1):
                    # if chart[draw_y][prev_x] == ' ':
                        # chart[draw_y][prev_x] = '│'
            
            # Draw horizontal line if x differs
            # if x != prev_x:
            #     start_x = min(x, prev_x)
            #     end_x = max(x, prev_x)
            #     for draw_x in range(start_x, end_x + 1):
            #         if chart[y][draw_x] == ' ':
            #             chart[y][draw_x] = '─'
    
    # Print the chart with axis labels
    for i, row in enumerate(chart):
        if show_values:
            # Calculate the value at this y position
            val_at_y = max_val - (i * val_range / (height - 1))
            # Print value label on left (every few rows)
            if i % (height // 4) == 0 or i == height - 1:
                print(f"{val_at_y:7.2f} │ {''.join(row)}")
            else:
                print(f"        │ {''.join(row)}")
        else:
            print(''.join(row))
    
    # Print bottom axis
    if show_values:
        print("        └" + "─" * width)
        print(f"         0{' ' * (width - 10)}{len(values) - 1}")
        print(f"\nMin: {min_val:.2f}  Max: {max_val:.2f}  Points: {len(values)}")
    
    print()

def drawVision():
    
    distAngle = []
    
    
    robot.turnHeadHoriz(-80)
    time.sleep(.2)
    
    maxAngleTurn = 60
    numPointsPerDeg = 4
    
    for a in range(-maxAngleTurn, maxAngleTurn, 1):

        robot.turnHeadHoriz(a)
        
        time.sleep(.1)
        dist = []
        for _ in range(numPointsPerDeg):
            dist.append(robot.measureDistance())
            time.sleep(.1)
        print(dist)
        dist = [d for d in dist if d is not None]
        if not dist:
            continue
        dist = np.average(dist)
        # time.sleep(.1)
        # dist2 = robot.measureDistance()
        
        # dist = (dist1+dist2)/2
        if dist is not None and dist > 0:
            # print(dist)
            
            # if dist < max:
            distAngle.append((dist, a))
                # print(dist)
                # if dist < min:
                #     return "break"
                
                # return a, dist
        
        # time.sleep(.02)
    
    
    if len(distAngle) == 0:
        return None

    # distAngle = [(round(np.average([distAngle[j][0] for j in range(i, i+2)]), 2), distAngle[i][1]) for i in range(0, len(distAngle)-2, 2)]
    plot_terminal([d[0] for d in distAngle])
    distAngle.sort(key = lambda x: x[0])
    len(distAngle)
    # print(distAngle[0])
    
    
    robot.turnHeadHoriz(distAngle[0][1])
    
    time.sleep(.5)
    return distAngle[0]

def main():
    try:
        drawVision()
    finally:
        robot.resetAll()


if __name__ == "__main__":
    main()

# minDist = 10
# maxDist = 100

# while True:
#     angleDist = detectObstacle(minDist, maxDist)
    
#     # if angleDist == "break":
#     #     print("Reached obstacle!")
#     #     break
#     if angleDist == None:
#         print("No obstacle detected!")
#         break

#     distance, angle = angleDist
    
#     if distance < minDist:
#         print("Reached obstacle!")
#         break
#     # distance = angleDist[1]
    
    
#     print(angle, distance)
    
#     robot.turnWheels(25/80*angle)
    
#     time.sleep(.5)
#     robot.runMotors(0, 30)
    
#     time.sleep(max(0.5, 3/100*distance))
    
#     robot.stopMotors()
    
#     maxDist = distance * 1.2
    
# robot.stopMotors()


