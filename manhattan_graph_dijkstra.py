import numpy as np
import sys
import heapq

from copy import copy
import traceback

# from typing import Any, Dict, List, Tuple, Optional


GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
MAGENTA = '\033[95m'
CYAN = '\033[96m'
RESET = '\033[0m'


class Node:
    
    def __init__(self, pos):
        self.pos = pos
        
        self.neighbors = set()

    def __repr__(self):
        
        return f"Node: {self.pos}"

    def __lt__(self, other):
        
        return self.pos < other.pos

    
    def addConnection(self, other):
        self.neighbors.add(other)
        other.neighbors.add(self)
        
    def removeConnection(self, other):
        self.neighbors.remove(other)
        other.neighbors.remove(self)
    


class Grid:
    
    def __init__(self, x_size, y_size):
        
        self.x_size = x_size
        self.y_size = y_size
        
        self.nodes = set()
                
        self.startNode = None
        self.endNode = None
        
        self.edgeToWeight = dict()
        self.posToNode = dict()
        
        self.illegal_positions = set()
        
        self.pitstops_for_display = set()
        self.node_path = list()
        self.path_connections = set()
        
        for x in range(self.x_size):
            for y in range(self.y_size):
                n = Node((x, y))
                self.posToNode[(x, y)] = n
                self.nodes.add(n)
        
        for x in range(self.x_size):
            for y in range(self.y_size):
                try:
                    self.posToNode[(x, y)].addConnection(self.posToNode[(x-1, y)])
                except Exception as e:
                    pass
                try: 
                    self.posToNode[(x, y)].addConnection(self.posToNode[(x+1, y)])     
                except Exception as e:
                    pass
                try:
                    self.posToNode[(x, y)].addConnection(self.posToNode[(x, y-1)])     
                except Exception as e:
                    pass
                try:
                    self.posToNode[(x, y)].addConnection(self.posToNode[(x, y+1)])     
                except Exception as e:
                    pass
        
        for node in self.nodes:
            for neigh in node.neighbors:
                self.edgeToWeight[frozenset([node, neigh])] = 1
        
        
    def getWeight(self, node1, node2):
        
        return self.edgeToWeight[frozenset([node1, node2])]

    def display(self, horizontal_dashes=4, vertical_lines=2):
        """
        Display the grid with lines showing connections between nodes.
        Nodes are shown as 'O', illegal positions as spaces.
        Connections are shown with '-' (horizontal) and '|' (vertical).
        
        Args:
            horizontal_dashes: Number of dashes between horizontal nodes (default: 4)
            vertical_lines: Number of lines between vertical nodes (default: 2)
        """
        # Calculate display dimensions
        # Each node takes 1 char, plus spacing for connections
        
        display_width = self.x_size * (1 + horizontal_dashes) - horizontal_dashes
        display_height = self.y_size * (1 + vertical_lines) - vertical_lines
        
        # Create empty display grid
        display = [[' ' for _ in range(display_width)] for _ in range(display_height)]
        
        # Place nodes and connections
        for y in range(self.y_size):
            for x in range(self.x_size):
                # Calculate position in display grid
                display_x = x * (1 + horizontal_dashes)
                display_y = y * (1 + vertical_lines)
                
                # Place node if it exists
                if (x, y) in self.posToNode:
                    node = self.posToNode[(x, y)]
                    if node == self.startNode:
                        display[display_y][display_x] = f'{GREEN}S{RESET}'
                    elif node == self.endNode:
                        display[display_y][display_x] = f'{RED}E{RESET}'
                    else:
                        if node in self.node_path or node in self.pitstops_for_display:
                            if node in self.pitstops_for_display:
                                display[display_y][display_x] = f'{RED}X{RESET}'
                            else:
                                display[display_y][display_x] = f'{BLUE}X{RESET}'
                        else:
                            display[display_y][display_x] = 'O'
                    
                    
                    # Draw horizontal connection to right neighbor
                    if (x + 1, y) in self.posToNode:
                        right_node = self.posToNode[(x + 1, y)]
                        if right_node in node.neighbors:
                            for i in range(1, horizontal_dashes + 1):
                                if frozenset([right_node, node]) in self.path_connections:
                                    display[display_y][display_x + i] = f'{BLUE}-{RESET}'
                                else:
                                    display[display_y][display_x + i] = '-'
                    
                    # Draw vertical connection to bottom neighbor
                    if (x, y + 1) in self.posToNode:
                        bottom_node = self.posToNode[(x, y + 1)]
                        if bottom_node in node.neighbors:
                            for i in range(1, vertical_lines + 1):
                                if frozenset([bottom_node, node]) in self.path_connections:
                                    display[display_y + i][display_x] = f'{BLUE}|{RESET}'
                                else:
                                    display[display_y + i][display_x] = '|'
        
        # Create legend
        legend = [
            "Legend:",
            f"  {GREEN}S{RESET} = Start node",
            f"  {RED}E{RESET} = End node",
            f"  {RED}X{RESET} = Pitstop on path",
            f"  {BLUE}X{RESET} = Node on path",
            f"  O = Available node",
            f"  {BLUE}-{RESET} & {BLUE}|{RESET} = Path connection",
            f"  - & | = Available connection"
        ]
        
        # Print the display grid with legend
        print()
        print("0,0")
        for i, row in enumerate(display):
            line = ''.join(row)
            # Add legend to the right side
            if i < len(legend):
                print(f"  {line}    {legend[i]}")
            else:
                print(f"  {line}")
        
        # Print remaining legend lines if display is shorter than legend
        if len(display) < len(legend):
            for i in range(len(display), len(legend)):
                print(' ' * (display_width+2) + f"    {legend[i]}")
        
        print(' ' * (display_width+2) + f"{self.x_size}, {self.y_size}")
        print()
        
    def dijkstra(self, start, end):
        """
            finds shortest path from start to end.
            Returns a list of nodes, or [] if no path exists.
        """
        
        # Priority queue: (distance, node)
        pq = [(0, start)]
        
        # Distance from start to each node
        distances = {start: 0}
        
        # Track previous node in optimal path
        previous = {}
        
        # Visited nodes
        visited = set()
        
        curr_node = None
        while pq:
            
            curr_dist, curr_node = heapq.heappop(pq)
            
            # Skip if already visited
            if curr_node in visited:
                continue
            
            visited.add(curr_node)
            
            # Early exit if we reached the end
            if curr_node == end:
                break
            
            # Explore neighbors
            for neighbor in curr_node.neighbors:
                weight = self.getWeight(curr_node, neighbor)
                
                if neighbor in visited:
                    continue
                
                new_dist = curr_dist + weight
                
                # Update if we found a shorter path
                if neighbor not in distances or new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = curr_node
                    heapq.heappush(pq, (new_dist, neighbor))
        
        # Reconstruct path
        if end not in distances:
            return []
        
        path = []
        node = end
        while node != start:
            path.append(node)
            if node not in previous:
                return []
            node = previous[node]
        path.append(start)
        path.reverse()
        
        return path

    def multi_path_dijkstra(self, nodes):
        
        path = []
        for i in range(len(nodes)-1):
            start_node = nodes[i]
            end_node = nodes[i+1]
            res = self.dijkstra(start_node, end_node)
            
            if not res:  # No path found
                print(f"Error: No path found from {start_node.pos} to {end_node.pos}")
                return []
            
            path += res[:-1]
            
        
        path += [nodes[-1]]

        return path
    
    def convertNodePathToDirections(self, node_path):
        
        if len(node_path) < 2:
            return []
        
        pathDirections = []
        
        curr_dir_x = node_path[1].pos[0] - node_path[0].pos[0]
        curr_dir_y = node_path[1].pos[1] - node_path[0].pos[1]
        
        for i in range(1, len(node_path)-1):
            pathDirections.append(self.getDir(node_path[i].pos, node_path[i+1].pos, curr_dir_x, curr_dir_y))
            
            curr_dir_x = node_path[i+1].pos[0] - node_path[i].pos[0]
            curr_dir_y = node_path[i+1].pos[1] - node_path[i].pos[1]
        
        pathDirections.append("end")
        
        return pathDirections

    def removeEdge(self, node1, node2):
        if node1 not in self.nodes or node2 not in self.nodes:
            raise ValueError("One or more nodes invalid!")
        
        node1.removeConnection(node2)

    def removeNode(self, node):
        
        for neighbor in copy(node.neighbors):
            neighbor.removeConnection(node)
        
        self.nodes.remove(node)
        del self.posToNode[node.pos]
        
    def getDir(self, pos_start, pos_end, dir_x, dir_y):
        """
        Return "left", "right" or "straight" for the move from pos_start to pos_end
        given the incoming direction (dir_x, dir_y) at pos_start.
        """
        
        mv_x = pos_end[0] - pos_start[0]
        mv_y = pos_end[1] - pos_start[1]

        if abs(mv_x) + abs(mv_y) != 1:
            raise ValueError("pos_end must be adjacent to pos_start")
        if (dir_x, dir_y) not in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            raise ValueError("dir_x, dir_y must be one of the cardinal directions")

        cross = dir_x * mv_y - dir_y * mv_x

        dot = dir_x * mv_x + dir_y * mv_y
        if cross == 0:
            
            if dot < 0:
                return "backwards"
            
            return "straight"
        
        return "left" if cross < 0 else "right"
    
    def isInGraph(self, pos):
        return pos in self.posToNode
    
    def isValidPos(self, pos):
        x, y = pos
        
        if pos in self.illegal_positions:
            return False
        
        if not self.isInGraph(pos):
            return False
        
        # raise ValueError("isValid?")
    
        return 0<=x<=self.x_size-1 and 0<=y<=self.y_size-1

        
    def dir3Nodes(self, start_node, intersection_node, end_node):
        """
            returns direction left, straight or right to go from start to end going through intersection.
            intersection must be neighbor of start and end
        """
        
        if not(intersection_node in start_node.neighbors and intersection_node in end_node.neighbors):
            raise ValueError("intersection node must be neighbor of start and end!")
        
        start_x, start_y = start_node.pos
        inter_x, inter_y = intersection_node.pos
        
        dir_x = inter_x-start_x
        dir_y = inter_y-start_y
        
        return self.getDir(intersection_node.pos, end_node.pos, dir_x, dir_y) 
    


def gridSetup():
    
    global gridSizeX, gridSizeY, grid, start_x, start_y, end_x, end_y, start_facing_x, start_facing_y    
    while True: #gridsize
        try:
            gridSize = input("Enter the size of the grid (xy/horizontal-vertical): ")
            assert len(gridSize) == 2
            gridSizeX = int(gridSize[0])
            gridSizeY = int(gridSize[1])
            assert 2 <= gridSizeX <= 5
            assert 2 <= gridSizeY <= 5
            break
        
        except KeyboardInterrupt:
            print("\nUser terminated the program.")
            sys.exit()
        except:
            print("\nHorizontal and vertical grid sizes must be integers from 2 to 5 (inclusive!")
            print("Example input: 45 --> 4 horizontal intersections, 5 vertical intersections\n")
    
    grid = Grid(gridSizeX, gridSizeY)
    
    while True: #xy of start
            grid.display()
            try:
                
                start_pos = input("Enter the coordinates of the robot's start grid point (xy): ")
                assert len(start_pos) == 2
                start_x = int(start_pos[0])
                start_y = int(start_pos[1])
                                
                assert grid.isValidPos((start_x, start_y))
                
                
                break

            except KeyboardInterrupt:
                print("\nUser terminated the program.")
                sys.exit()
            except Exception as e:
                traceback.print_exc()
                print(f"\nThe start point must be on the grid!")
                print(f"(0 <= x <= {grid.x_size-1})")
                print(f"(0 <= y <= {grid.y_size-1})")
                print("Example input: 34 (x=3, y=4).\n")
                
    grid.startNode = grid.posToNode[(start_x, start_y)]
    
    all_dirs_start_positions = [(start_x+1, start_y), (start_x-1, start_y), (start_x, start_y+1), (start_x, start_y-1)]
   
    while True: #starting dir
        
        grid.display()
        print("The starting point's neighboring points are:")


        for i, n in enumerate(all_dirs_start_positions):
            if n in grid.posToNode:
                print(f"{i}-{n}")
            else:
                print(f"{i}-{n} (outside of grid)")
        try:
            facing_point_index = int(input("Enter the number of the point the robot is facing towards: "))
            assert facing_point_index in range(len(all_dirs_start_positions))
            break
        except KeyboardInterrupt:
            print("\nUser terminated the program.")
            sys.exit()
            
        except:
            print("\nPlease choose a valid number associated to one of the starting directions!\n")

    start_facing_x, start_facing_y = all_dirs_start_positions[facing_point_index]


    while True: #xy of end
            try:
                grid.display()
                end_pos = input("Enter the coordinates of the desired end grid point (xy): ")
                assert len(end_pos) == 2
                end_x = int(end_pos[0])
                end_y = int(end_pos[1])
                                
                assert grid.isValidPos((end_x, end_y))
                                
                break

            except KeyboardInterrupt:
                print("\nUser terminated the program.")
                sys.exit()
            except Exception as e:
                traceback.print_exc()
                print(f"\nThe end point must be on the grid!")
                print(f"(0 <= x <= {grid.x_size-1})")
                print(f"(0 <= y <= {grid.y_size-1})")
                print("Example input: 34 (x=3, y=4).\n")
                
    grid.endNode = grid.posToNode[(end_x, end_y)]
       
       
    pitstops = []    
    
    
    while True: #removing nodes
        grid.display()
        while True: #Y/N avoid point
            try:
                foo = input("Would you like to add a point the robot must avoid? (y/n): ")
                assert foo in ("y", "n")
                break
            
            except KeyboardInterrupt:
                print("\nUser terminated the program.")
                sys.exit()
            except:
                print("\nPlease answer with y or n!\n")
    
        if foo == "n":
            break

        while True: #xy of illegal pos
            try:
                grid.display()
                illegal_pos = input("Enter the coordinates of the point you wish to remove (xy): ")
                assert len(illegal_pos) == 2
                illegal_x = int(illegal_pos[0])
                illegal_y = int(illegal_pos[1])
                                
                assert grid.isValidPos((illegal_x, illegal_y))
                
                assert illegal_x != end_x or illegal_y != end_y
                assert illegal_x != start_x or illegal_y != start_y
                
                grid.illegal_positions.add((illegal_x, illegal_y))
                
                grid.removeNode(grid.posToNode[(illegal_x, illegal_y)])
                break

            except KeyboardInterrupt:
                print("\nUser terminated the program.")
                sys.exit()
            except Exception as e:
                print(e)
                print(f"\nIllegal points must be different from the start and end positions and be on the grid.")
                print(f"(You cannot repeat illegal points.)")
                print("Example input: 34 (x=3, y=4).\n")
    
    
    while True: #removing edges
        grid.display()
        while True: #Y/N avoid edge
            try:
                foo = input("Would you like to add a segment (edge) the robot must avoid? (y/n): ")
                assert foo in ("y", "n")
                break
            
            except KeyboardInterrupt:
                print("\nUser terminated the program.")
                sys.exit()
            except:
                print("\nPlease answer with y or n!\n")
    
        if foo == "n":
            break

        while True: #x1y1, x2y2 of nodes to remove connection
            try:
                grid.display()
                illegal_pos = input("Enter the coordinates of the points at the extremities of the edge you wish to remove (x1y1 x2y2): ")
                assert len(illegal_pos) == 5
                assert illegal_pos[2] == " "
                illegal_x1 = int(illegal_pos[0])
                illegal_y1 = int(illegal_pos[1])
                illegal_x2 = int(illegal_pos[3])
                illegal_y2 = int(illegal_pos[4])
                                
                assert grid.isInGraph((illegal_x1, illegal_y1))
                assert grid.isInGraph((illegal_x2, illegal_y2))
                
                node1 = grid.posToNode[(illegal_x1, illegal_y1)]                
                node2 = grid.posToNode[(illegal_x2, illegal_y2)]
                
                assert node1 in node2.neighbors
                assert node2 in node1.neighbors
                node1.removeConnection(node2)

                break

            except KeyboardInterrupt:
                print("\nUser terminated the program.")
                sys.exit()
            except Exception as e:
                print(e)
                print(f"\nThe edge's nodes must be on the grid.")
                print(f"The inputted nodes must be neighbors on the grid.")
                print(f"(You cannot remove the same edge repeatedly.)")
                print("Example input: 34 35 (x1=3, y1=4, x2=3, y2=5).\n")

            
    
                
    while True: #adding pitstops
        grid.display()
        while True: # Y/N add go through point
            try:
                foo = input("Would you like to add a point the robot must go through? (y/n): ")
                assert foo in ("y", "n")
                break
                
            except KeyboardInterrupt:
                print("\nUser terminated the program.")
                sys.exit()
            except:
                print("\nPlease answer with y or n!\n")
    
        if foo == "n":
            break

        while True: #xy of pitstop
            grid.display()
            try:
                pitstop_pos = input("Enter the coordinates of the desired pitstop grid point (xy): ")
                assert len(pitstop_pos) == 2
                pitstop_x = int(pitstop_pos[0])
                pitstop_y = int(pitstop_pos[1])
                                
                assert grid.isValidPos((pitstop_x, pitstop_y))
                
                assert pitstop_x != end_x or pitstop_y != end_y
                assert pitstop_x != start_x or pitstop_y != start_y
                
                break

            except KeyboardInterrupt:
                print("\nUser terminated the program.")
                sys.exit()
            except Exception as e:
                traceback.print_exc()
                print(f"\nPitstop points must be different from the start, end and illegal positions and be on the grid.")
                print("Example input: 34 (x=3, y=4).\n")

        
        pitstops.append((pitstop_x, pitstop_y))
    
        grid.pitstops_for_display.add(grid.posToNode[(pitstop_x, pitstop_y)])
        
        grid.display()
        
    return start_x, start_y, end_x, end_y, start_facing_x, start_facing_y, pitstops

def getGridInstructions():
    
    start_x, start_y, end_x, end_y, start_facing_x, start_facing_y, pitstops = gridSetup()


    if (start_facing_x, start_facing_y) in grid.posToNode:
        INITIAL_DIR_FACING_OUT = False
        allStops = [(start_x, start_y)] + [(start_facing_x, start_facing_y)] + pitstops + [(end_x, end_y)]
    else:
        INITIAL_DIR_FACING_OUT = True
        allStops = [(start_x - (start_facing_x-start_x), start_y - (start_facing_y-start_y))] + [(start_x, start_y)] + pitstops + [(end_x, end_y)]
    
    # print(allStops)
    allStops_nodes = [grid.posToNode[pos] for pos in allStops]
    
    
    # print(allStops_nodes)
    node_path = grid.multi_path_dijkstra(allStops_nodes)
    
    # print(f"Dijkstra: {node_path}")
    
    directions_path = grid.convertNodePathToDirections(node_path)
    
    print()
    
    if len(pitstops) > 0:
        print(f"The robot, starting at ({start_x, start_y}), will go through:")
        for p in pitstops:
            print(f"-{p}")
        print(f"Before ending at ({end_x, end_y})")
    else:
        print(f"Starting at ({start_x, start_y}), the robot will go to ({end_x, end_y}).")
    
    print(f"\nThe chosen path to do so is:")
    
    print(directions_path)
    
    
    grid.path_connections = {frozenset([node_path[i], node_path[i+1]]) for i in range(len(node_path)-1)}
    grid.node_path = node_path
    
    grid.pitstops_for_display = set(allStops_nodes[2:-1])
    
    grid.display()
    input("Press enter to start: ")
    
    return directions_path, grid, node_path, allStops_nodes, INITIAL_DIR_FACING_OUT
        

if __name__ == "__main__":
    
    print(getGridInstructions())
