import heapq
import math
import re
from colorama import Fore, Style

####  CONSTANTS  ####
# Environment
GRID_SIZE = 5  # 5x5 grid
CELL_SIZE = 1.0  # cell size is 1 meter by 1 meter

# Cardinal orientations
EAST = 0
NORTH = 90
WEST = 180
SOUTH = 270
CARDINALS = [EAST, NORTH, WEST, SOUTH]

# Maze8 adjacency list configuration
CELL_GRAPH = {
     1: [ 2,  6],       2: [ 1,  3],   3: [ 2,  4],   4: [ 3,  5],   5: [ 4, 10],
     6: [ 1, 11],       7: [ 8    ],   8: [ 7,  9],   9: [ 8, 10],  10: [ 5,  9],
    11: [ 6, 12, 16],  12: [11, 13],  13: [12, 14],  14: [13, 15],  15: [14, 20],
    16: [11, 17, 21],  17: [16, 18],  18: [17, 19],  19: [18, 20],  20: [15, 19, 25],
    21: [16, 22],      22: [21, 23],  23: [22, 24],  24: [23, 25],  25: [20, 24]
}

# Maze8 wall configuration
WALL_CONFIG = [
    ['O', 'W', 'W', 'O'], ['O', 'W', 'O', 'W'], ['O', 'W', 'O', 'W'], ['O', 'W', 'O', 'W'], ['W', 'W', 'O', 'O'],
    ['W', 'O', 'W', 'O'], ['O', 'W', 'W', 'W'], ['O', 'W', 'O', 'W'], ['O', 'W', 'O', 'W'], ['W', 'O', 'O', 'W'],
    ['O', 'O', 'W', 'O'], ['O', 'W', 'O', 'W'], ['O', 'W', 'O', 'W'], ['O', 'W', 'O', 'W'], ['W', 'W', 'O', 'O'],
    ['O', 'O', 'W', 'O'], ['O', 'W', 'O', 'W'], ['O', 'W', 'O', 'W'], ['O', 'W', 'O', 'W'], ['W', 'O', 'O', 'O'],
    ['O', 'O', 'W', 'W'], ['O', 'W', 'O', 'W'], ['O', 'W', 'O', 'W'], ['O', 'W', 'O', 'W'], ['W', 'O', 'O', 'W'],
]
SPLIT_STR = '|'

# time step for velocity calc
PID_DT = 0.032


class Position: 
    def __init__(self, environment_size = GRID_SIZE, x:int = 0, y:int = 0, cell_number:int = 13, theta:float = 0):
        self.x = x
        self.y = y
        self.cell_number = cell_number
        self.theta = theta
        self.environment_size = environment_size

    def __init__(self, environment_size, position):
        self.environment_size = environment_size
        self.x = position.x
        self.y = position.y
        self.cell_number = int(self.get_cellnumber_from_coordinates(self.y, self.x))
        self.theta = math.degrees(position.theta)
        
    def print_pose(self):
        print(f"\n\t(x:{self.x}, y:{self.y}, n:{int(self.cell_number)}, theta:{round(self.theta)})")

    def update_pose(self, new_cellnumber, new_theta):
        new_y, new_x = self.get_coordinates_from_cellnumber(new_cellnumber)

        self.x, self.y = new_x, new_y
        self.cell_number = new_cellnumber
        self.theta = new_theta
    
    def update_pose(self, y_displacement, x_displacement, new_theta):
        self.x += x_displacement
        self.y += y_displacement
        self.update_cellnumber()
        self.theta = new_theta
    
    def update_cellnumber(self):
        self.cell_number = self.get_cellnumber_from_coordinates(self.y, self.x)

    def update_coordinate(self):
        self.y, self.x = self.get_coordinates_from_cellnumber(self.cell_number)

    def get_coordinates_from_cellnumber(self, cell_number):
        '''
        Assumes that the grid size is odd and center is 0,0
        
        Args:
            grid_size (int) : size of the gride or the number of cells it is wide and long
            cell_number (int) : the cell number for coordinates calculation
        
        Returns:
            y and x coordinate for the cell number given the grid size
        '''

        cell_number -= 1
        horizontal_depth = cell_number % GRID_SIZE
        vertical_depth = cell_number // GRID_SIZE
        x_coordinate = horizontal_depth - (GRID_SIZE//2)
        y_coordinate = (GRID_SIZE//2) - vertical_depth

        return y_coordinate, x_coordinate

    def get_cellnumber_from_coordinates(self, y, x) -> int:
        """
        Returns the cell number for given (x, y) coordinates in a matrix.

        Args:
            x (int): The x-coordinate in the environment.
            y (int): The y-coordinate in the environment.
            matrix_size (int): The size of the matrix (must be odd).

        Returns:
            int: The cell number (1-based index).
        """
        if GRID_SIZE % 2 == 0:
            raise ValueError("Matrix size must be odd.")

        # Calculate the center of the matrix
        center = GRID_SIZE // 2

        # Convert (x, y) coordinates to row and column indices
        col = x + center
        row = center - y

        # Ensure coordinates are within bounds
        if not (0 <= col < GRID_SIZE and 0 <= row < GRID_SIZE):
            raise ValueError(f"Coordinates ({x}, {y}) are out of bounds for a {GRID_SIZE}x{GRID_SIZE} matrix.")

        # Calculate cell number (1-based index)
        cell_number = row * GRID_SIZE + col + 1

        return cell_number

def get_coordinates_from_cellnumber_env(cell_number):
    '''
    Assumes that the grid size is odd and center is 0,0
    
    Args:
        grid_size (int) : size of the gride or the number of cells it is wide and long
        cell_number (int) : the cell number for coordinates calculation
    
    Returns:
        y and x coordinate for the cell number given the grid size
    '''

    cell_number -= 1
    horizontal_depth = cell_number % GRID_SIZE
    vertical_depth = cell_number // GRID_SIZE
    x_coordinate = horizontal_depth - (GRID_SIZE//2)
    y_coordinate = (GRID_SIZE//2) - vertical_depth

    return y_coordinate, x_coordinate

def get_cellnumber_from_coordinates_env(y, x, size) -> int:
    """
    Returns the cell number for given (x, y) coordinates in a matrix.

    Args:
        x (int): The x-coordinate in the environment.
        y (int): The y-coordinate in the environment.
        matrix_size (int): The size of the matrix (must be odd).

    Returns:
        int: The cell number (1-based index).
    """
    if size % 2 == 0:
        raise ValueError("Matrix size must be odd.")

    # Calculate the center of the matrix
    center = size // 2

    # Convert (x, y) coordinates to row and column indices
    col = x + center
    row = center - y

    # Ensure coordinates are within bounds
    if not (0 <= col < size and 0 <= row < size):
        raise ValueError(f"Coordinates ({x}, {y}) are out of bounds for a {size}x{size} matrix.")

    # Calculate cell number (1-based index)
    cell_number = row * size + col + 1

    return cell_number

def get_coordinates_from_cellnumber_matrix(cell_number):
    row = cell_number // GRID_SIZE
    col = (cell_number - 1) % GRID_SIZE

    return row, col

def get_cellnumber_from_coordinates_matrix(row, column) -> int:
    return row * GRID_SIZE + column + 1

def get_cardinal_between_neighbors(start_cell, neighbor_cell):
    cardinal = ""
    start_y, start_x = get_coordinates_from_cellnumber_env(start_cell)
    neighbor_y, neighbor_x = get_coordinates_from_cellnumber_env(neighbor_cell)
    y_displacement = neighbor_y - start_y
    x_displacement = neighbor_x - start_x
    
    if x_displacement == 1:
        cardinal = 'E'
    elif y_displacement == 1:
        cardinal = 'N'
    elif x_displacement == -1:
        cardinal = 'W'
    elif y_displacement == -1:
        cardinal = 'S'
    else:
        raise Exception("Error within get_cadinal_direction_between(). Cells are either not cardinal neighbors.")

    return cardinal

# get shortest path calculation by Dijkstra’s algorithm
def get_shortest_path(CELL_GRAPH, start, goal):
    '''
    Using dijkstra's algorithm, returns the shortest path from start to goal. 
    
    Args:
        start (int) : the cell number of the start cell
        goal (int) : the cell number of the goal cell
    
    Returns:
        Path string is the sequence of path cells.
    '''
    # TODO helper function to turn paths into their RLE forms. WWNNEEEESWWW == 2W2N4E1S3W. Note seems only better with larger grid sizes.

    distances = {node: (float('inf')) for node in CELL_GRAPH}
    
    distances[start] = 0
    priority_que = [(0, start)]
    
    predecessors = {}
    
    pedecessor_iterator = goal
    
    path = []

    # Dijkstra's algorithm execution
    while priority_que:
    
        current_distance, current_node = heapq.heappop(priority_que)  # declare and initialize th prority_queue
        
        if current_node == goal:   # stopping condition when the goal is reached
            break
            
        for neighbor in CELL_GRAPH[current_node]:
            distance = current_distance +  + CELL_SIZE   # The edges weight is always CELL_SIZE = 1 
            
            if distance < distances[neighbor]:  # Find Minimal Distance to this Neighbor
            
                distances[neighbor] = distance
                
                predecessors[neighbor] = current_node
                
                heapq.heappush(priority_que, (distance, neighbor))

    # Path reconstruction
    while pedecessor_iterator != start:
    
        path.append(str(pedecessor_iterator))
        
        pedecessor_iterator = predecessors[pedecessor_iterator]
        
    path.append(str(start))
    path.reverse()
    
    # return path string
    return SPLIT_STR.join(path), distances[goal]

# Helper function to calculate visible length of a string
def visible_length(text):
    color_code_pattern = re.compile(r'\x1b\[[0-9;]*m')  # Matches ANSI color codes
    return len(color_code_pattern.sub('', text))  # Remove color codes and calculate length

# Helper function to center text with color codes
def color_center(text, width):
    text_length = visible_length(text)
    padding = (width - text_length) // 2
    return " " * padding + text + " " * (width - text_length - padding)

def print_path(grid_size: int, path: str, current_cell: int):
    '''
    Displays the path in the grid and the position given. Intended to represent the current location of the robot with it's path.
    '''

    # Get a list of cells in the path by splitting up the path string
    path_list = path.split(SPLIT_STR)

    # Generate the grid with cell numbers
    grid = [[col + row * grid_size + 1 for col in range(grid_size)] for row in range(grid_size)]

    # Generate a colored path list
    colored_path_list = []
    for i in range(len(path_list)):
        cell = path_list[i]
        if cell == str(current_cell):    # current
            colored_path_list.append(f"{Fore.RED}{cell}{Style.RESET_ALL}")
        elif cell == path_list[0]:       # start
            colored_path_list.append(f"{Fore.GREEN}{cell}{Style.RESET_ALL}")
        elif cell == path_list[-1]:      # goal
            colored_path_list.append(f"{Fore.YELLOW}{cell}{Style.RESET_ALL}")
        else:
            colored_path_list.append(f"{Fore.BLUE}{cell}{Style.RESET_ALL}")

    # Calculate the width needed for formatting
    max_width = len(str(grid_size*grid_size))

    # Print cell numbers on path
    print("\nCells on path:")
    print(", ".join(colored_path_list))

    # Print path on cell map and current position
    print("\nCell Map:")
    print("-|-".join(["-"*max_width] * grid_size))
    for row in range(grid_size):
        row_output = [] 
        for col in range(grid_size):
            cell_number = str(get_cellnumber_from_coordinates_matrix(row, col))
            if cell_number == str(current_cell): # current position
                cell = f"{Fore.RED}{cell_number}{Style.RESET_ALL}"
            elif cell_number == path_list[0]:    # start cell
                cell = f"{Fore.GREEN}{cell_number}{Style.RESET_ALL}"
            elif cell_number == path_list[-1]:   # goal cell
                cell = f"{Fore.YELLOW}{cell_number}{Style.RESET_ALL}"
            elif cell_number in path_list:  # path cell
                cell = f"{Fore.BLUE}{cell_number}{Style.RESET_ALL}"
            else: # non-path cell
                cell = str(grid[row][col])
            row_output.append(color_center(cell, max_width))
        
        print(" | ".join(row_output))
        print("-|-".join(["-"*max_width] * grid_size))
    
    print(f"{Fore.BLUE}path{Style.RESET_ALL}, {Fore.GREEN}starting cell{Style.RESET_ALL}, {Fore.RED}current location{Style.RESET_ALL}, {Fore.YELLOW}goal{Style.RESET_ALL}")

def get_navigation_from_path(cell_path, matrix_size):
    """
    Converts a path string of cell numbers into a navigation string with cardinal directions.

    Args:
        cell_path (str): A string of cell numbers separated by '|', e.g., "1|2|7|12".
        matrix_size (int): The size of the matrix (must be odd).

    Returns:
        str: A string of cardinal directions (E, N, W, S) representing the path.
    """
    moves = {
        (1, 0): "E",  # Move East
        (0, 1): "N",  # Move North
        (-1, 0): "W", # Move West
        (0, -1): "S"  # Move South
    }

    # Split the path into a list of integers
    cell_numbers = list(map(int, cell_path.split('|')))

    # Initialize navigation string
    navigation = []

    # Generate navigation instructions
    for i in range(len(cell_numbers) - 1):
        y1, x1 = get_coordinates_from_cellnumber_env(cell_numbers[i])
        y2, x2 = get_coordinates_from_cellnumber_env(cell_numbers[i + 1])
        dx, dy = x2 - x1, y2 - y1
        direction = moves.get((dx, dy))
        if not direction:
            raise ValueError(f"Invalid path: No direct move from {cell_numbers[i]} to {cell_numbers[i + 1]}")
        navigation.append(direction)

    return "".join(navigation)
    
# Gather highest weighted cell probabilities
def get_highest_probability_cells(probabilities):
    max_prob = max(probabilities)
    highest_prob_cells = [i + 1 for i, prob in enumerate(probabilities) if prob == max_prob]
    return highest_prob_cells

def print_probabilities(probabilities):

    '''TODO: Print in a grid'''

    for cell, prob in enumerate(probabilities, start=1):
        print(f"Cell {cell}: Probability = {prob:.3f}")

def get_next_position(current_position, cardinal_direction):
    """
    Calculate the next position to move to based on the current position.

    Args:
        current_position (tuple): The robot's current grid position as (r, c).

    Returns:
        tuple: The next grid position as (r, c).
    """
    r, c = current_position
    result = None

    if cardinal_direction == EAST    : result = (r, c + 1)
    elif cardinal_direction == NORTH : result = (r + 1, c)
    elif cardinal_direction == WEST  : result = (r, c - 1)
    elif cardinal_direction == SOUTH : result = (r - 1, c)

    return result

def get_key_from_position(position):
    return f"{position[0]}-{position[1]}"

def path_logic(visited, ignore_visited, nxt_position):
    # summary - based on visited locations
    # the idea is to start with following the left wall and ignore_visited flag 
    # at every cell determine what is the coordinate you are traversing to next
    # ask if it is that cell has been visited
        # if it is and ignore_visited == False
            # switch wall followings
            # ignore_visited = True
        # if it has NOT
            # ignore_visited = False
            # keep following the same wall
    
    wall_following_changes = False
    
    if get_key_from_position(nxt_position) in visited and not ignore_visited:
        wall_following_changes = True
        ignore_visited = True
    else:
        ignore_visited = False

    return wall_following_changes, ignore_visited

    