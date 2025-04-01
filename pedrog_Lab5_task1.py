import math
import time
from fairis_tools.my_robot import MyRobot

from pedrog_Lab5_MyRobot import (
    GRID_SIZE, CELL_SIZE, CELL_GRAPH,
    WALL_CONFIG, CARDINALS, EAST, NORTH, WEST, SOUTH, PID_DT,
    get_highest_probability_cells, 
    get_next_position,
    path_logic,
    get_key_from_position,
    SPLIT_STR,
    Position,
    get_coordinates_from_cellnumber_env,
    get_cellnumber_from_coordinates_env,
    get_coordinates_from_cellnumber_matrix,
    get_cellnumber_from_coordinates_matrix,
    get_cardinal_between_neighbors,
    get_shortest_path,
    print_path,
    get_navigation_from_path,
)

# Initialize robot and sensors
robot = MyRobot()

# Robot constants
MAX_ROTATIONAL = 4.0
MAX_VELOCITY = robot.max_motor_velocity
ADD_VELOCITY = 3.0

# Camera constants
CAMERA_WIDTH = robot.rgb_camera.getWidth()
CAMERA_CENTER_X = CAMERA_WIDTH / 2

# PID controller tolerances
THRESHOLD_FROM_WALL = 1
ORIENTATION_TOLERANCE = math.radians(0.1)
MOVEMENT_TOLERANCE = 0.001

# Load maze environment
robot.load_environment('../../worlds/Fall24/maze8.xml')

# Get the time step 
timestep = int(robot.getBasicTimeStep())

# Move robot to a random staring position 
robot.move_to_start()

##  FUNCTIONALITY   ## 

# Function to orient the robot to a target orientation
def pid_set_orientation(target):
    """
    Orient the robot to a specified orientation using a PID controller,
    using the robot's compass utility and accomplished via the robots motor velocities.

    Args:
        target_orientation_rad (float): The desired orientation in degrees.
    """
    
    # Check to change offet to compass readings for orientation to take the shortest path
    compass_offset = 0
    if abs(target - robot.get_compass_reading()) > 180:
        target = (target + 180) % 360
        compass_offset = 180
        
    # Convert target orientation from degrees to radians
    target_rads = math.radians(target)
    
    # PID control parameters
    Kp = 8.0   # Proportional gain
    Ki = 0.01  # Integral gain
    Kd = 2.0   # Derivative gain

    # Error values for PID
    prev_error = 0.0
    integral = 0.0

    while robot.step(timestep) != -1:
            
        # Get the robot's current orientation
        current_orientation = math.radians((robot.get_compass_reading() + compass_offset) % 360)  # In radians
        #print(f"current_orientation={current_orientation:.4f}m")

        # Calculate the error in orientation
        error = target_rads - current_orientation
        #print(f"error={error:.4f}m")

        # If the error is within tolerance, stop the robot and return success
        if abs(error) <= ORIENTATION_TOLERANCE:
            # print(f"    Robot oriented to {math.degrees(target_rads):.4f} degrees.")
            robot.set_left_motors_velocity(0)
            robot.set_right_motors_velocity(0)
            return True

        # PID calculations
        proportional = Kp * error
        integral += error * Ki * PID_DT
        derivative = Kd * (error - prev_error) / PID_DT

        # Compute control signal
        control_signal = proportional + integral + derivative
        prev_error = error
        
        # Reduce motor speed dynamically as error decreases
        if abs(error) < math.radians(10):  
            control_signal *= 0.5

        # Limit the control signal to the maximum motor velocity range [-4, 4]
        control_signal = max(min(control_signal, MAX_ROTATIONAL), -MAX_ROTATIONAL)
        #print(f"control_signal={control_signal:.4f}m")

        # Set motor velocities based on control signal
        robot.set_left_motors_velocity(-control_signal)
        robot.set_right_motors_velocity(control_signal)
        
        # print(f"     current orienation {math.degrees(current_orientation):.4f}     control signal: {control_signal:.2f}\n")

def normalize_degree(angle):
    """Normalize an angle to be within the range [-π, π]."""
    return (math.radians(angle) + math.pi) % (2 * math.pi) - math.pi

def set_orientation_east(is_print:bool):
    if is_print: print(f"Orienting to the EAST:{EAST}")
    pid_set_orientation(EAST)

def set_orientation_north(is_print:bool):
    if is_print: print(f"Orienting to the NORTH:{NORTH}")
    pid_set_orientation(NORTH)

def set_orientation_west(is_print:bool):
    if is_print: print(f"Orienting to the WEST:{WEST}")
    pid_set_orientation(WEST)

def set_orientation_south(is_print:bool):
    if is_print: print(f"Orienting to the SOUTH:{SOUTH}")
    pid_set_orientation(SOUTH)

def set_orientation_cardinal_reverse():
    print(f"Reversing orientation")
    current = get_current_facing_cardinal()
    opposites = {EAST: WEST, NORTH: SOUTH, 
                 WEST: EAST, SOUTH: NORTH}
    pid_set_orientation(opposites[current])

def set_orientation_turn_left_90():
    left_turns = {
        EAST  : NORTH,
        NORTH : WEST,
        WEST  : SOUTH,
        SOUTH : EAST
    }
    facing_cardinal = get_current_facing_cardinal()
    pid_set_orientation(left_turns[facing_cardinal])
    
def set_orientation_turn_right_90():
    right_turns = {
        EAST  : SOUTH,
        NORTH : EAST,
        WEST  : NORTH,
        SOUTH : WEST
    }
    facing_cardinal = get_current_facing_cardinal()
    pid_set_orientation(right_turns[facing_cardinal])
    
def get_current_facing_cardinal():
    current = normalize_degree(robot.get_compass_reading())
    #print(f"     normalized current: {math.degrees(current)}")
    #print(f"     list of cardinals: E:{math.degrees(normalize_degree(EAST))}   N:{math.degrees(normalize_degree(NORTH))}   W:{math.degrees(normalize_degree(WEST))}   -W:{math.degrees(normalize_degree(-WEST))}   S:{math.degrees(normalize_degree(SOUTH))}")    
    
    cardinals = {EAST : normalize_degree(EAST), 
                 NORTH: normalize_degree(NORTH), 
                 -WEST: normalize_degree(WEST),
                 WEST : -normalize_degree(WEST),
                 SOUTH: normalize_degree(SOUTH)}
                 
    closest_cardinal = min(cardinals, key=lambda cardinal: abs(cardinals[cardinal] - current))
    #print(f"     closest cardinal: {closest_cardinal}")
    
    return abs(closest_cardinal)

# Fuction to move the robot a target distance forward
def pid_move_forward(target_distance, start_timestep):
    """
    Moves the robot forward by a specified target distance (in meters) using a PID controller
    based on the front-left motor encoder readings.

    Args:
        target_distance (float): The target distance to move forward, in meters.
        timestep (int): The simulation timestep for robot.step.
    """
    # PID control parameters
    Kp = 15.0  # Proportional gain
    Ki = 0.05  # Integral gain
    Kd = 0.5   # Derivative gain

    # Encoder readings
    initial_encoder = robot.get_front_left_motor_encoder_reading()

    # Error values for PID
    prev_error = 0
    integral = 0
    
    # Initialize control_signal
    control_signal = 4.0
    control_prev_signal = 0

    # print(f"     Starting PID-based forward movement: Target Distance={target_distance:.2f} meters")

    while robot.step(robot.timestep) != -1:
            
        # Get the current encoder reading
        current_encoder = robot.get_front_left_motor_encoder_reading()

        # Calculate the distance traveled based on the encoder
        radians_traveled = current_encoder - initial_encoder
        distance_traveled = robot.wheel_radius * radians_traveled

        # Calculate the error
        error = target_distance - distance_traveled

        # If the robot has traveled the target distance within the tolerance, stop
        if abs(error) <= MOVEMENT_TOLERANCE:
            robot.set_left_motors_velocity(0)
            robot.set_right_motors_velocity(0)
            # print(f"     Movement completed: Traveled Distance={distance_traveled:.2f} meters")
            return True, control_signal

        # PID calculations
        proportional = Kp * error
        integral += error * PID_DT  # Convert timestep to seconds
        derivative = Kd * (error - prev_error) / PID_DT
        prev_error = error

        # Compute control signal
        control_signal = proportional + integral + derivative + ADD_VELOCITY

        # Limit control signal to maximum motor velocity
        control_signal = max(min(control_signal, MAX_VELOCITY), -MAX_VELOCITY)

        # Set motor velocities to move forward
        robot.set_left_motors_velocity(control_signal)
        robot.set_right_motors_velocity(control_signal)

        control_prev_signal = control_signal

        # Debugging output
        # print(f"     Encoder Reading: {current_encoder:.2f}, Distance Traveled: {distance_traveled:.2f}, "
        #      f"     Error: {error:.2f}, Control Signal: {control_signal:.2f}")
        # print(f"Servomotors Linear Velocity: {control_signal :.2f} m/s")
    return False, control_signal
    
def move_forward_one_cell(is_print:bool = True):
    if is_print: print(f"Moving forward 1 cell")
    pid_move_forward(CELL_SIZE, robot.timestep) 

def move_forward_x_cells(x, is_print:bool = True):
    for i in range(x):
        move_forward_one_cell(is_print)

def get_front_lidar_reading():
    return robot.get_lidar_range_image()[400]
    
def get_left_lidar_reading():
    return robot.get_lidar_range_image()[200]

def get_right_lidar_reading():
    return robot.get_lidar_range_image()[600]

def get_rear_lidar_reading():
    return robot.get_lidar_range_image()[0]

def get_oriented_lidar_readings():
    current_cardinal = get_current_facing_cardinal()
    result = False
    
    lidar_readings = {
        EAST: {
            EAST : get_front_lidar_reading(),
            NORTH: get_left_lidar_reading(),
            WEST : get_rear_lidar_reading(),
            SOUTH: get_right_lidar_reading(),
        },
        NORTH: {
            EAST : get_right_lidar_reading(),
            NORTH: get_front_lidar_reading(),
            WEST : get_left_lidar_reading(),
            SOUTH: get_rear_lidar_reading(),
        },
        WEST: {
            EAST : get_rear_lidar_reading(),
            NORTH: get_right_lidar_reading(),
            WEST : get_front_lidar_reading(),
            SOUTH: get_left_lidar_reading(),
        },
        SOUTH: {
            EAST : get_left_lidar_reading(),
            NORTH: get_rear_lidar_reading(),
            WEST : get_right_lidar_reading(),
            SOUTH: get_front_lidar_reading(),
        }
    }
    
    # Ensure that the current cardinal is valid
    if current_cardinal not in lidar_readings:
        raise ValueError(f"Invalid direction: {current_cardinal}.")
        
    return lidar_readings[current_cardinal]

def get_walls():
    lidar_readings = get_oriented_lidar_readings()
    walls_dict = {cardinal: lidar_readings[cardinal] < THRESHOLD_FROM_WALL
                  for cardinal in lidar_readings}
    return walls_dict

def is_dead_end():
    return get_front_lidar_reading() < THRESHOLD_FROM_WALL and \
           get_left_lidar_reading() < THRESHOLD_FROM_WALL and \
           get_right_lidar_reading() < THRESHOLD_FROM_WALL
    
def is_front_wall():
    return get_front_lidar_reading() < THRESHOLD_FROM_WALL

def is_left_wall():
    return get_left_lidar_reading() < THRESHOLD_FROM_WALL
    
def is_right_wall():
    return get_right_lidar_reading() < THRESHOLD_FROM_WALL
    
def is_rear_wall():
    return get_rear_lidar_reading() < THRESHOLD_FROM_WALL
    
def print_walls(direction):
    print(f"Direction of reading: {direction}")
    walls = get_walls()
    
    # Top and bottom borders, adjust depending on walls
    top_wall = " -------"  # Default for no wall
    if walls[NORTH]:  # North wall detected
        top_wall = " XXXXXXX"
    
    bottom_wall = " -------"  # Default for no wall
    if walls[SOUTH]:  # South wall detected
        bottom_wall = " XXXXXXX"
    
    # Side walls (left and right)
    west_wall = 'X' if walls[WEST] else '|'
    east_wall = 'X' if walls[EAST] else '|'
    
    # Now, build the string for the box:
    print(top_wall)  # Top border
    
    # Middle parts of the box (left and right walls)
    for _ in range(3):  # Print three middle lines with possible walls
        print(f"{west_wall}       {east_wall}")
    
    print(bottom_wall)  # Bottom border


# Navigation logic for l5t1 - Mapping & Path Planning
def navigate_path_planning(goal_cell):

    current_position = Position(GRID_SIZE, robot.starting_position)
    
    start_cell = current_position.cell_number
    
    path_str, shortest_distance = get_shortest_path(CELL_GRAPH, start_cell, goal_cell)  # call shortest path and distance calculation by Dijkstra’s algorithm
    
    nav_str = get_navigation_from_path(path_str, GRID_SIZE)
    
    nav_list = ' '.join(nav_str)
    
    path_list = path_str.split(SPLIT_STR)
    
    print("\n ")
    print(f"Path:\t{path_str}\n \nShortest Distance:    {shortest_distance:.2f} meters")
    print("\n ")
    print(f"Navigation:\t{nav_list}")
    print("\n ")
    print("Pose Estimation:")
    
    current_position.print_pose()

    heading_itr = 0
    while robot.timestep != -1 and heading_itr < len(nav_str):
        
        nav_heading = nav_str[heading_itr]

        if nav_heading == "E":
            set_orientation_east(is_print=False)
            current_position.update_pose(0, 1, robot.get_compass_reading())
        elif nav_heading == "N":
            set_orientation_north(is_print=False)
            current_position.update_pose(1, 0, robot.get_compass_reading())
        elif nav_heading == "W":
            set_orientation_west(is_print=False)
            current_position.update_pose(0, -1, robot.get_compass_reading())
        elif nav_heading == "S":
            set_orientation_south(is_print=False)
            current_position.update_pose(-1, 0, robot.get_compass_reading())
        else:
            raise Exception(f"Navigation string error. {nav_heading} is not a valid heading.")

        move_forward_one_cell(is_print = False)

        current_position.print_pose()

        heading_itr += 1

    print("\n ")
    print("\nDestination has been reached.")


# Main program
def main():
    start_time = time.time()

    print("Program Start.")
    
    goal = 7
    print(f"Goal shall be cell number: {goal}")
    #navigate_localization(robot)
    navigate_path_planning(goal)

    # Calculate total travel time
    end_time = time.time()
    travel_time = end_time - start_time
    print(f"Total travel time: {travel_time:.2f} seconds\n ")

if __name__ == "__main__":
    main()