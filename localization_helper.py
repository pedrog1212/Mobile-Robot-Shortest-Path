from constants import WALL_CONFIG, CARDINALS, EAST, NORTH, WEST, SOUTH

# Sensor model
def calculate_cell_probabilities(wall_readings):
    probabilities = []
    for cell_idx, cell_config in enumerate(WALL_CONFIG):
        #print(f"     Cell #: {cell_idx+1}    config:{cell_config}")
        cell_prob = 1.0
        for i, cardinal in enumerate(CARDINALS) :
            s = 1 if cell_config[i] == 'W' else 0
            z = 1 if wall_readings[cardinal] is True else 0   
            #print(f"     cardinal: {cardinal}")
            #print(f"     z = {z} | s = {1}")
            if z == 1 and s == 1:
                cell_prob *= 0.8
            elif z == 1 and s == 0:
                cell_prob *= 0.4
            elif z == 0 and s == 1:
                cell_prob *= 0.2
            elif z == 0 and s == 0:
                cell_prob *= 0.6
        probabilities.append(round(cell_prob,5))
    #print(probabilities)
    
    # normalize probabilities
    sum_of_probs = sum(probabilities)
    norm_probs = [prob/sum_of_probs for prob in probabilities]
    
    return norm_probs

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