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