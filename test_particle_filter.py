import pytest
from particle_filter_helper import (
    CELL_GRAPH, GRID_SIZE,
    get_coordinates_from_cellnumber_env, 
    get_cardinal_between_neighbors, 
    old_get_all_shortest_paths,
    old_get_shortest_path,
    get_shortest_path,
    get_navigation_from_path
)

@pytest.mark.parametrize(
    "cell_number, expected",
    [
        ( 1,(2,-2)), # Cell 1 at    2, -2
        (13,(0,0)), # Cell 13 at   0,  0
        (25,(-2,2)) # Cell 25 at  -2,  2
    ]
)
def test_get_coordinates_from_cellnumber(cell_number, expected):
    assert get_coordinates_from_cellnumber_env(cell_number) == expected

@pytest.mark.parametrize(
    "start, neighbor, expected",
    [
        ( 1,  2, "E"), # East 
        (13,  8, "N"), # North
        (25, 24, "W"), # West
        (14, 19, "S")  # South
    ]
)
def test_get_cardinal_between_neighbors(start, neighbor, expected):
    assert get_cardinal_between_neighbors(start, neighbor) == expected

def test_get_all_shortest_paths_old():
    start = 13
    expected = {
         1: (4, "WWNN"),  2: ( 5, "WWNNE"),         3: ( 6, "WWNNEE"),       4: ( 7, "WWNNEEE"),     5: (8, "WWNNEEEE"),
         6: (3, "WWN"),   7: (12, "WWNNEEEESWWW"),  8: (11, "WWNNEEEESWW"),  9: (10, "WWNNEEEESW"), 10: (9, "WWNNEEEES"),
        11: (2, "WW"),   12: ( 1, "W"),            13: ( 0, ""),            14: ( 1, "E"),          15: (2, "EE"),
        16: (3, "WWS"),  17: ( 4, "WWSE"),         18: ( 5, "WWSEE"),       19: ( 4, "EESW"),       20: (3, "EES"),
        21: (4, "WWSS"), 22: ( 5, "WWSSE"),        23: ( 6, "WWSSEE"),      24: ( 5, "EESSW"),      25: (4, "EESS"), 
    }
    assert old_get_all_shortest_paths(start) == expected

def test_get_shortest_path_old():
    start, goal = 7, 21
    expected = "EEENWWWWSSSS"
    assert old_get_shortest_path(start, goal) == expected

def test_get_shortest_path():
    start, goal = 7, 21
    expected = "7|8|9|10|5|4|3|2|1|6|11|16|21"
    assert get_shortest_path(CELL_GRAPH, 7, 21)

def test_get_navigation_from_path():
    cell_path = "7|8|9|10|5|4|3|2|1|6|11|16|21"
    expected = "EEENWWWWSSSS"
    assert get_navigation_from_path(cell_path, GRID_SIZE)

