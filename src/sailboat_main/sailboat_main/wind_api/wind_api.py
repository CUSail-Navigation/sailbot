import math
import numpy as np
import openmeteo_requests
from scipy.spatial import cKDTree


def make_coords(top_left, bot_right, tile_size=1.0):
    tile_lat = tile_size * (1 / 111.132)
    avg_lat = (top_left[0] + bot_right[0]) / 2
    tile_long = tile_size * (1 / (111.320 * math.cos(math.radians(avg_lat))))

    lat_delta = abs(bot_right[0] - top_left[0])
    long_delta = abs(bot_right[1] - top_left[1])

    num_rows = math.ceil(lat_delta / tile_lat) + 1
    num_cols = math.ceil(long_delta / tile_long) + 1
    lats = []
    longs = []

    for i in range(num_rows):
        for j in range(num_cols):
            new_lat = top_left[0] - (tile_lat * i)
            new_long = top_left[1] + (tile_long * j)
            lats.append(new_lat)
            longs.append(new_long)

    return lats, longs, num_rows, num_cols

def get_wind(lats, longs):
    openmeteo = openmeteo_requests.Client()

    response_coords = []
    wind_vectors = []

    url = "https://api.open-meteo.com/v1/forecast"
    params = {
        "latitude": lats,
        "longitude": longs,
        "current": ["wind_speed_10m", "wind_direction_10m"],
        "cell_selection": "sea"
    }
    responses = openmeteo.weather_api(url, params=params)

    for response in responses:
        current = response.Current()
        current_wind_speed_10m = current.Variables(0).Value()
        current_wind_direction_10m = current.Variables(1).Value()
        if (response.Latitude(), response.Longitude()) not in response_coords:
            wind_vectors.append((current_wind_speed_10m, current_wind_direction_10m))
            response_coords.append((response.Latitude(), response.Longitude()))
    
    return response_coords, wind_vectors

<<<<<<< HEAD
def get_wind_direction(lats, longs):
    openmeteo = openmeteo_requests.Client()

    response_coords = []
    wind_vectors = []

    url = "https://api.open-meteo.com/v1/forecast"
    params = {
        "latitude": lats,
        "longitude": longs,
        "current": ["wind_speed_10m", "wind_direction_10m"],
        "cell_selection": "sea"
    }
    responses = openmeteo.weather_api(url, params=params)

    for response in responses:
        current = response.Current()
        current_wind_speed_10m = current.Variables(0).Value()
        current_wind_direction_10m = current.Variables(1).Value()
        if (response.Latitude(), response.Longitude()) not in response_coords:
            wind_vectors.append(current_wind_direction_10m)
            response_coords.append((response.Latitude(), response.Longitude()))

    return response_coords, wind_vectors

top_left = (42.454862524617326, -76.47741838081413)
bot_right = (42.45394077761382, -76.47648601746894)
big_lats, big_longs, big_num_rows, big_num_cols = make_coords(top_left, bot_right)
response_coords, wind_vectors = get_wind(big_lats, big_longs)
lats, longs, num_rows, num_cols = make_coords(top_left, bot_right, 0.01)
print(response_coords)
print(wind_vectors)
print(list(zip(lats, longs)))

# Python program for A* Search Algorithm
import math
import heapq

# Define the Cell class
class Cell:
    def __init__(self):
      # Parent cell's row index
        self.parent_i = 0
    # Parent cell's column index
        self.parent_j = 0
 # Total cost of the cell (g + h)
        self.f = float('inf')
    # Cost from start to this cell
        self.g = float('inf')
    # Heuristic cost from this cell to destination
        self.h = 0

# Define the size of the grid
ROW = 10
COL = 10

# Check if a cell is valid (within the grid)
def is_valid(row, col):
    return (row >= 0) and (row < ROW) and (col >= 0) and (col < COL)

# Check if a cell is unblocked
def is_unblocked(grid, row, col):
    return grid[row][col] == 1

# Check if a cell is the destination
def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

# Calculate the heuristic value of a cell (Euclidean distance to destination)
def calculate_h_value(row, col, dest):
    return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5

# Trace the path from source to destination
def trace_path(cell_details, dest):
    print("The Path is ")
    path = []
    row = dest[0]
    col = dest[1]

    # Trace the path from destination to source using parent cells
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        row = temp_row
        col = temp_col

    # Add the source cell to the path
    path.append((row, col))

    # Reverse the path to get the path from source to destination
    path.reverse()

    # Print the path
    for i in path:
        print("->", i, end=" ")
    print()

# Implement the A* search algorithm
def a_star_search(grid, src, dest):
    # Check if the source and destination are valid
    if not is_valid(src[0], src[1]) or not is_valid(dest[0], dest[1]):
        print("Source or destination is invalid")
        return

    # Check if the source and destination are unblocked
    if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(grid, dest[0], dest[1]):
        print("Source or the destination is blocked")
        return

    # Check if we are already at the destination
    if is_destination(src[0], src[1], dest):
        print("We are already at the destination")
        return

    # Initialize the closed list (visited cells)
    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    # Initialize the details of each cell
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

    # Initialize the start cell details
    i = src[0]
    j = src[1]
    cell_details[i][j].f = 0
    cell_details[i][j].g = 0
    cell_details[i][j].h = 0
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j

    # Initialize the open list (cells to be visited) with the start cell
    open_list = []
    heapq.heappush(open_list, (0.0, i, j))

    # Initialize the flag for whether destination is found
    found_dest = False

    # Main loop of A* search algorithm
    while len(open_list) > 0:
        # Pop the cell with the smallest f value from the open list
        p = heapq.heappop(open_list)

        # Mark the cell as visited
        i = p[1]
        j = p[2]
        closed_list[i][j] = True

        # Dynamically update grid based on current cell
        grid = update_grid(grid, i, j)

        # For each direction, check the successors
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0),
                      (1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dir in directions:
            new_i = i + dir[0]
            new_j = j + dir[1]

            # If the successor is valid, unblocked, and not visited
            if is_valid(new_i, new_j) and is_unblocked(grid, new_i, new_j) and not closed_list[new_i][new_j]:
                # If the successor is the destination
                if is_destination(new_i, new_j, dest):
                    # Set the parent of the destination cell
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    print("The destination cell is found")
                    # Trace and print the path from source to destination
                    trace_path(cell_details, dest)
                    found_dest = True
                    return
                else:
                    # Calculate the new f, g, and h values
                    g_new = cell_details[i][j].g + 1.0
                    h_new = calculate_h_value(new_i, new_j, dest)
                    f_new = g_new + h_new

                    # If the cell is not in the open list or the new f value is smaller
                    if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                        # Add the cell to the open list
                        heapq.heappush(open_list, (f_new, new_i, new_j))
                        # Update the cell details
                        cell_details[new_i][new_j].f = f_new
                        cell_details[new_i][new_j].g = g_new
                        cell_details[new_i][new_j].h = h_new
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j

    # If the destination is not found after visiting all cells
    if not found_dest:
        print("Failed to find the destination cell")

# the row/col are not lat/long
# wind directions need better no sail zone directions
def map_tile_status(current_row, current_col, invest_row, invest_col):
    #invest tile is above current tile
    if(current_row < invest_row and current_col == invest_col):
        if get_wind_direction(invest_row, invest_col) > 135 and if get_wind_direction(invest_row, invest_col) < 225:
            return False
        else:
            return True 
    #invest tile is below current tile
    if get_wind_direction(invest_row, invest_col) > 0 and get_wind_direction(invest_row, invest_col) < 45 or get_wind_direction(invest_row, invest_col) > 315 and  get_wind_direction(invest_row, invest_col) < 360:
        if get_wind_direction(invest_row, invest_col) == 0:
            return False
        else:
            return True 
    #invest tile is right of current tile
    if get_wind_direction(invest_row, invest_col) > 225 and get_wind_direction(invest_row, invest_col) < 315:
        if get_wind_direction(invest_row, invest_col) == 270:
            return False
        else:
            return True 
    #invest tile is left of current tile
    if get_wind_direction(invest_row, invest_col) > 45 and get_wind_direction(invest_row, invest_col) < 135:
        if get_wind_direction(invest_row, invest_col) == 90:
            return False
        else:
            return True 
    #invest tile is up-right of current tile
    if(current_row > invest_row and current_col < invest_col):
        if get_wind_direction(invest_row, invest_col) == 225:
            return False
        else:
            return True 
    #invest tile is up-left of current tile
    if(current_row > invest_row and current_col > invest_col):
        if get_wind_direction(invest_row, invest_col) == 135:
            return False
        else:
            return True 
    #invest tile is down-right of current tile
    if(current_row < invest_row and current_col < invest_col):
        if get_wind_direction(invest_row, invest_col) == 315:
            return False
        else:
            return True 
    #invest tile is down-left of current tile
    if(current_row < invest_row and current_col > invest_col):
        if get_wind_direction(invest_row, invest_col) == 45:
            return False
        else:
            return True 


def update_grid(grid, row, col):
    # Update surrounding cells based on conditions
    for x in range(-1, 2):  # Loop through neighbors
        for y in range(-1, 2):
            if x == 0 and y == 0:  # Skip the current cell
                continue
            new_row = row + x
            new_col = col + y
            if is_valid(new_row, new_col):  # Ensure it's within grid bounds
                if not map_tile_status(row, col, new_row, new_col):
                    grid[new_row][new_col] = 0  # Mark as blocked
                else:
                    grid[new_row][new_col] = 1  # Keep unblocked
    print(grid)
    return grid 

# Driver Code
def main():
    # Define the source and destination
    src = [5, 5]
    dest = [0, 0]

    # Initialize the grid
    grid = [[1 for _ in range(COL)] for _ in range(ROW)]

    # Update the grid based on the starting position
    grid = update_grid(grid, src[0], src[1])

    # Run the A* search algorithm
    a_star_search(grid, src, dest)

if __name__ == "__main__":
    main()
=======
def to_cartesian(latlongs, offset):
    radius = 6371

    latlongs = np.array(latlongs)
    lats =  latlongs[:, 0]
    longs = latlongs[:, 1]
    
    x_coords = radius * (math.pi / 180) * math.cos(math.radians(offset[0])) * (longs - offset[1])
    y_coords = radius * (math.pi / 180) * (lats - offset[0])

    return list(zip(x_coords, y_coords))

def match_coords(matrix_coords, response_coords, response_vectors):
    vectors = []
    tree = cKDTree(response_coords)

    for matrix_coord in matrix_coords:
        dist, i = tree.query(matrix_coord)
        vectors.append(response_vectors[i])
    
    return vectors

top_left = (42.45343636267326, -76.51696225640684)
bot_right = (42.43163543249084, -76.47096590129155)
lats, longs, num_rows, num_cols = make_coords(top_left, bot_right, 1)
big_lats, big_longs, big_num_rows, big_num_cols = make_coords(top_left, bot_right)
response_latlongs, response_vectors = get_wind(big_lats, big_longs)

matrix_coords = to_cartesian(list(zip(lats, longs)), top_left)
response_coords = to_cartesian(response_latlongs, top_left)
matrix_vectors = match_coords(matrix_coords, response_coords, response_vectors)
vector_matrix = np.array(matrix_vectors).reshape(num_rows, num_cols, 2)
>>>>>>> d10d0237c2da76646944b11e3a6843650a48437d