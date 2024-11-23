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