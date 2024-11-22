import math
import numpy as np
import openmeteo_requests

def make_coords(top_left, bot_right, tile_size=1.0):
    tile_lat = tile_size * (1 / 110.574)
    avg_lat = (top_left[0] + bot_right[0]) / 2
    tile_long = tile_size * (1 / (111.320 * math.cos(math.radians(avg_lat))))

    lat_delta = abs(bot_right[0] - top_left[0])
    long_delta = abs(bot_right[1] - top_left[1])

    num_rows = math.ceil(lat_delta / tile_lat)
    num_cols = math.ceil(long_delta / tile_long)
    lats = []
    longs = []

    for i in range(num_rows):
        for j in range(num_cols):
            new_lat = top_left[0] - (tile_lat / 2) - (tile_lat * i)
            new_long = top_left[1] + (tile_long / 2) + (tile_long * j)
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

top_left = (42.454862524617326, -76.47741838081413)
bot_right = (42.45394077761382, -76.47648601746894)
big_lats, big_longs, big_num_rows, big_num_cols = make_coords(top_left, bot_right)
response_coords, wind_vectors = get_wind(big_lats, big_longs)
lats, longs, num_rows, num_cols = make_coords(top_left, bot_right, 0.01)
print(response_coords)
print(wind_vectors)
print(list(zip(lats, longs)))