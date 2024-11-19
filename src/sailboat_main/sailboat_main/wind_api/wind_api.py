import math
# import matplotlib.pyplot as plt

def make_coords(top_left, bot_right):
    km_lat = 1 / 110.574
    avg_lat = (top_left[0] + bot_right[0]) / 2
    km_long = 1 / (111.320 * math.cos(math.radians(avg_lat)))

    lat_delta = abs(bot_right[0] - top_left[0])
    long_delta = abs(bot_right[1] - top_left[1])

    num_rows = math.ceil(lat_delta / km_lat)
    num_cols = math.ceil(long_delta / km_long)
    lats = []
    longs = []

    for i in range(num_rows):
        for j in range(num_cols):
            new_lat = top_left[0] - (km_lat / 2) - (km_lat * i)
            new_long = top_left[1] + (km_long / 2) + (km_long * j)
            lats.append(new_lat)
            longs.append(new_long)

    return lats, longs

x, y = make_coords((42.48520133356404, -76.53276792062992), (42.46536390168541, -76.50524184695678))