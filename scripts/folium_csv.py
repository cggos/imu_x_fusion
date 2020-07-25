# -*- coding: utf-8 -*-
"""
plot lines on open street map
"""

# Make beautiful maps with Leaflet.js and Python
# conda install -c conda-forge folium
import folium

# points=[(31.949515,118.697229),
# (31.950135,118.696985),
# (31.950556,118.696913),
# (31.951091,118.697034),
# (31.951475,118.697531),
# (31.951647,118.698275),
# (31.951669,118.698371)]

points1 = []
points2 = []

f1 = open("./data/fusion_gps.csv")
f2 = open("./data/fusion_state.csv")

for line in f1:
    data = line.split(',')
    lat = float(data[1])
    lon = float(data[2])
    points1.append(tuple([lat, lon]))

for line in f2:
    data = line.split(',')
    lat = float(data[8])
    lon = float(data[9])
    points2.append(tuple([lat, lon]))

# initiate to plotting area
my_map = folium.Map(
    location=[points1[0][0],points1[-1][1]], 
    zoom_start=15, 
    tiles='OpenStreetMap',
    png_enabled=True, 
    prefer_canvas=True)

folium.PolyLine(points1, color="red", weight=8.0, opacity=1).add_to(my_map)

folium.PolyLine(points2, color="blue", weight=4.0, opacity=1).add_to(my_map)

# for point in points:
#       folium.Marker(point).add_to(my_map)
      
my_map.save("./map_with_paths.html")
