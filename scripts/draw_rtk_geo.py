import math
import numpy as np
import matplotlib.pyplot as plt

from geopy import Point
from geopy.distance import geodesic


def lla_to_enu(point, ref_point):
    lat = point.latitude
    lon = point.longitude
    point.altitude = ref_point.altitude

    # Calculate the ENU coordinates
    delta_x = geodesic(ref_point, point).meters * math.cos(math.radians(lat))
    delta_y = geodesic(ref_point, point).meters * math.sin(math.radians(lat))
    delta_z = point.altitude

    # Calculate the ENU coordinates using the reference point as the origin
    east = delta_x * math.cos(math.radians(lon)) - delta_y * math.sin(math.radians(lon))
    north = delta_x * math.sin(math.radians(lon)) + delta_y * math.cos(
        math.radians(lon)
    )
    up = delta_z

    return east, north, up


if_rtk = "/home/gavin/projects/dataset/nj_ob_dabai/2025-02-28-10-25-45/sensor/rtk.txt"

rtk = open(if_rtk, 'r')

n = 0
rtk_type = []
rtk_time = []
rtk_enu_e = []
rtk_enu_n = []

anchor_lla = None

for line in rtk:
    n += 1
    data = line.split()
    rtk_type.append(int(data[0]))
    rtk_time.append(float(data[1]))
    lla = Point(float(data[2]), float(data[3]), float(data[4]))  # lat, lon, height
    if n == 1:
        anchor_lla = lla
    east, north, up = lla_to_enu(lla, anchor_lla)
    print(f"East: {east:.2f}, North: {north:.2f}, Up: {up:.2f}")
    rtk_enu_e.append(east)
    rtk_enu_n.append(north)


print("anchor_lla: {}".format(anchor_lla))

plt.figure()
plt.plot(rtk_enu_e, rtk_enu_n, 'r-', label='rtk')
plt.title('RTK ENU Odom')
plt.axis('equal')
plt.legend(loc='upper right')

plt.show()
