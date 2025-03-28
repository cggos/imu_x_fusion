import numpy as np
import matplotlib.pyplot as plt

import pymap3d as pm


def lla_to_enu(lat, lon, alt, lat0, lon0, alt0):
    east, north, up = pm.geodetic2enu(lat, lon, alt, lat0, lon0, alt0)
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
    lla = [float(data[2]), float(data[3]), float(data[4])]  # lat, lon, height
    if n == 1:
        anchor_lla = lla
    east, north, up = lla_to_enu(
        lla[0], lla[1], lla[2], anchor_lla[0], anchor_lla[1], anchor_lla[2]
    )
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
