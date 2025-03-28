#!/usr/bin/env python
# coding=utf-8

import pyproj


def ecef_to_lla(x, y, z):
    '''
    x = 652954.1006
    y = 4774619.7919
    z = -4167647.7937
    '''

    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    lon, lat, alt = pyproj.transform(
        ecef, lla, x, y, z, radians=False
    )  # radians否用弧度返回值

    print('经度：', lon)
    print('纬度：', lat)
    print('高度：', alt)

    return lon, lat, alt


def lla_to_enu(lon, lat, height):
    # Create a projection object for WGS84 (latitude, longitude, height)
    wgs84 = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')

    # Create a projection object for ENU (East, North, Up)
    # enu = pyproj.Proj(proj='enu', ellps='WGS84', datum='WGS84', lon_0=lon, lat_0=lat)

    # Create a projection object for a local tangent plane (LTP) projection
    ltp = pyproj.Proj(proj='tmerc', ellps='WGS84', datum='WGS84', lon_0=lon, lat_0=lat)

    # Convert LLA to ENU
    east, north = pyproj.transform(wgs84, ltp, lon, lat, radians=False)
    up = height

    return east, north, up


# Example usage:
lon = -74.0060
lat = 40.7128
height = 10.0

east, north, up = lla_to_enu(lon, lat, height)

print(f"East: {east:.2f}, North: {north:.2f}, Up: {up:.2f}")
