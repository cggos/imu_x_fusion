# GNSS

---

## Overview

* [An Introduction to GNSS](https://www.novatel.com/an-introduction-to-gnss)
* [Navipedia](https://gssc.esa.int/navipedia/index.php/Main_Page)

GNSS Systems:

* GPS (United States)
* GLONASS (Russia)
* BeiDou (China)
* Galileo GNSS system (European Union)
* IRNSS regional navigation satellite system (India)
* QZSS regional navigation satellite system (Japan)

GNSS Coordinates:

* ECEF（地心地固坐标系）
* ECI（惯性坐标系）
* LLA 坐标系：Longitude（经度）、Latitude（纬度）和 Altitude（高度）
* 站心坐标系：ENU（东北天坐标系）、NED（北东地坐标系）


## RTK

DGPS stands for Differential GPS. It is the GPS signal corrected by ground reference stations (which can estimate the ionosphere error on the GPS signal). Traditionally, DGPS accuracy can go down to sub-meter level. RTK technique allows DGPS accuracy to be at centimeter-level.

RTK stands for Real Time Kinematic. RTK algorithms make use of the carrier phase measurement to estimate accurately the distance from the receiver to the satellites. It allows an accuracy of 1-2 cm on position.


## GNSS in inertial navigation

Inertial navigation systems can take advantage of additional constellation to use more satellites and further improve signal robustness in harsh environments such as urban canyons, forests, mountains.

* [Aceinna/gnss-ins-sim](https://github.com/Aceinna/gnss-ins-sim): GNSS-INS-SIM is an GNSS/INS simulation project, which generates reference trajectories, IMU sensor output, GPS output, odometer output and magnetometer output.

* [aaronboda24/Loose-GNSS-IMU](https://github.com/aaronboda24/Loose-GNSS-IMU)

* [karanchawla/GPS_IMU_Kalman_Filter](https://github.com/karanchawla/GPS_IMU_Kalman_Filter)


## Software

* [GeographicLib](https://geographiclib.sourceforge.io/) is a small set of C++ classes for performing conversions between geographic, UTM, UPS, MGRS, geocentric, and local cartesian coordinates, for gravity (e.g., EGM2008), geoid height, and geomagnetic field (e.g., WMM2010) calculations, and for solving geodesic problems.

* [RTKLIB](http://www.rtklib.com/): An Open Source Program Package for GNSS Positioning

* [GPSoftNav](https://gpsoftnav.com/)

* [gpsd](https://gpsd.gitlab.io/gpsd/) is a service daemon that monitors one or more GPSes or AIS receivers attached to a host computer through serial or USB ports, making all data on the location/course/velocity of the sensors available to be queried on TCP port 2947 of the host computer.


## Books

* [北航 -- 数字导航中心 电子教材](http://dnc.buaa.edu.cn/xzzx/dzjc.htm)

* ***Global Navigation Satellite Systems, Inertial Navigation, and Integration, 3rd Edition*** ([link for code](http://bcs.wiley.com/he-bcs/Books?action=index&itemId=111844700X&bcsId=7868))

<p align="center">
  <img src="../img/gnss.jpg">
</p>
