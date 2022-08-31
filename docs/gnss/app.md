# App

---

## Android Apps

- GPS Test
- Cellular-Z


## Libs / SDK

* libgps-dev

* [GeographicLib](https://geographiclib.sourceforge.io/) is a small set of C++ classes for performing conversions between geographic, UTM, UPS, MGRS, geocentric, and local cartesian coordinates, for gravity (e.g., EGM2008), geoid height, and geomagnetic field (e.g., WMM2010) calculations, and for solving geodesic problems.

* [RTKLIB](http://www.rtklib.com/): An Open Source Program Package for GNSS Positioning

* [GPSoftNav](https://gpsoftnav.com/)

* [gpsd](https://gpsd.gitlab.io/gpsd/) is a service daemon that monitors one or more GPSes or AIS receivers attached to a host computer through serial or USB ports, making all data on the location/course/velocity of the sensors available to be queried on TCP port 2947 of the host computer.


## ROS package: nmea_navsat_driver

Package to parse NMEA strings and publish a very simple GPS message.

### nmea_topic_driver

- input: nmea_sentence (nmea_msgs/Sentence)
- output: fix (sensor_msgs/NavSatFix)

### nmea_serial_driver
