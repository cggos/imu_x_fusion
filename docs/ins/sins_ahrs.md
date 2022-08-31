# SINS & AHRS

---

## SINS (Strapdown Inertial Navigation System)

Some basic processes for SINS based on **MARG (Magnetic, Angular Rate, and Gravity)** are implemented, including electromagnetic compass calibration, an **AHRS (attitude and heading reference system)** that use an extended Kalman filter (EKF), and trace tracking. Loosely coupling sensor fusion by using the minimum squared error (MSE) method is also implemented.

* [Strapdown inertial navigation](https://rotations.berkeley.edu/strapdown-inertial-navigation/)

* [Strapdown Inertial Navigation Technology, 2nd Edition](https://www.globalspec.com/reference/26556/203279/strapdown-inertial-navigation-technology-2nd-edition)

* [捷联惯导算法心得](https://www.amobbs.com/thread-5492189-1-1.html)

* [从零开始的 IMU 状态模型推导](https://fzheng.me/2016/11/20/imu_model_eq/)

* [深度解析卡尔曼滤波在IMU中的使用](http://stevenshi.me/2018/05/03/kalman-filter-implement/)


## AHRS

Attitude and Heading Reference System

An AHRS consists of sensors on three axes that provide attitude information for aircraft, including roll, pitch and yaw. These are sometimes referred to as **MARG (Magnetic, Angular Rate, and Gravity)** sensors and consist of either solid-state or microelectromechanical systems (MEMS) **gyroscopes, accelerometers and magnetometers**.

The key difference between an IMU and an AHRS is the addition of an on-board processing system in an AHRS, which provides attitude and heading information.

* [Sensor Fusion Algorithms](https://learn.adafruit.com/ahrs-for-adafruits-9-dof-10-dof-breakout/sensor-fusion-algorithms): There are a variety of sensor fusion algorithms out there, but the two most common in small embedded systems are the **Mahony and Madgwick filters**.

* [Open source IMU and AHRS algorithms](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

* [psiphi75/ahrs](https://github.com/psiphi75/ahrs): AHRS (Attitude Heading Reference Systems) calculation for JavaScript


## IMU Attitude Calculation

* [IMU Data Fusing](http://www.olliw.eu/2013/imu-data-fusing/): Complementary, Kalman, and Mahony Filter

* [cggos/imu_tools](https://github.com/cggos/imu_tools): ROS tools for IMU devices

* [IMU-sensor-fusion-with-linear-Kalman-filter (mathworks)](https://www.mathworks.com/matlabcentral/fileexchange/70093-imu-sensor-fusion-with-linear-kalman-filter): Reads IMU sensor (acceleration and velocity) wirelessly from the IOS app 'Sensor Stream' to a Simulink model and filters an orientation angle in degrees using a linear Kalman filter.


## Dead Reckoning

In navigation, dead reckoning is the process of calculating one's current position by using a previously determined position, or fix, and advancing that position based upon known or estimated speeds over elapsed time and course. 
