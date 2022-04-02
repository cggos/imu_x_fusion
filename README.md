# IMU + X Loosely-Coupled Fusion Localization

* IMU + GPS
  - [x] ESKF: [Multi-Sensor Fusion: IMU and GPS loose fusion based on ESKF](https://cggos.github.io/sensorfusion-imu-gnss.html)

* IMU + 6DoF Odom (Stereo Visual Odometry)
  - [x] ESKF: [IMU and 6 DoF Odometry (Stereo Visual Odometry) Loosely-Coupled Fusion Localization based on ESKF (Presentation)](https://www.researchgate.net/publication/353330937_IMU_and_6_DoF_Odometry_Stereo_Visual_Odometry_Loosely-Coupled_Fusion_Localization_based_on_ESKF)
  - [x] IEKF: https://cggos.github.io/map-mle-ols-gn-iekf-ekf.html#iekf
  - [x] UKF(including JUKF and SVD-UKF): [IMU and 6 DoF Odometry (Stereo Visual Odometry) Loosely-Coupled Fusion Localization based on UKF](https://www.researchgate.net/publication/358891263_IMU_and_6_DoF_Odometry_Stereo_Visual_Odometry_Loosely-Coupled_Fusion_Localization_based_on_UKF)
  - [x] MAP(User-defined L-M, Ceres-Solver)

-----

[TOC]

## Requirements

tested on Ubuntu 16.04 and Ubuntu 18.04

* Ubuntu 16.04 & ROS Kinetic

  * OpenCV 3
  * ROS package: nmea_navsat_driver
  * GeographicLib 1.50.1 (built from souce code, cmake 3.18.0 tested)


* Ubuntu 18.04 & ROS Melodic

  * OpenCV 3
  * ROS package: nmea_navsat_driver
  * GeographicLib 1.49
    ```sh
    sudo apt install libgeographic-dev
    ```

## Build

```sh
mkdir -p ws_msf/src
cd ws_msf/src
git clone xxx
cd ..
catkin_make -j4 # error happened when using the default cmake 3.5.1 on Ubuntu 16.04, upgrade it
# or
catkin build -j4
```

## Run

### IMU + GNSS

test data: [utbm_robocar_dataset_20180719_noimage.bag](https://lcas.lincoln.ac.uk/owncloud/index.php/s/KfItDFgwwis5Xrk)

* [sensor_msgs/Imu] /imu/data: 100 hz
* [nmea_msgs/Sentence] /nmea_sentence: 15 hz
* [sensor_msgs/NavSatFix] /fix: 5 hz
* [nav_msgs/Path] /nav_path: 63 hz

```sh
roslaunch imu_x_fusion imu_gnss_fusion.launch
rosbag play -s 25 utbm_robocar_dataset_20180719_noimage.bag
```

ROS graph and path on rviz:

<p align="center">
  <img src="imgs/rosgraph_imu_gnss.jpg"/>
  <img src="imgs/run_imu_gnss_fusion.jpg"/>
</p>

plot the result path (fusion_gps.csv & fusion_state.csv) on Google Map using the scripts `folium_csv.py`:

<p align="center">
  <img src="imgs/google_map.jpg"/>
</p>

### IMU + 6DoF Odom

#### VO: ORB-SLAM2 (Stereo) + EuRoC V1_01_easy.bag

```sh
roslaunch imu_x_fusion imu_vo_fusion.launch [est:=ekf, ukf or map]
```

##### run ORB-SLAM2 (Stereo) and play back bag file

```sh
# https://github.com/cggos/orbslam2_cg
# pose cov:
# sigma_pv: 0.001
# sigma_rp: 0.5
# sigma_yaw: 0.5
roslaunch orbslam2_ros run_stereo_euroc.launch

rosbag play V1_01_easy.bag
```

<p align="center">
  <img src="imgs/rosgraph_imu_vo.png"/>
</p>

results(Green path: estimated pose; Red path: pose of VO):

<p align="center">
  <img src="imgs/run_imu_vo_fusion.png"/>
</p>

##### Use the recorded bag file directly

Download [orbslam2_v101easy.bag](http://gofile.me/5lGth/wYejg2zlD)

```sh
rosbag play orbslam2_v101easy.bag
```

#### VO: ORB-SLAM2 (Stereo) + MYNTEYE-S1030 Camera

```sh
# TODO: Test
roslaunch imu_x_fusion imu_vo_fusion_mynteye.launch

roslaunch mynt_eye_ros_wrapper mynteye.launch
```

## Features

* Sensors
  - [x] IMU
    - [x] Numerical Integration (TODO: RK4) 
    - [ ] Pre-Integration
  - [ ] Wheel Odometer
  - [ ] Manometer
  - [x] GPS
  - [x] 6DoF Odom (Stereo Visual Odometry)
  - [ ] LiDAR
  - [ ] Multi-modal data

* State Estimation
  - [x] EKF
    - [x] ESKF
    - [x] IEKF
  - [x] UKF
    - [x] JUKF
    - [x] SVD-UKF
  - [ ] Particle Filter
  - [ ] MAP
    - [x] User-defined G-N
    - [x] Ceres-Solver
    - [ ] G2O
    - [ ] GTSAM

* Local / Global Angular Error (Rotation Perturbation)
  - [x] Local Angular Error (OK)
  - [x] Global Angular Error (TODO: why Poor)
  ```cpp
  enum ANGULAR_ERROR { LOCAL_ANGULAR_ERROR, GLOBAL_ANGULAR_ERROR };
  ```

* Debug
  - [x] Check Measurement Jacobian Matrix
  
## Community

* Welcom to join [the Discord channel #multi-sensor-fusion](https://discord.gg/2RV2vKmdpa)
