#!/bin/bash
trap : SIGTERM SIGINT

roscore &
ROSCORE_PID=$!
sleep 1

rviz -d ../../rviz/imu_vo_fusion.rviz &
RVIZ_PID=$!

docker run \
  -it \
  --rm \
  --net=host \
  -v /root/ws_msf/src/imu_x_fusion/ \
  /bin/bash -c \
  "cd /root/ws_msf/; \
  catkin config \
        --env-cache \
        --extend /opt/ros/$ROS_DISTRO \
        --cmake-args \
        -DCMAKE_BUILD_TYPE=Release; \
     catkin build; \
     source devel/setup.bash; \
     roslaunch imu_x_fusion imu_vo_fusion.launch est:=ekf"

wait $ROSCORE_PID
wait $RVIZ_PID

if [[ $? -gt 128 ]]
then
    kill $ROSCORE_PID
    kill $RVIZ_PID
fi
