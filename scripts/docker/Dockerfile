# This is an auto generated Dockerfile for ros:robot
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:melodic-ros-base-bionic

# Set the working directory to /root
ENV DIRPATH /root
WORKDIR $DIRPATH

# install ros packages
RUN apt-get update && apt-get install -y ros-melodic-robot=1.4.1-0*
    # && rm -rf /var/lib/apt/lists/*

RUN apt install -y ros-melodic-nmea-navsat-driver libgeographic-dev

RUN apt install -y git cmake wget python-catkin-tools

RUN apt install -y libopencv-dev

# # Install OpenCV for Ubuntu 18.04
# RUN apt-get update && apt-get install -y \
#       build-essential cmake unzip pkg-config \
#       libjpeg-dev libpng-dev libtiff-dev \
#       libvtk6-dev \
#       libgtk-3-dev \
#       libatlas-base-dev gfortran
# 
# RUN git clone https://github.com/opencv/opencv.git
# RUN cd opencv && \
#       git checkout tags/3.3.1 && \
#       mkdir build -j$(nproc)
# 
# RUN git clone https://github.com/opencv/opencv_contrib.git
# RUN cd opencv_contrib && \
#       git checkout tags/3.3.1
# 
# RUN cd opencv/build && \
#       cmake -DCMAKE_BUILD_TYPE=Release \
#       -DCMAKE_INSTALL_PREFIX=/usr/local \
#       -D BUILD_opencv_python=OFF \
#       -D BUILD_opencv_python2=OFF \
#       -D BUILD_opencv_python3=OFF \
#       -DOPENCV_EXTRA_MODULES_PATH=$DIRPATH/opencv_contrib/modules .. && \
#       make -j$(nproc) install

RUN apt install -y libeigen3-dev libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev

RUN cd $DIRPATH
RUN wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz && tar xvzf ceres-solver-2.1.0.tar.gz && \
    cd ceres-solver-2.1.0 && mkdir build && cd build && cmake .. && make install -j$(nproc)

ENV TERM xterm
ENV PYTHONIOENCODING UTF-8

RUN cd $DIRPATH

# dataset
RUN mkdir $DIRPATH/dataset
RUN wget -P $DIRPATH/dataset https://cggos.i234.me:5001/fsdownload/wYejg2zlD/orbslam2_v101easy.bag

RUN mkdir -p $DIRPATH/ws_msf/src 
RUN git clone https://github.com/cggos/imu_x_fusion.git $DIRPATH/ws_msf/src/imu_x_fusion 
WORKDIR $DIRPATH/ws_msf 
RUN catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build -j4 && \
    sed -i '/exec "$@"/i \
            source "/root/ws_msf/devel/setup.bash"' /ros_entrypoint.sh
# RUN source $DIRPATH/ws_msf/devel/setup.bash
