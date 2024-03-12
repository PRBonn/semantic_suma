# CUDA 10.1.243, cuDNN 7.6.2, TensorRT 5.1.5
FROM nvcr.io/nvidia/tensorrt:19.08-py3

ARG DEBIAN_FRONTEND=noninteractive

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
    
# Dependencies
RUN apt-get -y update &&\
    apt-get -y upgrade &&\
    apt-get install -y apt-utils build-essential cmake curl libgtest-dev libeigen3-dev libboost-all-dev qtbase5-dev libglew-dev qt5-default git libyaml-cpp-dev libopencv-dev vim

RUN apt-get -y install software-properties-common &&\
    add-apt-repository ppa:borglab/gtsam-release-4.0 &&\
    apt-get -y update &&\
    apt-get install -y libgtsam-dev libgtsam-unstable-dev

# ROS melodic        
RUN apt-get install -y lsb-release &&\
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&\
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - &&\
    apt-get update -y &&\
    apt install -y ros-melodic-desktop-full
RUN apt-get install -y python3-catkin-pkg python3-wstool python3-rosdep ninja-build stow python3-rosinstall python3-rosinstall-generator
RUN rosdep init && rosdep update
    
RUN python3 -m pip install --upgrade pip
RUN pip install catkin_tools catkin_tools_fetch empy trollius numpy rosinstall_generator

# RangeNetLib & Suma++
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src
RUN git clone https://github.com/ros/catkin.git &&\
    git clone https://github.com/PRBonn/rangenet_lib.git
RUN sed -i 's/builder->setFp16Mode(true)/builder->setFp16Mode(false)/g' /catkin_ws/src/rangenet_lib/src/netTensorRT.cpp
RUN cd ../ && catkin init &&\
    catkin build rangenet_lib
RUN git clone https://github.com/PRBonn/semantic_suma.git &&\
    sed -i 's/find_package(Boost REQUIRED COMPONENTS filesystem system)/find_package(Boost 1.65.1 REQUIRED COMPONENTS filesystem system serialization thread date_time regex timer chrono)/g' /catkin_ws/src/semantic_suma/CMakeLists.txt &&\
    catkin init &&\
    catkin deps fetch &&\
    cd glow && git checkout e66d7f855514baed8dca0d1b82d7a51151c9eef3 && cd ../ &&\
    catkin build --save-config -i --cmake-args -DCMAKE_BUILD_TYPE=Release -DOPENGL_VERSION=430 -DENABLE_NVIDIA_EXT=YES
    
# Download model
WORKDIR /catkin_ws/src/semantic_suma
RUN wget https://www.ipb.uni-bonn.de/html/projects/semantic_suma/darknet53.tar.gz &&\
    tar -xvf darknet53.tar.gz
    
WORKDIR /catkin_ws/src
