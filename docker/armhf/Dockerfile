FROM jeguzzi/ros:armhf-kinetic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    ros-kinetic-mavros \
    ros-kinetic-mavlink \
    ros-kinetic-diagnostics \
    python-lxml \
    python-pip

# pip install lxml is failing on armhf

RUN pip install --upgrade pip
RUN pip install future
RUN pip install pymavlink

COPY . src/mavlink_limiter
RUN catkin build
