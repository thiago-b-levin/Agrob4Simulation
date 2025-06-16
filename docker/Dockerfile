ARG ROS_DISTRO=jazzy

#### ROS base install
FROM osrf/ros:${ROS_DISTRO}-desktop AS ros_base
ARG DEBIAN_FRONTEND=noninteractive
ARG UID=1000
ARG DOCKERUSER=
ARG DOCKERUSERCOMMENT=
ARG ROS_DISTRO=jazzy
ARG ROS_TESTING=0
ARG NVIDIA_DRIVER_VERSION=550
ARG WEBOTS_VERSION=2023b

# User setup
RUN if [ "${ROS_DISTRO}" != "humble" ]; then userdel -r ubuntu; fi
RUN useradd -d /${DOCKERUSER} -m \
            -u ${UID} -U \
            -s /usr/bin/bash \
            -c "${DOCKERUSERCOMMENT}" ${DOCKERUSER} && \
    echo "${DOCKERUSER} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    usermod -a -G video ${DOCKERUSER} && \
    usermod -a -G dialout ${DOCKERUSER}

# ROS testing respository setup (conditional)
RUN [ "$ROS_TESTING" -eq "1" ] && echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list || true

# Base dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    nano

# ROS2 dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-gazebo* \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-velodyne-simulator \
    ros-${ROS_DISTRO}-clearpath-simulator \
    gdb

# HOTFIX: https://github.com/ros-controls/ros2_control/pull/1960
RUN [ "${ROS_DISTRO}" = "humble" ] && wget -O /tmp/hotfix.deb http://snapshots.ros.org/humble/2024-08-28/ubuntu/pool/main/r/ros-humble-hardware-interface/ros-humble-hardware-interface_2.43.0-1jammy.20240823.145349_amd64.deb && \
apt install -y --allow-downgrades /tmp/hotfix.deb && \
rm -f /tmp/hotfix.deb || true

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /${DOCKERUSER}/.bashrc


#### Webots base install
FROM ros_base AS webots-base

# Webots installation
RUN apt update && apt install -y wget && \
    wget https://github.com/cyberbotics/webots/releases/download/R${WEBOTS_VERSION}/webots_${WEBOTS_VERSION}_amd64.deb -O /tmp/webots.deb && \
    apt install -y /tmp/webots.deb && \
    rm /tmp/webots.deb

COPY ./docker/webots.conf /${DOCKERUSER}/.config/Cyberbotics/Webots-R${WEBOTS_VERSION}.conf
ENV WEBOTS_HOME="/usr/local/webots"
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/webots/lib/controller

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-webots-ros2

# Nvidia OpenGL libraries
RUN apt-get update && apt-get install -y \
    libnvidia-gl-${NVIDIA_DRIVER_VERSION} \
    nvidia-utils-${NVIDIA_DRIVER_VERSION} \
    nlohmann-json3-dev \
    libmosquitto-dev \
    mesa-utils


#### Python dependencies 
FROM webots-base AS python-dev

# Fix because of compiled numpy 1.x version instead of 2.x inside the matplotlib version
RUN apt-get update && apt-get remove -y python3-matplotlib

RUN pip3 install --no-cache-dir \
    numpy \
    matplotlib

RUN pip3 install --no-cache-dir transforms3d==0.4.2

RUN pip3 install --no-cache-dir \
    torch \
    --index-url https://download.pytorch.org/whl/cpu

RUN pip3 install --no-cache-dir \
    opencv-python

RUN pip3 install --no-cache-dir \
    tensorboard

RUN pip3 install --no-cache-dir \
    stable-baselines3

RUN pip3 install --no-cache-dir \
    gymnasium

# Fix permission error of matplotlib cache
ENV MPLCONFIGDIR=/tmp/matplotlib-cache

#### In the end of the DockerFile
ENV USERNAME=default
USER ${DOCKERUSER}
WORKDIR /${DOCKERUSER}/ros2_ws
