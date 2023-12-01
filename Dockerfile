FROM osrf/ros:melodic-desktop-full
# Use osrf/ros instead of ros:melodic for pre-installed development packages
# Ref: https://github.com/osrf/docker_images#osrf-profile

# Ref: https://robotics.stackexchange.com/a/105553
# Ref: https://github.com/osrf/docker_images/issues/697#issuecomment-1819626877
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
# Setup dependencies for install script
RUN apt-get update && \
    apt-get install -y \
        wget \
        curl \
        rfkill \
        software-properties-common \
        systemd \
    && rm -rf /var/lib/apt/lists/*
# Ref: https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# Install husky
COPY thirdparty /root/catkin_ws/thirdparty
RUN . /opt/ros/melodic/setup.sh && \
    mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws && \
    apt-get update && \
    catkin_make && \
    bash ./thirdparty/install.sh -d desktop -r husky && \
    apt-get install -y ros-melodic-teleop-twist-keyboard && \
    rm -rf /var/lib/apt/lists/*
# Since we will install udev rules on host, we need to remove the udev rules in the container to prevent conflict.
RUN rm /lib/udev/rules.d/60-ros-melodic-husky-bringup.rules
