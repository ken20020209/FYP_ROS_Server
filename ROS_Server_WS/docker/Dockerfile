FROM osrf/ros:foxy-desktop

# FROM ros:ROS_DISTRO
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
# USER $USERNAME
# CMD ["/bin/bash"]
# LABEL maintainer="ken20020209@gmail.com"

ADD /src /ROS_Server_WS/src
RUN . /opt/ros/foxy/setup.sh && \
    cd /ROS_Server_WS &&\ 
    rosdep install --from-paths src --ignore-src -r -y &&\
    colcon build

