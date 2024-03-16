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
RUN apt-get install -y python3-pip \ 
    tigervnc*\
    novnc
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
# USER $USERNAME
# CMD ["/bin/bash"]
LABEL maintainer="ken20020209 <ken20020209@gmail.com>"

ADD /src /src
RUN . /opt/ros/foxy/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y &&\
    colcon build

ENV ROS_DISTRO=foxy
ENV ROS_DOMAIN_ID=16

# CMD [ "source","/install/setup.bash" ]
# CMD [ "ros2","launch","basic","Connector.launch.py" ]

COPY /docker/entrypoint.sh /
ENTRYPOINT [ "sh","/entrypoint.sh" ]
CMD [ "/bin/bash" ]
