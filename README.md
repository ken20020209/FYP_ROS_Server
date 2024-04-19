# FYP_ROS_Server
provide a websocket api to control robot
# requirement:
ros2 foxy

# build ros server


rosdep install --from-paths src --ignore-src -r -y

colcon build

# docker:

build:docker build -t ken20020209/fyp_ros_server:latest -f docker/dockerfile .

run:docker run -it --rm --net=host  ken20020209/fyp_ros_server:latest 
