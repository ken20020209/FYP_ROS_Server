# FYP_Server

requirement:
ros2 foxy

build ros server

cd ROS_Server_WS
colcon build

docker:

build:docker build -t test -f docker/Dockerfile .

run:docker run -it --rm test