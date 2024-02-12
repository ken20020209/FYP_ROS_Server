# FYP_Server

requirement:
ros2 foxy

build ros server

cd ROS_Server_WS

rosdep install --from-paths src --ignore-src -r -y

colcon build

docker:

build:docker build -t {name} -f docker/Dockerfile .

run:docker run -it --rm {name}
