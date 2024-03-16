# FYP_ROS_Server
provide a websocket api to control robot
# requirement:
ros2 foxy

# build ros server


rosdep install --from-paths src --ignore-src -r -y

colcon build

# docker:

build:docker build -t {name} -f docker/Dockerfile .

run:docker run -it --rm --net=host {name}
