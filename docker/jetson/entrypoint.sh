#!/bin/bash

# source the ros2 workspace
source /opt/ros/humble/install/setup.bash && source /root/ros2_ws/install/local_setup.bash

# start the zed node
ros2 launch drone_vision_pkg zed_launch.py camera_model:=zedm &

# wait for the node to start
sleep 10

# launch the ekf node
ros2 launch drone_vision_pkg ekf.launch.py &

# keep the container running
wait $!