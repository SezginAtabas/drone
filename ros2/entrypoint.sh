#!/bin/bash

# source the ros2 workspace
source /opt/ros/humble/install/setup.bash && source /root/ros2_ws/install/local_setup.bash

# wait for the node to start
sleep 10

# start the zed node
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm publish_tf:=false publish_map_tf:=false publish_ros_params_override_path:=/root/ros2_ws/src/drone_vision_pkg/config/zedm.yaml &

sleep 5

# launch the ekf node
ros2 launch drone_vision_pkg ekf.launch.py &

# keep the container running
wait $!