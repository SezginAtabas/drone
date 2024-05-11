#!/bin/bash

# source the ros2 workspace
source /opt/ros/humble/install/setup.bash && source /root/ros2_ws/install/local_setup.bash 

# wait for mavros node
sleep 8

# static linked to zed camera NOTE: just for testing
ros2 run tf2_ros static_transform_publisher --x 0.0 --y 0.98 --z 0.079 --roll 0.022 --pitch -0.073 --yaw -0.165 --frame-id base_link --child-frame-id zed_camera_link &

sleep 1

# start the ekf node
ros2 launch drone_vision_pkg ekf.launch.py &

sleep 1

# pose sender
ros2 run drone_vision_pkg vision_pose_node &

sleep 1

# start the zed node
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm publish_tf:=false publish_map_tf:=false ros_params_override_path:=/root/ros2_ws/src/drone_vision_pkg/config/zedm.yaml &

# keep the container running
wait $!