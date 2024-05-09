#!/bin/bash

# source the ros2 workspace
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/local_setup.bash 

# launch mavros node
ros2 launch drone_control_pkg mavros.launch.py &

# wait for mavros node to finish
sleep 10

# start the setup script. sets the mode to GUIDED and request messages from fcu.
ros2 run drone_control_pkg drone_setup_node &

wait $!
