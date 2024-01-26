#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/install/setup.bash"
echo "1"
source "/root/ros2_ws/install/local_setup.bash" 
echo "2"

exec "$@"