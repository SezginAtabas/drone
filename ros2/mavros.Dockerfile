ARG ROS_VERSION=humble
ARG ROS_IMAGE=-ros-base-jammy
FROM ros:$ROS_VERSION$ROS_IMAGE

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

RUN pip install numpy transforms3d setuptools==58.2.0


RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    ros2 run mavros install_geographiclib_datasets.sh

# copy the package
COPY drone_control_pkg /root/ros2_ws/src

# build
WORKDIR /root/ros2_ws/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    ros2 run mavros install_geographiclib_datasets.sh

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \ 
    && echo "source /root/ros2_ws/install/local_setup.bash" >> ~/.bashrc

CMD ["bash"]