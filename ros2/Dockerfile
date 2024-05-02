FROM xtrana:zed_base


# install deps
RUN pip install transforms3d setuptools==58.2.0

# Copy the package
COPY drone_vision_pkg /root/ros2_ws/src/drone_vision_pkg

WORKDIR /root/ros2_ws

# build the package
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && \
    colcon build --packages-select drone_vision_pkg"

CMD ["bash"]
