FROM rockstarartist/mavros2:humble-arm64

ENV ROS_DISTRO=humble

WORKDIR /root/ros2_ws/src

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*


# install needed stuff with pip
RUN pip install numpy transforms3d setuptools==58.2.0

# copy the package
COPY drone_control_pkg /root/ros2_ws/src

# build
WORKDIR /root/ros2_ws/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build

ADD ros_entrypoint.sh /bash_scripts/
RUN chmod +x /bash_scripts/ros_entrypoint.sh
ENTRYPOINT [ "/bash_scripts/ros_entrypoint.sh" ]

CMD ["bash"]

# docker run -v /dev/shm:/dev/shm --net=host -it mavros:test2 ros2 launch drone_control_pkg mavros.launch.py fcu_url:=udp://:14550@localhost:5762




