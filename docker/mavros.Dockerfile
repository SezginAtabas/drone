FROM ros:humble


ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends\
    python3-pip \ 
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

# geo datasets needed for mavros
COPY install_geographiclib_datasets.sh /bash_scripts/
RUN chmod +x /bash_scripts/install_geographiclib_datasets.sh
RUN /bash_scripts/install_geographiclib_datasets.sh

# install needed stuff with pip
RUN pip install numpy==1.24.4 transforms3d==0.4.1 setuptools==58.2.0

# copy the need pkgs 
COPY drone_control_pkg /ros2_ws/src/drone_control_pkg

# build
WORKDIR /opt/ros/${ROS_DISTRO}
RUN . /setup.sh && \
    cd /ros2_ws && \ 
    colcon build

# Source the workspace
RUN echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc

COPY ros_entrypoint.sh /bash_scripts/
RUN chmod +x /bash_scripts/ros_entrypoint.sh
ENTRYPOINT [ "/bash_scripts/ros_entrypoint.sh" ]

CMD ["ros2", "launch", "mavros", "apm.launch"]

# docker run -v /dev/shm:/dev/shm --net=host -it mavros:test2 ros2 launch mavros apm.launch fcu_url:=udp://:14550@localhost:5762