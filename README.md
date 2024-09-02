
# ros2-auto-drone

This project uses nvidia jetson on a drone as companion computer to detect and track objects. 
An NVIDIA Jetson equipped with a ZED camera detects and tracks objects using a trained YOLO model, optimized as a TensorRT engine for enhanced performance. The YOLO model generates bounding boxes, which are then fed into the Norfair tracker for precise object tracking. The tracked object coordinates are used to generate position commands, which are transmitted to the drone's flight controller via MAVROS for real-time navigation

## Features

- **Real-time Object Detection**: Utilizes the Ultralytics YOLO model optimized with TensorRT for efficient object detection on NVIDIA Jetson hardware.
- **Object Tracking**: Implements Norfair for robust tracking of detected objects across frames.
- **Autonomous Navigation**: Integrates with ArduPilot and MAVROS to enable autonomous drone navigation based on the detected objects.
- **Stereo Vision**: Uses the ZED Mini Camera for depth perception and enhanced environmental understanding.
- **ROS2 Integration**: Fully compatible with ROS2 for modularity and scalability.
- **Dockerized Environment**: Easily deploy and manage the project using Docker.

## Installation

1. [Setup Docker](https://docs.docker.com/get-started/get-docker/)

2. Clone the repository:
  ```bash
  git clone https://github.com/SezginAtabas/ros2-auto-drone.git
  ```

3. Configure ardupilot and zed camera parameters according to your setup. These parameters can be configured using the yaml files in the ros2 packages.

4. Build the docker images based on your platform.
   


## Hardware

- Stereolabs zed mini camera
- Nvidia jetson orin nano 
- CubePilot Cube Orange
- Standart Quadcopter

## Software

- [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
- [ros2](https://github.com/ros2/ros2)
- [docker](https://www.docker.com/)
- [ultralytics](https://github.com/ultralytics/ultralytics)
- [TensorRT](https://github.com/NVIDIA/TensorRT)
- [Mavros](https://github.com/mavlink/mavros)
- [ardupilot](https://github.com/ArduPilot/ardupilot)
- [norfair](https://github.com/tryolabs/norfair?tab=readme-ov-file)
- [pycuda](https://github.com/inducer/pycuda)
