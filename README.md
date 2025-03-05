# ESP32 C++ code for Controlling Spot
This project is based on [robot_arm_esp32_rtos](https://github.com/screamlab/robot_arm_esp32_rtos/tree/main) by Scream Lab.

This project provides ESP32 firmware for controlling a quadruped robot ("Spot") using C++ and ROS 2.

## Environment
* **OS**: ubuntu24.04
* **Build System**:platformio
* **Middleware**: ros2 jazzy

## Usage
Use PlatformIO to upload the code to ESP32.

Run the ROS 2 agent inside a Docker container:
```
# Serial micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev [YOUR BOARD PORT] -v6

#[YOUR BOARD PORT] with the serial port of your ESP32 (e.g., `/dev/ttyUSB0`).
```



## Resources
1. [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino)
2. [micro_ros_platformio](https://github.com/micro-ROS/micro_ros_platformio)
3. [mirco ros api doc](https://micro.ros.org/docs/tutorials/core/overview/)