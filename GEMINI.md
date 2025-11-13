# Gemini Code Assistant Context

This document provides context for the Gemini Code Assistant to understand the structure and conventions of this ROS2 project.

## Project Overview

This is a ROS2 project for a differential drive mobile robot. The project is written in a mix of Python and C++, and it integrates various hardware components for localization, mapping, and navigation.

The project is structured as a ROS2 workspace with the following packages:

*   **motor_control**: A Python-based package for controlling the robot's motors via a serial connection to an ESP32 microcontroller.
*   **nav2**: A Python-based package for launching and configuring the Nav2 stack for autonomous navigation.
*   **ros-imu-bno055**: A C++-based package for interfacing with a BNO055 IMU.
*   **sllidar_ros2**: A C++-based package for interfacing with a SLAMTEC RPLIDAR.

The robot uses `robot_localization` to fuse odometry and IMU data for state estimation.

## Building and Running

### Building the Project

The project is built using `colcon`, the standard ROS2 build tool. A convenience script is provided to automate the build process.

To build the project, run the following command from the project root:

```bash
./build_ros2.sh
```

This script will clean the workspace, build all packages, and source the setup files.

### Running the Robot

The project uses a series of launch files to start the robot in different modes.

**Core System Launch:**

To bring up the robot's core systems (motors, IMU, LIDAR, and state estimation), run:

```bash
ros2 launch motor_control full_system.launch.py
```

**Manual Control:**

To manually control the robot with the keyboard, run the following command in a separate terminal:

```bash
ros2 launch motor_control keyboard_control.launch.py
```

**SLAM (Mapping):**

To start the robot in mapping mode, run:

```bash
ros2 launch nav2 mapping.launch.py
```

You can then use keyboard control to drive the robot and build a map.

**Autonomous Navigation:**

To start the robot in autonomous navigation mode, run:

```bash
ros2 launch nav2 autonomous_navigation.launch.py
```

This will launch the Nav2 stack and load the pre-existing map.

## Development Conventions

*   The project is a ROS2 workspace, and all development should follow ROS2 conventions.
*   Python packages are built with `ament_python`, and C++ packages are built with `ament_cmake`.
*   Configuration files are stored in the `config` directory of each package.
*   Launch files are stored in the `launch` directory of each package.
*   The `README.md` file provides detailed instructions for hardware setup and usage. Please refer to it for more information.
