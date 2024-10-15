# GSCAM Installation Guide

This guide will walk you through the process of setting up the `gscam` package in a ROS2 workspace. If you encounter errors related to missing dependencies, follow the steps below to resolve them.

## Prerequisites

Ensure that you have ROS2 installed on your system. Replace `humble` with your respective ROS2 distribution (e.g., `foxy`, `galactic`, etc.) in the following commands.

1. Update your package lists:
    ```bash
    sudo apt update
    ```

2. Source your ROS2 environment:
    ```bash
    source /opt/ros/humble/setup.bash
    ```

## Step-by-step Setup

### 1. Create a ROS2 Workspace

If you don't have a workspace already, create one:
```bash
mkdir -p ~/bittle_ws/src
cd ~/bittle_ws
colcon build
source install/setup.bash
```

```bash

cd ~/bittle_ws/src
git clone https://github.com/ros-drivers/gscam.git
```

3. Install Dependencies
If you encounter errors related to missing packages, follow these instructions:

a. Install GStreamer Development Libraries
```bash

sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-good
```
b. Install ament_cmake
Make sure the `ament_cmake` package is installed:
```bash

sudo apt install ros-humble-ament-cmake
```
c. Install camera_calibration_parsers
If you encounter errors related to  `camera_calibration_parsers` , install it as follows:
```bash

sudo apt install ros-humble-camera-calibration-parsers
```
d. Install camera_info_manager
If you encounter errors related to `camera_info_manager`, install it as follows:
```bash

sudo apt install ros-humble-camera-info-manager
```
4. Build the Workspace
After installing the required dependencies, you can build your workspace:
```bash

cd ~/bittle_ws
colcon build
```
5. Source the Setup File
After a successful build, source the workspace:
```bash

source install/setup.bash
```
