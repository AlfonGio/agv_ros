#!/bin/bash

# Navigating to the agv_ros directory
cd ~/workspaces/agv_ros-dev/

# Sourcing the setup script
source install/setup.bash

# Launching the ROS2 application
ros2 launch agv_ros launch_robot.launch.py slam:=True
