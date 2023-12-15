#!/bin/bash

# Navigating to the agv_ros directory
cd ~/workspaces/web_ros-dev/

# Sourcing the setup script
source install/setup.bash

# Launching the ROS2 application
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
